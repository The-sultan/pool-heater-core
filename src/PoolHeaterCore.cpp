#include "PoolHeaterCore.h"
#include <cstring> // Required for std::memcpy, std::memset, and std::memcmp
#include <Arduino.h>

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
PoolHeaterCore::PoolHeaterCore(IHeaterHardware* hardware) : _hardware(hardware) {
    // Initialize frames with zeros to prevent garbage data on startup
    std::memset(_lastTempFrame, 0, sizeof(_lastTempFrame));
    std::memset(_lastCtrlFrame, 0, sizeof(_lastCtrlFrame));
}

// -----------------------------------------------------------------------------
// Initialization & Main Loop
// -----------------------------------------------------------------------------
void PoolHeaterCore::begin() {
    if (_hardware) {
        _hardware->begin();
        _hardware->enableRx(true);
    }
}

bool PoolHeaterCore::loop() {
    if (!_hardware) return false;

    // --- OPTIMISTIC STATE UPDATE ---
    // Initialize the return flag using our local predicted state change
    bool stateChanged = _pendingStateChange;
    _pendingStateChange = false;

    // --- ASYNC SEQUENCE HANDLING ---
    if (_currentSequence != AsyncSequence::NONE) {
        // Check if 500ms have passed without blocking the execution thread
        if (millis() - _sequenceTimer >= 500) {
            switch (_currentSequence) {
                case AsyncSequence::TEMP_CHANGE_WAIT_OFF:
                    // Heater is now off. Send the new target temperature.
                    sendControlFrame(2, _pendingValue); 
                    
                    if (_restorePower) {
                        // If it was originally on, start the wait to power it back on
                        _currentSequence = AsyncSequence::TEMP_CHANGE_WAIT_SET;
                        _sequenceTimer = millis();
                    } else {
                        // Finish sequence if it shouldn't restore power
                        _currentSequence = AsyncSequence::NONE;
                    }
                    break;
                
                case AsyncSequence::TEMP_CHANGE_WAIT_SET:
                    // Temperature is set. Restore power to ON.
                    sendControlFrame(0, 1); 
                    _currentSequence = AsyncSequence::NONE;
                    break;

                case AsyncSequence::MODE_CHANGE_WAIT_SET:
                    // Mode is set. Force power ON.
                    sendControlFrame(0, 1); 
                    _currentSequence = AsyncSequence::NONE;
                    break;

                default:
                    _currentSequence = AsyncSequence::NONE;
                    break;
            }
        }
    }

    // --- RAW FRAME PROCESSING ---
    uint8_t frame[10];

    // Process all available frames in the hardware buffer
    while (_hardware->receiveRawFrame(frame)) {
        
        // Temperature frame identifier (0xdd)
        if (frame[0] == 0xdd) {
            if (std::memcmp(_lastTempFrame, frame, 10) != 0) {
                std::memcpy(_lastTempFrame, frame, 10);
                parseTemperatureFrame(frame);
                stateChanged = true;
            }
        } 
        // Control frame identifier (0xd2)
        else if (frame[0] == 0xd2) {
            if (std::memcmp(_lastCtrlFrame, frame, 10) != 0) {
                std::memcpy(_lastCtrlFrame, frame, 10);
                parseControlFrame(frame);
                stateChanged = true;
            }
        }
    }

    // Returns true if a local optimistic change occurred OR a new different frame was parsed
    return stateChanged;
}

// -----------------------------------------------------------------------------
// High-Level User Interface
// -----------------------------------------------------------------------------
HeaterState PoolHeaterCore::getState() const {
    return _currentState;
}



void PoolHeaterCore::setMode(HeaterMode mode) {
    if (mode == HeaterMode::OFF) {
        sendControlFrame(0, 0);
        return;
    }

    uint8_t modeValue = 0;
    switch (mode) {
        case HeaterMode::AUTO: modeValue = 1; break;
        case HeaterMode::COOL: modeValue = 2; break;
        case HeaterMode::HEAT: modeValue = 3; break;
        default: return;
    }

    if (_currentSequence != AsyncSequence::NONE) return;

    // Cambiamos el modo e iniciamos la secuencia para prenderla después
    sendControlFrame(1, modeValue);
    _currentSequence = AsyncSequence::MODE_CHANGE_WAIT_SET;
    _sequenceTimer = millis();
}

void PoolHeaterCore::setTargetTemperature(int temp) {
    // Si ya hay una secuencia asíncrona en curso, ignoramos el nuevo comando
    if (_currentSequence != AsyncSequence::NONE) return;

    bool wasOn = _currentState.powerOn;
    _pendingValue = temp;
    _restorePower = wasOn;

    if (wasOn) {
        // Apagamos y delegamos la espera a la máquina de estados
        sendControlFrame(0, 0); 
        _currentSequence = AsyncSequence::TEMP_CHANGE_WAIT_OFF;
        _sequenceTimer = millis();
    } else {
        // Si ya estaba apagada, mandamos la temperatura directo
        sendControlFrame(2, temp); 
    }
}

// -----------------------------------------------------------------------------
// Low-Level Internal Functions
// -----------------------------------------------------------------------------
uint8_t PoolHeaterCore::calculateChecksum(const uint8_t* buffer, int len) {
    int res = 0;
    for (int i = 0; i < len; i++) {
        res += buffer[i];
    }
    return res;
}

void PoolHeaterCore::parseTemperatureFrame(const uint8_t* frame) {
    // Extract sensor data based on the reverse-engineered byte positions
    _currentState.currentTempIn  = frame[1];
    _currentState.currentTempOut = frame[2];
    _currentState.ambientTemp    = frame[5];
    _currentState.errorCode      = frame[7];
}

void PoolHeaterCore::parseControlFrame(const uint8_t* frame) {
    // Extract power state (Byte 7, bit 6)
    _currentState.powerOn = (frame[7] & 0x40) == 0x40;
    
    // OVERRIDE: If power is off, the mode is strictly OFF to the outside world
    if (!_currentState.powerOn) {
        _currentState.mode = HeaterMode::OFF;
    } else {
        // Only evaluate internal modes if the machine is actually running
        if ((frame[8] & 0x80) == 0x80) {
            _currentState.mode = HeaterMode::AUTO;
        } else if ((frame[7] & 0x20) == 0) {
            _currentState.mode = HeaterMode::COOL;
        } else {
            _currentState.mode = HeaterMode::HEAT;
        }
    }
    
    // Extract target temperature (Byte 2, bottom 7 bits)
    _currentState.targetTemp = frame[2] & 0x7F;
}

void PoolHeaterCore::sendControlFrame(uint8_t tag, uint8_t value) {
    if (!_hardware) return;

    // --- GUARD CLAUSE: Prevent sending if base hardware state is unknown ---
    if (_lastCtrlFrame[0] != 0xd2) {
        Serial.println("[Core] ERROR: Cannot send command, base hardware state is unknown.");
        return; 
    }

    Serial.printf("[Core] Preparing command. Tag: %d, Value: %d\n", tag, value);

    uint8_t sendBuffer[10];
    
    // Copy the last known control state to avoid overwriting other settings
    std::memcpy(sendBuffer, _lastCtrlFrame, 10);

    // Apply the specific modification based on the requested tag
    switch (tag) {
        case 0: // Power
            sendBuffer[7] = (sendBuffer[7] & 0xBF) | (value > 0 ? 0x40 : 0);
            break;
            
        case 1: // Mode
            if (value == 1) { // Auto
                sendBuffer[7] = sendBuffer[7] & 0xDF;
                sendBuffer[8] = sendBuffer[8] | 0x80;
            } else if (value == 2) { // Cool
                sendBuffer[7] = sendBuffer[7] & 0xDF;
                sendBuffer[8] = sendBuffer[8] & 0x7F;
            } else if (value == 3) { // Heat
                sendBuffer[7] = sendBuffer[7] | 0x20;
                sendBuffer[8] = sendBuffer[8] & 0x7F;
            } else {
                return; // Invalid mode, abort
            }
            break;
            
        case 2: // Target Temperature
            sendBuffer[2] = (sendBuffer[2] & 0x80) | (value & 0x7F);
            break;
            
        default:
            return; // Unknown tag, abort
    }

    sendBuffer[0] = 0xcc; 
    sendBuffer[9] = calculateChecksum(sendBuffer + 1, 8);

    Serial.print("[Core] Dispatched Frame to Driver: ");
    for(int i = 0; i < 10; i++) Serial.printf("%02X ", sendBuffer[i]);
    Serial.println();

    _hardware->sendRawFrame(sendBuffer, 4000);

    // --- NEW: Optimistic State Update ---
    // Update local cache so consecutive commands in a sequence build on each other properly.
    // This guarantees the 'Target Temp' frame gets constructed with the 'Power OFF' bit.
    std::memcpy(_lastCtrlFrame, sendBuffer, 10);
    _lastCtrlFrame[0] = 0xd2;
    parseControlFrame(_lastCtrlFrame);
    _pendingStateChange = true;
}

void PoolHeaterCore::setPower(bool on) {
    // Internal tag 0 represents Power
    sendControlFrame(0, on ? 1 : 0);
}