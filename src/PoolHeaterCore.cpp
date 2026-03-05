#include "PoolHeaterCore.h"
#include <cstring> // Required for std::memcpy, std::memset, and std::memcmp

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

    uint8_t frame[10];
    bool stateChanged = false;

    // Process all available frames in the hardware buffer
    while (_hardware->receiveRawFrame(frame)) {
        
        // Temperature frame identifier (0xdd)
        if (frame[0] == 0xdd) {
            // Check if the frame content actually changed
            if (std::memcmp(_lastTempFrame, frame, 10) != 0) {
                std::memcpy(_lastTempFrame, frame, 10);
                parseTemperatureFrame(frame);
                stateChanged = true;
            }
        } 
        // Control frame identifier (0xd2)
        else if (frame[0] == 0xd2) {
            // Check if the frame content actually changed
            if (std::memcmp(_lastCtrlFrame, frame, 10) != 0) {
                std::memcpy(_lastCtrlFrame, frame, 10);
                parseControlFrame(frame);
                stateChanged = true;
            }
        }
    }

    // Returns true only if a new, different frame was parsed
    return stateChanged;
}

// -----------------------------------------------------------------------------
// High-Level User Interface
// -----------------------------------------------------------------------------
HeaterState PoolHeaterCore::getState() const {
    return _currentState;
}

void PoolHeaterCore::setPower(bool on) {
    // Internal tag 0 represents Power
    sendControlFrame(0, on ? 1 : 0);
}

void PoolHeaterCore::setMode(HeaterMode mode) {
    uint8_t modeValue = 0;
    
    // Map our clean enum to the heater's expected integers
    switch (mode) {
        case HeaterMode::AUTO: modeValue = 1; break;
        case HeaterMode::COOL: modeValue = 2; break;
        case HeaterMode::HEAT: modeValue = 3; break;
        default: return; // Do nothing if mode is UNKNOWN
    }
    
    // Internal tag 1 represents Mode
    sendControlFrame(1, modeValue);
}

void PoolHeaterCore::setTargetTemperature(int temp) {
    // Safety clamp to avoid sending out-of-bounds temperatures
    if (temp >= 15 && temp <= 38) {
        // Internal tag 2 represents Target Temperature
        sendControlFrame(2, static_cast<uint8_t>(temp));
    }
}

void PoolHeaterCore::safeChangeTemperature(int temp) {
    // Store the current power state
    bool wasRunning = _currentState.powerOn;
    
    // If the heater is on, turn it off before changing the temperature
    if (wasRunning) {
        setPower(false);
    }
    
    setTargetTemperature(temp);
    
    // If the heater was originally on, turn it back on
    if (wasRunning) {
        setPower(true);
    }
}

// -----------------------------------------------------------------------------
// Low-Level Internal Functions
// -----------------------------------------------------------------------------
int PoolHeaterCore::calculateChecksum(const uint8_t* buffer, int len) {
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
    
    // Extract operating mode (Bytes 7 and 8)
    if ((frame[8] & 0x80) == 0x80) {
        _currentState.mode = HeaterMode::AUTO;
    } else if ((frame[7] & 0x20) == 0) {
        _currentState.mode = HeaterMode::COOL;
    } else {
        _currentState.mode = HeaterMode::HEAT;
    }
    
    // Extract target temperature (Byte 2, bottom 7 bits)
    _currentState.targetTemp = frame[2] & 0x7F;
}

void PoolHeaterCore::sendControlFrame(uint8_t tag, uint8_t value) {
    if (!_hardware) return;

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

    // Set the header byte for command transmission
    sendBuffer[0] = 0xcc; 
    
    // Calculate and append the checksum to the final byte
    sendBuffer[9] = calculateChecksum(sendBuffer + 1, 8);

    // Dispatch the frame to the hardware layer with a 4000ms timeout
    _hardware->sendRawFrame(sendBuffer, 4000);
}