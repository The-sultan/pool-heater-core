// --- PREPROCESSOR SHIELD ---
// This entire file will only be compiled if the target board is an ESP8266
#if defined(ESP8266) || defined(ARDUINO_ARCH_ESP8266)

#include "ESP8266HeaterDriver.h"
#include <cstring> // For std::memcpy

// -----------------------------------------------------------------------------
// Constructor & Initialization
// -----------------------------------------------------------------------------
ESP8266HeaterDriver::ESP8266HeaterDriver(uint8_t txPin, uint8_t rxPin, bool openDrainTx, bool invertTx, bool invertRx) 
    : _txPin(txPin), _rxPin(rxPin), _openDrainTx(openDrainTx), _rxPos(0), _txPos(0), _lastFrameRecvTime(0) {
    
   // CORRECCIÓN: Si está invertido por hardware (BJT), 
    // el ESP manda HIGH para el pulso y LOW para el espacio.
    _txPulseLevel = invertTx ? HIGH : LOW;
    _txSpaceLevel = invertTx ? LOW : HIGH;
    
    // CORRECCIÓN: Si el hardware RX invierte (BJT), 
    // el estado de reposo (bus HIGH) se lee como LOW en el ESP.
    _rxSpaceLevel = invertRx ? LOW : HIGH;

    resetDecoder();
}

void ESP8266HeaterDriver::begin() {
    if (_openDrainTx) {
        pinMode(_txPin, OUTPUT_OPEN_DRAIN);
    } else {
        pinMode(_txPin, OUTPUT);
    }
    
    digitalWrite(_txPin, _txSpaceLevel); // Set default idle state on the TX line
    enableRx(true);
}

void ESP8266HeaterDriver::enableRx(bool enable) {
    if (enable) {
        pinMode(_rxPin, INPUT);
        // Pass 'this' pointer so the static ISR knows which object instance to update
        attachInterruptArg(digitalPinToInterrupt(_rxPin), isrHandler, this, CHANGE);
    } else {
        detachInterrupt(digitalPinToInterrupt(_rxPin));
        // Restore TX pin mode in case we are using the same pin for 1-wire
        if (_openDrainTx) {
            pinMode(_txPin, OUTPUT_OPEN_DRAIN);
        } else {
            pinMode(_txPin, OUTPUT); 
        }
    }
    resetDecoder();
}

bool ESP8266HeaterDriver::receiveRawFrame(uint8_t* buffer) {
    if (_txPos == _rxPos) return false;

    // Copy the oldest complete frame from the ring buffer
    std::memcpy(buffer, (const void*)_frames[_txPos], HEATER_FRAME_SIZE);

    // Advance the buffer tail pointer
    _txPos = (_txPos + 1) % HEATER_RX_BUFFER_SIZE;

    return true; 
}

bool ESP8266HeaterDriver::sendRawFrame(const uint8_t* buffer, unsigned int timeoutMs) {
    unsigned long startTime = millis();
    Serial.printf("[Driver] Attempting to TX frame. Timeout: %dms\n", timeoutMs);

    // Try to send the frame within the allowed timeout window
    while ((millis() - startTime) < timeoutMs) {
        unsigned long lastRcv = micros() - _lastFrameRecvTime; 
        
        // Relaxed upper bound constraint. As long as it is > 15ms silent, send it.
        if (lastRcv > 15000) {     
            enableRx(false); // Stop listening while transmitting
            digitalWrite(_txPin, _txSpaceLevel); // Ensure line is idle before starting
            
            Serial.printf("[Driver] Silence window found (%lu us). Sending pulses...\n", lastRcv);

            // Repeat the frame sequence for reliability (protocol requirement)
            for (size_t i = 0; i < HEATER_REPEAT_SEND; i++) {                            
                
                // --- SHIELD ON: Block Wi-Fi interrupts for precise timing ---
                noInterrupts(); 

                // Transmit leading pulse & space
                digitalWrite(_txPin, _txPulseLevel);
                delayMicroseconds(HEATER_LEADING_PULSE);
                digitalWrite(_txPin, _txSpaceLevel);
                delayMicroseconds(HEATER_LEADING_SPACE);

                // --- SHIELD OFF: Let ESP process Wi-Fi momentarily ---
                interrupts(); 

                // Bit-bang the payload
                for (size_t j = 0; j < HEATER_FRAME_SIZE; j++) {                    
                    for (size_t k = 0; k < 8; k++) {
                        
                        // CRITICAL PROTOCOL FIX: The pump expects inverted durations natively.
                        // We must logically invert the bit to map it to the correct space length.
                        bool bitValue = !((buffer[j] >> k) & 1); 

                        // --- SHIELD ON ---
                        noInterrupts();
                        
                        digitalWrite(_txPin, _txPulseLevel);
                        delayMicroseconds(HEATER_PULSE_LENGTH);

                        digitalWrite(_txPin, _txSpaceLevel);
                        delayMicroseconds(bitValue ? HEATER_SPACE_ONE : HEATER_SPACE_ZERO);
                        
                        // --- SHIELD OFF ---
                        interrupts(); 
                    }                                    
                }

                // --- SHIELD ON ---
                noInterrupts();
                
                // Transmit ending pulse
                digitalWrite(_txPin, _txPulseLevel);         
                delayMicroseconds(HEATER_PULSE_LENGTH);
                digitalWrite(_txPin, _txSpaceLevel);
                
                // --- SHIELD OFF ---
                interrupts();

                // Wait before sending the next repeated frame
                delay(HEATER_REPEAT_DELAY_US / 1000); 
            }

            enableRx(true); // Resume listening
            Serial.println("[Driver] TX completed successfully.");
            return true;
        }
        delay(1); // Yield to the hardware watchdog
    }
    
    Serial.println("[Driver] ERROR: TX timeout. No silence window found on the bus.");
    return false; // Timed out waiting for a clear bus
}

// -----------------------------------------------------------------------------
// ISR (Interrupt) & Decoding State Machine
// -----------------------------------------------------------------------------
void IRAM_ATTR ESP8266HeaterDriver::isrHandler(void* arg) {
    ESP8266HeaterDriver* instance = static_cast<ESP8266HeaterDriver*>(arg);
    
    unsigned long currentMicros = micros();
    unsigned long duration = currentMicros - instance->_lastIsrTime;
    
    // Read the physical RX pin state
    int newPinState = digitalRead(instance->_rxPin);
    
    // If the pin state matches the calculated idle (space) level, the logic bit is TRUE (1)
    bool bitValue = (newPinState == instance->_rxSpaceLevel);
    instance->appendBit(bitValue, duration);
    
    instance->_lastIsrTime = currentMicros;
}

uint8_t IRAM_ATTR ESP8266HeaterDriver::reverseBit(uint8_t d) {
    uint8_t result = 0;
    for (int i = 0; i < 8; i++) {
        result = (result << 1) | (d & 1);
        d >>= 1;
    } 
    return result;
}

void IRAM_ATTR ESP8266HeaterDriver::appendByte(uint8_t d) {
    _frames[_rxPos][_bytePos++] = d;

    if (_bytePos == HEATER_FRAME_SIZE) {
        _lastFrameRecvTime = micros();
        _rxPos = (_rxPos + 1) % HEATER_RX_BUFFER_SIZE; 
        _bytePos = 0; 
    }
}

void IRAM_ATTR ESP8266HeaterDriver::resetDecoder() {
    _bytePos = 0;
    _bitBuffer = 1;
    _state = WAIT_LEADING_PULSE;
}

void IRAM_ATTR ESP8266HeaterDriver::handleError(bool bit, unsigned long duration) {
    resetDecoder();
}

void IRAM_ATTR ESP8266HeaterDriver::appendBit(bool bit, unsigned long duration) {
    if (duration > (HEATER_LEADING_SPACE + HEATER_MAX_NOISE)) {
        resetDecoder();
    }

    switch (_state) {
        case WAIT_LEADING_PULSE:
            if (CHECK_TIME(duration, HEATER_LEADING_PULSE) && !bit) {
                _state = WAIT_LEADING_SPACE;
            }
            break;

        case WAIT_LEADING_SPACE:
            if (CHECK_TIME(duration, HEATER_LEADING_SPACE) && bit) {
                _state = WAIT_PULSE;
            } else {
                handleError(bit, duration);
            }
            break;

        case WAIT_PULSE:
            if (CHECK_TIME(duration, HEATER_PULSE_LENGTH) && !bit) {
                _state = WAIT_BIT;
            } else {
                handleError(bit, duration);
            }
            break;

        case WAIT_BIT:
            if (!bit) {
                handleError(bit, duration);
                return;
            }

            _state = WAIT_PULSE;
            uint8_t currentBit = 0;
            
            if (CHECK_TIME(duration, HEATER_SPACE_ONE)) {
                currentBit = 1;
            }
            
            _bitBuffer = (_bitBuffer << 1) | currentBit;
            
            if (_bitBuffer & 0b100000000) {
                appendByte(reverseBit(_bitBuffer & 0xFF));
                _bitBuffer = 1; 
            }
            break;
    }
}

#endif // ESP8266