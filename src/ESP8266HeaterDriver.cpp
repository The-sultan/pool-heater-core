// --- PREPROCESSOR SHIELD ---
// This entire file will only be compiled if the target board is an ESP8266
#if defined(ESP8266) || defined(ARDUINO_ARCH_ESP8266)

#include "ESP8266HeaterDriver.h"
#include <cstring> // For std::memcpy

// -----------------------------------------------------------------------------
// Constructor & Initialization
// -----------------------------------------------------------------------------
ESP8266HeaterDriver::ESP8266HeaterDriver(uint8_t rxPin, uint8_t txPin, bool inverted) 
    : _rxPin(rxPin), _txPin(txPin), _inverted(inverted), _rxPos(0), _txPos(0), _lastFrameRecvTime(0) {
    resetDecoder();
}

void ESP8266HeaterDriver::begin() {
    pinMode(_txPin, OUTPUT);
    digitalWrite(_txPin, _inverted ? LOW : HIGH); // Set default idle state on the TX line
    enableRx(true);
}

void ESP8266HeaterDriver::enableRx(bool enable) {
    if (enable) {
        pinMode(_rxPin, INPUT);
        // Pass 'this' pointer so the static ISR knows which object instance to update
        attachInterruptArg(digitalPinToInterrupt(_rxPin), isrHandler, this, CHANGE);
    } else {
        detachInterrupt(digitalPinToInterrupt(_rxPin));
        pinMode(_txPin, OUTPUT); 
    }
    resetDecoder();
}

// -----------------------------------------------------------------------------
// Core Interface Methods
// -----------------------------------------------------------------------------
bool ESP8266HeaterDriver::receiveRawFrame(uint8_t* buffer) {
    // If transmit and receive ring buffer positions match, no new data is available
    if (_txPos == _rxPos) {
        return false;
    }

    // Copy the oldest complete frame from the ring buffer
    std::memcpy(buffer, (const void*)_frames[_txPos], HEATER_FRAME_SIZE);

    // Advance the buffer tail pointer
    _txPos = (_txPos + 1) % HEATER_RX_BUFFER_SIZE;

    // Physical layer inversion correction: 
    // The protocol logic assumes inverted data if the first byte is < 0x40.
    if (buffer[0] < 0x40) {
        for (int i = 0; i < HEATER_FRAME_SIZE; i++) {
            buffer[i] ^= 0xFF;
        }
    }

    return true; // Frame successfully retrieved
}

bool ESP8266HeaterDriver::sendRawFrame(const uint8_t* buffer, unsigned int timeoutMs) {
    // Create a local copy to handle physical layer bit-inversions without mutating the source
    uint8_t localFrame[HEATER_FRAME_SIZE];
    std::memcpy(localFrame, buffer, HEATER_FRAME_SIZE);

    // Only XOR if the hardware level shifter requires inversion
    if (_inverted) {
        for(int i = 0; i < HEATER_FRAME_SIZE; i++) {
            localFrame[i] ^= 0xFF;
        }
    }

    unsigned long startTime = millis();

    // Try to send the frame within the allowed timeout window
    while ((millis() - startTime) < timeoutMs) {
        unsigned long lastRcv = micros() - _lastFrameRecvTime; 
        
        // Wait for a quiet window on the bus (between 15ms and 1000ms since last receive)
        if (lastRcv < 1000000 && lastRcv > 15000) {     
            
            enableRx(false); // Stop listening while transmitting
            digitalWrite(_txPin, HIGH);

            // Repeat the frame sequence for reliability (protocol requirement)
            for (size_t i = 0; i < HEATER_REPEAT_SEND; i++) {                            
                
                // Transmit leading pulse & space
                digitalWrite(_txPin, _inverted ? LOW : HIGH);
                delayMicroseconds(HEATER_LEADING_PULSE);
                digitalWrite(_txPin, _inverted ? HIGH : LOW);
                delayMicroseconds(HEATER_LEADING_SPACE);

                // Bit-bang the payload payload
                for (size_t j = 0; j < HEATER_FRAME_SIZE; j++) {                    
                    for (size_t k = 0; k < 8; k++) {
                        digitalWrite(_txPin, _inverted ? LOW : HIGH);
                        delayMicroseconds(HEATER_PULSE_LENGTH);

                        // Extract the current bit (LSB first)
                        bool bitValue = (localFrame[j] >> k) & 1;

                        digitalWrite(_txPin, _inverted ? HIGH : LOW);
                        delayMicroseconds(bitValue ? HEATER_SPACE_ONE : HEATER_SPACE_ZERO);
                    }                                    
                }

                // Transmit ending pulse
                digitalWrite(_txPin, HIGH);         
                delayMicroseconds(HEATER_PULSE_LENGTH);
                
                // Wait before sending the next repeated frame
                digitalWrite(_txPin, LOW);
                delayMicroseconds(HEATER_REPEAT_DELAY_US);
            }

            enableRx(true); // Resume listening
            return true;
        }
        delay(1); // Yield to the hardware watchdog to prevent crashes
    }
    return false; // Timed out waiting for a clear bus
}

// -----------------------------------------------------------------------------
// ISR (Interrupt) & Decoding State Machine
// -----------------------------------------------------------------------------
void IRAM_ATTR ESP8266HeaterDriver::isrHandler(void* arg) {
    // Cast the void pointer back to our driver instance
    ESP8266HeaterDriver* instance = static_cast<ESP8266HeaterDriver*>(arg);
    
    unsigned long currentMicros = micros();
    unsigned long duration = currentMicros - instance->_lastIsrTime;
    
    // Read the physical RX pin state
    int newPinState = digitalRead(instance->_rxPin);
    
    // Logic swap based on hardware inversion
    bool bitValue = instance->_inverted ? !newPinState : (bool)newPinState;
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
    // Save the byte into the current frame slot
    _frames[_rxPos][_bytePos++] = d;

    // If the frame is fully assembled (10 bytes)
    if (_bytePos == HEATER_FRAME_SIZE) {
        _lastFrameRecvTime = micros();
        _rxPos = (_rxPos + 1) % HEATER_RX_BUFFER_SIZE; // Advance the ring buffer head
        _bytePos = 0; // Reset byte index for the next frame
    }
}

void IRAM_ATTR ESP8266HeaterDriver::resetDecoder() {
    _bytePos = 0;
    _bitBuffer = 1;
    _state = WAIT_LEADING_PULSE;
}

void IRAM_ATTR ESP8266HeaterDriver::handleError(bool bit, unsigned long duration) {
    // Reset the decoding state machine on error. 
    // Logging is omitted here because calling print() inside an ISR causes panics.
    resetDecoder();
}

void IRAM_ATTR ESP8266HeaterDriver::appendBit(bool bit, unsigned long duration) {
    // If duration is abnormally long, reset the decoder
    if (duration > (HEATER_LEADING_SPACE + HEATER_MAX_NOISE)) {
        resetDecoder();
    }

    // Decoding state machine
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
            
            // Shift the bit into the buffer
            _bitBuffer = (_bitBuffer << 1) | currentBit;
            
            // If we have collected 8 bits (the 9th bit is our sentinel marker)
            if (_bitBuffer & 0b100000000) {
                appendByte(reverseBit(_bitBuffer & 0xFF));
                _bitBuffer = 1; // Reset sentinel
            }
            break;
    }
}

#endif // ESP8266