#pragma once

// --- PREPROCESSOR SHIELD ---
// This entire file will only be compiled if the target board is an ESP8266
#if defined(ESP8266) || defined(ARDUINO_ARCH_ESP8266)

#include <Arduino.h>
#include "IHeaterHardware.h"

// Hardware timing constants (Reverse-engineered from the physical protocol)
#define HEATER_REPEAT_SEND        3
#define HEATER_REPEAT_DELAY_US    243000
#define HEATER_LEADING_PULSE      9300
#define HEATER_LEADING_SPACE      4700
#define HEATER_PULSE_LENGTH       1152
#define HEATER_SPACE_ONE          1141
#define HEATER_SPACE_ZERO         3180
#define HEATER_MAX_NOISE          500

#define HEATER_FRAME_SIZE         10
#define HEATER_RX_BUFFER_SIZE     5

// Helper macro for timing validation
#define CHECK_TIME(value, expected) (abs(long((value)-(expected))) < HEATER_MAX_NOISE)

// State machine steps for the RX decoding
enum CaptureState {
    WAIT_LEADING_PULSE, 
    WAIT_LEADING_SPACE, 
    WAIT_PULSE, 
    WAIT_BIT
};

class ESP8266HeaterDriver : public IHeaterHardware {
private:
    uint8_t _rxPin;
    uint8_t _txPin;

    // RX Ring Buffer to store decoded frames
    uint8_t _frames[HEATER_RX_BUFFER_SIZE][HEATER_FRAME_SIZE];
    volatile int _rxPos;
    volatile int _txPos;

    // Decoding state machine variables
    volatile uint16_t _bitBuffer;
    volatile uint8_t  _bytePos;
    volatile CaptureState _state;
    volatile unsigned long _lastFrameRecvTime;
    volatile unsigned long _lastIsrTime;

    // Internal helpers for the Interrupt Service Routine (ISR)
    void appendByte(uint8_t d);
    void appendBit(bool bit, unsigned long duration);
    void resetDecoder();
    void handleError(bool bit, unsigned long duration);
    
    // Utility to flip bits (endianness/protocol requirement)
    static uint8_t reverseBit(uint8_t d);

    // Static ISR handler required by the ESP8266 hardware core
    static void IRAM_ATTR isrHandler(void* arg);

public:
    // Constructor allows dynamic pin assignment (e.g., D7, D2)
    ESP8266HeaterDriver(uint8_t rxPin, uint8_t txPin);
    
    // Virtual destructor is mandatory when implementing interfaces
    virtual ~ESP8266HeaterDriver() = default;

    // --- IHeaterHardware Interface Implementation ---
    void begin() override;
    void enableRx(bool enable) override;
    bool receiveRawFrame(uint8_t* buffer) override;
    bool sendRawFrame(const uint8_t* buffer, unsigned int timeoutMs) override;
};

#endif // ESP8266