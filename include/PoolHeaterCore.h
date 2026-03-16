#pragma once

#include <cstdint>
#include "PoolHeaterTypes.h"
#include "IHeaterHardware.h"

class PoolHeaterCore {
private:
    // Pointer to the hardware abstraction layer (Dependency Injection)
    IHeaterHardware* _hardware;
    
    // The current, human-readable state of the heater
    HeaterState _currentState;

    // Caches for the last received raw frames to detect state changes
    // (Replaces the old tempframe and ctrlframe global variables)
    uint8_t _lastTempFrame[10];
    uint8_t _lastCtrlFrame[10];

    // --- Low-Level Internal Functions ---
    
    // Calculates the checksum for a given frame buffer
    uint8_t calculateChecksum(const uint8_t* buffer, int len);
    
    // Parses incoming temperature frames (starting with 0xdd)
    void parseTemperatureFrame(const uint8_t* frame);
    
    // Parses incoming control frames (starting with 0xd2)
    void parseControlFrame(const uint8_t* frame);
    
    // Constructs and dispatches a control frame via the hardware layer
    // (Equivalent to the old sendFrame(tag, value) function)
    void sendControlFrame(uint8_t tag, uint8_t value);

    void setPower(bool on);

    // --- Async Sequence State Machine ---
    enum class AsyncSequence {
        NONE,
        TEMP_CHANGE_WAIT_OFF,
        TEMP_CHANGE_WAIT_SET,
        MODE_CHANGE_WAIT_SET
    };

    AsyncSequence _currentSequence = AsyncSequence::NONE;
    unsigned long _sequenceTimer = 0;
    int _pendingValue = 0;
    bool _restorePower = false;

    bool _pendingStateChange = false;

public:
    // Constructor: Requires a valid hardware implementation
    PoolHeaterCore(IHeaterHardware* hardware);

    // Initializes the underlying hardware
    void begin();

    // Main processing engine. Should be called continuously from the main loop.
    // Returns 'true' ONLY if the heater's state has changed (useful for MQTT publishing).
    bool loop();

    // --- High-Level User Interface ---
    
    // Retrieves the current state snapshot of the heater
    HeaterState getState() const;

    // Direct commands to control the heater
    
    void setMode(HeaterMode mode);
    void setTargetTemperature(int temp);
};