#pragma once

// Operating modes supported by the Heater
enum class HeaterMode {
    COOL,
    HEAT,
    AUTO,
    UNKNOWN
};

// Human readable Heater state
struct HeaterState {
    bool powerOn;
    HeaterMode mode;
    int currentTempIn;
    int currentTempOut;
    int targetTemp;
    int ambientTemp;
    int errorCode;
    
    // Default constructor
    HeaterState() : 
        powerOn(false), 
        mode(HeaterMode::UNKNOWN), 
        currentTempIn(0), 
        currentTempOut(0), 
        targetTemp(0), 
        ambientTemp(0), 
        errorCode(0) {}
};