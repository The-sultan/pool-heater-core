#pragma once
#include <Arduino.h>
#include <TelnetStream.h>

class MultiLogger : public Print {
public:
    MultiLogger();
    
    // Starts the Telnet server
    void begin();

    // The core methods that handle the actual data output
    virtual size_t write(uint8_t character) override;
    virtual size_t write(const uint8_t *buffer, size_t size) override;
};

// Extern declaration to make a global 'Log' instance available everywhere
extern MultiLogger Log;