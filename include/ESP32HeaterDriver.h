#pragma once

// --- PREPROCESSOR SHIELD ---
// This file will only be compiled if the target board is an ESP32
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>
#include "IHeaterHardware.h"
#include <driver/rmt.h> // Native ESP-IDF RMT driver

// Hardware timing constants (same as ESP8266)
#define HEATER_LEADING_PULSE      9300
#define HEATER_LEADING_SPACE      4700
#define HEATER_PULSE_LENGTH       1152
#define HEATER_SPACE_ONE          1141
#define HEATER_SPACE_ZERO         3180
#define HEATER_REPEAT_SEND        3
#define HEATER_REPEAT_DELAY_US    243000
#define HEATER_MAX_NOISE          500
#define HEATER_FRAME_SIZE         10

class ESP32HeaterDriver : public IHeaterHardware {
private:
    gpio_num_t _rxPin;
    gpio_num_t _txPin;
    bool _inverted; // Flag for DIY level shifter inversion
    
    rmt_channel_t _txChannel;
    rmt_channel_t _rxChannel;

    // Ring buffer for receiving decoded frames
    uint8_t _frames[5][HEATER_FRAME_SIZE];
    volatile int _rxPos;
    volatile int _txPos;

    // Internal helper to convert raw timings from the RMT peripheral into bytes
    void processRmtRxItems(rmt_item32_t* item, size_t item_num);
    static uint8_t reverseBit(uint8_t d);

    // FreeRTOS Task handle for the RX monitoring loop
    TaskHandle_t _rxTaskHandle;
    static void rxTask(void* arg); // Runs in the background waiting for RMT data

public:
    // Constructor allows pin and RMT channel assignment
    // Updated constructor with 'inverted' parameter
    ESP32HeaterDriver(uint8_t rxPin, uint8_t txPin, bool inverted = false,
                      rmt_channel_t txChannel = RMT_CHANNEL_0, 
                      rmt_channel_t rxChannel = RMT_CHANNEL_1);
                      
    virtual ~ESP32HeaterDriver();

    // --- IHeaterHardware Interface Implementation ---
    void begin() override;
    void enableRx(bool enable) override;
    bool receiveRawFrame(uint8_t* buffer) override;
    bool sendRawFrame(const uint8_t* buffer, unsigned int timeoutMs) override;
};

#endif // ESP32