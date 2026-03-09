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

    bool _openDrainTx; // Flag for physical bus topology

    bool _invertTx;
    bool _invertRx;

    rmt_channel_t _txChannel;
    rmt_channel_t _rxChannel;

    // Resolved physical levels for clean code
    uint32_t _txPulseLevel;
    uint32_t _txSpaceLevel;
    uint32_t _rxSpaceLevel;

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
    ESP32HeaterDriver(uint8_t rxPin, uint8_t txPin, bool openDrainTx = false, 
                      bool invertTx = false, bool invertRx = false);

    /**
     * @brief Advanced configuration for ESP32 RMT hardware channels.
     * * The RMT peripheral has multiple channels, but physical hardware constraints 
     * vary drastically depending on the specific ESP32 chip family:
     * * - ESP32 Classic (WROOM/WROVER): 8 universal channels (0-7). Any combination works.
     * - ESP32-C3 (e.g., Super Mini): 4 channels. Channels 0-1 are strictly TX ONLY, 
     * and channels 2-3 are strictly RX ONLY. (Using 0 and 1 will silently fail).
     * - ESP32-S3: 8 channels. Channels 0-3 are strictly TX ONLY, and channels 4-7 
     * are strictly RX ONLY.
     * * By default, this driver uses Channel 0 (TX) and Channel 2 (RX). 
     * This specific default configuration guarantees out-of-the-box compatibility 
     * with roughly 90% of the most common DIY boards (Classic, C3, and S2).
     * * WARNING: If deploying on an ESP32-S3, you MUST override these defaults 
     * (e.g., driver.setRmtChannels(0, 4)) before calling begin(), as channel 2 
     * lacks the hardware capability to receive data on the S3 architecture.
     * * @param txChannel The RMT channel used for transmitting pulses.
     * @param rxChannel The RMT channel used for receiving pulses.
     */
    void setRmtChannels(uint8_t txChannel, uint8_t rxChannel);                  

    virtual ~ESP32HeaterDriver();

    // --- IHeaterHardware Interface Implementation ---
    void begin() override;
    void enableRx(bool enable) override;
    bool receiveRawFrame(uint8_t* buffer) override;
    bool sendRawFrame(const uint8_t* buffer, unsigned int timeoutMs) override;
};

#endif // ESP32