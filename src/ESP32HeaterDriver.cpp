// --- PREPROCESSOR SHIELD ---
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)

#include "ESP32HeaterDriver.h"
#include <cstring>

// -----------------------------------------------------------------------------
// Constructor & Initialization
// -----------------------------------------------------------------------------
ESP32HeaterDriver::ESP32HeaterDriver(uint8_t txPin, uint8_t rxPin, bool openDrainTx, bool invertTx, bool invertRx)
    : _txPin((gpio_num_t)txPin), _rxPin((gpio_num_t)rxPin), _openDrainTx(openDrainTx),
    _invertTx(invertTx), _invertRx(invertRx), _rxPos(0), _txPos(0), _rxTaskHandle(NULL) {
    
    // Resolve logical levels ONCE at initialization for maximum readability
    _txPulseLevel = invertTx ? 0 : 1;
    _txSpaceLevel = invertTx ? 1 : 0;
    _rxSpaceLevel = invertRx ? 1 : 0;

    if (_openDrainTx) {
        pinMode(_txPin, OUTPUT_OPEN_DRAIN);
    } else {
        pinMode(_txPin, OUTPUT);
    }

    // Default channels assigned silently
    _txChannel = RMT_CHANNEL_0;
    _rxChannel = RMT_CHANNEL_2;

    for (int i = 0; i < 5; i++) {
        memset(_frames[i], 0, HEATER_FRAME_SIZE);
    }
}

ESP32HeaterDriver::~ESP32HeaterDriver() {
    if (_rxTaskHandle) {
        vTaskDelete(_rxTaskHandle);
    }
    rmt_driver_uninstall(_txChannel);
    rmt_driver_uninstall(_rxChannel);
}

void ESP32HeaterDriver::begin() {
    // Determine the high/low levels based on TX hardware inversion
    // If inverted (DIY transistors), a High from MCU = Low at the heater
    uint8_t txSpaceLevel = _invertTx ? 1 : 0;

    // --- Configure TX Channel ---
    rmt_config_t tx_config = RMT_DEFAULT_CONFIG_TX(_txPin, _txChannel);
    tx_config.clk_div = 80; // 1 tick = 1 microsecond
    rmt_config(&tx_config);
    rmt_driver_install(_txChannel, 0, 0);

    // --- Configure RX Channel ---
    rmt_config_t rx_config = RMT_DEFAULT_CONFIG_RX(_rxPin, _rxChannel);
    rx_config.clk_div = 80;
    rx_config.rx_config.idle_threshold = 15000; 
    rmt_config(&rx_config);
    rmt_driver_install(_rxChannel, 1000, 0);

    // Initial idle state for TX line
    digitalWrite(_txPin, txSpaceLevel);

    xTaskCreate(rxTask, "RMT_RX_Task", 4096, this, 10, &_rxTaskHandle);
}

void ESP32HeaterDriver::enableRx(bool enable) {
    if (enable) rmt_rx_start(_rxChannel, true);
    else rmt_rx_stop(_rxChannel);
}

// -----------------------------------------------------------------------------
// Core Interface Methods
// -----------------------------------------------------------------------------
bool ESP32HeaterDriver::receiveRawFrame(uint8_t* buffer) {
    if (_txPos == _rxPos) return false;

    std::memcpy(buffer, (const void*)_frames[_txPos], HEATER_FRAME_SIZE);
    _txPos = (_txPos + 1) % 5;
    
    // Note: Software byte inversion (^= 0xFF) was removed.
    // Inversion is now handled purely at the physical RMT level.
    return true;
}

// -----------------------------------------------------------------------------
// Transmit Logic (Super Clean)
// -----------------------------------------------------------------------------
bool ESP32HeaterDriver::sendRawFrame(const uint8_t* buffer, unsigned int timeoutMs) {
    size_t num_items = 1 + (HEATER_FRAME_SIZE * 8) + 1;
    rmt_item32_t* items = (rmt_item32_t*)malloc(sizeof(rmt_item32_t) * num_items);
    if (!items) return false;

    for (size_t repeat = 0; repeat < HEATER_REPEAT_SEND; repeat++) {
        size_t item_idx = 0;

        // 1. Leading Pulse & Space using our class variables directly!
        items[item_idx++] = {{{ (uint32_t)HEATER_LEADING_PULSE, _txPulseLevel, (uint32_t)HEATER_LEADING_SPACE, _txSpaceLevel }}};

        // 2. Data payload
        for (size_t j = 0; j < HEATER_FRAME_SIZE; j++) {
            for (size_t k = 0; k < 8; k++) {
                bool bitValue = (buffer[j] >> k) & 1;
                uint16_t space_len = bitValue ? HEATER_SPACE_ONE : HEATER_SPACE_ZERO;
                
                // Beautiful and readable injection
                items[item_idx++] = {{{ (uint32_t)HEATER_PULSE_LENGTH, _txPulseLevel, (uint32_t)space_len, _txSpaceLevel }}};
            }
        }

        // 3. Ending Pulse
        items[item_idx++] = {{{ (uint32_t)HEATER_PULSE_LENGTH, _txPulseLevel, 0, _txSpaceLevel }}};

        rmt_write_items(_txChannel, items, item_idx, true);
        rmt_wait_tx_done(_txChannel, pdMS_TO_TICKS(100));
        delayMicroseconds(HEATER_REPEAT_DELAY_US);
    }

    free(items);
    return true;
}

// -----------------------------------------------------------------------------
// RX Task & Decoding
// -----------------------------------------------------------------------------
void ESP32HeaterDriver::rxTask(void* arg) {
    ESP32HeaterDriver* instance = static_cast<ESP32HeaterDriver*>(arg);
    RingbufHandle_t rb = NULL;
    
    // Get the internal FreeRTOS ringbuffer from the RMT driver
    rmt_get_ringbuf_handle(instance->_rxChannel, &rb);

    while (true) {
        if (rb) {
            size_t rx_size = 0;
            // Block and wait until the RMT hardware detects an idle line (frame received)
            rmt_item32_t* item = (rmt_item32_t*)xRingbufferReceive(rb, &rx_size, portMAX_DELAY);
            
            if (item) {
                size_t item_num = rx_size / sizeof(rmt_item32_t);
                instance->processRmtRxItems(item, item_num);
                // Return the memory block to the ringbuffer
                vRingbufferReturnItem(rb, (void*)item);
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(100)); // Fallback delay
        }
    }
}

uint8_t ESP32HeaterDriver::reverseBit(uint8_t d) {
    uint8_t result = 0;
    for (int i = 0; i < 8; i++) {
        result = (result << 1) | (d & 1);
        d >>= 1;
    } 
    return result;
}

void ESP32HeaterDriver::processRmtRxItems(rmt_item32_t* item, size_t item_num) {
    if (item_num < (HEATER_FRAME_SIZE * 8)) return;

    uint8_t bytePos = 0;
    uint16_t bitBuffer = 1;
 
    for (size_t i = 0; i < item_num; i++) {
        // Find the correct duration within the RMT item based on the logical space level
        uint16_t duration;
        if (item[i].level0 == _rxSpaceLevel) {
            duration = item[i].duration0;
        } else if (item[i].level1 == _rxSpaceLevel) {
            duration = item[i].duration1;
        } else {
            continue; // Skip if no logical space is found (noise)
        }
        
        // Discard the massive leading space
        if (duration > (HEATER_LEADING_SPACE - HEATER_MAX_NOISE) && 
            duration < (HEATER_LEADING_SPACE + HEATER_MAX_NOISE)) {
            continue; 
        }

        // Decode the bit based on physical time duration, not voltage level
        uint8_t currentBit = 0;
        if (abs((long)duration - HEATER_SPACE_ONE) < HEATER_MAX_NOISE) {
            currentBit = 1;
        } else if (abs((long)duration - HEATER_SPACE_ZERO) < HEATER_MAX_NOISE) {
            currentBit = 0;
        } else {
            continue; // Invalid duration, line noise
        }

        bitBuffer = (bitBuffer << 1) | currentBit;
        
        if (bitBuffer & 0b100000000) {
            _frames[_rxPos][bytePos++] = reverseBit(bitBuffer & 0xFF);
            bitBuffer = 1;

            if (bytePos == HEATER_FRAME_SIZE) {
                _rxPos = (_rxPos + 1) % 5;
                bytePos = 0;
                break; // Complete frame parsed
            }
        }
    }
}

void ESP32HeaterDriver::setRmtChannels(uint8_t txChannel, uint8_t rxChannel) {
    // Only allow changes before begin() is called (when RMT drivers are installed)
    _txChannel = (rmt_channel_t)txChannel;
    _rxChannel = (rmt_channel_t)rxChannel;
}

#endif // ESP32