// --- PREPROCESSOR SHIELD ---
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)

#include "ESP32HeaterDriver.h"
#include <cstring>

// -----------------------------------------------------------------------------
// Constructor & Initialization
// -----------------------------------------------------------------------------
ESP32HeaterDriver::ESP32HeaterDriver(uint8_t rxPin, uint8_t txPin, bool inverted, rmt_channel_t txChannel, rmt_channel_t rxChannel)
    : _rxPin((gpio_num_t)rxPin), _txPin((gpio_num_t)txPin), _inverted(inverted), 
      _txChannel(txChannel), _rxChannel(rxChannel), _rxPos(0), _txPos(0), _rxTaskHandle(NULL) {
}

ESP32HeaterDriver::~ESP32HeaterDriver() {
    if (_rxTaskHandle) {
        vTaskDelete(_rxTaskHandle);
    }
    rmt_driver_uninstall(_txChannel);
    rmt_driver_uninstall(_rxChannel);
}

void ESP32HeaterDriver::begin() {
    // Determine the high/low levels based on hardware inversion
    // If inverted (DIY transistors), a High from MCU = Low at the heater
    uint8_t pulseLevel = _inverted ? 0 : 1;
    uint8_t spaceLevel = _inverted ? 1 : 0;

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
    digitalWrite(_txPin, spaceLevel);

    xTaskCreate(rxTask, "RMT_RX_Task", 4096, this, 10, &_rxTaskHandle);
    
    //Log.printf("[Driver] ESP32 RMT initialized. RX:%d TX:%d (Inverted: %s)\n", 
    //           _rxPin, _txPin, _inverted ? "YES" : "NO");
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
    
    // Physical layer inversion correction
    if (buffer[0] < 0x40) {
        for (int i = 0; i < HEATER_FRAME_SIZE; i++) buffer[i] ^= 0xFF;
    }
    return true;
}

bool ESP32HeaterDriver::sendRawFrame(const uint8_t* buffer, unsigned int timeoutMs) {
    uint8_t localFrame[HEATER_FRAME_SIZE];
    std::memcpy(localFrame, buffer, HEATER_FRAME_SIZE);
    
    // Physical layer inversion correction
    if (_inverted) {
        for (int i = 0; i < HEATER_FRAME_SIZE; i++) localFrame[i] ^= 0xFF;
    }

    // Determine levels for RMT items
    unsigned int pL = _inverted ? 0 : 1; // Pulse level
    unsigned int sL = _inverted ? 1 : 0; // Space level

    // Allocate memory for the RMT sequence. 
    // 1 start bit + (10 bytes * 8 bits) + 1 stop bit = 82 items per frame
    size_t num_items = 1 + (HEATER_FRAME_SIZE * 8) + 1;
    rmt_item32_t* items = (rmt_item32_t*)malloc(sizeof(rmt_item32_t) * num_items);
    if (!items) return false;

    enableRx(false); // Stop listening during TX

    for (size_t repeat = 0; repeat < HEATER_REPEAT_SEND; repeat++) {
        size_t item_idx = 0;

        // 1. Leading Pulse & Space using our dynamic levels
        items[item_idx++] = {{{ (uint32_t)HEATER_LEADING_PULSE, pL, (uint32_t)HEATER_LEADING_SPACE, sL }}};

        // 2. Data payload
        for (size_t j = 0; j < HEATER_FRAME_SIZE; j++) {
            for (size_t k = 0; k < 8; k++) {
                bool bitValue = (localFrame[j] >> k) & 1;
                uint16_t space_len = bitValue ? HEATER_SPACE_ONE : HEATER_SPACE_ZERO;
                items[item_idx++] = {{{ (uint32_t)HEATER_PULSE_LENGTH, pL, (uint32_t)space_len, sL }}};
            }
        }

        // 3. Ending Pulse
        items[item_idx++] = {{{ (uint32_t)HEATER_PULSE_LENGTH, pL, 0, sL }}};

        // Send the sequence using hardware acceleration!
        rmt_write_items(_txChannel, items, item_idx, true);
        
        // Wait for transmission to finish, then apply inter-frame delay
        rmt_wait_tx_done(_txChannel, pdMS_TO_TICKS(100));
        delayMicroseconds(HEATER_REPEAT_DELAY_US);
    }

    free(items);
    enableRx(true);
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
    // Basic sanity check: we need enough pulses to form a valid frame
    if (item_num < (HEATER_FRAME_SIZE * 8)) return;

    uint8_t bytePos = 0;
    uint16_t bitBuffer = 1;
    
     // Iterate over the captured hardware pulses
   for (size_t i = 0; i < item_num; i++) {
        // In RMT RX, duration0 is the first captured level.
        // If inverted, the "Space" (Low) is seen as High by the MCU
        uint16_t duration = item[i].duration0; 
        
        // Skip leading space parsing for brevity, assume data starts after sync
        if (duration > (HEATER_LEADING_SPACE - HEATER_MAX_NOISE) && 
            duration < (HEATER_LEADING_SPACE + HEATER_MAX_NOISE)) {
            continue; 
        }

        uint8_t currentBit = 0;
        if (abs((long)duration - HEATER_SPACE_ONE) < HEATER_MAX_NOISE) {
            currentBit = 1;
        }

        bitBuffer = (bitBuffer << 1) | currentBit;

        if (bitBuffer & 0b100000000) {
            _frames[_rxPos][bytePos++] = reverseBit(bitBuffer & 0xFF);
            bitBuffer = 1;

            if (bytePos == HEATER_FRAME_SIZE) {
                _rxPos = (_rxPos + 1) % 5;
                bytePos = 0;
                break; // Frame complete
            }
        }
    }
}

#endif // ESP32