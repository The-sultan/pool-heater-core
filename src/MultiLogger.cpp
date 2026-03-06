#include "MultiLogger.h"

// Instantiate the global object
MultiLogger Log;

MultiLogger::MultiLogger() {}

void MultiLogger::begin() {
    TelnetStream.begin();
}

size_t MultiLogger::write(uint8_t character) {
    // Write to both streams
    Serial.write(character);
    TelnetStream.write(character);
    return 1;
}

size_t MultiLogger::write(const uint8_t *buffer, size_t size) {
    // Write the whole buffer to both streams
    Serial.write(buffer, size);
    TelnetStream.write(buffer, size);
    return size;
}