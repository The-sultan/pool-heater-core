// include/IHeaterHardware.h
#pragma once
#include <cstdint> // Librería estándar de C++, cero dependencias de Arduino

class IHeaterHardware {
public:
    virtual ~IHeaterHardware() = default;

    virtual void begin() = 0;

    virtual void enableRx(bool enable) = 0;

    // Cambiamos 'byte' por 'uint8_t'
    virtual bool receiveRawFrame(uint8_t* buffer) = 0;

    virtual bool sendRawFrame(const uint8_t* buffer, unsigned int timeoutMs) = 0;
};