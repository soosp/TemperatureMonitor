#pragma once
/**
 * @file OneWire.h (host test stub)
 * @brief Only what DS18B20Sensor uses: the class name and crc8().
 */
#include <stdint.h>

// The ESP32 core defines DISABLED (an interrupt mode) as a macro, and every
// sketch sees it through Arduino.h. Defined here too, so that a library
// header using the name unprotected fails the host build as well.
#define DISABLED 0x00

class OneWire {
public:
    OneWire() {}
    explicit OneWire(uint8_t /*pin*/) {}

    /// Dallas/Maxim CRC-8 (polynomial x^8 + x^5 + x^4 + 1), as in OneWire.
    static uint8_t crc8(const uint8_t* addr, uint8_t len) {
        uint8_t crc = 0;
        while (len--) {
            uint8_t inbyte = *addr++;
            for (uint8_t i = 8; i; i--) {
                uint8_t mix = (crc ^ inbyte) & 0x01;
                crc >>= 1;
                if (mix) crc ^= 0x8C;
                inbyte >>= 1;
            }
        }
        return crc;
    }
};
