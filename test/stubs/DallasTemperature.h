#pragma once
/**
 * @file DallasTemperature.h (host test stub)
 * @brief A scriptable stand-in for DallasTemperature 4.x.
 *
 * Holds a small table of devices. Tests set each device's next raw reading,
 * whether it answers, and its resolution, then check what DS18B20Sensor makes
 * of it. The constants are copied from DallasTemperature 4.0.x.
 */
#include <stdint.h>
#include <string.h>
#include "OneWire.h"

typedef uint8_t DeviceAddress[8];

#define DS18S20MODEL  0x10
#define DS18B20MODEL  0x28
#define DS1822MODEL   0x22
#define DS1825MODEL   0x3B
#define DS28EA00MODEL 0x42

#define DEVICE_DISCONNECTED_C         -127
#define DEVICE_DISCONNECTED_RAW       -7040
#define DEVICE_FAULT_OPEN_RAW         -32512
#define DEVICE_FAULT_SHORTGND_RAW     -32384
#define DEVICE_FAULT_SHORTVDD_RAW     -32256
#define DEVICE_POWER_ON_RESET_RAW     -32128
#define DEVICE_INSUFFICIENT_POWER_RAW -32000

class DallasTemperature {
public:
    struct Device {
        DeviceAddress addr;
        bool    present    = true;
        int32_t raw        = 0;       ///< Returned by getTemp()
        int     crcFails   = 0;       ///< getTemp() attempts that fail before success
        uint8_t resolution = 12;
        bool    setResolutionFails = false;
    };

    static constexpr int MAX = 32;
    Device dev[MAX];
    int    count = 0;

    // Call log
    int requestCalls = 0;
    int recallCalls = 0;
    int getTempCalls = 0;
    uint8_t lastRetryCount = 0xFF;

    explicit DallasTemperature(OneWire* = nullptr) {}

    // Bus-level calls the examples make; no effect on the device table.
    bool    waitForConversion = true;
    void    begin() {}
    void    setWaitForConversion(bool w) { waitForConversion = w; }
    /// Devices that answer a search (present ones), like the real bus.
    uint8_t getDS18Count() {
        uint8_t n = 0;
        for (int i = 0; i < count; i++) if (dev[i].present) n++;
        return n;
    }
    void    requestTemperatures() { requestCalls++; }
    static uint16_t millisToWaitForConversion(uint8_t bits) {
        switch (bits) { case 9: return 94; case 10: return 188; case 11: return 375; default: return 750; }
    }

    Device& add(const uint8_t* a) {
        memcpy(dev[count].addr, a, 8);
        return dev[count++];
    }

    Device* find(const uint8_t* a) {
        for (int i = 0; i < count; i++)
            if (memcmp(dev[i].addr, a, 8) == 0) return &dev[i];
        return nullptr;
    }

    /// index-th *present* device, as a real bus search would number them.
    bool getAddress(uint8_t* out, uint8_t index) {
        int n = 0;
        for (int i = 0; i < count; i++) {
            if (!dev[i].present) continue;
            if (n++ == index) { memcpy(out, dev[i].addr, 8); return true; }
        }
        return false;
    }

    bool isConnected(const uint8_t* a) {
        Device* d = find(a);
        return d && d->present;
    }

    bool setResolution(const uint8_t* a, uint8_t bits, bool = false) {
        Device* d = find(a);
        if (!d || !d->present || d->setResolutionFails) return false;
        if (a[0] != DS18S20MODEL) d->resolution = bits;
        return true;
    }

    uint8_t getResolution(const uint8_t* a) {
        Device* d = find(a);
        if (!d) return 0;
        return (a[0] == DS18S20MODEL) ? 9 : d->resolution;
    }

    bool recallScratchPad(const uint8_t* = nullptr) { recallCalls++; return true; }

    int32_t getTemp(const uint8_t* a, uint8_t retryCount = 0) {
        getTempCalls++;
        lastRetryCount = retryCount;
        Device* d = find(a);
        if (!d || !d->present) return DEVICE_DISCONNECTED_RAW;
        for (uint8_t attempt = 0; attempt <= retryCount; attempt++) {
            if (d->crcFails > 0) { d->crcFails--; continue; }
            return d->raw;
        }
        return DEVICE_DISCONNECTED_RAW;
    }

    static float rawToCelsius(int32_t raw) {
        if (raw <= DEVICE_DISCONNECTED_RAW) return DEVICE_DISCONNECTED_C;
        return (float)raw * 0.0078125f;
    }

    /// Helper for tests: °C to the raw 1/128 °C fixed point.
    static int32_t celsiusToRaw(float c) { return (int32_t)(c * 128.0f); }
};
