/**
 * @file tm_platform.h
 * @brief Platform glue for TemperatureMonitor: lock and key-value storage.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Péter Soós — https://github.com/soosp
 *
 * Lock: a FreeRTOS mutex in a static buffer on ESP32 (no heap), a no-op
 * elsewhere — ESP8266, AVR and the host test build have no preemptive
 * threads.
 *
 * Storage: the Preferences API on every platform, from three libraries:
 * native on ESP32, vshymanskyy/Preferences on ESP8266, soosp/EEPROMPreferences
 * on AVR. Host builds pick up the stub in test/stubs.
 */

#pragma once

#include <stdint.h>

#if defined(ARDUINO_ARCH_ESP32)
    #include <freertos/FreeRTOS.h>
    #include <freertos/semphr.h>
#endif

#if defined(ARDUINO_ARCH_AVR)
    #include <EEPROMPreferences.h>
    namespace tm_detail { typedef EEPROMPreferences Prefs; }
#else
    #include <Preferences.h>
    namespace tm_detail { typedef Preferences Prefs; }
#endif

/// Mutex acquisition timeout in milliseconds (ESP32 only).
#ifndef TEMPERATURE_MONITOR_MUTEX_TIMEOUT
    #define TEMPERATURE_MONITOR_MUTEX_TIMEOUT 1000
#endif

namespace tm_detail {

/**
 * @brief Non-recursive mutex; static storage on ESP32, no-op elsewhere.
 */
class Lock {
public:
#if defined(ARDUINO_ARCH_ESP32)
    Lock()  { _h = xSemaphoreCreateMutexStatic(&_buf); }
    ~Lock() { vSemaphoreDelete(_h); }
    bool take() const {
        return xSemaphoreTake(_h, pdMS_TO_TICKS(TEMPERATURE_MONITOR_MUTEX_TIMEOUT)) == pdTRUE;
    }
    void give() const { xSemaphoreGive(_h); }
private:
    mutable StaticSemaphore_t _buf;
    SemaphoreHandle_t         _h;
#else
    Lock() {}
    bool take() const { return true; }
    void give() const {}
#endif
public:
    Lock(const Lock&)            = delete;
    Lock& operator=(const Lock&) = delete;
};

/**
 * @brief Scope guard: releases the lock on every return path.
 *
 * Check ok() before touching the protected data.
 */
class Guard {
public:
    explicit Guard(const Lock& l) : _l(l), _ok(l.take()) {}
    ~Guard() { if (_ok) _l.give(); }
    bool ok() const { return _ok; }
    Guard(const Guard&)            = delete;
    Guard& operator=(const Guard&) = delete;
private:
    const Lock& _l;
    bool        _ok;
};

} // namespace tm_detail
