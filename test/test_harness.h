#pragma once

/**
 * @file test_harness.h
 * @brief Assertion helpers and a simulated bus for the TemperatureMonitor
 *        host tests.
 *
 * stubs/ replaces OneWire, DallasTemperature, CheckDS18B20 and Preferences;
 * third_party/ holds copies of the published RangeMonitor and DS18B20Sensor
 * headers, so the suite builds without a library manager.
 */

#include "../src/TemperatureMonitor.h"
#include <cstdio>
#include <cmath>
#include <cstring>
#include <vector>

using State  = TemperatureChannel::State;
using Result = TemperatureMonitor::Result;
using Cfg    = TemperatureMonitor::Config;

inline int& testFailures() { static int failures = 0; return failures; }

inline void checkImpl(bool ok, const char* expr, const char* file, int line) {
    if (!ok) {
        testFailures()++;
        printf("  FAIL %s:%d  %s\n", file, line, expr);
    }
}

#define CHECK(expr) checkImpl((expr), #expr, __FILE__, __LINE__)
#define CHECK_NEAR(a, b, tol) checkImpl(std::fabs((double)(a) - (double)(b)) <= (tol), \
                                        #a " ~= " #b, __FILE__, __LINE__)
#define SECTION(name) printf("- %s\n", name)

inline void checkStateImpl(State a, State e, const char* expr, const char* file, int line) {
    if (a != e) {
        char an[TemperatureChannel::STATE_NAME_SIZE], en[TemperatureChannel::STATE_NAME_SIZE];
        TemperatureChannel::stateName(a, an, sizeof(an));
        TemperatureChannel::stateName(e, en, sizeof(en));
        testFailures()++;
        printf("  FAIL %s:%d  %s is %s, expected %s\n", file, line, expr, an, en);
    }
}
#define CHECK_STATE(a, e) checkStateImpl((a), (e), #a, __FILE__, __LINE__)

inline int finish(const char* name) {
    if (testFailures() == 0) printf("%s: OK\n", name);
    else                     printf("%s: %d FAILED\n", name, testFailures());
    return testFailures() == 0 ? 0 : 1;
}

/// A valid DS18B20 ROM address (family 0x28, correct CRC) with serial @p n.
inline void makeAddress(uint8_t* a, uint8_t n) {
    a[0] = DS18B20MODEL;
    for (uint8_t i = 1; i < 7; i++) a[i] = (uint8_t)(n * 17 + i);
    a[7] = OneWire::crc8(a, 7);
}

/// Wipe the simulated flash.
inline void resetStore() {
    Preferences::store().clear();
    Preferences::failWrites() = false;
    Preferences::writeCount() = 0;
}

/**
 * @brief A bus with sensors, a monitor on it, and the events it produced.
 */
struct Rig {
    OneWire                         ow;
    DallasTemperature               dt;
    TemperatureMonitor              m;
    std::vector<TemperatureMonitor::Event> events;
    uint32_t                        now = 0;

    Rig() : dt(&ow), m(ow, dt) {
        m.onEvent([](const TemperatureMonitor::Event& e, void* ctx) {
            static_cast<Rig*>(ctx)->events.push_back(e);
        }, this);
        m.setPollInterval(10000);
    }

    /// Put sensor @p n on the bus reading @p celsius; returns its index.
    int add(uint8_t n, float celsius = -18.0f) {
        uint8_t a[8];
        makeAddress(a, n);
        dt.add(a).raw = DallasTemperature::celsiusToRaw(celsius);
        return dt.count - 1;
    }

    DallasTemperature::Device& dev(uint8_t n) {
        uint8_t a[8];
        makeAddress(a, n);
        return *dt.find(a);
    }

    void setTemp(uint8_t n, float c) { dev(n).raw = DallasTemperature::celsiusToRaw(c); }

    /// One full measuring cycle: request, wait out the conversion, read.
    void cycle() {
        m.poll(now);
        now += 800;
        m.poll(now);
        now += 10000 - 800;
    }

    /// Run @p seconds of cycles.
    void run(uint32_t seconds) {
        uint32_t end = now + seconds * 1000UL;
        while ((int32_t)(end - now) > 0) cycle();
    }

    /// Last event for @p slot, or nullptr.
    const TemperatureMonitor::Event* lastFor(uint8_t slot) const {
        for (auto it = events.rbegin(); it != events.rend(); ++it)
            if (it->slot == slot) return &*it;
        return nullptr;
    }

    State state(uint8_t slot) {
        TemperatureMonitor::Status st;
        m.getStatus(slot, st, now);
        return st.state;
    }
};

/// Freezer configuration, monitoring on.
inline Cfg freezer(const char* name = "Freezer") {
    Cfg c;
    TemperatureChannel::copyName(c.name, name);
    c.enabled = true;
    c.range = RangeMonitor::Config::range(-22.0f, -15.0f, 1.0f, 0);
    c.range.hiHard = -5.0f;
    c.range.alarmDelaySeconds = 600;
    return c;
}
