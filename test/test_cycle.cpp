/**
 * @file test_cycle.cpp
 * @brief poll(): timing, readings, events, MISSING/DISABLED, reconnection.
 */

#include "test_harness.h"

int main() {
    printf("test_cycle\n");

    SECTION("poll() requests, waits out the conversion, then reads");
    {
        resetStore();
        Rig r;
        r.add(1);
        r.m.begin(nullptr);
        r.m.poll(0);
        CHECK(r.dt.requestCalls == 1);
        CHECK(r.m.isConverting());
        r.m.poll(500);                               // conversion not done
        CHECK(r.events.empty());
        r.m.poll(750);
        CHECK(!r.m.isConverting());
        CHECK(r.events.size() == 1);                 // initial state reported
        r.m.poll(5000);                              // interval not elapsed
        CHECK(r.dt.requestCalls == 1);
        r.m.poll(10000);
        CHECK(r.dt.requestCalls == 2);
    }

    SECTION("conversion time follows the resolution");
    {
        resetStore();
        Rig r;
        r.add(1);
        r.m.begin(nullptr, false, 9);
        r.m.poll(0);
        r.m.poll(94);
        CHECK(r.events.size() == 1);
    }

    SECTION("new sensor: DISABLED, value still measured");
    {
        resetStore();
        Rig r;
        r.add(1, 30.0f);
        r.m.begin(nullptr);
        r.cycle();
        CHECK(r.events.size() == 1);
        CHECK_STATE(r.events[0].to, State::DISABLED);
        CHECK(!TemperatureChannel::isAlarm(r.events[0].to));
        CHECK_NEAR(r.events[0].value, 30.0f, 1e-3);
    }

    SECTION("enabled channel: OK, then soft excursion, then alarm after the delay");
    {
        resetStore();
        uint8_t a[8]; makeAddress(a, 1);
        Rig r;
        r.add(1, -18.0f);
        r.m.begin(nullptr);
        r.m.setConfig(0, a, freezer());
        r.cycle();
        CHECK_STATE(r.state(0), State::OK);
        r.setTemp(1, -12.0f);
        r.run(30);
        CHECK_STATE(r.state(0), State::PENDING_HIGH);
        r.run(600);
        CHECK_STATE(r.state(0), State::ALARM_HIGH);
        const TemperatureMonitor::Event* e = r.lastFor(0);
        CHECK(e && e->from == State::PENDING_HIGH && e->to == State::ALARM_HIGH);
        CHECK(e && memcmp(e->address, a, 8) == 0);
        CHECK_NEAR(e->value, -12.0f, 1e-3);
    }

    SECTION("one event per change, none while steady");
    {
        resetStore();
        uint8_t a[8]; makeAddress(a, 1);
        Rig r;
        r.add(1, -18.0f);
        r.m.begin(nullptr);
        r.m.setConfig(0, a, freezer());
        r.run(300);
        CHECK(r.events.size() == 1);                 // UNKNOWN -> OK only
    }

    SECTION("missing at start: MISSING event, then reconnects by itself");
    {
        resetStore();
        uint8_t a[8]; makeAddress(a, 2);
        {
            Rig r;
            r.add(1); r.add(2);
            r.m.begin("tm");
            r.m.setConfig(1, a, freezer());
        }
        Rig r;
        r.add(1); r.add(2);
        r.dev(2).present = false;
        r.m.begin("tm");
        r.cycle();
        CHECK_STATE(r.state(1), State::MISSING);
        CHECK(TemperatureChannel::isAlarm(State::MISSING));
        const TemperatureMonitor::Event* e = r.lastFor(1);
        CHECK(e && e->to == State::MISSING);
        r.dev(2).present = true;                     // connector re-seated
        r.cycle();
        CHECK_STATE(r.state(1), State::OK);
        e = r.lastFor(1);
        CHECK(e && e->from == State::MISSING && e->to == State::OK);
    }

    SECTION("disabled channel never alarms; enabling reports the real state");
    {
        resetStore();
        uint8_t a[8]; makeAddress(a, 1);
        Rig r;
        r.add(1, 24.0f);                             // chamber not in operation
        r.m.begin(nullptr);
        r.run(120);
        CHECK_STATE(r.state(0), State::DISABLED);
        Cfg c = freezer();
        r.m.setConfig(0, a, c);
        CHECK_STATE(r.state(0), State::DISABLED);    // applied at the next cycle
        r.cycle();
        CHECK_STATE(r.state(0), State::CRITICAL_HIGH);
        const TemperatureMonitor::Event* e = r.lastFor(0);
        CHECK(e && e->from == State::DISABLED && e->to == State::CRITICAL_HIGH);
    }

    SECTION("disabled beats missing");
    {
        resetStore();
        { Rig r; r.add(1); r.m.begin("tm"); }
        Rig r;
        r.add(1);
        r.dev(1).present = false;
        r.m.begin("tm");
        r.cycle();
        CHECK_STATE(r.state(0), State::DISABLED);
    }

    SECTION("sensor error: FAULT with the reason");
    {
        resetStore();
        uint8_t a[8]; makeAddress(a, 1);
        Rig r;
        r.add(1);
        r.m.begin(nullptr);
        r.m.setConfig(0, a, freezer());              // fault delay 0
        r.cycle();
        r.dev(1).raw = DEVICE_POWER_ON_RESET_RAW;
        r.cycle();
        CHECK_STATE(r.state(0), State::FAULT);
        const TemperatureMonitor::Event* e = r.lastFor(0);
        CHECK(e && e->sensorError == DS18B20Sensor::Error::POWER_ON_RESET);
        CHECK(e && std::isnan(e->raw));
        r.setTemp(1, -18.0f);
        r.cycle();
        CHECK_STATE(r.state(0), State::OK);
    }

    SECTION("callback may query the monitor (no lock held)");
    {
        resetStore();
        Rig r;
        r.add(1);
        r.m.begin(nullptr);
        static int seen = 0;
        r.m.onEvent([](const TemperatureMonitor::Event& e, void* ctx) {
            TemperatureMonitor* mon = static_cast<TemperatureMonitor*>(ctx);
            TemperatureMonitor::Status st;
            if (mon->getStatus(e.slot, st, e.atMs) && st.state == e.to) seen++;
        }, &r.m);
        r.cycle();
        CHECK(seen == 1);
    }

    SECTION("poll() before begin() does nothing");
    {
        Rig r;
        r.add(1);
        r.m.poll(0);
        r.m.poll(1000);
        CHECK(r.dt.requestCalls == 0);
        CHECK(r.events.empty());
    }

    return finish("test_cycle");
}
