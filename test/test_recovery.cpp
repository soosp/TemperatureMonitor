/**
 * @file test_recovery.cpp
 * @brief Sensors that come and go at runtime, slot release, unreadable
 *        records, default config validation.
 *
 * Regression tests for the issues found in review before 0.1.0.
 */

#include "test_harness.h"

int main() {
    printf("test_recovery\n");
    uint8_t a1[8], a2[8];
    makeAddress(a1, 1); makeAddress(a2, 2);

    SECTION("sensor missing at start reconnects at the begin() resolution");
    {
        resetStore();
        { Rig r; r.add(1); r.add(2); r.m.begin("tm", false, 9); }
        Rig r;
        r.add(1); r.add(2);
        r.dev(2).present    = false;
        r.dev(2).resolution = 12;                    // EEPROM default after power-up
        r.m.begin("tm", false, 9);
        r.cycle();
        CHECK_STATE(r.state(1), State::OFF);
        r.dev(2).present = true;
        r.cycle();
        CHECK(r.dev(2).resolution == 9);
    }

    SECTION("sensor that resets at runtime gets its resolution back");
    {
        resetStore();
        Rig r;
        r.add(1);
        r.m.begin(nullptr, false, 10);
        r.cycle();
        CHECK(r.dev(1).resolution == 10);
        r.dev(1).resolution = 12;                    // brown-out: EEPROM defaults
        r.dev(1).raw = DEVICE_POWER_ON_RESET_RAW;
        r.cycle();
        CHECK(r.dev(1).resolution == 10);
    }

    SECTION("sensor unplugged and re-plugged at runtime gets its resolution back");
    {
        resetStore();
        Rig r;
        r.add(1);
        r.m.begin(nullptr, false, 11);
        r.cycle();
        r.dev(1).present = false;
        r.cycle();
        r.dev(1).present    = true;
        r.dev(1).resolution = 12;
        r.setTemp(1, -18.0f);
        r.cycle();
        CHECK(r.dev(1).resolution == 11);
    }

    SECTION("forget() allowed once a sensor is gone at runtime");
    {
        resetStore();
        Rig r;
        r.add(1);
        r.m.begin("tm");
        r.cycle();
        CHECK(r.m.forget(0, a1) == Result::IN_USE);
        r.dev(1).present = false;
        r.cycle();
        CHECK(r.m.forget(0, a1) == Result::OK);
        CHECK(!r.m.isAssigned(0));
    }

    SECTION("forget() clears the MISSING alarm through an event");
    {
        resetStore();
        {
            Rig r; r.add(1); r.add(2); r.m.begin("tm");
            r.m.setConfig(1, a2, freezer());
        }
        Rig r;
        r.add(1); r.add(2);
        r.dev(2).present = false;
        r.m.begin("tm");
        r.cycle();
        CHECK_STATE(r.state(1), State::MISSING);
        CHECK(r.m.forget(1, a2) == Result::OK);
        r.events.clear();
        r.cycle();
        const TemperatureMonitor::Event* e = r.lastFor(1);
        CHECK(e != nullptr);
        CHECK(e && e->from == State::MISSING && e->to == State::UNKNOWN);
        CHECK(e && !TemperatureChannel::isAlarm(e->to));
        r.events.clear();
        r.run(60);
        CHECK(r.lastFor(1) == nullptr);              // and only once
    }

    SECTION("an invalid default config is refused; new sensors still get slots");
    {
        resetStore();
        Rig r;
        r.add(1); r.add(2);
        Cfg bad;
        bad.range = RangeMonitor::Config::range(5.0f, 1.0f, 0.0f, 0);
        CHECK(r.m.setDefaultConfig(bad) == RangeMonitor::ConfigError::SOFT_ORDER);
        CHECK(r.m.begin("tm"));
        TemperatureMonitor::Census c = r.m.census();
        CHECK(c.added == 2 && c.unassigned == 0);
        CHECK(r.m.findSlot(a1) == 0 && r.m.findSlot(a2) == 1);
        uint8_t zero[8] = {0};
        CHECK(r.m.findSlot(zero) < 0);
    }

    SECTION("unreadable record: reported, and can be discarded");
    {
        resetStore();
        Preferences::store()["tm"]["s0"] = std::vector<uint8_t>{ 99, 1, 2, 3 };
        {
            Rig r;
            r.add(1);
            r.m.begin("tm");
            CHECK(r.m.hasUnreadableRecord(0));
            CHECK(!r.m.hasUnreadableRecord(1));
            CHECK(r.m.discardRecord(1) == Result::EMPTY_SLOT);
            CHECK(r.m.discardRecord(0) == Result::OK);
            CHECK(!r.m.hasUnreadableRecord(0));
            CHECK(Preferences::store()["tm"].count("s0") == 0);
        }
        Rig r;                                       // slot 0 usable again
        r.add(1); r.add(2);
        r.m.begin("tm");
        CHECK(r.m.findSlot(a2) == 0);
    }

    SECTION("event carries the reading of the same update");
    {
        resetStore();
        Rig r;
        r.add(1, 30.0f);
        r.m.begin(nullptr);
        r.cycle();
        CHECK(r.events.size() == 1);
        CHECK_NEAR(r.events[0].raw, 30.0f, 1e-3);
        CHECK(r.events[0].sensorError == DS18B20Sensor::Error::NONE);
        CHECK(memcmp(r.events[0].address, a1, 8) == 0);
    }

    return finish("test_recovery");
}
