/**
 * @file test_config.cpp
 * @brief setConfig()/forget() guards, persistence records, names, helpers.
 */

#include "test_harness.h"

int main() {
    printf("test_config\n");
    uint8_t a1[8], a2[8], a9[8];
    makeAddress(a1, 1); makeAddress(a2, 2); makeAddress(a9, 9);

    SECTION("setConfig: applied, stored, survives a restart");
    {
        resetStore();
        {
            Rig r;
            r.add(1);
            r.m.begin("tm");
            CHECK(r.m.setConfig(0, a1, freezer("Fagyasztó")) == Result::OK);
        }
        Rig r;
        r.add(1);
        r.m.begin("tm");
        Cfg c;
        CHECK(r.m.getConfig(0, c));
        CHECK(strcmp(c.name, "Fagyasztó") == 0);
        CHECK(c.enabled);
        CHECK_NEAR(c.range.hiHard, -5.0f, 1e-6);
        CHECK(std::isnan(c.range.loHard));
        CHECK(c.range.alarmDelaySeconds == 600);
    }

    SECTION("setConfig: guards");
    {
        resetStore();
        Rig r;
        r.add(1);
        CHECK(r.m.setConfig(0, a1, freezer()) == Result::NOT_STARTED);
        r.m.begin("tm");
        CHECK(r.m.setConfig(0, a2, freezer()) == Result::ADDRESS_MISMATCH);
        CHECK(r.m.setConfig(1, a1, freezer()) == Result::EMPTY_SLOT);
        CHECK(r.m.setConfig(TemperatureMonitor::MAX_CHANNELS, a1, freezer()) == Result::INVALID_SLOT);
        Cfg bad = freezer();
        bad.range.hiHard = -20.0f;                   // inside the soft range
        RangeMonitor::ConfigError why;
        CHECK(r.m.setConfig(0, a1, bad, true, &why) == Result::INVALID_CONFIG);
        CHECK(why == RangeMonitor::ConfigError::HI_HARD_INSIDE);
        Cfg c;
        r.m.getConfig(0, c);
        CHECK(!c.enabled);                           // nothing changed
    }

    SECTION("setConfig: storage failure keeps the RAM copy");
    {
        resetStore();
        Rig r;
        r.add(1);
        r.m.begin("tm");
        Preferences::failWrites() = true;
        CHECK(r.m.setConfig(0, a1, freezer()) == Result::STORAGE_ERROR);
        Preferences::failWrites() = false;
        Cfg c;
        r.m.getConfig(0, c);
        CHECK(c.enabled);
    }

    SECTION("setConfig without saving");
    {
        resetStore();
        Rig r;
        r.add(1);
        r.m.begin("tm");
        Preferences::writeCount() = 0;
        CHECK(r.m.setConfig(0, a1, freezer(), false) == Result::OK);
        CHECK(Preferences::writeCount() == 0);
    }

    SECTION("forget: refused while present, allowed when missing");
    {
        resetStore();
        { Rig r; r.add(1); r.add(2); r.m.begin("tm"); }
        Rig r;
        r.add(1); r.add(2);
        r.dev(2).present = false;
        r.m.begin("tm");
        CHECK(r.m.forget(0, a1) == Result::IN_USE);
        CHECK(r.m.forget(1, a1) == Result::ADDRESS_MISMATCH);
        CHECK(r.m.forget(1, a2) == Result::OK);
        CHECK(!r.m.isAssigned(1));
        CHECK(Preferences::store()["tm"].count("s1") == 0);
    }

    SECTION("stale form after a sensor swap cannot write onto the new sensor");
    {
        resetStore();
        { Rig r; r.add(1); r.add(2); r.m.begin("tm"); }
        {   // sensor 2 removed and forgotten
            Rig r; r.add(1); r.add(2); r.dev(2).present = false;
            r.m.begin("tm");
            r.m.forget(1, a2);
        }
        Rig r;                                       // sensor 9 installed, takes slot 1
        r.add(1); r.add(9);
        r.m.begin("tm");
        CHECK(r.m.findSlot(a9) == 1);
        CHECK(r.m.setConfig(1, a2, freezer("Old")) == Result::ADDRESS_MISMATCH);
    }

    SECTION("names are cut at a UTF-8 character boundary");
    {
        char out[TemperatureChannel::NAME_SIZE];
        char in[64];
        memset(in, 'x', 31);
        strcpy(in + 31, "é");                        // 31 + 2 bytes = 33 > 32
        TemperatureChannel::copyName(out, in);
        CHECK(strlen(out) == 31);
        memset(in, 'x', 30);
        strcpy(in + 30, "é");                        // exactly 32 bytes
        TemperatureChannel::copyName(out, in);
        CHECK(strlen(out) == 32);
        TemperatureChannel::copyName(out, nullptr);
        CHECK(out[0] == '\0');
    }

    SECTION("record: round trip, size, rejects damage");
    {
        Cfg c = freezer("Kamra 1");
        c.range.spikeFilter = false;
        c.range.loHard = -30.0f;
        uint8_t buf[TemperatureMonitor::RECORD_SIZE];
        size_t len = TemperatureMonitor::packRecord(buf, a1, CheckDS18B20::FAMILY_C, c);
        CHECK(len <= TemperatureMonitor::RECORD_SIZE);
        DeviceAddress a;
        DS18B20Sensor::Family f;
        Cfg d;
        CHECK(TemperatureMonitor::unpackRecord(buf, len, a, f, d));
        CHECK(memcmp(a, a1, 8) == 0);
        CHECK(f == CheckDS18B20::FAMILY_C);
        CHECK(strcmp(d.name, "Kamra 1") == 0 && d.enabled && !d.range.spikeFilter);
        CHECK_NEAR(d.range.loHard, -30.0f, 1e-6);
        CHECK(std::isnan(d.range.hiHard) == false);
        CHECK(d.range.tauSeconds == c.range.tauSeconds);

        Cfg full;
        memset(full.name, 'n', TemperatureChannel::NAME_LEN);
        full.name[TemperatureChannel::NAME_LEN] = '\0';
        CHECK(TemperatureMonitor::packRecord(buf, a1, CheckDS18B20::FAMILY_A1, full)
              == TemperatureMonitor::RECORD_SIZE);

        len = TemperatureMonitor::packRecord(buf, a1, CheckDS18B20::FAMILY_C, c);
        CHECK(!TemperatureMonitor::unpackRecord(buf, len - 1, a, f, d));   // truncated
        buf[0] = 2;
        CHECK(!TemperatureMonitor::unpackRecord(buf, len, a, f, d));       // unknown version
        buf[0] = TemperatureMonitor::RECORD_VERSION;
        buf[9] = 200;
        CHECK(!TemperatureMonitor::unpackRecord(buf, len, a, f, d));       // bad family
    }

    SECTION("record: a config that no longer validates is rejected");
    {
        Cfg c = freezer();
        c.range.loSoft = 0.0f;                       // lo > hi once stored
        uint8_t buf[TemperatureMonitor::RECORD_SIZE];
        size_t len = TemperatureMonitor::packRecord(buf, a1, CheckDS18B20::FAMILY_A1, c);
        DeviceAddress a; DS18B20Sensor::Family f; Cfg d;
        CHECK(!TemperatureMonitor::unpackRecord(buf, len, a, f, d));
    }

    SECTION("state and result names");
    {
        char b[TemperatureChannel::STATE_NAME_SIZE];
        CHECK(strcmp(TemperatureChannel::stateName(State::MISSING, b, sizeof(b)), "missing") == 0);
        CHECK(strcmp(TemperatureChannel::stateName(State::OFF, b, sizeof(b)), "disabled") == 0);
        CHECK(strcmp(TemperatureChannel::stateName(State::CRITICAL_LOW, b, sizeof(b)), "critical_low") == 0);
        char rb[TemperatureMonitor::RESULT_NAME_SIZE];
        CHECK(strcmp(TemperatureMonitor::resultName(Result::ADDRESS_MISMATCH, rb, sizeof(rb)), "address_mismatch") == 0);
        CHECK(strlen(rb) == TemperatureMonitor::RESULT_NAME_LEN);
        CHECK(strcmp(TemperatureMonitor::resultName(Result::OK, rb, sizeof(rb)), "ok") == 0);
        CHECK(!TemperatureChannel::isAlarm(State::OFF));
#pragma push_macro("DISABLED")
#undef DISABLED
        CHECK(State::DISABLED == State::OFF);     // the old name, where usable
#pragma pop_macro("DISABLED")
        CHECK(!TemperatureChannel::isAlarm(State::PENDING_LOW));
        CHECK(TemperatureChannel::isAlarm(State::FAULT));
    }

    return finish("test_config");
}
