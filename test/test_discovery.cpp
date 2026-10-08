/**
 * @file test_discovery.cpp
 * @brief begin(): stored slots, bus search, address matching, persistence.
 */

#include "test_harness.h"

static void addr(uint8_t* a, uint8_t n) { makeAddress(a, n); }

int main() {
    printf("test_discovery\n");

    SECTION("first start: every sensor gets a slot, monitoring off, stored");
    {
        resetStore();
        Rig r;
        r.add(1); r.add(2); r.add(3);
        CHECK(r.m.begin("tm"));
        TemperatureMonitor::Census c = r.m.census();
        CHECK(c.found == 3 && c.added == 3 && c.matched == 0 && c.missing == 0);
        for (uint8_t s = 0; s < 3; s++) {
            CHECK(r.m.isAssigned(s));
            Cfg cfg;
            CHECK(r.m.getConfig(s, cfg));
            CHECK(!cfg.enabled);
        }
        CHECK(!r.m.isAssigned(3));
        CHECK(Preferences::store()["tm"].size() == 3);
    }

    SECTION("restart with the bus order changed: every sensor keeps its slot");
    {
        resetStore();
        uint8_t a1[8], a2[8], a3[8];
        addr(a1, 1); addr(a2, 2); addr(a3, 3);
        {
            Rig r;
            r.add(1); r.add(2); r.add(3);
            r.m.begin("tm");
            r.m.setConfig(0, a1, freezer("One"));
            r.m.setConfig(2, a3, freezer("Three"));
        }
        Rig r;
        r.add(3); r.add(1); r.add(2);                // different search order
        r.m.begin("tm");
        TemperatureMonitor::Census c = r.m.census();
        CHECK(c.matched == 3 && c.added == 0);
        CHECK(r.m.findSlot(a1) == 0);
        CHECK(r.m.findSlot(a2) == 1);
        CHECK(r.m.findSlot(a3) == 2);
        Cfg cfg;
        r.m.getConfig(0, cfg); CHECK(strcmp(cfg.name, "One") == 0 && cfg.enabled);
        r.m.getConfig(2, cfg); CHECK(strcmp(cfg.name, "Three") == 0);
    }

    SECTION("sensor replaced: old slot MISSING, new sensor gets a free slot");
    {
        resetStore();
        uint8_t a2[8], a4[8];
        addr(a2, 2); addr(a4, 4);
        {
            Rig r;
            r.add(1); r.add(2); r.add(3);
            r.m.begin("tm");
        }
        Rig r;
        r.add(1); r.add(3); r.add(4);                // 2 gone, 4 new
        r.m.begin("tm");
        TemperatureMonitor::Census c = r.m.census();
        CHECK(c.found == 3 && c.matched == 2 && c.added == 1 && c.missing == 1);
        CHECK(r.m.findSlot(a2) == 1);                // still bound
        CHECK(r.m.findSlot(a4) == 3);                // not slot 1
        TemperatureMonitor::Status st;
        r.m.getStatus(1, st, 0);
        CHECK(st.assigned && !st.present);
    }

    SECTION("more sensors than slots: the rest are counted, not assigned");
    {
        resetStore();
        Rig r;
        for (uint8_t i = 0; i < TemperatureMonitor::MAX_CHANNELS + 2; i++) r.add(i + 1);
        r.m.begin("tm");
        TemperatureMonitor::Census c = r.m.census();
        CHECK(c.added == TemperatureMonitor::MAX_CHANNELS);
        CHECK(c.unassigned == 2);
    }

    SECTION("identification: family stored and kept without re-identifying");
    {
        resetStore();
        CheckDS18B20::stubFamilies()[0] = CheckDS18B20::FAMILY_B1;
        {
            Rig r;
            r.add(1);
            r.m.begin("tm", true);
            TemperatureMonitor::Status st;
            r.m.getStatus(0, st, 0);
            CHECK(st.family == CheckDS18B20::FAMILY_B1);
        }
        CheckDS18B20::stubFamilies()[0] = CheckDS18B20::FAMILY_A1;
        Rig r;
        r.add(1);
        r.m.begin("tm", false);
        TemperatureMonitor::Status st;
        r.m.getStatus(0, st, 0);
        CHECK(st.family == CheckDS18B20::FAMILY_B1);
    }

    SECTION("unchanged restart writes nothing");
    {
        resetStore();
        { Rig r; r.add(1); r.add(2); r.m.begin("tm"); }
        Preferences::writeCount() = 0;
        { Rig r; r.add(1); r.add(2); r.m.begin("tm"); }
        CHECK(Preferences::writeCount() == 0);
    }

    SECTION("no namespace: works, stores nothing");
    {
        resetStore();
        Rig r;
        r.add(1);
        CHECK(r.m.begin(nullptr));
        CHECK(r.m.isAssigned(0));
        CHECK(Preferences::store().empty());
    }

    SECTION("unreadable record: counted, slot not reused");
    {
        resetStore();
        Preferences::store()["tm"]["s0"] = std::vector<uint8_t>{ 99, 1, 2, 3 };
        Rig r;
        r.add(1);
        r.m.begin("tm");
        TemperatureMonitor::Census c = r.m.census();
        CHECK(c.invalidRecords == 1);
        CHECK(!r.m.isAssigned(0));
        CHECK(r.m.isAssigned(1));                    // new sensor went elsewhere
        CHECK(Preferences::store()["tm"]["s0"].size() == 4);   // left alone
    }

    SECTION("storage write failure is reported, sensors still run");
    {
        resetStore();
        Preferences::failWrites() = true;
        Rig r;
        r.add(1);
        CHECK(!r.m.begin("tm"));
        CHECK(r.m.isAssigned(0));
        Preferences::failWrites() = false;
    }

    return finish("test_discovery");
}
