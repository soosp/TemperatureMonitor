/**
 * TemperatureMonitor — Monitor
 *
 * Finds the sensors on the bus, matches them to their stored slots, and
 * prints every state change. A sensor seen for the first time gets the
 * default configuration — here a freezer range with monitoring switched on,
 * so the example does something visible; a real application would leave new
 * sensors off until someone names them.
 *
 * Restart with a sensor unplugged to see MISSING; plug it back in to see it
 * reconnect. Heat a sensor in your hand to see PENDING_HIGH, then ALARM_HIGH
 * after the delay.
 *
 * Wiring: DS18B20 data line to ONEWIRE_PIN with a 4.7 kΩ pull-up.
 */

#include <OneWire.h>
#include <DallasTemperature.h>
#include <TemperatureMonitor.h>

#ifndef ONEWIRE_PIN
#define ONEWIRE_PIN 4
#endif

OneWire            oneWire(ONEWIRE_PIN);
DallasTemperature  dallas(&oneWire);
TemperatureMonitor monitor(oneWire, dallas);

// Runs after poll() has released its locks. Keep it short: in a FreeRTOS
// application, copy the event into a queue here and act on it elsewhere.
void onEvent(const TemperatureMonitor::Event& e, void*) {
    char addr[DS18B20Sensor::ADDR_STR_SIZE];
    char from[TemperatureChannel::STATE_NAME_SIZE];
    char to[TemperatureChannel::STATE_NAME_SIZE];
    DS18B20Sensor::formatAddress(e.address, addr, sizeof(addr));
    TemperatureChannel::stateName(e.from, from, sizeof(from));
    TemperatureChannel::stateName(e.to, to, sizeof(to));

    Serial.print(F("slot "));
    Serial.print(e.slot);
    Serial.print(F(" ("));
    Serial.print(addr);
    Serial.print(F("): "));
    Serial.print(from);
    Serial.print(F(" -> "));
    Serial.print(to);
    if (!isnan(e.value)) {
        Serial.print(F("  "));
        Serial.print(e.value, 1);
        Serial.print(F(" C"));
    }
    if (TemperatureChannel::isAlarm(e.to) && !TemperatureChannel::isAlarm(e.from)) {
        Serial.print(F("  ** ALARM **"));
    } else if (!TemperatureChannel::isAlarm(e.to) && TemperatureChannel::isAlarm(e.from)) {
        Serial.print(F("  (cleared)"));
    }
    Serial.println();
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    TemperatureMonitor::Config def;
    def.enabled = true;
    def.range   = RangeMonitor::Config::range(-22.0f, -15.0f, 1.0f, 60);
    def.range.hiHard            = -5.0f;
    def.range.alarmDelaySeconds = 120;     // short, for a demo
    def.range.faultDelaySeconds = 30;
    monitor.setDefaultConfig(def);

    monitor.setPollInterval(5000);
    monitor.onEvent(onEvent, nullptr);
    monitor.begin("tmdemo");

    TemperatureMonitor::Census c = monitor.census();
    Serial.print(F("Found "));     Serial.print(c.found);
    Serial.print(F(", matched ")); Serial.print(c.matched);
    Serial.print(F(", new "));     Serial.print(c.added);
    Serial.print(F(", missing ")); Serial.print(c.missing);
    Serial.print(F(", no slot ")); Serial.println(c.unassigned);
}

void loop() {
    monitor.poll();        // never blocks; reads after the conversion time
    delay(20);
}
