# TemperatureMonitor

Monitored temperature channels on one 1-Wire bus, for ESP32, ESP8266 and AVR
(Arduino). Each channel binds a DS18B20-compatible sensor — by its ROM
address — to a name, a range and an enabled flag, measures it, and reports
every state change as an event.

Built on [RangeMonitor](https://github.com/soosp/RangeMonitor) (spike filter,
EMA, soft limits with delay, hard limits, fault detection) and
[DS18B20Sensor](https://github.com/soosp/DS18B20Sensor) (address-based reads,
classified errors, clone identification).

## Features

- **Identity is the ROM address.** A channel's address, name and limits live
  in one object and are stored in one record, and the sensor is read by that
  address. Readings and configuration cannot drift apart when sensors are
  added, removed or found in a different order.
- **Discovery at start-up.** Known sensors return to their slots; new ones get
  a free slot with a default configuration (monitoring off).
- **Missing sensors are reported, and reconnect by themselves.** A stored
  slot whose sensor is absent reports `MISSING` (an alarm); if the sensor
  answers later — a re-seated connector — the channel resumes. A sensor that
  comes back after a power loss gets its resolution written back, so its
  conversion time keeps matching the cycle.
- **Disabled channels** are still measured and published but never alarm:
  for a chamber that is built but not in use.
- **Guarded changes.** Changing or releasing a slot requires the address the
  caller expects to find there, so a web form rendered before a sensor swap
  cannot write onto the new sensor.
- **Non-blocking cycle.** `poll()` starts one broadcast conversion and reads
  every channel once it has finished; it never waits.
- **Events after the locks.** Every state change goes to one callback, called
  with no lock held, so it can query the monitor or hand the event to a queue.
- **Embedded-friendly.** No dynamic allocation; fixed slot count
  (`TEMPERATURE_MONITOR_MAX_CHANNELS`); C++11 (Arduino AVR core). Thread-safe
  on ESP32 with mutexes in static buffers.

## Usage

```cpp
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TemperatureMonitor.h>

OneWire            oneWire(4);
DallasTemperature  dallas(&oneWire);
TemperatureMonitor monitor(oneWire, dallas);

void onEvent(const TemperatureMonitor::Event& e, void*) {
    if (TemperatureChannel::isAlarm(e.to) && !TemperatureChannel::isAlarm(e.from)) {
        // alarm raised on slot e.slot — queue an SMS, publish, ...
    }
}

void setup() {
    monitor.onEvent(onEvent, nullptr);
    monitor.setPollInterval(10000);
    monitor.begin("tempmon");            // Preferences namespace
}

void loop() {
    monitor.poll();
    delay(20);
}
```

Configuring a slot, e.g. from a web form that carried the address:

```cpp
DeviceAddress expected;
DS18B20Sensor::parseAddress(formAddress, expected);

TemperatureMonitor::Config cfg;
TemperatureChannel::copyName(cfg.name, formName);    // UTF-8, cut at a character boundary
cfg.enabled = true;
cfg.range   = RangeMonitor::Config::range(-22.0f, -12.0f, 1.0f, 60);
cfg.range.hiHard            = 3.0f;
cfg.range.alarmDelaySeconds = 3600;

RangeMonitor::ConfigError why;
TemperatureMonitor::Result r = monitor.setConfig(slot, expected, cfg, true, &why);
// OK, ADDRESS_MISMATCH (stale form), INVALID_CONFIG (see why), STORAGE_ERROR, ...
```

See `examples/Monitor`.

## Channel states

|State|Meaning|Alarm|
|---|---|---|
|`UNKNOWN`|No valid sample yet|no|
|`OK`|Within range|no|
|`PENDING_HIGH` / `PENDING_LOW`|Outside a soft limit, delay running|no|
|`ALARM_HIGH` / `ALARM_LOW`|Outside a soft limit for at least the delay|yes|
|`CRITICAL_HIGH` / `CRITICAL_LOW`|Outside a hard limit|yes|
|`FAULT`|Sensor errors for at least the fault delay|yes|
|`MISSING`|Stored slot, sensor not on the bus|yes|
|`OFF`|Monitoring off; measured, never alarms. Also `DISABLED` (see below)|no|

Values 0–8 are the RangeMonitor states; all values are stable. `OFF`
takes precedence over `MISSING`. `TemperatureChannel::isAlarm()` and
`stateName()` cover all of them.

On ESP32 the core's `Arduino.h` defines `DISABLED` as a macro (an interrupt
mode), so write `State::OFF` there. `State::DISABLED` remains as an alias for
code on the other platforms; `stateName()` still returns `"disabled"`.

## Slots and persistence

There are `TEMPERATURE_MONITOR_MAX_CHANNELS` slots (16; 4 on AVR). Each bound
slot is stored as one record under key `s<slot>` in the namespace given to
`begin()`: version, address, family, flags (enabled, spike filter,
`confirmBeyondLimit`), name and limits, serialised field by field (no struct
dump), at most `RECORD_SIZE` (76) bytes. Storage is the
Preferences API on every platform: native on ESP32,
[vshymanskyy/Preferences](https://github.com/vshymanskyy/Preferences) on
ESP8266, [EEPROMPreferences](https://github.com/soosp/EEPROMPreferences) on
AVR. A record that cannot be read is left alone, counted in
`Census::invalidRecords`, and its slot is not reused.

`begin()` reports what it found:

|`Census` field|Meaning|
|---|---|
|`found`|Sensors on the bus|
|`matched` / `added` / `unassigned`|... with a stored slot / given a new slot / no free slot|
|`missing`|Stored slots whose sensor is absent|
|`invalidRecords`|Stored records that could not be read|

A slot whose sensor is gone for good is released with `forget()` — allowed
when the sensor was missing at start-up or its last reading found it
disconnected, refused (`IN_USE`) while it still answers, since it would only
come back as new at the next start. The channel's transition to `UNKNOWN` is
reported at the next cycle, so an alarm raised for it is cleared through the
normal event path.

A record that could not be read keeps its slot out of use, so nothing
overwrites a record a newer firmware might still understand;
`hasUnreadableRecord()` reports it and `discardRecord()` deletes it.

## Measuring cycle and events

`poll(nowMs)` / `poll()`:

1. When the poll interval has elapsed, start a broadcast conversion.
2. When the conversion time for the configured resolution has passed (750 ms
   at 12 bits), read every channel, update its state, and collect the changes.
3. Release all locks, then call the event callback once per change.

The first cycle after `begin()` reports every channel's initial state, so
missing sensors and alarms present at start-up are not lost.

```cpp
struct Event {
    uint8_t              slot;
    DeviceAddress        address;     // all zero if the slot was released
    State                from, to;
    float                value;       // smoothed; NAN if none
    float                raw;         // last raw reading; NAN if invalid
    DS18B20Sensor::Error sensorError; // why the last reading failed, if it did
    uint32_t             atMs;
};
```

The callback runs in the task that calls `poll()`. Keep it short — copy the
event into a queue — and do not call `poll()` from it.

With parasite-powered sensors, nothing else may use the bus while
`isConverting()` is true.

## API summary

### TemperatureMonitor

|Method|Description|
|---|---|
|`TemperatureMonitor(OneWire&, DallasTemperature&)`|Bind to a bus|
|`ConfigError setDefaultConfig(const Config&)`|Configuration for new sensors (default: no name, off, no limits); refused if invalid|
|`setPollInterval(uint32_t ms)`|Conversion interval (global; default 10 s)|
|`onEvent(EventCallback, void* ctx)`|Event callback|
|`bool begin(const char* ns, bool identify = true, uint8_t resolution = 12)`|Load, discover, match; `ns == nullptr`: no persistence. Call once, before other tasks use the monitor|
|`void poll(uint32_t nowMs)` / `poll()`|Advance the cycle|
|`Census census()`|What `begin()` found|
|`int8_t findSlot(const DeviceAddress)`|Slot of an address, or -1|
|`bool isAssigned(uint8_t slot)`|Slot bound?|
|`bool getStatus(uint8_t slot, Status&, uint32_t nowMs)` / `getStatus(slot, Status&)`|Everything about a slot|
|`bool getConfig(uint8_t slot, Config&)`|A slot's configuration|
|`Result setConfig(slot, expected, cfg, save = true, ConfigError* detail = nullptr)`|Change a slot|
|`Result forget(slot, expected)`|Release a slot whose sensor is gone|
|`bool hasUnreadableRecord(slot)` / `Result discardRecord(slot)`|Inspect / delete a stored record that could not be read|
|`static char* resultName(Result, char*, size_t)`|`"address_mismatch"`, ...|
|`static packRecord()` / `unpackRecord()`|The stored record format, for tests and tools|

### TemperatureChannel

Usable on its own for a fixed sensor (`assign()`, `beginAt()` / `connect()`,
`update()`), and the type behind every slot: `Config`, `Status`, `State`,
`isAlarm()`, `stateName()`, `copyName()`.

`Status` holds the channel state, the assigned/present/enabled flags, the
name, the family, the full `DS18B20Sensor::Status` and the full
`RangeMonitor::Status` — one consistent copy.

## Thread safety

Lock order: monitor → channel → RangeMonitor; nothing takes them in reverse.
ESP32 uses FreeRTOS mutexes in static buffers (timeout
`TEMPERATURE_MONITOR_MUTEX_TIMEOUT`, 1000 ms); ESP8266 and AVR have no
preemptive threads and the locks are no-ops. Only `begin()` and `poll()` touch
the bus: call them from one task. Status and configuration calls may come from
any task.

## Memory

No dynamic allocation. On AVR a monitor with 4 slots takes about 860 bytes of
RAM (a channel about 160): comfortable on an ATmega2560, too much for a
2 KB part with anything else running — lower
`TEMPERATURE_MONITOR_MAX_CHANNELS` there.

## Installation

### PlatformIO

```ini
lib_deps =
    soosp/TemperatureMonitor
```

RangeMonitor, DS18B20Sensor (with OneWire, DallasTemperature, CheckDS18B20)
and the platform's Preferences library are installed with it.

### Arduino IDE

Search for **TemperatureMonitor** in the Library Manager; RangeMonitor and
DS18B20Sensor are offered as dependencies. The Library Manager cannot express
per-platform dependencies, so install the storage library yourself where
needed: **Preferences** (vshymanskyy) on ESP8266, **EEPROMPreferences** on
AVR.

## Host tests

`Ctrl+Shift+B` in VS Code, or see [test/README.md](test/README.md).

## License

MIT — see [LICENSE](LICENSE).
