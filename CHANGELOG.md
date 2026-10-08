# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0] - 2026-10-08

### Added

- `TemperatureChannel`: one monitored channel — DS18B20Sensor, RangeMonitor,
  name and enabled flag bound to a ROM address; `MISSING` and `DISABLED`
  states on top of the RangeMonitor states; UTF-8-safe name truncation
- `TemperatureMonitor`: the channels of one bus
  - discovery at start-up with address-based slot matching
  - one versioned record per slot (address and configuration together),
    on the Preferences API of each platform
  - automatic reconnection of missing sensors
  - non-blocking `poll()` with one broadcast conversion per cycle
  - state-change events delivered after all locks are released
  - `setConfig()` / `forget()` guarded by the expected address; `forget()`
    clears a channel's alarm through the normal event path
  - resolution written back to sensors that reconnect or reset
  - `hasUnreadableRecord()` / `discardRecord()` for records that cannot be read
  - `Census` of what `begin()` found
- Host test suite in `test/` with stubs of OneWire, DallasTemperature,
  CheckDS18B20 and Preferences, runnable from VS Code with Ctrl+Shift+B
- Example: `Monitor`

[Unreleased]: https://github.com/soosp/TemperatureMonitor/compare/0.1.0...HEAD
[0.1.0]: https://github.com/soosp/TemperatureMonitor/releases/tag/0.1.0
