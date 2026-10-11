# Host tests

Without `ARDUINO` defined, the library compiles as plain C++ with no-op locks.
The tests replace the bus and the flash with the stubs in `stubs/`: no board,
no sensors, no real time.

## Running

Press `Ctrl+Shift+B` in VS Code — the default build task builds and runs every
test and reports a summary.

From a shell:

```sh
bash test/run_tests.sh   # Linux / macOS
test\run_tests.bat       # Windows
```

Both scripts exit non-zero if any test fails. They need nothing but a C++17
compiler (`g++` by default; override with `CXX`).

On Windows the VSCode task prepends `C:\msys64\ucrt64\bin` to `PATH` so the
MSYS2 toolchain is found without changing the system PATH. If MSYS2 lives
elsewhere, edit that entry in `.vscode/tasks.json`.

## What is covered

|File|Area|
|---|---|
|`test_discovery.cpp`|First start, restart in a different bus order, sensor replaced, slots full, family kept, no needless writes, unreadable records, storage failure|
|`test_cycle.cpp`|Non-blocking timing, resolution, OFF/MISSING, reconnection, delay to ALARM, FAULT with its reason, callback without locks|
|`test_config.cpp`|Address guard (including the stale-form case), validation, storage failure, `forget()`, UTF-8 names, record format|
|`test_recovery.cpp`|Sensors coming and going at runtime (resolution restored), `forget()` after a runtime loss and its clearing event, default-config validation, discarding unreadable records|

`test_harness.h` provides `Rig`: a simulated bus with a monitor on it, a
clock, and the list of events it produced.

## stubs/ and third_party/

|File|Stands in for|
|---|---|
|`stubs/OneWire.h`|OneWire: the class and the real CRC-8|
|`stubs/DallasTemperature.h`|DallasTemperature 4.x: a scriptable device table (readings, presence, CRC failures, resolution)|
|`stubs/CheckDS18B20.h`|CheckDS18B20: the family enum and a scriptable identifier|
|`stubs/Preferences.h`|Preferences: an in-memory store the tests can inspect, corrupt and make fail|
|`third_party/RangeMonitor.h`|Copy of RangeMonitor 0.2.0|
|`third_party/DS18B20Sensor.h`|Copy of DS18B20Sensor 0.1.0|
|`third_party/ExponentialAverage.h`|Copy of RunningStatistics 1.1.1|

The copies let the suite build without a library manager; device builds use
the installed libraries. Update them when a dependency's version changes.
