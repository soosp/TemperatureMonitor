/**
 * @file TemperatureChannel.h
 * @brief One monitored temperature channel: a DS18B20Sensor, a RangeMonitor,
 *        a name and an enabled flag, bound to one ROM address.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Péter Soós — https://github.com/soosp
 *
 * -----------------------------------------------------------------------------
 * IDENTITY
 * -----------------------------------------------------------------------------
 *
 * A channel is bound to a ROM address by assign(), and everything the
 * application attaches to it — name, limits, enabled flag — lives in the same
 * object as that address. The sensor is read by that address. There is no
 * step at which a reading is matched to a name afterwards, so readings and
 * configuration cannot drift apart.
 *
 * -----------------------------------------------------------------------------
 * STATE
 * -----------------------------------------------------------------------------
 *
 * State extends RangeMonitor::State (same values 0..8) with two channel-level
 * states:
 *
 *   MISSING   assigned, but the sensor was not on the bus at start-up (or has
 *             not answered since). Counts as an alarm.
 *   OFF       monitoring switched off: the channel is still read and its
 *             RangeMonitor still runs, so values stay available, but the
 *             state never alarms. Takes precedence over MISSING. Also named
 *             DISABLED, except on ESP32, whose core defines DISABLED as a
 *             macro (an interrupt mode).
 *
 * The state is recomputed by update() only, so every transition — including
 * the ones caused by setConfig(), assign() and unassign() — is reported by
 * update(), in order. An unassigned channel's state is UNKNOWN.
 *
 * -----------------------------------------------------------------------------
 * SENSOR RESETS
 * -----------------------------------------------------------------------------
 *
 * A sensor that was unplugged or lost power comes back with its EEPROM
 * defaults (usually 12 bits) and the "85 °C" power-on value in its
 * scratchpad. update() therefore writes the channel's resolution back
 * whenever a reading reports POWER_ON_RESET or the sensor answers again after
 * DISCONNECTED, so its conversion time keeps matching the owner's.
 *
 * -----------------------------------------------------------------------------
 * THREAD SAFETY AND MEMORY
 * -----------------------------------------------------------------------------
 *
 * Every public method takes the channel's lock (static FreeRTOS mutex on
 * ESP32, no-op elsewhere). Lock order: TemperatureMonitor → channel →
 * RangeMonitor; nothing takes them in reverse. Only update(), beginAt() and
 * connect() touch the bus; the owner calls them from one task.
 *
 * No dynamic allocation. Non-copyable (owns a mutex on ESP32).
 */

#pragma once

#include <stdint.h>
#include <string.h>
#include <math.h>

#ifdef ARDUINO
    #include <Arduino.h>
#endif

#include <RangeMonitor.h>   // >= 0.2.0
#include <DS18B20Sensor.h>  // >= 0.1.0
#include "detail/tm_platform.h"

/// Maximum channel name length in bytes (UTF-8), excluding the terminator.
#ifndef TEMPERATURE_MONITOR_NAME_LEN
    #define TEMPERATURE_MONITOR_NAME_LEN 32
#endif

/**
 * @brief One monitored temperature channel.
 *
 * Usually owned by a TemperatureMonitor, which handles discovery,
 * persistence and the measuring cycle. Usable on its own for a fixed sensor.
 */
class TemperatureChannel {
public:
    // =========================================================================
    // TYPES
    // =========================================================================

    /**
     * @brief Channel state. 0..8 are RangeMonitor::State values; stable.
     */
#pragma push_macro("DISABLED")   // a macro on ESP32 (esp32-hal-gpio.h)
#undef DISABLED
    enum class State : uint8_t {
        UNKNOWN       = 0,  ///< Not assigned, or no valid sample yet
        OK            = 1,
        PENDING_HIGH  = 2,
        PENDING_LOW   = 3,
        ALARM_HIGH    = 4,
        ALARM_LOW     = 5,
        CRITICAL_HIGH = 6,
        CRITICAL_LOW  = 7,
        FAULT         = 8,  ///< Sensor errors for at least the fault delay
        MISSING       = 9,  ///< Assigned, sensor not on the bus
        OFF           = 10, ///< Monitoring off; never alarms
        DISABLED      = OFF ///< Same as OFF; on ESP32 write OFF (see above)
    };
#pragma pop_macro("DISABLED")

    /// Number of State values.
    static constexpr uint8_t STATE_COUNT = 11;

    /// Channel name: UTF-8 bytes, excluding the null terminator.
    static constexpr size_t NAME_LEN  = TEMPERATURE_MONITOR_NAME_LEN;
    /// Channel name buffer size, including the null terminator.
    static constexpr size_t NAME_SIZE = NAME_LEN + 1;

    /// Longest name returned by stateName(), excluding the null terminator.
    static constexpr size_t STATE_NAME_LEN  = 13;  // "critical_high"
    /// Buffer size for stateName(), including the null terminator.
    static constexpr size_t STATE_NAME_SIZE = STATE_NAME_LEN + 1;

    /**
     * @brief What the application configures for a channel.
     */
    struct Config {
        char                 name[NAME_SIZE]; ///< UTF-8; may be empty
        bool                 enabled;         ///< Monitoring on (alarms possible)
        RangeMonitor::Config range;           ///< Limits, smoothing, delays

        /// Empty name, monitoring off, no limits.
        Config() : enabled(false), range() { name[0] = '\0'; }
    };

    /**
     * @brief Everything about the channel, copied under one lock.
     */
    struct Status {
        State                 state;      ///< Channel state
        bool                  assigned;   ///< Bound to an address
        bool                  present;    ///< Sensor answers on the bus
        bool                  enabled;    ///< Monitoring on
        char                  name[NAME_SIZE];
        DS18B20Sensor::Family family;     ///< As identified (kept across reconnects)
        DS18B20Sensor::Status sensor;     ///< Hardware layer (address, last reading, errors)
        RangeMonitor::Status  monitor;    ///< Algorithm layer (smoothed value, state)
    };

    /**
     * @brief State change produced by one update().
     */
    struct Transition {
        State                from;
        State                to;
        DeviceAddress        address;      ///< Bound address; all zero if unassigned
        float                value;        ///< Smoothed value after the update; NAN if none
        float                raw;          ///< This update's raw reading; NAN if invalid or not read
        DS18B20Sensor::Error sensorError;  ///< Result of this update's reading
        bool changed() const { return from != to; }
    };

    // =========================================================================
    // LIFECYCLE
    // =========================================================================

    /// Unassigned channel.
    TemperatureChannel()
        : _family(CheckDS18B20::FAMILY_UNKNOWN)
        , _resolution(12)
        , _assigned(false)
        , _present(false)
        , _enabled(false)
        , _state(State::UNKNOWN)
    {
        memset(_address, 0, sizeof(DeviceAddress));
        _name[0] = '\0';
    }

    TemperatureChannel(const TemperatureChannel&)            = delete;
    TemperatureChannel& operator=(const TemperatureChannel&) = delete;

    /**
     * @brief Bind the channel to an address and a configuration.
     *
     * No bus access: the sensor is marked not present until beginAt() or
     * connect() succeeds. The RangeMonitor starts afresh (UNKNOWN). The name
     * is truncated to NAME_LEN bytes at a UTF-8 character boundary.
     *
     * @param address     ROM address.
     * @param cfg         Configuration; cfg.range must validate.
     * @param family      Family known from an earlier identification, if any.
     * @param resolution  Resolution in bits (9..12) the sensor will be set to
     *                    whenever it is (re)connected.
     * @return            ConfigError::NONE, the validation error, or
     *                    LOCK_TIMEOUT.
     */
    RangeMonitor::ConfigError assign(const DeviceAddress address, const Config& cfg,
                                     DS18B20Sensor::Family family = CheckDS18B20::FAMILY_UNKNOWN,
                                     uint8_t resolution = 12) {
        RangeMonitor::ConfigError e = cfg.range.validate();
        if (e != RangeMonitor::ConfigError::NONE) return e;
        tm_detail::Guard g(_lock);
        if (!g.ok()) return RangeMonitor::ConfigError::LOCK_TIMEOUT;
        memcpy(_address, address, sizeof(DeviceAddress));
        copyName(_name, cfg.name);
        _enabled    = cfg.enabled;
        _family     = family;
        _resolution = _clampResolution(resolution);
        _assigned   = true;
        _present    = false;
        _sensor     = DS18B20Sensor();
        _monitor.reconfigure(cfg.range);
        _monitor.reset();
        return RangeMonitor::ConfigError::NONE;
    }

    /**
     * @brief Release the binding. The channel becomes unassigned.
     *
     * The state (and its transition to UNKNOWN) is reported by the next
     * update(), so an alarm the application raised for this channel is
     * cleared through the normal path.
     *
     * @return false on lock timeout.
     */
    bool unassign() {
        tm_detail::Guard g(_lock);
        if (!g.ok()) return false;
        memset(_address, 0, sizeof(DeviceAddress));
        _name[0]  = '\0';
        _enabled  = false;
        _family   = CheckDS18B20::FAMILY_UNKNOWN;
        _assigned = false;
        _present  = false;
        _sensor   = DS18B20Sensor();
        _monitor.reset();
        return true;
    }

    // =========================================================================
    // BUS
    // =========================================================================

    /**
     * @brief Start the sensor found at bus @p index (discovery).
     *
     * The device at @p index must have the assigned address. With
     * @p identify, CheckDS18B20 runs and the family is updated.
     *
     * @return true if the sensor is now present.
     */
    bool beginAt(DallasTemperature& dallas, OneWire& oneWire, uint8_t index,
                 bool identify, uint8_t resolution) {
        tm_detail::Guard g(_lock);
        if (!g.ok() || !_assigned) return false;
        DeviceAddress found;
        if (!dallas.getAddress(found, index) ||
            memcmp(found, _address, sizeof(DeviceAddress)) != 0) {
            return false;
        }
        _resolution = _clampResolution(resolution);
        _present    = _sensor.begin(dallas, oneWire, index, identify, _resolution);
        if (_present && identify) _family = _sensor.family();
        return _present;
    }

    /**
     * @brief Start the sensor by its assigned address (no identification).
     * @return true if the sensor is now present.
     */
    bool connect(DallasTemperature& dallas, uint8_t resolution) {
        tm_detail::Guard g(_lock);
        if (!g.ok() || !_assigned) return false;
        _resolution = _clampResolution(resolution);
        _present    = _sensor.begin(dallas, _address, _resolution);
        return _present;
    }

    /**
     * @brief Read the sensor after a broadcast conversion and update the state.
     *
     * A sensor that is not present is first looked for by address (one
     * scratchpad read); if it answers, it is reconnected and read in the
     * same call. A missing sensor does not feed the RangeMonitor. A sensor
     * that reset (see the file header) gets its resolution back.
     *
     * The returned Transition also carries this update's reading, captured
     * under the same lock, so an event built from it is always complete.
     *
     * @param dallas  DallasTemperature on the bus.
     * @param nowMs   Time of the reading.
     * @return        The channel state transition (from == to if unchanged,
     *                or on lock timeout).
     */
    Transition update(DallasTemperature& dallas, uint32_t nowMs) {
        Transition t;
        t.from = t.to  = State::UNKNOWN;
        t.value        = NAN;
        t.raw          = NAN;
        t.sensorError  = DS18B20Sensor::Error::NOT_INITIALISED;
        memset(t.address, 0, sizeof(DeviceAddress));
        tm_detail::Guard g(_lock);
        if (!g.ok()) return t;
        t.from = _state;
        memcpy(t.address, _address, sizeof(DeviceAddress));
        if (_assigned) {
            if (!_present && dallas.isConnected(_address)) {
                _present = _sensor.begin(dallas, _address, _resolution);
            }
            if (_present) {
                DS18B20Sensor::Error before = _sensor.lastError();
                float v = _sensor.read(dallas, nowMs);
                DS18B20Sensor::Error now = _sensor.lastError();
                bool reset = (now == DS18B20Sensor::Error::POWER_ON_RESET) ||
                             (before == DS18B20Sensor::Error::DISCONNECTED &&
                              now != DS18B20Sensor::Error::DISCONNECTED);
                if (reset) {
                    // Back to our resolution; the sensor's error accounting
                    // (this reading's outcome, the counters) stays as it is.
                    dallas.setResolution(_address, _resolution);
                }
                _monitor.update(v, nowMs);
                t.raw         = v;
                t.sensorError = _sensor.lastError();
            }
            t.value = _monitor.value();
        }
        _state = _computeState();
        t.to   = _state;
        return t;
    }

    // =========================================================================
    // CONFIGURATION
    // =========================================================================

    /**
     * @brief Replace name, enabled flag and limits.
     *
     * The RangeMonitor keeps its filter and state (see
     * RangeMonitor::reconfigure()); the channel state follows at the next
     * update(), which reports the transition.
     *
     * @return ConfigError::NONE, the validation error, or LOCK_TIMEOUT.
     */
    RangeMonitor::ConfigError setConfig(const Config& cfg) {
        RangeMonitor::ConfigError e = cfg.range.validate();
        if (e != RangeMonitor::ConfigError::NONE) return e;
        tm_detail::Guard g(_lock);
        if (!g.ok()) return RangeMonitor::ConfigError::LOCK_TIMEOUT;
        e = _monitor.reconfigure(cfg.range);
        if (e != RangeMonitor::ConfigError::NONE) return e;
        copyName(_name, cfg.name);
        _enabled = cfg.enabled;
        return RangeMonitor::ConfigError::NONE;
    }

    /// Copy the configuration. @return false on lock timeout.
    bool getConfig(Config& out) const {
        tm_detail::Guard g(_lock);
        if (!g.ok()) return false;
        copyName(out.name, _name);
        out.enabled = _enabled;
        return _monitor.getConfig(out.range);
    }

    // =========================================================================
    // STATE
    // =========================================================================

    /**
     * @brief Everything about the channel, under one lock.
     * @param nowMs  Current time (for RangeMonitor's settled flag).
     * @return       false on lock timeout (@p out unchanged).
     */
    bool getStatus(Status& out, uint32_t nowMs) const {
        tm_detail::Guard g(_lock);
        if (!g.ok()) return false;
        out.state    = _state;
        out.assigned = _assigned;
        out.present  = _present;
        out.enabled  = _enabled;
        copyName(out.name, _name);
        out.family   = _family;
        out.sensor   = _sensor.getStatus();
        memcpy(out.sensor.address, _address, sizeof(DeviceAddress));
        out.monitor  = _monitor.getStatus(nowMs);
        return true;
    }

#ifdef ARDUINO
    /// Status at millis(). @see getStatus(Status&, uint32_t)
    bool getStatus(Status& out) const { return getStatus(out, (uint32_t)millis()); }
#endif

    /// Current state; UNKNOWN on lock timeout.
    State state() const {
        tm_detail::Guard g(_lock);
        return g.ok() ? _state : State::UNKNOWN;
    }

    /// True if bound to an address.
    bool isAssigned() const {
        tm_detail::Guard g(_lock);
        return g.ok() && _assigned;
    }

    /// True if bound to @p address.
    bool hasAddress(const DeviceAddress address) const {
        tm_detail::Guard g(_lock);
        return g.ok() && _assigned &&
               memcmp(_address, address, sizeof(DeviceAddress)) == 0;
    }

    /// Copy the bound address (all zero if unassigned). @return false on lock timeout.
    bool getAddress(DeviceAddress out) const {
        tm_detail::Guard g(_lock);
        if (!g.ok()) return false;
        memcpy(out, _address, sizeof(DeviceAddress));
        return true;
    }

    /// Identified family (FAMILY_UNKNOWN if never identified).
    DS18B20Sensor::Family family() const {
        tm_detail::Guard g(_lock);
        return g.ok() ? _family : CheckDS18B20::FAMILY_UNKNOWN;
    }

    /// Resolution in bits the channel asks for.
    uint8_t resolution() const {
        tm_detail::Guard g(_lock);
        return g.ok() ? _resolution : 12;
    }

    // =========================================================================
    // STATIC HELPERS
    // =========================================================================

    /// True for the alarm states: ALARM_x, CRITICAL_x, FAULT, MISSING.
    static bool isAlarm(State s) {
        return s == State::MISSING ||
               RangeMonitor::isAlarm(static_cast<RangeMonitor::State>(s));
    }

    /**
     * @brief Lower-case state name into a caller buffer.
     *
     * The RangeMonitor names, plus "missing" and "disabled". In flash on AVR.
     *
     * @return @p buf, or nullptr if @p buf is null or @p len is 0.
     */
    static char* stateName(State s, char* buf, size_t len) {
        if (buf == nullptr || len == 0) return nullptr;
        if (s == State::MISSING)  return _copyP(_missingName(), buf, len);
        if (s == State::OFF) return _copyP(_disabledName(), buf, len);
        return RangeMonitor::stateName(static_cast<RangeMonitor::State>(s), buf, len);
    }

    /**
     * @brief Copy a name, truncating to NAME_LEN bytes at a UTF-8 boundary.
     *
     * A multi-byte character that does not fit is dropped whole rather than
     * cut, so the result is always valid UTF-8 if the input was.
     *
     * @param dst  Destination, NAME_SIZE bytes.
     * @param src  Source; nullptr is treated as empty.
     */
    static void copyName(char* dst, const char* src) {
        if (src == nullptr) { dst[0] = '\0'; return; }
        size_t n = strlen(src);
        if (n > NAME_LEN) {
            n = NAME_LEN;
            // Step back over continuation bytes (10xxxxxx) to the start of the
            // character that would be cut, and drop it.
            while (n > 0 && ((uint8_t)src[n] & 0xC0) == 0x80) n--;
        }
        memcpy(dst, src, n);
        dst[n] = '\0';
    }

private:
    static uint8_t _clampResolution(uint8_t r) {
        return r < 9 ? 9 : (r > 12 ? 12 : r);
    }

    State _computeState() const {
        if (!_assigned) return State::UNKNOWN;
        if (!_enabled)  return State::OFF;
        if (!_present)  return State::MISSING;
        return static_cast<State>(_monitor.state());
    }

    static char* _copyP(PGM_P src, char* buf, size_t len) {
        size_t n = strlen_P(src);
        if (n > len - 1) n = len - 1;
        memcpy_P(buf, src, n);
        buf[n] = '\0';
        return buf;
    }

    static PGM_P _missingName()  { static const char s[] PROGMEM = "missing";  return s; }
    static PGM_P _disabledName() { static const char s[] PROGMEM = "disabled"; return s; }

    tm_detail::Lock       _lock;
    DS18B20Sensor         _sensor;
    RangeMonitor          _monitor;
    DeviceAddress         _address;
    char                  _name[NAME_SIZE];
    DS18B20Sensor::Family _family;
    uint8_t               _resolution;
    bool                  _assigned;
    bool                  _present;
    bool                  _enabled;
    State                 _state;
};
