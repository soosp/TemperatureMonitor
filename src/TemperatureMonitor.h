/**
 * @file TemperatureMonitor.h
 * @brief A set of temperature channels on one 1-Wire bus: discovery,
 *        address-based slot matching, persistence, a non-blocking measuring
 *        cycle and state-change events.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Péter Soós — https://github.com/soosp
 *
 * -----------------------------------------------------------------------------
 * SLOTS
 * -----------------------------------------------------------------------------
 *
 * The monitor has MAX_CHANNELS slots. A slot is bound to a sensor's ROM
 * address, and the binding is stored together with the channel's
 * configuration in one record, so the two can never be saved apart.
 *
 * begin() loads the stored records, searches the bus once, and matches every
 * sensor found to the slot holding its address. A sensor without a slot gets
 * the first free one, with the default configuration (monitoring off). A
 * stored slot whose sensor is not on the bus stays bound and reports MISSING;
 * it is reconnected automatically if the sensor answers later, and can be
 * released with forget() once it is known to be gone.
 *
 * Slot numbers are handles, not identities. Operations that change a slot
 * take the address the caller expects to find there and refuse with
 * ADDRESS_MISMATCH otherwise — a web form rendered before a sensor swap
 * cannot write its old settings onto the new sensor.
 *
 * -----------------------------------------------------------------------------
 * MEASURING CYCLE
 * -----------------------------------------------------------------------------
 *
 * poll() never blocks. It starts a broadcast conversion when the poll
 * interval has elapsed, and reads every channel once the conversion time
 * (from the highest resolution in use) has passed. Call it often — every few
 * tens of milliseconds — from the task that owns the bus.
 *
 * With parasite-powered sensors, nothing else may use the bus during the
 * conversion; poll() itself does not.
 *
 * -----------------------------------------------------------------------------
 * EVENTS
 * -----------------------------------------------------------------------------
 *
 * Every channel state change is delivered to the event callback, after all
 * locks are released, in the context of the task calling poll(). The callback
 * should only hand the event on (e.g. to a queue); the application decides
 * which transitions matter (TemperatureChannel::isAlarm()).
 *
 * At most one event per channel per cycle. The first cycle after begin()
 * reports every channel's initial state, so the application learns about
 * missing sensors and alarms present at start-up.
 *
 * -----------------------------------------------------------------------------
 * PERSISTENCE
 * -----------------------------------------------------------------------------
 *
 * One record per slot, key "s<slot>" in the namespace passed to begin(),
 * written with the Preferences API (ESP32 native, vshymanskyy/Preferences on
 * ESP8266, soosp/EEPROMPreferences on AVR). Fields are serialised one by one
 * (no struct dump, so padding and layout changes cannot corrupt it) behind a
 * version byte; at most RECORD_SIZE bytes. A record with an unknown version
 * or a bad length is ignored and counted in Census::invalidRecords.
 *
 * -----------------------------------------------------------------------------
 * THREAD SAFETY AND MEMORY
 * -----------------------------------------------------------------------------
 *
 * Lock order: monitor → channel → RangeMonitor. Only begin() and poll() touch
 * the bus. Status and configuration calls may come from any task.
 *
 * No dynamic allocation: channels and the pending-event buffer are members.
 */

#pragma once

#include "TemperatureChannel.h"

/// Number of slots. AVR defaults to 4 to fit 2 KB parts; override in build flags.
#ifndef TEMPERATURE_MONITOR_MAX_CHANNELS
    #if defined(ARDUINO_ARCH_AVR)
        #define TEMPERATURE_MONITOR_MAX_CHANNELS 4
    #else
        #define TEMPERATURE_MONITOR_MAX_CHANNELS 16
    #endif
#endif

/**
 * @brief A set of temperature channels on one 1-Wire bus.
 *
 * @par Typical use
 * @code
 * OneWire            oneWire(PIN);
 * DallasTemperature  dallas(&oneWire);
 * TemperatureMonitor monitor(oneWire, dallas);
 *
 * void onEvent(const TemperatureMonitor::Event& e, void*) {
 *     xQueueSend(events, &e, 0);                  // hand on, return quickly
 * }
 *
 * monitor.onEvent(onEvent, nullptr);
 * monitor.setPollInterval(10000);
 * monitor.begin("tempmon");
 *
 * for (;;) { monitor.poll(); vTaskDelay(pdMS_TO_TICKS(50)); }
 * @endcode
 */
class TemperatureMonitor {
public:
    // =========================================================================
    // TYPES
    // =========================================================================

    typedef TemperatureChannel::State  State;   ///< Channel state
    typedef TemperatureChannel::Config Config;  ///< Channel configuration
    typedef TemperatureChannel::Status Status;  ///< Channel status

    /**
     * @brief Result of the slot-changing operations. Stable values.
     */
    enum class Result : uint8_t {
        OK               = 0,
        INVALID_SLOT     = 1, ///< Slot number out of range
        EMPTY_SLOT       = 2, ///< Slot not bound to a sensor
        ADDRESS_MISMATCH = 3, ///< Slot is bound to a different sensor
        INVALID_CONFIG   = 4, ///< Config::range failed validation
        IN_USE           = 5, ///< forget(): the sensor is still on the bus
        STORAGE_ERROR    = 6, ///< Applied in RAM, but could not be saved
        LOCK_TIMEOUT     = 7,
        NOT_STARTED      = 8  ///< begin() has not run
    };

    /**
     * @brief One channel state change.
     */
    struct Event {
        uint8_t              slot;
        DeviceAddress        address;     ///< All zero if the slot was released (forget())
        State                from;
        State                to;
        float                value;       ///< Smoothed value; NAN if none
        float                raw;         ///< Last raw reading; NAN if invalid
        DS18B20Sensor::Error sensorError; ///< Why the last reading failed, if it did
        uint32_t             atMs;
    };

    /// Event callback. Runs with no lock held; @p ctx is the pointer given to onEvent().
    typedef void (*EventCallback)(const Event& event, void* ctx);

    /**
     * @brief What begin() found.
     */
    struct Census {
        uint8_t found;          ///< Sensors on the bus
        uint8_t matched;        ///< ... that had a stored slot
        uint8_t added;          ///< ... that got a new slot
        uint8_t unassigned;     ///< ... that found no free slot
        uint8_t missing;        ///< Stored slots whose sensor is not on the bus
        uint8_t invalidRecords; ///< Stored records that could not be read
    };

    // =========================================================================
    // CONSTANTS
    // =========================================================================

    static constexpr uint8_t  MAX_CHANNELS   = TEMPERATURE_MONITOR_MAX_CHANNELS;
    static constexpr uint8_t  RECORD_VERSION = 1;
    /// Largest stored record: version, address, family, flags, name, limits.
    static constexpr size_t   RECORD_SIZE    = 1 + 8 + 1 + 1 + 1 + TemperatureChannel::NAME_LEN
                                             + 5 * 4 + 3 * 4;
    static constexpr uint32_t DEFAULT_POLL_INTERVAL_MS = 10000;

    // =========================================================================
    // LIFECYCLE
    // =========================================================================

    /**
     * @brief Bind to a bus. Nothing happens until begin().
     */
    TemperatureMonitor(OneWire& oneWire, DallasTemperature& dallas)
        : _oneWire(oneWire)
        , _dallas(dallas)
        , _callback(nullptr)
        , _callbackCtx(nullptr)
        , _pollIntervalMs(DEFAULT_POLL_INTERVAL_MS)
        , _conversionMs(750)
        , _lastRequestMs(0)
        , _resolution(12)
        , _identify(true)
        , _started(false)
        , _converting(false)
        , _firstCycle(true)
    {
        _ns[0] = '\0';
        memset(&_census, 0, sizeof(_census));
        memset(_unreadable, 0, sizeof(_unreadable));
    }

    TemperatureMonitor(const TemperatureMonitor&)            = delete;
    TemperatureMonitor& operator=(const TemperatureMonitor&) = delete;

    /**
     * @brief Configuration given to newly found sensors.
     *
     * Default: no name, monitoring off, no limits.
     *
     * @return ConfigError::NONE, or the validation error (default unchanged).
     */
    RangeMonitor::ConfigError setDefaultConfig(const Config& cfg) {
        RangeMonitor::ConfigError e = cfg.range.validate();
        if (e == RangeMonitor::ConfigError::NONE) _default = cfg;
        return e;
    }

    /// Interval between conversions (global: one broadcast serves all sensors).
    void setPollInterval(uint32_t ms) { _pollIntervalMs = ms; }

    /// Register the event callback (nullptr to remove).
    void onEvent(EventCallback cb, void* ctx) { _callback = cb; _callbackCtx = ctx; }

    /**
     * @brief Load stored slots, search the bus once, match and start sensors.
     *
     * Call once at start-up, before other tasks use the monitor: it holds the
     * monitor's lock throughout, and identification takes a few hundred
     * milliseconds per sensor.
     *
     * @param ns          Preferences namespace (max 15 chars); nullptr: no persistence.
     * @param identify    Identify each found sensor's family (CheckDS18B20).
     * @param resolution  Resolution for every sensor, 9..12.
     * @return            false on lock timeout or if storage could not be opened
     *                    (the bus is still searched; nothing is stored).
     */
    bool begin(const char* ns, bool identify = true, uint8_t resolution = 12) {
        tm_detail::Guard g(_lock);
        if (!g.ok()) return false;

        _identify   = identify;
        _resolution = (resolution < 9) ? 9 : (resolution > 12 ? 12 : resolution);
        _conversionMs = DallasTemperature::millisToWaitForConversion(_resolution);
        memset(&_census, 0, sizeof(_census));
        _ns[0] = '\0';
        if (ns != nullptr) {
            strncpy(_ns, ns, sizeof(_ns) - 1);
            _ns[sizeof(_ns) - 1] = '\0';
        }

        for (uint8_t s = 0; s < MAX_CHANNELS; s++) _channels[s].unassign();

        bool storageOk = _loadAll();

        _dallas.begin();
        _dallas.setWaitForConversion(false);

        uint8_t found = _dallas.getDS18Count();
        _census.found = found;
        for (uint8_t i = 0; i < found; i++) {
            DeviceAddress addr;
            if (!_dallas.getAddress(addr, i)) continue;

            int8_t slot = _findSlot(addr);
            bool   isNew = (slot < 0);
            if (isNew) {
                slot = _freeSlot();
                if (slot < 0 ||
                    _channels[slot].assign(addr, _default, CheckDS18B20::FAMILY_UNKNOWN,
                                           _resolution) != RangeMonitor::ConfigError::NONE) {
                    _census.unassigned++;
                    continue;
                }
            }
            DS18B20Sensor::Family before = _channels[slot].family();
            _channels[slot].beginAt(_dallas, _oneWire, i, identify, _resolution);
            if (isNew) {
                _census.added++;
                storageOk &= _save((uint8_t)slot);
            } else {
                _census.matched++;
                if (_channels[slot].family() != before) storageOk &= _save((uint8_t)slot);
            }
        }

        for (uint8_t s = 0; s < MAX_CHANNELS; s++) {
            Status st;
            if (_channels[s].getStatus(st, 0) && st.assigned && !st.present) _census.missing++;
        }

        _started    = true;
        _converting = false;
        _firstCycle = true;
        return storageOk;
    }

    // =========================================================================
    // MEASURING CYCLE
    // =========================================================================

    /**
     * @brief Advance the measuring cycle; never blocks on a conversion.
     *
     * @param nowMs  Current time in milliseconds.
     */
    void poll(uint32_t nowMs) {
        uint8_t n = 0;
        {
            tm_detail::Guard g(_lock);
            if (!g.ok() || !_started) return;

            if (!_converting) {
                if (_firstCycle || nowMs - _lastRequestMs >= _pollIntervalMs) {
                    _dallas.requestTemperatures();
                    _lastRequestMs = nowMs;
                    _converting    = true;
                    _firstCycle    = false;
                }
                return;
            }
            if (nowMs - _lastRequestMs < _conversionMs) return;

            _converting = false;
            for (uint8_t s = 0; s < MAX_CHANNELS; s++) {
                // The transition carries its reading, captured under the
                // channel's lock: no second lock that could time out and
                // lose an event whose state change is already committed.
                TemperatureChannel::Transition t = _channels[s].update(_dallas, nowMs);
                if (!t.changed()) continue;
                Event& e = _pending[n++];
                e.slot        = s;
                memcpy(e.address, t.address, sizeof(DeviceAddress));
                e.from        = t.from;
                e.to          = t.to;
                e.value       = t.value;
                e.raw         = t.raw;
                e.sensorError = t.sensorError;
                e.atMs        = nowMs;
            }
        }
        // Locks released: deliver.
        if (_callback) {
            for (uint8_t i = 0; i < n; i++) _callback(_pending[i], _callbackCtx);
        }
    }

#ifdef ARDUINO
    /// poll() at millis().
    void poll() { poll((uint32_t)millis()); }
#endif

    /// True while a conversion is in progress (the bus must stay quiet if parasite-powered).
    bool isConverting() const {
        tm_detail::Guard g(_lock);
        return g.ok() && _converting;
    }

    // =========================================================================
    // QUERIES
    // =========================================================================

    /// What begin() found.
    Census census() const {
        tm_detail::Guard g(_lock);
        Census c;
        if (g.ok()) c = _census;
        else memset(&c, 0, sizeof(c));
        return c;
    }

    /// Slot bound to @p address, or -1.
    int8_t findSlot(const DeviceAddress address) const {
        tm_detail::Guard g(_lock);
        return g.ok() ? _findSlot(address) : -1;
    }

    /// True if @p slot is bound to a sensor.
    bool isAssigned(uint8_t slot) const {
        return slot < MAX_CHANNELS && _channels[slot].isAssigned();
    }

    /**
     * @brief Status of one slot.
     * @return false for an invalid slot or on lock timeout.
     */
    bool getStatus(uint8_t slot, Status& out, uint32_t nowMs) const {
        if (slot >= MAX_CHANNELS) return false;
        return _channels[slot].getStatus(out, nowMs);
    }

#ifdef ARDUINO
    /// Status at millis(). @see getStatus(uint8_t, Status&, uint32_t)
    bool getStatus(uint8_t slot, Status& out) const {
        return getStatus(slot, out, (uint32_t)millis());
    }
#endif

    /**
     * @brief Configuration of one slot.
     * @return false for an invalid or empty slot, or on lock timeout.
     */
    bool getConfig(uint8_t slot, Config& out) const {
        if (slot >= MAX_CHANNELS || !_channels[slot].isAssigned()) return false;
        return _channels[slot].getConfig(out);
    }

    // =========================================================================
    // CHANGES
    // =========================================================================

    /**
     * @brief Replace a slot's configuration, if it still holds @p expected.
     *
     * Applied at once in RAM; the state follows at the next cycle and is
     * reported as an event.
     *
     * @param slot      Slot number.
     * @param expected  The address the caller believes is in the slot.
     * @param cfg       New configuration.
     * @param save      Also store it.
     * @param detail    If not null, receives the RangeMonitor validation error.
     * @return          OK, or why nothing (or only the RAM copy) changed.
     */
    Result setConfig(uint8_t slot, const DeviceAddress expected, const Config& cfg,
                     bool save = true, RangeMonitor::ConfigError* detail = nullptr) {
        if (detail) *detail = RangeMonitor::ConfigError::NONE;
        tm_detail::Guard g(_lock);
        if (!g.ok())                  return Result::LOCK_TIMEOUT;
        Result r = _checkSlot(slot, expected);
        if (r != Result::OK)          return r;
        RangeMonitor::ConfigError e = _channels[slot].setConfig(cfg);
        if (detail) *detail = e;
        if (e == RangeMonitor::ConfigError::LOCK_TIMEOUT) return Result::LOCK_TIMEOUT;
        if (e != RangeMonitor::ConfigError::NONE)         return Result::INVALID_CONFIG;
        if (save && !_save(slot))     return Result::STORAGE_ERROR;
        return Result::OK;
    }

    /**
     * @brief Release a slot whose sensor is gone, if it still holds @p expected.
     *
     * Allowed when the sensor was missing at start-up or its last reading
     * found it disconnected. Refused with IN_USE while it answers on the bus:
     * it would only be found again, as a new sensor, at the next start.
     *
     * The channel's transition to UNKNOWN is reported at the next cycle, so
     * an alarm raised for it is cleared through the normal event path.
     */
    Result forget(uint8_t slot, const DeviceAddress expected) {
        tm_detail::Guard g(_lock);
        if (!g.ok())         return Result::LOCK_TIMEOUT;
        Result r = _checkSlot(slot, expected);
        if (r != Result::OK) return r;
        Status st;
        if (!_channels[slot].getStatus(st, 0)) return Result::LOCK_TIMEOUT;
        if (st.present && st.sensor.lastError != DS18B20Sensor::Error::DISCONNECTED)
            return Result::IN_USE;
        _channels[slot].unassign();
        return _erase(slot) ? Result::OK : Result::STORAGE_ERROR;
    }

    /**
     * @brief True if @p slot holds a stored record that could not be read.
     *
     * Such a slot is not reused for new sensors, so that nothing overwrites a
     * record a newer firmware might still understand. discardRecord() frees it.
     */
    bool hasUnreadableRecord(uint8_t slot) const {
        tm_detail::Guard g(_lock);
        return g.ok() && slot < MAX_CHANNELS && _unreadable[slot];
    }

    /**
     * @brief Delete the unreadable record in @p slot and free the slot.
     * @return OK, INVALID_SLOT, EMPTY_SLOT (no unreadable record there),
     *         STORAGE_ERROR, LOCK_TIMEOUT or NOT_STARTED.
     */
    Result discardRecord(uint8_t slot) {
        tm_detail::Guard g(_lock);
        if (!g.ok())              return Result::LOCK_TIMEOUT;
        if (!_started)            return Result::NOT_STARTED;
        if (slot >= MAX_CHANNELS) return Result::INVALID_SLOT;
        if (!_unreadable[slot])   return Result::EMPTY_SLOT;
        return _erase(slot) ? Result::OK : Result::STORAGE_ERROR;
    }

    // =========================================================================
    // HELPERS
    // =========================================================================

    /// Number of Result values.
    static constexpr uint8_t RESULT_COUNT = 9;
    /// Longest name returned by resultName(), excluding the null terminator.
    static constexpr size_t RESULT_NAME_LEN  = 16;  // "address_mismatch"
    /// Buffer size for resultName(), including the null terminator.
    static constexpr size_t RESULT_NAME_SIZE = RESULT_NAME_LEN + 1;

    /**
     * @brief Lower-case name of a Result ("ok", "address_mismatch", ...)
     *        into a caller buffer. In flash on AVR.
     * @return @p buf, or nullptr if @p buf is null or @p len is 0.
     */
    static char* resultName(Result r, char* buf, size_t len) {
        if (buf == nullptr || len == 0) return nullptr;
        static const char n0[] PROGMEM = "ok";
        static const char n1[] PROGMEM = "invalid_slot";
        static const char n2[] PROGMEM = "empty_slot";
        static const char n3[] PROGMEM = "address_mismatch";
        static const char n4[] PROGMEM = "invalid_config";
        static const char n5[] PROGMEM = "in_use";
        static const char n6[] PROGMEM = "storage_error";
        static const char n7[] PROGMEM = "lock_timeout";
        static const char n8[] PROGMEM = "not_started";
        static const char* const t[RESULT_COUNT] PROGMEM = { n0, n1, n2, n3, n4, n5, n6, n7, n8 };
        uint8_t i = static_cast<uint8_t>(r);
        if (i >= RESULT_COUNT) i = static_cast<uint8_t>(Result::INVALID_SLOT);
        PGM_P  src = (PGM_P)pgm_read_ptr(&t[i]);
        size_t n   = strlen_P(src);
        if (n > len - 1) n = len - 1;
        memcpy_P(buf, src, n);
        buf[n] = '\0';
        return buf;
    }

    /**
     * @brief Serialise a slot record. Public for tests and backup tools.
     * @return Bytes written (<= RECORD_SIZE).
     */
    static size_t packRecord(uint8_t* buf, const DeviceAddress address,
                             DS18B20Sensor::Family family, const Config& cfg) {
        size_t p = 0;
        buf[p++] = RECORD_VERSION;
        memcpy(buf + p, address, 8);                      p += 8;
        buf[p++] = (uint8_t)family;
        buf[p++] = (uint8_t)((cfg.enabled ? 0x01 : 0) | (cfg.range.spikeFilter ? 0x02 : 0));
        size_t nameLen = strlen(cfg.name);
        if (nameLen > TemperatureChannel::NAME_LEN) nameLen = TemperatureChannel::NAME_LEN;
        buf[p++] = (uint8_t)nameLen;
        memcpy(buf + p, cfg.name, nameLen);               p += nameLen;
        p = _putF(buf, p, cfg.range.loSoft);
        p = _putF(buf, p, cfg.range.hiSoft);
        p = _putF(buf, p, cfg.range.loHard);
        p = _putF(buf, p, cfg.range.hiHard);
        p = _putF(buf, p, cfg.range.hysteresis);
        p = _putU(buf, p, cfg.range.tauSeconds);
        p = _putU(buf, p, cfg.range.alarmDelaySeconds);
        p = _putU(buf, p, cfg.range.faultDelaySeconds);
        return p;
    }

    /**
     * @brief Parse a slot record written by packRecord().
     * @return false on an unknown version, a bad length or an invalid config.
     */
    static bool unpackRecord(const uint8_t* buf, size_t len, DeviceAddress address,
                             DS18B20Sensor::Family& family, Config& cfg) {
        const size_t FIXED = 1 + 8 + 1 + 1 + 1 + 5 * 4 + 3 * 4;
        if (len < FIXED || buf[0] != RECORD_VERSION) return false;
        size_t nameLen = buf[11];
        if (nameLen > TemperatureChannel::NAME_LEN || len != FIXED + nameLen) return false;
        if (buf[9] > (uint8_t)CheckDS18B20::NO_SENSOR) return false;
        size_t p = 1;
        memcpy(address, buf + p, 8);                       p += 8;
        family = (DS18B20Sensor::Family)buf[p++];
        uint8_t flags = buf[p++];
        p++;                                               // name length
        memcpy(cfg.name, buf + p, nameLen);                p += nameLen;
        cfg.name[nameLen]      = '\0';
        cfg.enabled            = (flags & 0x01) != 0;
        cfg.range.spikeFilter  = (flags & 0x02) != 0;
        cfg.range.loSoft       = _getF(buf, p);            p += 4;
        cfg.range.hiSoft       = _getF(buf, p);            p += 4;
        cfg.range.loHard       = _getF(buf, p);            p += 4;
        cfg.range.hiHard       = _getF(buf, p);            p += 4;
        cfg.range.hysteresis   = _getF(buf, p);            p += 4;
        cfg.range.tauSeconds        = _getU(buf, p);       p += 4;
        cfg.range.alarmDelaySeconds = _getU(buf, p);       p += 4;
        cfg.range.faultDelaySeconds = _getU(buf, p);
        return cfg.range.validate() == RangeMonitor::ConfigError::NONE;
    }

private:
    // =========================================================================
    // SLOTS (lock held)
    // =========================================================================

    int8_t _findSlot(const DeviceAddress address) const {
        for (uint8_t s = 0; s < MAX_CHANNELS; s++)
            if (_channels[s].hasAddress(address)) return (int8_t)s;
        return -1;
    }

    int8_t _freeSlot() const {
        for (uint8_t s = 0; s < MAX_CHANNELS; s++)
            if (!_channels[s].isAssigned() && !_unreadable[s]) return (int8_t)s;
        return -1;
    }

    Result _checkSlot(uint8_t slot, const DeviceAddress expected) const {
        if (!_started)                           return Result::NOT_STARTED;
        if (slot >= MAX_CHANNELS)                return Result::INVALID_SLOT;
        if (!_channels[slot].isAssigned())       return Result::EMPTY_SLOT;
        if (!_channels[slot].hasAddress(expected)) return Result::ADDRESS_MISMATCH;
        return Result::OK;
    }

    // =========================================================================
    // PERSISTENCE (lock held)
    // =========================================================================

    static void _key(char* buf, uint8_t slot) {
        buf[0] = 's';
        if (slot >= 10) { buf[1] = (char)('0' + slot / 10); buf[2] = (char)('0' + slot % 10); buf[3] = '\0'; }
        else            { buf[1] = (char)('0' + slot);      buf[2] = '\0'; }
    }

    bool _loadAll() {
        for (uint8_t s = 0; s < MAX_CHANNELS; s++) _unreadable[s] = false;
        if (_ns[0] == '\0') return true;
        tm_detail::Prefs p;
        if (!p.begin(_ns, true)) return true;   // namespace not created yet: nothing stored
        for (uint8_t s = 0; s < MAX_CHANNELS; s++) {
            char key[4];
            _key(key, s);
            if (!p.isKey(key)) continue;
            uint8_t buf[RECORD_SIZE];
            size_t len = p.getBytesLength(key);
            if (len == 0 || len > RECORD_SIZE || p.getBytes(key, buf, len) != len) {
                _unreadable[s] = true;
                _census.invalidRecords++;
                continue;
            }
            DeviceAddress addr;
            DS18B20Sensor::Family fam;
            Config cfg;
            if (!unpackRecord(buf, len, addr, fam, cfg) || _findSlot(addr) >= 0) {
                _unreadable[s] = true;
                _census.invalidRecords++;
                continue;
            }
            _channels[s].assign(addr, cfg, fam, _resolution);
        }
        p.end();
        return true;
    }

    bool _save(uint8_t slot) {
        if (_ns[0] == '\0') return true;
        DeviceAddress addr;
        Config cfg;
        if (!_channels[slot].getAddress(addr) || !_channels[slot].getConfig(cfg)) return false;
        uint8_t buf[RECORD_SIZE];
        size_t len = packRecord(buf, addr, _channels[slot].family(), cfg);
        tm_detail::Prefs p;
        if (!p.begin(_ns, false)) return false;
        char key[4];
        _key(key, slot);
        bool ok = p.putBytes(key, buf, len) == len;
        p.end();
        if (ok) _unreadable[slot] = false;
        return ok;
    }

    bool _erase(uint8_t slot) {
        if (_ns[0] == '\0') return true;
        tm_detail::Prefs p;
        if (!p.begin(_ns, false)) return false;
        char key[4];
        _key(key, slot);
        bool ok = !p.isKey(key) || p.remove(key);
        p.end();
        if (ok) _unreadable[slot] = false;
        return ok;
    }

    // Little-endian field codecs; explicit, so the record does not depend on
    // the compiler's struct layout.
    static size_t _putU(uint8_t* b, size_t p, uint32_t v) {
        b[p] = (uint8_t)v; b[p + 1] = (uint8_t)(v >> 8);
        b[p + 2] = (uint8_t)(v >> 16); b[p + 3] = (uint8_t)(v >> 24);
        return p + 4;
    }
    static uint32_t _getU(const uint8_t* b, size_t p) {
        return (uint32_t)b[p] | ((uint32_t)b[p + 1] << 8) |
               ((uint32_t)b[p + 2] << 16) | ((uint32_t)b[p + 3] << 24);
    }
    static size_t _putF(uint8_t* b, size_t p, float f) {
        uint32_t v;
        memcpy(&v, &f, 4);
        return _putU(b, p, v);
    }
    static float _getF(const uint8_t* b, size_t p) {
        uint32_t v = _getU(b, p);
        float f;
        memcpy(&f, &v, 4);
        return f;
    }

    // =========================================================================
    // MEMBER VARIABLES
    // =========================================================================

    tm_detail::Lock    _lock;
    OneWire&           _oneWire;
    DallasTemperature& _dallas;
    TemperatureChannel _channels[MAX_CHANNELS];
    bool               _unreadable[MAX_CHANNELS]; ///< Slot holds a record we could not read
    Event              _pending[MAX_CHANNELS];    ///< Events of the current cycle
    Config             _default;
    Census             _census;
    EventCallback      _callback;
    void*              _callbackCtx;
    uint32_t           _pollIntervalMs;
    uint32_t           _conversionMs;
    uint32_t           _lastRequestMs;
    char               _ns[16];
    uint8_t            _resolution;
    bool               _identify;
    bool               _started;
    bool               _converting;
    bool               _firstCycle;
};
