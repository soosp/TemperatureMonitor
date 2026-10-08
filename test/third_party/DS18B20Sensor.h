/**
 * @file DS18B20Sensor.h
 * @brief One DS18B20-compatible 1-Wire temperature sensor: address, family,
 *        reading with classified errors.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Péter Soós — https://github.com/soosp
 *
 * -----------------------------------------------------------------------------
 * SCOPE
 * -----------------------------------------------------------------------------
 *
 * The hardware layer and nothing else. DS18B20Sensor knows one sensor's ROM
 * address, its family (genuine or which clone, via CheckDS18B20), its
 * resolution and its last reading. It does not own the bus — OneWire and
 * DallasTemperature are passed in — and it has no persistence, no
 * description, no alarm logic, no JSON and no lock. Those belong to the
 * layer that owns the sensor (TemperatureMonitor), which is also the layer
 * that knows what an error means for the application.
 *
 * -----------------------------------------------------------------------------
 * READING
 * -----------------------------------------------------------------------------
 *
 * The owner issues one broadcast dallas.requestTemperatures() for the whole
 * bus, then calls read() on each sensor. read() reads by ROM address, never
 * by bus index: an index is a position in a bus search, and a search that
 * glitches once returns a neighbour's value under the wrong index.
 *
 * read() takes the raw value from DallasTemperature::getTemp() rather than
 * getTempC(), because getTempC() folds every error code into -127 °C. The raw
 * value keeps the reason apart, and the most useful of them is the power-on
 * reset: the 85 °C a DS18B20 reports after a brown-out is classified as
 * POWER_ON_RESET, not returned as a temperature.
 *
 * On error read() returns NAN and lastError() tells why. A valid reading
 * clears the error and the consecutive-error counter.
 *
 * Limitation inherited from DallasTemperature: error codes are raw values at
 * or below -7040, which is exactly -55 °C in its 1/128 °C fixed point. A
 * reading of -55 °C or below is therefore reported as DISCONNECTED. That is
 * the DS18B20's lower limit anyway, but it also cuts the MAX31850's range.
 *
 * -----------------------------------------------------------------------------
 * IDENTIFICATION
 * -----------------------------------------------------------------------------
 *
 * CheckDS18B20 tells a genuine Maxim DS18B20 from the common clones by their
 * reactions to undocumented commands, and in doing so writes the scratchpad
 * (alarm registers and configuration). begin() therefore recalls the
 * scratchpad from the sensor's EEPROM afterwards and then sets the requested
 * resolution, so the sensor ends up configured exactly as asked whether or
 * not it was identified.
 *
 * Identification works by bus index only (that is CheckDS18B20's API), so it
 * is available from begin(..., index, ...), not from begin(..., address, ...).
 *
 * -----------------------------------------------------------------------------
 * THREAD SAFETY AND MEMORY
 * -----------------------------------------------------------------------------
 *
 * Not thread-safe: drive it from one task, or let the owner serialise access.
 * getStatus() copies every field in one call, so an owner holding its own
 * lock gets a consistent picture with the shortest possible critical section.
 *
 * No dynamic allocation, no strings held (the family is an enum, its name
 * is copied out of flash on request). About 24 bytes of RAM per instance.
 * C++11-compatible (the Arduino AVR core compiles with -std=gnu++11).
 */

#pragma once

#include <stdint.h>
#include <string.h>
#include <math.h>

#ifdef ARDUINO
    #include <Arduino.h>
#endif

#include <OneWire.h>
#include <DallasTemperature.h>  // >= 4.0.5: raw error codes
#include <CheckDS18B20.h>

#if defined(__AVR__)
    #include <avr/pgmspace.h>
#elif !defined(ARDUINO)
    // Host build: flash and RAM are the same thing.
    #ifndef PROGMEM
        #define PROGMEM
    #endif
    #ifndef PGM_P
        #define PGM_P const char*
    #endif
    #ifndef strlen_P
        #define strlen_P(s) strlen(s)
    #endif
    #ifndef memcpy_P
        #define memcpy_P(d, s, n) memcpy((d), (s), (n))
    #endif
    #ifndef pgm_read_ptr
        #define pgm_read_ptr(p) (*(p))
    #endif
#endif

/**
 * @brief One DS18B20-compatible 1-Wire temperature sensor.
 *
 * @par Typical use
 * @code
 * OneWire           oneWire(PIN);
 * DallasTemperature dallas(&oneWire);
 * DS18B20Sensor     sensors[MAX_SENSORS];
 * uint8_t           count = 0;
 *
 * dallas.begin();
 * uint8_t found = dallas.getDS18Count();
 * for (uint8_t i = 0; i < found && count < MAX_SENSORS; i++) {
 *     if (sensors[count].begin(dallas, oneWire, i)) count++;
 * }
 *
 * dallas.requestTemperatures();                 // broadcast, all convert at once
 * for (uint8_t i = 0; i < count; i++) {
 *     float t = sensors[i].read(dallas);        // NAN on error
 *     if (isnan(t)) { DS18B20Sensor::Error e = sensors[i].lastError(); ... }
 * }
 * @endcode
 */
class DS18B20Sensor {
public:
    // =========================================================================
    // TYPES
    // =========================================================================

    /// Sensor family as identified by CheckDS18B20 (genuine Maxim or a clone).
    typedef CheckDS18B20::DS18B20_family_enum Family;

    /**
     * @brief Why the last read() failed. The numeric values are stable.
     */
    enum class Error : uint8_t {
        NONE               = 0, ///< Last read was valid
        NOT_INITIALISED    = 1, ///< begin() has not succeeded
        DISCONNECTED       = 2, ///< No answer, or scratchpad CRC failed after the retries
        POWER_ON_RESET     = 3, ///< Sensor reset since the conversion (the "85 °C" value)
        INSUFFICIENT_POWER = 4, ///< Conversion did not complete (weak parasite power)
        FAULT              = 5, ///< MAX31850 thermocouple open or shorted
        OUT_OF_RANGE       = 6  ///< Above the device's measuring range (below it
                                ///< DallasTemperature reports DISCONNECTED)
    };

    /// Number of Error values.
    static constexpr uint8_t ERROR_COUNT = 7;

    /**
     * @brief Everything the sensor knows, copied in one call.
     */
    struct Status {
        DeviceAddress address;           ///< ROM address (all zero before begin())
        Family        family;            ///< Identified family; FAMILY_UNKNOWN if not identified
        bool          identified;        ///< True if CheckDS18B20 ran on this sensor
        bool          initialised;       ///< True after a successful begin()
        uint8_t       resolution;        ///< Resolution in bits (9..12)
        float         lastTemperature;   ///< Last valid reading in °C; NAN after an error
        Error         lastError;         ///< Result of the last read()
        uint16_t      consecutiveErrors; ///< Failed reads since the last valid one (saturates)
        uint32_t      lastValidMs;       ///< Time of the last valid reading; 0 if none yet
    };

    // =========================================================================
    // CONSTANTS
    // =========================================================================

    /// ROM address as hex: 16 digits, excluding the null terminator.
    static constexpr size_t ADDR_STR_LEN  = 16;
    /// ROM address string buffer size, including the null terminator.
    static constexpr size_t ADDR_STR_SIZE = ADDR_STR_LEN + 1;

    /// Longest name returned by errorName(), excluding the null terminator.
    static constexpr size_t ERROR_NAME_LEN  = 18;   // "insufficient_power"
    /// Buffer size for errorName(), including the null terminator.
    static constexpr size_t ERROR_NAME_SIZE = ERROR_NAME_LEN + 1;

    /// Longest name returned by familyName(), excluding the null terminator.
    static constexpr size_t FAMILY_NAME_LEN  = 22;  // "D1 (no parasite power)"
    /// Buffer size for familyName(), including the null terminator.
    static constexpr size_t FAMILY_NAME_SIZE = FAMILY_NAME_LEN + 1;

    /// Measuring range of the DS18x20 devices [°C].
    static constexpr float TEMP_MIN = -55.0f;
    static constexpr float TEMP_MAX = 125.0f;   ///< @see TEMP_MIN

    /// Upper limit of a MAX31850 thermocouple interface [°C]. Its lower
    /// limit is cut to -55 °C by DallasTemperature's error encoding.
    static constexpr float TC_TEMP_MAX = 1800.0f;

    /// Default scratchpad read retries on CRC failure.
    static constexpr uint8_t DEFAULT_READ_RETRIES = 1;

    // =========================================================================
    // LIFECYCLE
    // =========================================================================

    /// Uninitialised sensor; call begin().
    DS18B20Sensor() { _clear(); }

    /**
     * @brief Initialise from a bus index (discovery).
     *
     * Reads the ROM address of the @p index-th device, optionally identifies
     * its family with CheckDS18B20, restores the scratchpad from the sensor's
     * EEPROM (identification overwrites it), and sets the resolution.
     *
     * Call after dallas.begin(), before the measuring loop. Identification
     * takes a few hundred milliseconds per sensor.
     *
     * @param dallas      DallasTemperature on the bus.
     * @param oneWire     The same bus, for CheckDS18B20.
     * @param index       Device index in the bus search (0-based).
     * @param identify    Run CheckDS18B20 identification.
     * @param resolution  Resolution in bits, 9..12 (ignored by DS18S20).
     * @return            false if no device answers at @p index; the
     *                    sensor is then left uninitialised.
     */
    bool begin(DallasTemperature& dallas, OneWire& oneWire, uint8_t index,
               bool identify = true, uint8_t resolution = 12) {
        _clear();
        if (!dallas.getAddress(_address, index)) return false;
        if (identify) {
            _family     = CheckDS18B20::ds18b20_family(&oneWire, index);
            _identified = true;
            dallas.recallScratchPad(_address);
        }
        return _finishBegin(dallas, resolution);
    }

    /**
     * @brief Initialise from a known ROM address.
     *
     * For a sensor whose address was stored earlier. No bus search and no
     * identification; the family stays FAMILY_UNKNOWN.
     *
     * @param dallas      DallasTemperature on the bus.
     * @param address     ROM address.
     * @param resolution  Resolution in bits, 9..12 (ignored by DS18S20).
     * @return            false if the device does not answer; the address is
     *                    kept, so addressStr() still names the missing sensor.
     */
    bool begin(DallasTemperature& dallas, const DeviceAddress address,
               uint8_t resolution = 12) {
        _clear();
        memcpy(_address, address, sizeof(DeviceAddress));
        if (!dallas.isConnected(_address)) return false;
        return _finishBegin(dallas, resolution);
    }

    // =========================================================================
    // READING
    // =========================================================================

    /**
     * @brief Read the temperature converted by the last broadcast.
     *
     * Call after dallas.requestTemperatures() has completed.
     *
     * @param dallas  DallasTemperature on the bus.
     * @param nowMs   Time of the reading, stored as lastValidMs on success.
     * @return        Temperature in °C, or NAN (see lastError()).
     */
    float read(DallasTemperature& dallas, uint32_t nowMs) {
        if (!_initialised) {
            _fail(Error::NOT_INITIALISED);
            return NAN;
        }
        int32_t raw = dallas.getTemp(_address, _retries);
        Error   e   = classifyRaw(raw);
        if (e != Error::NONE) {
            _fail(e);
            return NAN;
        }
        float t = DallasTemperature::rawToCelsius(raw);
        if (!_inRange(t)) {
            _fail(Error::OUT_OF_RANGE);
            return NAN;
        }
        _lastTemperature   = t;
        _lastError         = Error::NONE;
        _consecutiveErrors = 0;
        _lastValidMs       = nowMs;
        return t;
    }

#ifdef ARDUINO
    /// Read with millis() as the time. @see read(DallasTemperature&, uint32_t)
    float read(DallasTemperature& dallas) { return read(dallas, (uint32_t)millis()); }
#endif

    /**
     * @brief Check that the sensor answers on the bus.
     *
     * Reads the scratchpad (address match + CRC); does not start a conversion.
     */
    bool isPresent(DallasTemperature& dallas) const {
        return _initialised && dallas.isConnected(_address);
    }

    /**
     * @brief Set the scratchpad read retries used on CRC failure.
     * @param retries  Additional attempts after the first (0 = no retry).
     */
    void setReadRetries(uint8_t retries) { _retries = retries; }

    // =========================================================================
    // STATE
    // =========================================================================

    /// Everything the sensor knows, in one copy.
    Status getStatus() const {
        Status s;
        memcpy(s.address, _address, sizeof(DeviceAddress));
        s.family            = _family;
        s.identified        = _identified;
        s.initialised       = _initialised;
        s.resolution        = _resolution;
        s.lastTemperature   = _lastTemperature;
        s.lastError         = _lastError;
        s.consecutiveErrors = _consecutiveErrors;
        s.lastValidMs       = _lastValidMs;
        return s;
    }

    float          lastTemperature()   const { return _lastTemperature; }   ///< NAN after an error
    Error          lastError()         const { return _lastError; }         ///< Result of the last read()
    uint16_t       consecutiveErrors() const { return _consecutiveErrors; } ///< Since the last valid read
    uint32_t       lastValidMs()       const { return _lastValidMs; }       ///< 0 if none yet
    bool           isInitialised()     const { return _initialised; }       ///< begin() succeeded
    Family         family()            const { return _family; }            ///< FAMILY_UNKNOWN if not identified
    uint8_t        resolution()        const { return _resolution; }        ///< Bits, 9..12
    const uint8_t* address()           const { return _address; }           ///< 8-byte ROM address

    /// True if this sensor has the given ROM address.
    bool hasAddress(const DeviceAddress address) const {
        return memcmp(_address, address, sizeof(DeviceAddress)) == 0;
    }

    /// ROM address as 16 upper-case hex digits. @see formatAddress()
    char* addressStr(char* buf, size_t len) const {
        return formatAddress(_address, buf, len);
    }

    // =========================================================================
    // STATIC HELPERS
    // =========================================================================

    /**
     * @brief Classify a raw DallasTemperature reading.
     *
     * @param raw  Value from DallasTemperature::getTemp().
     * @return     The error it encodes, or Error::NONE for a measurement.
     */
    static Error classifyRaw(int32_t raw) {
        if (raw > DEVICE_DISCONNECTED_RAW)       return Error::NONE;
        if (raw == DEVICE_POWER_ON_RESET_RAW)     return Error::POWER_ON_RESET;
        if (raw == DEVICE_INSUFFICIENT_POWER_RAW) return Error::INSUFFICIENT_POWER;
        if (raw == DEVICE_FAULT_OPEN_RAW ||
            raw == DEVICE_FAULT_SHORTGND_RAW ||
            raw == DEVICE_FAULT_SHORTVDD_RAW)     return Error::FAULT;
        return Error::DISCONNECTED;
    }

    /**
     * @brief Format a ROM address as 16 upper-case hex digits.
     *
     * @param address  ROM address.
     * @param buf      Destination; @ref ADDR_STR_SIZE bytes for the full address.
     * @param len      Size of @p buf, including the null terminator.
     * @return         @p buf (truncated if too small), or nullptr if @p buf
     *                 is null or @p len is 0.
     */
    static char* formatAddress(const DeviceAddress address, char* buf, size_t len) {
        if (buf == nullptr || len == 0) return nullptr;
        size_t n = 0;
        for (uint8_t i = 0; i < 8 && n + 1 < len; i++) {
            buf[n++] = _hexDigit(address[i] >> 4);
            if (n + 1 < len) buf[n++] = _hexDigit(address[i] & 0x0F);
        }
        buf[n] = '\0';
        return buf;
    }

    /**
     * @brief Parse 16 hex digits (either case) into a ROM address.
     *
     * @param str       Exactly 16 hex digits, null-terminated.
     * @param out       Destination; unchanged on failure.
     * @param checkCrc  Also require a valid ROM CRC (last byte).
     * @return          false on wrong length, a non-hex character or a bad CRC.
     */
    static bool parseAddress(const char* str, DeviceAddress out, bool checkCrc = true) {
        if (str == nullptr || strlen(str) != ADDR_STR_LEN) return false;
        DeviceAddress tmp;
        for (uint8_t i = 0; i < 8; i++) {
            int hi = _hexValue(str[2 * i]);
            int lo = _hexValue(str[2 * i + 1]);
            if (hi < 0 || lo < 0) return false;
            tmp[i] = (uint8_t)((hi << 4) | lo);
        }
        if (checkCrc && OneWire::crc8(tmp, 7) != tmp[7]) return false;
        memcpy(out, tmp, sizeof(DeviceAddress));
        return true;
    }

    /**
     * @brief Copy the lower-case name of @p e into a caller-supplied buffer.
     *
     * Names: "none", "not_initialised", "disconnected", "power_on_reset",
     * "insufficient_power", "fault", "out_of_range". Stable; in flash on AVR.
     *
     * @return @p buf, or nullptr if @p buf is null or @p len is 0.
     */
    static char* errorName(Error e, char* buf, size_t len) {
        uint8_t i = static_cast<uint8_t>(e);
        if (i >= ERROR_COUNT) i = static_cast<uint8_t>(Error::DISCONNECTED);
        return _copyP(_errorNames()[i], buf, len);
    }

    /**
     * @brief Copy a readable name of @p f into a caller-supplied buffer.
     *
     * Code letter (CheckDS18B20's classification) and manufacturer where
     * known, e.g. "A1 Maxim DS18B20", "B1 GXCAS 18B20", "?" for unknown.
     * In flash on AVR.
     *
     * @return @p buf, or nullptr if @p buf is null or @p len is 0.
     */
    static char* familyName(Family f, char* buf, size_t len) {
        uint8_t i = static_cast<uint8_t>(f);
        if (i > static_cast<uint8_t>(CheckDS18B20::NO_SENSOR))
            i = static_cast<uint8_t>(CheckDS18B20::FAMILY_UNKNOWN);
        return _copyP(_familyNames()[i], buf, len);
    }

    /// Name of this sensor's family. @see familyName(Family, char*, size_t)
    char* familyName(char* buf, size_t len) const { return familyName(_family, buf, len); }

private:
    // =========================================================================
    // HELPERS
    // =========================================================================

    bool _finishBegin(DallasTemperature& dallas, uint8_t resolution) {
        if (resolution < 9)  resolution = 9;
        if (resolution > 12) resolution = 12;
        if (!dallas.setResolution(_address, resolution)) return false;
        uint8_t actual = dallas.getResolution(_address);
        _resolution  = (actual >= 9 && actual <= 12) ? actual : resolution;
        _initialised = true;
        _lastError   = Error::NONE;
        return true;
    }

    void _fail(Error e) {
        _lastTemperature = NAN;
        _lastError       = e;
        if (_consecutiveErrors < 0xFFFF) _consecutiveErrors++;
    }

    bool _inRange(float t) const {
        if (_address[0] == DS1825MODEL)          // also MAX31850 (thermocouple)
            return t <= TC_TEMP_MAX;
        return t >= TEMP_MIN && t <= TEMP_MAX;
    }

    void _clear() {
        memset(_address, 0, sizeof(DeviceAddress));
        _family            = CheckDS18B20::FAMILY_UNKNOWN;
        _identified        = false;
        _initialised       = false;
        _resolution        = 12;
        _retries           = DEFAULT_READ_RETRIES;
        _lastTemperature   = NAN;
        _lastError         = Error::NOT_INITIALISED;
        _consecutiveErrors = 0;
        _lastValidMs       = 0;
    }

    /// Upper-case hex digit; arithmetic rather than a table, so no RAM on AVR.
    static char _hexDigit(uint8_t v) {
        return (char)(v < 10 ? '0' + v : 'A' + (v - 10));
    }

    static int _hexValue(char c) {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        return -1;
    }

    /// Bounded copy of a flash (PROGMEM) string; plain memory on host builds.
    static char* _copyP(PGM_P src, char* buf, size_t len) {
        if (buf == nullptr || len == 0) return nullptr;
        size_t n = strlen_P(src);
        if (n > len - 1) n = len - 1;
        memcpy_P(buf, src, n);
        buf[n] = '\0';
        return buf;
    }

    // Name tables. Function-local statics keep the header self-contained
    // under C++11 (no inline variables); PROGMEM puts them in flash on AVR
    // and ESP8266, where reading them needs the _P functions.

    static const char* const* _errorTable() {
        static const char n0[] PROGMEM = "none";
        static const char n1[] PROGMEM = "not_initialised";
        static const char n2[] PROGMEM = "disconnected";
        static const char n3[] PROGMEM = "power_on_reset";
        static const char n4[] PROGMEM = "insufficient_power";
        static const char n5[] PROGMEM = "fault";
        static const char n6[] PROGMEM = "out_of_range";
        static const char* const t[ERROR_COUNT] PROGMEM = { n0, n1, n2, n3, n4, n5, n6 };
        return t;
    }

    /// Indexed like CheckDS18B20::DS18B20_family_enum.
    static const char* const* _familyTable() {
        static const char f0[]  PROGMEM = "A1 Maxim DS18B20";
        static const char f1[]  PROGMEM = "A2 Maxim DS18B20";
        static const char f2[]  PROGMEM = "A";
        static const char f3[]  PROGMEM = "B1 GXCAS 18B20";
        static const char f4[]  PROGMEM = "B2 7Q-Tek QT18B20";
        static const char f5[]  PROGMEM = "B";
        static const char f6[]  PROGMEM = "C Sensylink CT1820B";
        static const char f7[]  PROGMEM = "D1 (no parasite power)";
        static const char f8[]  PROGMEM = "D1";
        static const char f9[]  PROGMEM = "D2 XSEC SE18B20";
        static const char f10[] PROGMEM = "D";
        static const char f11[] PROGMEM = "E Novosense NS18B20";
        static const char f12[] PROGMEM = "?";
        static const char f13[] PROGMEM = "no sensor";
        static const char* const t[] PROGMEM =
            { f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13 };
        return t;
    }

    struct _Names {
        const char* const* table;
        PGM_P operator[](uint8_t i) const { return (PGM_P)pgm_read_ptr(&table[i]); }
    };
    static _Names _errorNames()  { _Names n = { _errorTable() };  return n; }
    static _Names _familyNames() { _Names n = { _familyTable() }; return n; }

    // =========================================================================
    // MEMBER VARIABLES
    // =========================================================================

    DeviceAddress _address;           ///< ROM address
    float         _lastTemperature;   ///< Last valid reading, NAN after an error
    uint32_t      _lastValidMs;       ///< Time of the last valid reading
    uint16_t      _consecutiveErrors; ///< Failed reads since the last valid one
    Family        _family;            ///< Identified family
    Error         _lastError;         ///< Result of the last read()
    uint8_t       _resolution;        ///< Bits, 9..12
    uint8_t       _retries;           ///< Scratchpad CRC retries
    bool          _identified;        ///< CheckDS18B20 ran
    bool          _initialised;       ///< begin() succeeded
};
