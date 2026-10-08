#pragma once
/**
 * @file Preferences.h (host test stub)
 * @brief In-memory Preferences with the subset of the API TemperatureMonitor
 *        uses, plus hooks to inspect and corrupt the store.
 */
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <vector>

class Preferences {
public:
    typedef std::map<std::string, std::vector<uint8_t>> Space;

    /// The whole store: namespace -> key -> bytes. Survives Preferences objects.
    static std::map<std::string, Space>& store() {
        static std::map<std::string, Space> s;
        return s;
    }
    static bool& failWrites() { static bool f = false; return f; }
    static int&  writeCount() { static int n = 0; return n; }

    bool begin(const char* name, bool readOnly = false) {
        if (readOnly && store().find(name) == store().end()) return false;
        _ns = name;
        _ro = readOnly;
        store()[_ns];
        _open = true;
        return true;
    }
    void end() { _open = false; }

    bool isKey(const char* key) {
        Space& s = store()[_ns];
        return s.find(key) != s.end();
    }
    size_t getBytesLength(const char* key) {
        Space& s = store()[_ns];
        auto it = s.find(key);
        return it == s.end() ? 0 : it->second.size();
    }
    size_t getBytes(const char* key, void* buf, size_t maxLen) {
        Space& s = store()[_ns];
        auto it = s.find(key);
        if (it == s.end() || it->second.size() > maxLen) return 0;
        memcpy(buf, it->second.data(), it->second.size());
        return it->second.size();
    }
    size_t putBytes(const char* key, const void* value, size_t len) {
        if (!_open || _ro || failWrites()) return 0;
        const uint8_t* v = static_cast<const uint8_t*>(value);
        store()[_ns][key] = std::vector<uint8_t>(v, v + len);
        writeCount()++;
        return len;
    }
    bool remove(const char* key) {
        if (!_open || _ro || failWrites()) return false;
        return store()[_ns].erase(key) > 0;
    }

private:
    std::string _ns;
    bool        _ro   = false;
    bool        _open = false;
};
