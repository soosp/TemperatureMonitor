#pragma once
/**
 * @file CheckDS18B20.h (host test stub)
 * @brief The family enum of CheckDS18B20 1.0.x and a scriptable identifier.
 */
#include "OneWire.h"

namespace CheckDS18B20 {
    typedef enum {FAMILY_A1, FAMILY_A2, FAMILY_A_UNKNOWN, FAMILY_B1, FAMILY_B2,
                  FAMILY_B_UNKNOWN, FAMILY_C, FAMILY_D1_NO_PARASITIC, FAMILY_D1,
                  FAMILY_D2, FAMILY_D_UNKNOWN, FAMILY_E, FAMILY_UNKNOWN,
                  NO_SENSOR} DS18B20_family_enum;

    /// What ds18b20_family() returns, per bus index. Set by the tests.
    inline DS18B20_family_enum* stubFamilies() {
        static DS18B20_family_enum f[32] = {
            FAMILY_A1, FAMILY_A1, FAMILY_A1, FAMILY_A1, FAMILY_A1, FAMILY_A1, FAMILY_A1, FAMILY_A1,
            FAMILY_A1, FAMILY_A1, FAMILY_A1, FAMILY_A1, FAMILY_A1, FAMILY_A1, FAMILY_A1, FAMILY_A1,
            FAMILY_A1, FAMILY_A1, FAMILY_A1, FAMILY_A1, FAMILY_A1, FAMILY_A1, FAMILY_A1, FAMILY_A1,
            FAMILY_A1, FAMILY_A1, FAMILY_A1, FAMILY_A1, FAMILY_A1, FAMILY_A1, FAMILY_A1, FAMILY_A1 };
        return f;
    }
    inline int& stubCalls() { static int n = 0; return n; }

    inline DS18B20_family_enum ds18b20_family(OneWire*, int sensor_number) {
        stubCalls()++;
        return stubFamilies()[sensor_number];
    }
}
