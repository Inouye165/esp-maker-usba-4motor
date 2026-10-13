#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>
#include <stdio.h>

template<typename T>
T constrain(T val, T min_val, T max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

using std::min;
using std::max;

inline uint32_t millis() {
    extern uint32_t g_mockMillis;
    return g_mockMillis;
}

inline uint64_t micros() {
    extern uint64_t g_mockMicros;
    return g_mockMicros;
}

inline int64_t esp_timer_get_time() {
    extern uint64_t g_mockMicros;
    return (int64_t)g_mockMicros;
}
