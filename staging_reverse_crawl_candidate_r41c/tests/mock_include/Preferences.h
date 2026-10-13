#pragma once
#include <stddef.h>

class Preferences {
public:
    bool begin(const char* name, bool readOnly = false) { return true; }
    void end() {}
    size_t putBytes(const char* key, const void* value, size_t len) { return len; }
    size_t getBytes(const char* key, void* buf, size_t maxLen) { return 0; }
    size_t putBool(const char* key, bool val) { return 1; }
    bool getBool(const char* key, bool default_val = false) { return default_val; }
    size_t putFloat(const char* key, float val) { return sizeof(float); }
    float getFloat(const char* key, float default_val = 0.0f) { return default_val; }
};
