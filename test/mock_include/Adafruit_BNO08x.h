#pragma once
#include <stdint.h>

struct sh2_SensorValue_t {
    uint8_t sensorId;
};

class Adafruit_BNO08x {
public:
    bool begin_I2C(uint8_t = 0x4B) { return true; }
    bool enableReport(uint8_t, uint32_t = 20000) { return true; }
    bool getSensorEvent(sh2_SensorValue_t*) { return false; }
    bool wasReset() { return false; }
};
