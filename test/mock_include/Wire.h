#pragma once
#include <stdint.h>

class TwoWire {
public:
    void begin() {}
    void begin(int, int) {}
};

extern TwoWire Wire;
