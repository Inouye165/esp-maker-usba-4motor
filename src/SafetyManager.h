#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include <Arduino.h>
#include "MotorDriver.h"

enum FaultType {
    FAULT_NONE          = 0,
    FAULT_STALL_M1      = 1 << 0,
    FAULT_STALL_M2      = 1 << 1,
    FAULT_STALL_M3      = 1 << 2,
    FAULT_STALL_M4      = 1 << 3,
    FAULT_ENCODER_M1    = 1 << 4,
    FAULT_ENCODER_M2    = 1 << 5,
    FAULT_ENCODER_M3    = 1 << 6,
    FAULT_ENCODER_M4    = 1 << 7,
    FAULT_MISMATCH_LEFT = 1 << 8,
    FAULT_MISMATCH_RIGHT= 1 << 9
};

class SafetyManager {
public:
    SafetyManager();
    void begin();
    
    // Evaluate safety checks at 100Hz
    // Returns active fault bitmask (0 if all safe)
    uint32_t update(const float *targets, const float *measured, const int *pwms, float dt, const MotorDriver &driver);
    
    uint32_t getFaults() const { return activeFaults; }
    bool hasFaults() const { return activeFaults != FAULT_NONE; }
    void clearFaults();

private:
    uint32_t activeFaults;
    
    // Accumulators for fault detection timers
    uint32_t stallTicks[4];
    uint32_t encFaultTicks[4];
    uint32_t directionMismatchTicks[4];
    uint32_t sideMismatchTicks[2]; // 0=left, 1=right
    
    // Stiction breakaway grace period tracking
    uint32_t nonzeroTargetTicks[4];
    float lastTargets[4];
};

#endif // SAFETY_MANAGER_H
