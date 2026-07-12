#include "SafetyManager.h"

SafetyManager::SafetyManager() 
    : activeFaults(FAULT_NONE) {
    clearFaults();
}

void SafetyManager::begin() {
    clearFaults();
}

uint32_t SafetyManager::update(const float *targets, const float *measured, const int *pwms, float dt, const MotorDriver &driver) {
    if (dt <= 0.0f) return activeFaults;
    
    // Bypass safety updates during non-driving control phases (Locked, Maintenance, Calibration, Faulted)
    if (driver.getMode() != MotorOutputMode::NORMAL_DRIVE) {
        return activeFaults;
    }
    
    for (int i = 0; i < 4; i++) {
        float target = targets[i];
        float speed = measured[i];
        int pwm = pwms[i];
        
        // Zero target reset
        if (abs(target) < 0.01f) {
            nonzeroTargetTicks[i] = 0;
            stallTicks[i] = 0;
            encFaultTicks[i] = 0;
            directionMismatchTicks[i] = 0;
            lastTargets[i] = 0.0f;
            continue;
        }
        
        // Direction switch deadtime detection
        if ((target > 0.0f && lastTargets[i] < -0.01f) || (target < 0.0f && lastTargets[i] > 0.01f)) {
            nonzeroTargetTicks[i] = 0;
            stallTicks[i] = 0;
            encFaultTicks[i] = 0;
            directionMismatchTicks[i] = 0;
        }
        
        nonzeroTargetTicks[i]++;
        lastTargets[i] = target;
        
        // Breakaway grace period: ignore faults for the first 100 ticks (1.0 second) of motion
        if (nonzeroTargetTicks[i] <= 100) {
            continue;
        }
        
        // 1. Motor Stall Detection:
        // Commanded non-zero, high PWM output, but wheel speed remains near-zero
        if (abs(target) > 0.5f && abs(pwm) > 90 && abs(speed) < 0.1f) {
            stallTicks[i]++;
            if (stallTicks[i] > 200) { // 2.0 seconds at 100Hz
                activeFaults |= (FAULT_STALL_M1 << i);
            }
        } else {
            stallTicks[i] = 0;
        }
        
        // 2. Encoder Fault Detection (disconnected lines):
        // Driving PWM output but zero speed reported
        if (abs(target) > 0.5f && abs(pwm) > 90 && abs(speed) == 0.0f) {
            encFaultTicks[i]++;
            if (encFaultTicks[i] > 150) { // 1.5 seconds
                activeFaults |= (FAULT_ENCODER_M1 << i);
            }
        } else {
            encFaultTicks[i] = 0;
        }
        
        // 3. Encoder Direction Fault Detection:
        // Target and measured speeds are opposite sign for more than 500ms
        if (abs(target) > 0.8f && abs(speed) > 0.2f && ((target > 0.0f && speed < -0.1f) || (target < 0.0f && speed > 0.1f))) {
            directionMismatchTicks[i]++;
            if (directionMismatchTicks[i] > 50) { // 500ms
                activeFaults |= (FAULT_ENCODER_M1 << i); // Treat as encoder fault
            }
        } else {
            directionMismatchTicks[i] = 0;
        }
    }
    
    // 4. Side-to-Side Motor Disagreement Checks (only if both track wheels have completed their grace periods)
    // Left Track Front (M1) vs Left Track Rear (M3)
    if (abs(targets[0]) > 0.8f && abs(targets[2]) > 0.8f && nonzeroTargetTicks[0] > 100 && nonzeroTargetTicks[2] > 100) {
        float diffLeft = abs(measured[0] - measured[2]);
        if (diffLeft > 2.0f && (abs(measured[0]) < 0.1f || abs(measured[2]) < 0.1f)) {
            sideMismatchTicks[0]++;
            if (sideMismatchTicks[0] > 150) { // 1.5 seconds
                activeFaults |= FAULT_MISMATCH_LEFT;
            }
        } else {
            sideMismatchTicks[0] = 0;
        }
    } else {
        sideMismatchTicks[0] = 0;
    }
    
    // Right Track Front (M2) vs Right Track Rear (M4)
    if (abs(targets[1]) > 0.8f && abs(targets[3]) > 0.8f && nonzeroTargetTicks[1] > 100 && nonzeroTargetTicks[3] > 100) {
        float diffRight = abs(measured[1] - measured[3]);
        if (diffRight > 2.0f && (abs(measured[1]) < 0.1f || abs(measured[3]) < 0.1f)) {
            sideMismatchTicks[1]++;
            if (sideMismatchTicks[1] > 150) { // 1.5 seconds
                activeFaults |= FAULT_MISMATCH_RIGHT;
            }
        } else {
            sideMismatchTicks[1] = 0;
        }
    } else {
        sideMismatchTicks[1] = 0;
    }
    
    return activeFaults;
}

void SafetyManager::clearFaults() {
    activeFaults = FAULT_NONE;
    for (int i = 0; i < 4; i++) {
        stallTicks[i] = 0;
        encFaultTicks[i] = 0;
        directionMismatchTicks[i] = 0;
        nonzeroTargetTicks[i] = 0;
        lastTargets[i] = 0.0f;
    }
    sideMismatchTicks[0] = 0;
    sideMismatchTicks[1] = 0;
}
