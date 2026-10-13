#ifndef MAINTENANCE_MANAGER_H
#define MAINTENANCE_MANAGER_H

#include <Arduino.h>
#include "MotorDriver.h"

class MaintenanceManager {
public:
    MaintenanceManager();
    void begin();
    
    // Attempt to enter maintenance mode
    bool enter(bool ack, int motorIdx, uint32_t sessId, MotorDriver &driver);
    void exit(MotorDriver &driver);
    
    // Set current PWM output (must be called continuously as a deadman signal)
    void setOutput(int pwm);
    
    // Process step (called at loop frequency, 100Hz)
    void update(MotorDriver &driver);
    
    bool isActive() const { return active; }
    int getActiveMotor() const { return activeMotor; }
    int getTestPwm() const { return testPwm; }
    uint32_t getSessionId() const { return sessionId; }
    bool getSafetyAck() const { return safetyAck; }
    uint32_t getRemainingTimeoutMs() const;
    bool isDeadmanActive() const;

private:
    bool active;
    int activeMotor; // 0..3
    int testPwm;
    uint32_t sessionId;
    bool safetyAck;
    
    uint32_t lastCommandTimeMs;
    uint32_t sessionStartTimeMs;
    
    const uint32_t deadmanTimeoutMs = 500; // 500ms deadman refresh required
    const uint32_t sessionMaxDurationMs = 30000; // 30s maximum session duration
};

#endif // MAINTENANCE_MANAGER_H
