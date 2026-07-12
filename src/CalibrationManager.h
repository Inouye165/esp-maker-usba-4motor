#ifndef CALIBRATION_MANAGER_H
#define CALIBRATION_MANAGER_H

#include <Arduino.h>

enum CalibrationState {
    CAL_IDLE,
    CAL_PRECHECK,      // Warning and safety acknowledgement validation
    CAL_MEASURING_FWD,
    CAL_MEASURING_REV,
    CAL_COOLDOWN,      // Pause between direction changes
    CAL_DONE,
    CAL_ABORTED,
    CAL_FAILED
};

class CalibrationManager {
public:
    CalibrationManager();
    void begin();
    
    // Start calibration sequence
    void startCalibration(bool ack, bool sim, uint32_t sessId);
    void cancelCalibration();
    void failCalibration(const char *reason);
    
    // Process step (called at loop frequency, 100Hz)
    // returns true if calibration state changed
    bool update(const int32_t *currentTicks, class MotorDriver &driver);
    
    CalibrationState getState() const { return state; }
    int getActiveMotor() const { return activeMotor; }
    int getCurrentPwm() const { return testPwm; }
    bool getIsSimulation() const { return isSimulation; }
    uint32_t getSessionId() const { return sessionId; }
    bool getSafetyAck() const { return safetyAck; }
    const char* getFailureReason() const { return failureReason; }
    
    int getForwardBreakaway(int motorIndex) const { return (motorIndex >= 0 && motorIndex < 4) ? fwdBreakaway[motorIndex] : 0; }
    int getReverseBreakaway(int motorIndex) const { return (motorIndex >= 0 && motorIndex < 4) ? revBreakaway[motorIndex] : 0; }
    int getSimulatedDelta() const;
    void getStatusMessage(char *buf, size_t len) const;

    // Readiness gate verification flags (Step 8)
    bool motorDirectionsVerified;
    bool encoderDirectionsVerified;
    bool maintenanceStopVerified;
    bool emergencyStopVerified;
    bool deadmanVerified;

private:
    CalibrationState state;
    int activeMotor; // 0..3
    int testPwm;
    bool nextIsFwd;  // Handles transition between fwd and rev measurements
    bool isSimulation;
    uint32_t sessionId;
    bool safetyAck;
    char failureReason[32];
    
    uint32_t stateTimerTicks;
    int32_t startTicks;
    
    // Simulation state variables
    int32_t simTicks[4];
    int32_t simStartTicks;
    
    // Captured limits
    int fwdBreakaway[4];
    int revBreakaway[4];
};

#endif // CALIBRATION_MANAGER_H
