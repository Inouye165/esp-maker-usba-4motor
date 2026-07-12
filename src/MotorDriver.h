#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

constexpr bool PHASE4A1_NORMAL_DRIVE_OUTPUT_DISABLED = false; // Normal drive physical outputs enabled for floor testing

enum class MotorOutputMode {
    LOCKED,
    SINGLE_MOTOR_MAINTENANCE,
    REAL_CALIBRATION,
    NORMAL_DRIVE,
    EMERGENCY_STOP,
    FAULTED
};

class MotorDriver {
public:
    MotorDriver();
    void begin();
    
    // Set raw motor PWM (-255 to 255)
    // index: 0=M1 (LF), 1=M2 (RF), 2=M3 (LR), 3=M4 (RR)
    void setPWM(int index, int pwm);
    
    // Hard stop all motors immediately
    void emergencyStop();

    // Mode and authorization settings
    void setMode(MotorOutputMode mode, int authorizedMotor = -1);
    MotorOutputMode getMode() const { return currentMode; }
    int getAuthorizedMotor() const { return authorizedMotorIndex; }
    bool allows(int motorIndex, int pwm) const;

private:
    struct MotorPinMap {
        int in1;
        int in2;
        int chan1;
        int chan2;
        bool invert;
    };

    MotorPinMap motors[4];
    MotorOutputMode currentMode;
    int authorizedMotorIndex;
};

#endif // MOTOR_DRIVER_H
