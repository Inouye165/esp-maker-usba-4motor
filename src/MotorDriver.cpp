#include "MotorDriver.h"
#include "RoverConfig.h"

// ESP32 LEDC PWM core version compatibility macro helpers
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  #define setupLEDC(pin, freq, res, chan) ledcAttachChannel(pin, freq, res, chan)
  #define writeLEDC(pin, chan, val)       ledcWrite(pin, val)
#else
  #define setupLEDC(pin, freq, res, chan) { ledcSetup(chan, freq, res); ledcAttachPin(pin, chan); }
  #define writeLEDC(pin, chan, val)       ledcWrite(chan, val)
#endif

MotorDriver::MotorDriver() 
    : currentMode(MotorOutputMode::LOCKED)
    , authorizedMotorIndex(-1)
    , actuationState(DrivetrainActuationState::COAST) {
    // Chassis rebuild 2026-10-18: motors are plugged into different board ports than
    // the original chassis and every motor is mounted mirrored (spins backward on a
    // positive command). Logical index (LF, RF, LR, RR) is remapped to the port each
    // wheel is actually on, with per-port polarity verified by raised-wheel tests.
    // Logical 0 (Left Front)  -> board port M2
    motors[0] = {M2_IN1, M2_IN2, 2, 3, false};
    // Logical 1 (Right Front) -> board port M1
    motors[1] = {M1_IN1, M1_IN2, 0, 1, true};
    // Logical 2 (Left Rear)   -> board port M4
    motors[2] = {M4_IN1, M4_IN2, 6, 7, true};
    // Logical 3 (Right Rear)  -> board port M3
    motors[3] = {M3_IN1, M3_IN2, 4, 5, true};
}

void MotorDriver::begin() {
    for (int i = 0; i < 4; i++) {
        pinMode(motors[i].in1, OUTPUT);
        pinMode(motors[i].in2, OUTPUT);
        digitalWrite(motors[i].in1, LOW);
        digitalWrite(motors[i].in2, LOW);
        
        // 1000Hz frequency, 8-bit resolution
        setupLEDC(motors[i].in1, 1000, 8, motors[i].chan1);
        setupLEDC(motors[i].in2, 1000, 8, motors[i].chan2);
        
        writeLEDC(motors[i].in1, motors[i].chan1, 0);
        writeLEDC(motors[i].in2, motors[i].chan2, 0);
    }
    currentMode = MotorOutputMode::LOCKED;
    authorizedMotorIndex = -1;
    actuationState = DrivetrainActuationState::COAST;
}

void MotorDriver::setMode(MotorOutputMode mode, int authorizedMotor) {
    currentMode = mode;
    authorizedMotorIndex = authorizedMotor;
    if (mode == MotorOutputMode::LOCKED || mode == MotorOutputMode::EMERGENCY_STOP || mode == MotorOutputMode::FAULTED) {
        coastAll();
    }
    Serial.printf("[MotorDriver] Mode changed to %d (Authorized: %d)\n", (int)mode, authorizedMotor);
}

bool MotorDriver::allows(int motorIndex, int pwm) const {
    if (currentMode == MotorOutputMode::LOCKED) return false;
    if (currentMode == MotorOutputMode::EMERGENCY_STOP) return false;
    if (currentMode == MotorOutputMode::FAULTED) return false;
    
    if (currentMode == MotorOutputMode::SINGLE_MOTOR_MAINTENANCE) {
        return (motorIndex == authorizedMotorIndex);
    }
    if (currentMode == MotorOutputMode::REAL_CALIBRATION) {
        return (motorIndex == authorizedMotorIndex);
    }
    if (currentMode == MotorOutputMode::NORMAL_DRIVE) {
        if (PHASE4A1_NORMAL_DRIVE_OUTPUT_DISABLED) {
            return false;
        }
        return true;
    }
    return false;
}

void MotorDriver::setPWM(int index, int pwm) {
    if (index < 0 || index >= 4) return;
    
    // Guard: Normal PWM writes must NEVER overwrite active dynamic braking pulse
    if (actuationState == DrivetrainActuationState::BRAKE) {
        return;
    }
    
    // Safe output mode authorization guard
    if (!allows(index, pwm)) {
        writeLEDC(motors[index].in1, motors[index].chan1, 0);
        writeLEDC(motors[index].in2, motors[index].chan2, 0);
        return;
    }
    
    // Invert configured side
    int outputPwm = pwm;
    if (motors[index].invert) {
        outputPwm = -outputPwm;
    }
    
    // Clamp to valid 8-bit resolution
    outputPwm = constrain(outputPwm, -255, 255);
    
    if (outputPwm > 0) {
        writeLEDC(motors[index].in1, motors[index].chan1, outputPwm);
        writeLEDC(motors[index].in2, motors[index].chan2, 0);
        actuationState = DrivetrainActuationState::DRIVE;
    } else if (outputPwm < 0) {
        writeLEDC(motors[index].in1, motors[index].chan1, 0);
        writeLEDC(motors[index].in2, motors[index].chan2, -outputPwm);
        actuationState = DrivetrainActuationState::DRIVE;
    } else {
        writeLEDC(motors[index].in1, motors[index].chan1, 0);
        writeLEDC(motors[index].in2, motors[index].chan2, 0);
    }
}

void MotorDriver::brakeAll() {
    if (currentMode != MotorOutputMode::NORMAL_DRIVE) {
        coastAll();
        return;
    }
    // RZ7889 / SS6625E Truth Table: FI=1, BI=1 -> FO=0, BO=0 (Dynamic Brake to GND)
    // Applying 255 duty to both LEDC channels holds IN1 and IN2 HIGH (100% duty)
    for (int i = 0; i < 4; i++) {
        writeLEDC(motors[i].in1, motors[i].chan1, 255);
        writeLEDC(motors[i].in2, motors[i].chan2, 255);
    }
    actuationState = DrivetrainActuationState::BRAKE;
}

void MotorDriver::coastAll() {
    for (int i = 0; i < 4; i++) {
        writeLEDC(motors[i].in1, motors[i].chan1, 0);
        writeLEDC(motors[i].in2, motors[i].chan2, 0);
    }
    actuationState = DrivetrainActuationState::COAST;
}

void MotorDriver::emergencyStop() {
    currentMode = MotorOutputMode::EMERGENCY_STOP;
    authorizedMotorIndex = -1;
    coastAll();
}
