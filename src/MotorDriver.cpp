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
    , authorizedMotorIndex(-1) {
    // M1: Left Front
    motors[0] = {M1_IN1, M1_IN2, 0, 1, false};
    // M2: Right Front
    motors[1] = {M2_IN1, M2_IN2, 2, 3, true};
    // M3: Left Rear
    motors[2] = {M3_IN1, M3_IN2, 4, 5, false};
    // M4: Right Rear
    motors[3] = {M4_IN1, M4_IN2, 6, 7, false};
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
}

void MotorDriver::setMode(MotorOutputMode mode, int authorizedMotor) {
    currentMode = mode;
    authorizedMotorIndex = authorizedMotor;
    if (mode == MotorOutputMode::LOCKED || mode == MotorOutputMode::EMERGENCY_STOP || mode == MotorOutputMode::FAULTED) {
        // Immediately zero all outputs
        for (int i = 0; i < 4; i++) {
            writeLEDC(motors[i].in1, motors[i].chan1, 0);
            writeLEDC(motors[i].in2, motors[i].chan2, 0);
        }
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
    } else if (outputPwm < 0) {
        writeLEDC(motors[index].in1, motors[index].chan1, 0);
        writeLEDC(motors[index].in2, motors[index].chan2, -outputPwm);
    } else {
        writeLEDC(motors[index].in1, motors[index].chan1, 0);
        writeLEDC(motors[index].in2, motors[index].chan2, 0);
    }
}

void MotorDriver::emergencyStop() {
    currentMode = MotorOutputMode::EMERGENCY_STOP;
    authorizedMotorIndex = -1;
    for (int i = 0; i < 4; i++) {
        writeLEDC(motors[i].in1, motors[i].chan1, 0);
        writeLEDC(motors[i].in2, motors[i].chan2, 0);
    }
}
