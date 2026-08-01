#include "MaintenanceManager.h"

MaintenanceManager::MaintenanceManager()
    : active(false)
    , activeMotor(-1)
    , testPwm(0)
    , sessionId(0)
    , safetyAck(false)
    , lastCommandTimeMs(0)
    , sessionStartTimeMs(0) {}

void MaintenanceManager::begin() {
    active = false;
    activeMotor = -1;
    testPwm = 0;
    sessionId = 0;
    safetyAck = false;
    lastCommandTimeMs = 0;
    sessionStartTimeMs = 0;
}

bool MaintenanceManager::enter(bool ack, int motorIdx, uint32_t sessId, MotorDriver &driver) {
    if (active) return false;
    if (driver.getMode() == MotorOutputMode::EMERGENCY_STOP) {
        Serial.println("[Maintenance] Rejecting enter: Emergency Stop is active!");
        return false;
    }
    if (motorIdx < 0 || motorIdx >= 4) {
        Serial.printf("[Maintenance] Rejecting enter: invalid motor index %d\n", motorIdx);
        return false;
    }
    if (!ack) {
        Serial.println("[Maintenance] Rejecting enter: safety acknowledgement required!");
        return false;
    }
    
    active = true;
    activeMotor = motorIdx;
    testPwm = 0;
    sessionId = sessId;
    safetyAck = ack;
    
    sessionStartTimeMs = millis();
    lastCommandTimeMs = millis();
    
    driver.setMode(MotorOutputMode::SINGLE_MOTOR_MAINTENANCE, motorIdx);
    Serial.printf("[Maintenance] ENTERED Session: %u on Motor: %d\n", sessId, motorIdx + 1);
    return true;
}

void MaintenanceManager::exit(MotorDriver &driver) {
    if (!active) return;
    active = false;
    activeMotor = -1;
    testPwm = 0;
    
    driver.setMode(MotorOutputMode::LOCKED, -1);
    Serial.println("[Maintenance] EXITED. Outputs locked.");
}

void MaintenanceManager::setOutput(int pwm) {
    if (!active) return;
    
    // Central ceiling safety limit: max 60 PWM magnitude for safe low maintenance cap
    int maxCeiling = 60;
    testPwm = constrain(pwm, -maxCeiling, maxCeiling);
    
    lastCommandTimeMs = millis();
}

void MaintenanceManager::update(MotorDriver &driver) {
    if (!active) return;
    
    if (driver.getMode() == MotorOutputMode::EMERGENCY_STOP) {
        Serial.println("[Maintenance] Watchdog trigger: EMERGENCY_STOP active! Stopping output.");
        exit(driver);
        return;
    }
    
    uint32_t now = millis();
    
    // Deadman timeout check
    if (now - lastCommandTimeMs > deadmanTimeoutMs) {
        Serial.println("[Maintenance] Watchdog trigger: Deadman refresh timeout! Stopping output.");
        exit(driver);
        return;
    }
    
    // Max session duration check
    if (now - sessionStartTimeMs > sessionMaxDurationMs) {
        Serial.println("[Maintenance] Watchdog trigger: Max session duration reached! Stopping output.");
        exit(driver);
        return;
    }
    
    // Write authorized motor PWM, and explicitly command inactive motors to 0 PWM
    for (int i = 0; i < 4; i++) {
        if (i == activeMotor) {
            driver.setPWM(i, testPwm);
        } else {
            driver.setPWM(i, 0);
        }
    }
}

uint32_t MaintenanceManager::getRemainingTimeoutMs() const {
    if (!active) return 0;
    uint32_t elapsed = millis() - sessionStartTimeMs;
    if (elapsed >= sessionMaxDurationMs) return 0;
    return sessionMaxDurationMs - elapsed;
}

bool MaintenanceManager::isDeadmanActive() const {
    if (!active) return false;
    return (millis() - lastCommandTimeMs <= deadmanTimeoutMs);
}
