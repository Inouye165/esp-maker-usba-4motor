#include "CalibrationManager.h"
#include "MotorDriver.h"
#include "RoverConfig.h"

CalibrationManager::CalibrationManager() 
    : state(CAL_IDLE)
    , activeMotor(0)
    , testPwm(0)
    , nextIsFwd(true)
    , isSimulation(true)
    , sessionId(0)
    , safetyAck(false)
    , stateTimerTicks(0)
    , startTicks(0)
    , simStartTicks(0)
    , motorDirectionsVerified(false)
    , encoderDirectionsVerified(false)
    , maintenanceStopVerified(false)
    , emergencyStopVerified(false)
    , deadmanVerified(false) {
    memset(failureReason, 0, sizeof(failureReason));
    for (int i = 0; i < 4; i++) {
        fwdBreakaway[i] = 0;
        revBreakaway[i] = 0;
        simTicks[i] = 0;
    }
}

void CalibrationManager::begin() {
    state = CAL_IDLE;
    activeMotor = 0;
    testPwm = 0;
    nextIsFwd = true;
    isSimulation = true;
    sessionId = 0;
    safetyAck = false;
    memset(failureReason, 0, sizeof(failureReason));
    for (int i = 0; i < 4; i++) {
        simTicks[i] = 0;
        fwdBreakaway[i] = motorCalibrations[i].forwardBreakawayPwm;
        revBreakaway[i] = motorCalibrations[i].reverseBreakawayPwm;
    }
    motorDirectionsVerified = false;
    encoderDirectionsVerified = false;
    maintenanceStopVerified = false;
    emergencyStopVerified = false;
    deadmanVerified = false;
}

void CalibrationManager::startCalibration(bool ack, bool sim, uint32_t sessId) {
    if (state != CAL_IDLE) return;
    
    isSimulation = sim;
    sessionId = sessId;
    safetyAck = ack;
    activeMotor = 0;
    testPwm = 0;
    nextIsFwd = true;
    memset(failureReason, 0, sizeof(failureReason));
    
    for (int i = 0; i < 4; i++) {
        fwdBreakaway[i] = 0;
        revBreakaway[i] = 0;
        simTicks[i] = 0;
    }

    if (!ack) {
        failCalibration("No Safety Ack");
        return;
    }

    if (!sim) {
        // Step 8 readiness gates check
        if (!motorDirectionsVerified || !encoderDirectionsVerified || !maintenanceStopVerified || !emergencyStopVerified || !deadmanVerified) {
            failCalibration("GATE_LOCKED");
            return;
        }
    }

    state = CAL_PRECHECK;
    stateTimerTicks = 100; // 1 second precheck at 100Hz
    if (isSimulation) {
        LOG_SERIAL_PRINTF("[Calibration] SIMULATION START (Session: %u)\n", sessionId);
    } else {
        LOG_SERIAL_PRINTF("[Calibration] REAL START (Session: %u). WARNING: Wheels will spin!\n", sessionId);
    }
}

void CalibrationManager::cancelCalibration() {
    if (state == CAL_IDLE) return;
    state = CAL_ABORTED;
    stateTimerTicks = 50; // Hold aborted state for 0.5s
    testPwm = 0;
    LOG_SERIAL_PRINTLN("[Calibration] Cancelled/Aborted by user.");
}

void CalibrationManager::failCalibration(const char *reason) {
    state = CAL_FAILED;
    stateTimerTicks = 100; // Hold failed state for 1s
    testPwm = 0;
    strncpy(failureReason, reason, sizeof(failureReason) - 1);
    LOG_SERIAL_PRINTF("[Calibration] FAILED: %s\n", reason);
}

bool CalibrationManager::update(const int32_t *currentTicks, MotorDriver &driver) {
    if (state == CAL_IDLE) return false;
    
    bool changed = false;
    
    // Simulated breakaway thresholds
    const int simFwdThreshold[4] = {40, 42, 39, 41};
    const int simRevThreshold[4] = {44, 45, 43, 46};

    switch (state) {
        case CAL_PRECHECK:
            driver.setMode(MotorOutputMode::LOCKED, -1);
            if (stateTimerTicks > 0) {
                stateTimerTicks--;
            } else {
                state = CAL_COOLDOWN;
                stateTimerTicks = 50; // 0.5s cooldown
                nextIsFwd = true;
                activeMotor = 0;
                changed = true;
            }
            break;
            
        case CAL_COOLDOWN:
            driver.setMode(MotorOutputMode::LOCKED, -1);
            if (stateTimerTicks > 0) {
                stateTimerTicks--;
            } else {
                testPwm = 35; // Start breakaway search PWM
                if (isSimulation) {
                    simStartTicks = simTicks[activeMotor];
                } else {
                    startTicks = currentTicks[activeMotor];
                }
                
                if (nextIsFwd) {
                    state = CAL_MEASURING_FWD;
                    stateTimerTicks = 5; // Increment PWM every 5 ticks (50ms)
                    LOG_SERIAL_PRINTF("[Calibration] Motor %d FWD starting...\n", activeMotor + 1);
                } else {
                    state = CAL_MEASURING_REV;
                    stateTimerTicks = 5;
                    LOG_SERIAL_PRINTF("[Calibration] Motor %d REV starting...\n", activeMotor + 1);
                }
                changed = true;
            }
            break;
            
        case CAL_MEASURING_FWD:
            if (isSimulation) {
                driver.setMode(MotorOutputMode::LOCKED, -1);
                if (testPwm >= simFwdThreshold[activeMotor]) {
                    simTicks[activeMotor] += 2;
                }
                
                int32_t delta = abs(simTicks[activeMotor] - simStartTicks);
                if (delta >= 8) {
                    fwdBreakaway[activeMotor] = testPwm;
                    state = CAL_COOLDOWN;
                    stateTimerTicks = 50;
                    nextIsFwd = false;
                    changed = true;
                    LOG_SERIAL_PRINTF("[Calibration] Motor %d FWD breakaway (Simulated): %d\n", activeMotor + 1, testPwm);
                } else {
                    if (stateTimerTicks > 0) {
                        stateTimerTicks--;
                    } else {
                        testPwm++;
                        stateTimerTicks = 5;
                        if (testPwm > 210) {
                            failCalibration("FWD Limit Timeout");
                            changed = true;
                        }
                    }
                }
            } else {
                // Real calibration
                driver.setMode(MotorOutputMode::REAL_CALIBRATION, activeMotor);
                driver.setPWM(activeMotor, testPwm);
                
                if (abs(currentTicks[activeMotor] - startTicks) >= 8) {
                    fwdBreakaway[activeMotor] = testPwm;
                    driver.setPWM(activeMotor, 0);
                    state = CAL_COOLDOWN;
                    stateTimerTicks = 100; // 1.0s settle time on real motors
                    nextIsFwd = false;
                    changed = true;
                    LOG_SERIAL_PRINTF("[Calibration] Motor %d FWD breakaway detected: %d\n", activeMotor + 1, testPwm);
                } else {
                    if (stateTimerTicks > 0) {
                        stateTimerTicks--;
                    } else {
                        testPwm++;
                        stateTimerTicks = 10; // Increment every 100ms for physical stiction break search
                        if (testPwm > 210) {
                            fwdBreakaway[activeMotor] = 210;
                            driver.setPWM(activeMotor, 0);
                            state = CAL_COOLDOWN;
                            stateTimerTicks = 100;
                            nextIsFwd = false;
                            changed = true;
                            LOG_SERIAL_PRINTF("[Calibration] Motor %d FWD failed to break, default to 210\n", activeMotor + 1);
                        }
                    }
                }
            }
            break;
            
        case CAL_MEASURING_REV:
            if (isSimulation) {
                driver.setMode(MotorOutputMode::LOCKED, -1);
                if (testPwm >= simRevThreshold[activeMotor]) {
                    simTicks[activeMotor] -= 2;
                }
                
                int32_t delta = abs(simTicks[activeMotor] - simStartTicks);
                if (delta >= 8) {
                    revBreakaway[activeMotor] = testPwm;
                    activeMotor++;
                    if (activeMotor >= 4) {
                        state = CAL_DONE;
                        stateTimerTicks = 10;
                    } else {
                        state = CAL_COOLDOWN;
                        stateTimerTicks = 50;
                        nextIsFwd = true;
                    }
                    changed = true;
                    LOG_SERIAL_PRINTF("[Calibration] Motor %d REV breakaway (Simulated): %d\n", activeMotor, testPwm);
                } else {
                    if (stateTimerTicks > 0) {
                        stateTimerTicks--;
                    } else {
                        testPwm++;
                        stateTimerTicks = 5;
                        if (testPwm > 210) {
                            failCalibration("REV Limit Timeout");
                            changed = true;
                        }
                    }
                }
            } else {
                // Real calibration
                driver.setMode(MotorOutputMode::REAL_CALIBRATION, activeMotor);
                driver.setPWM(activeMotor, -testPwm);
                
                if (abs(currentTicks[activeMotor] - startTicks) >= 8) {
                    revBreakaway[activeMotor] = testPwm;
                    driver.setPWM(activeMotor, 0);
                    activeMotor++;
                    if (activeMotor >= 4) {
                        state = CAL_DONE;
                        stateTimerTicks = 10;
                    } else {
                        state = CAL_COOLDOWN;
                        stateTimerTicks = 100;
                        nextIsFwd = true;
                    }
                    changed = true;
                    LOG_SERIAL_PRINTF("[Calibration] Motor %d REV breakaway detected: %d\n", activeMotor, testPwm);
                } else {
                    if (stateTimerTicks > 0) {
                        stateTimerTicks--;
                    } else {
                        testPwm++;
                        stateTimerTicks = 10;
                        if (testPwm > 210) {
                            revBreakaway[activeMotor] = 210;
                            driver.setPWM(activeMotor, 0);
                            activeMotor++;
                            if (activeMotor >= 4) {
                                state = CAL_DONE;
                                stateTimerTicks = 10;
                            } else {
                                state = CAL_COOLDOWN;
                                stateTimerTicks = 100;
                                nextIsFwd = true;
                            }
                            changed = true;
                            LOG_SERIAL_PRINTF("[Calibration] Motor %d REV failed to break, default to 210\n", activeMotor);
                        }
                    }
                }
            }
            break;
            
        case CAL_DONE:
            driver.setMode(MotorOutputMode::LOCKED, -1);
            if (isSimulation) {
                LOG_SERIAL_PRINTLN("[Calibration] SIMULATION DONE. Breakaway limits (not persisted to NVS):");
                for (int i = 0; i < 4; i++) {
                    LOG_SERIAL_PRINTF("  Motor %d: FWD=%d, REV=%d\n", i + 1, fwdBreakaway[i], revBreakaway[i]);
                }
            } else {
                // Persist real calibration values to preferences (NVS)
                for (int i = 0; i < 4; i++) {
                    motorCalibrations[i].forwardBreakawayPwm = fwdBreakaway[i];
                    motorCalibrations[i].reverseBreakawayPwm = revBreakaway[i];
                }
                saveCalibrations();
                LOG_SERIAL_PRINTLN("[Calibration] REAL DONE. Persisted breakaway limits to NVS.");
            }
            state = CAL_IDLE;
            changed = true;
            break;

        case CAL_ABORTED:
        case CAL_FAILED:
            driver.setMode(MotorOutputMode::LOCKED, -1);
            if (stateTimerTicks > 0) {
                stateTimerTicks--;
            } else {
                state = CAL_IDLE;
                changed = true;
            }
            break;
    }
    
    return changed;
}

void CalibrationManager::getStatusMessage(char *buf, size_t len) const {
    switch (state) {
        case CAL_IDLE: snprintf(buf, len, "Idle"); break;
        case CAL_PRECHECK: snprintf(buf, len, "Prechecking..."); break;
        case CAL_MEASURING_FWD: snprintf(buf, len, "M%d FWD (PWM=%d)", activeMotor + 1, testPwm); break;
        case CAL_MEASURING_REV: snprintf(buf, len, "M%d REV (PWM=%d)", activeMotor + 1, testPwm); break;
        case CAL_COOLDOWN: snprintf(buf, len, "Cooldown M%d", activeMotor + 1); break;
        case CAL_DONE: snprintf(buf, len, "Complete!"); break;
        case CAL_ABORTED: snprintf(buf, len, "Aborted"); break;
        case CAL_FAILED: snprintf(buf, len, "Failed (%s)", failureReason); break;
        default: snprintf(buf, len, "Unknown"); break;
    }
}

int CalibrationManager::getSimulatedDelta() const {
    if (state == CAL_MEASURING_FWD || state == CAL_MEASURING_REV) {
        return abs(simTicks[activeMotor] - simStartTicks);
    }
    return 0;
}
