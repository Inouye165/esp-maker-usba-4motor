// Rebuild triggered: Updated GEAR_RATIO parameters to 45.0f
#include "SerialProtocol.h"
#include "RoverConfig.h"
#include "CommandManager.h"
#include "CalibrationManager.h"
#include "SafetyManager.h"
#include "DifferentialDrive.h"
#include "MotorDriver.h"
#include "MaintenanceManager.h"
#include "MotionLimiter.h"

extern MotorDriver motorDriver;

SerialProtocol::SerialProtocol()
    : parserState(WAIT_HEAD), extLen(0), payloadIdx(0), lastCharTimeMs(0),
      imuSequenceNum(0), imuTxDropped(0), _pendingFaultReport(false), _pendingFaultFlags(0) {}

bool SerialProtocol::update(CommandManager &cmdManager, CalibrationManager &calManager, MaintenanceManager &maintenanceManager, SafetyManager &safetyManager, ControlLoopStats &stats) {
    bool commandReceived = false;
    uint32_t now = millis();
    
    // Parser timeout: reset state machine if transmission hangs for >100ms
    if (parserState != WAIT_HEAD && (now - lastCharTimeMs > 100)) {
        parserState = WAIT_HEAD;
    }
    
    while (Serial.available() > 0) {
        uint8_t b = Serial.read();
        lastCharTimeMs = now;
        
        switch (parserState) {
            case WAIT_HEAD:
                if (b == 0xFF) {
                    parserState = WAIT_DEVICE;
                }
                break;
                
            case WAIT_DEVICE:
                if (b == 0xFC) {
                    parserState = WAIT_LEN;
                } else if (b != 0xFF) {
                    parserState = WAIT_HEAD;
                }
                break;
                
            case WAIT_LEN:
                extLen = b;
                if (extLen > sizeof(payloadBuf) || extLen < 2) {
                    parserState = WAIT_HEAD;
                } else {
                    payloadIdx = 0;
                    parserState = WAIT_PAYLOAD;
                }
                break;
                
            case WAIT_PAYLOAD:
                payloadBuf[payloadIdx++] = b;
                if (payloadIdx >= extLen) {
                    processPacket(cmdManager, calManager, maintenanceManager, safetyManager, stats);
                    parserState = WAIT_HEAD;
                    commandReceived = true;
                }
                break;
        }
    }
    
    return commandReceived;
}

void SerialProtocol::processPacket(CommandManager &cmdManager, CalibrationManager &calManager, MaintenanceManager &maintenanceManager, SafetyManager &safetyManager, ControlLoopStats &stats) {
    uint8_t funcId = payloadBuf[0];
    uint8_t receivedChecksum = payloadBuf[extLen - 1];

    uint16_t sum = extLen;
    for (int i = 0; i < extLen - 1; i++) {
        sum += payloadBuf[i];
    }
    uint8_t calculatedChecksum = sum & 0xFF;

    if (calculatedChecksum != receivedChecksum) {
        return;
    }

    cmdManager.resetWatchdog();

    switch (funcId) {
        case 0x10: { // CMD_MOTOR (Set individual speeds -100..100)
            if (extLen >= 6) {
                if (maintenanceManager.isActive() || calManager.getState() != CAL_IDLE) break;
                
                int8_t m1 = (int8_t)payloadBuf[1];
                int8_t m2 = (int8_t)payloadBuf[2];
                int8_t m3 = (int8_t)payloadBuf[3];
                int8_t m4 = (int8_t)payloadBuf[4];
                
                float maxWheelRadps = MAX_LINEAR_VELOCITY_MPS / WHEEL_RADIUS_M;
                float w1 = (m1 / 100.0f) * maxWheelRadps;
                float w2 = (m2 / 100.0f) * maxWheelRadps;
                float w3 = (m3 / 100.0f) * maxWheelRadps;
                float w4 = (m4 / 100.0f) * maxWheelRadps;
                
                float linear = 0.0f;
                float angular = 0.0f;
                DifferentialDrive::wheelsToVelocity(w1, w2, w3, w4, linear, angular);
                
                cmdManager.setCommand(linear, angular, SOURCE_USB_SERIAL);
            }
            break;
        }
        
        case 0x12: { // CMD_MOTION (vx, vy, vz as int16 LE * 1000)
            if (extLen >= 8) {
                if (maintenanceManager.isActive() || calManager.getState() != CAL_IDLE) break;
                
                int16_t vx = (int16_t)(payloadBuf[1] | (payloadBuf[2] << 8));
                int16_t vy = (int16_t)(payloadBuf[3] | (payloadBuf[4] << 8));
                int16_t vz = (int16_t)(payloadBuf[5] | (payloadBuf[6] << 8));
                
                float linear = (float)vx / 1000.0f;
                float angular = (float)vz / 1000.0f;
                
                cmdManager.setCommand(linear, angular, SOURCE_ROS);
            }
            break;
        }
        
        case 0x20: { // CMD_START_CALIBRATION_SIMULATION
            if (extLen >= 7) {
                bool safetyAck = (payloadBuf[1] == 1);
                bool simFlag = (payloadBuf[2] == 1);
                uint32_t sessId = (payloadBuf[3] | (payloadBuf[4] << 8) | (payloadBuf[5] << 16) | (payloadBuf[6] << 24));
                
                // Safety check: force simulation in current phase
                calManager.startCalibration(safetyAck, simFlag, sessId);
                cmdManager.requestSourceChange(SOURCE_CALIBRATION, 0.0f, 0.0f);
            }
            break;
        }
        
        case 0x21: { // CMD_ABORT_CALIBRATION
            calManager.cancelCalibration();
            cmdManager.requestSourceChange(SOURCE_NONE, 0.0f, 0.0f);
            break;
        }
        
        case 0x22: { // CMD_CLEAR_FAULTS
            safetyManager.clearFaults();
            cmdManager.clearEmergencyStop();
            Serial.println("[Safety] Faults cleared via serial command.");
            break;
        }
        
        case 0x23: { // CMD_GET_FIRMWARE_INFO
            sendFirmwareInfo();
            break;
        }
        
        case 0x24: { // CMD_RESET_TIMING_STATS
            stats.lastDurationUs = 0;
            stats.minDurationUs = 9999999;
            stats.avgDurationUs = 0;
            stats.maxDurationUs = 0;
            stats.missedDeadlines = 0;
            stats.totalIterations = 0;
            LOG_SERIAL_PRINTLN("[Stats] Control loop timing statistics reset.");
            break;
        }

        case 0x25: { // CMD_START_CALIBRATION_REAL
            if (extLen >= 7) {
                uint32_t sessId = (payloadBuf[3] | (payloadBuf[4] << 8) | (payloadBuf[5] << 16) | (payloadBuf[6] << 24));
                calManager.startCalibration(true, false, sessId);
                cmdManager.requestSourceChange(SOURCE_CALIBRATION, 0.0f, 0.0f);
            }
            break;
        }

        case 0x26: { // CMD_ENTER_MAINTENANCE
            if (extLen >= 7) {
                bool safetyAck = (payloadBuf[1] == 1);
                int motorIdx = (int)payloadBuf[2];
                uint32_t sessId = (payloadBuf[3] | (payloadBuf[4] << 8) | (payloadBuf[5] << 16) | (payloadBuf[6] << 24));
                
                if (calManager.getState() == CAL_IDLE) {
                    maintenanceManager.enter(safetyAck, motorIdx, sessId, motorDriver);
                }
            }
            break;
        }

        case 0x27: { // CMD_MAINTENANCE_SET_OUTPUT
            if (extLen >= 11) {
                uint32_t sessId = (payloadBuf[3] | (payloadBuf[4] << 8) | (payloadBuf[5] << 16) | (payloadBuf[6] << 24));
                int motorIdx = (int)payloadBuf[7];
                int dir = (int)payloadBuf[8];
                int rawPwm = (int)payloadBuf[9];
                bool enabled = (payloadBuf[10] == 1);
                
                if (maintenanceManager.isActive() && maintenanceManager.getSessionId() == sessId && motorIdx == maintenanceManager.getActiveMotor()) {
                    if (enabled) {
                        int pwm = (dir == 0) ? rawPwm : -rawPwm;
                        maintenanceManager.setOutput(pwm);
                    } else {
                        maintenanceManager.setOutput(0);
                    }
                }
            }
            break;
        }

        case 0x28: { // CMD_EXIT_MAINTENANCE
            maintenanceManager.exit(motorDriver);
            break;
        }

        case 0x29: { // CMD_EMERGENCY_STOP
            cmdManager.triggerEmergencyStop();
            motorDriver.emergencyStop();
            maintenanceManager.exit(motorDriver);
            calManager.cancelCalibration();
            Serial.println("[Safety] Emergency Stop triggered via serial command.");
            break;
        }

        case 0x2A: { // CMD_VERIFY_READY
            if (extLen >= 7) {
                calManager.motorDirectionsVerified = (payloadBuf[1] == 1);
                calManager.encoderDirectionsVerified = (payloadBuf[2] == 1);
                calManager.maintenanceStopVerified = (payloadBuf[3] == 1);
                calManager.emergencyStopVerified = (payloadBuf[4] == 1);
                calManager.deadmanVerified = (payloadBuf[5] == 1);
                Serial.printf("[Calibration] Readiness gates updated: MotorDir=%d, EncDir=%d, MaintStop=%d, EStop=%d, Deadman=%d\n",
                    calManager.motorDirectionsVerified, calManager.encoderDirectionsVerified,
                    calManager.maintenanceStopVerified, calManager.emergencyStopVerified, calManager.deadmanVerified);
            }
            break;
        }

        case 0x2C: { // CMD_ARM_NORMAL_DRIVE
            if (!maintenanceManager.isActive() && calManager.getState() == CAL_IDLE && !safetyManager.hasFaults() && !cmdManager.isEmergencyStopped()) {
                if (cmdManager.armNormalDrive()) {
                    motorDriver.setMode(MotorOutputMode::NORMAL_DRIVE);
                    Serial.println("[Command] NORMAL_DRIVE ARMED successfully.");
                } else {
                    Serial.println("[Command] Rejecting ARM: safety locks active.");
                }
            } else {
                Serial.println("[Command] Rejecting ARM: calibration, maintenance, faults, or E-STOP active.");
            }
            break;
        }

        case 0x2D: { // CMD_DISARM_NORMAL_DRIVE
            cmdManager.disarmNormalDrive();
            Serial.println("[Command] NORMAL_DRIVE DISARMED. Initiating controlled stop.");
            break;
        }

        case 0x37: { // CMD_ROVER_PARAMS (Set or Query)
            if (extLen >= 9) {
                float newDiameter = 0.0f;
                float newSeparation = 0.0f;
                memcpy(&newDiameter, &payloadBuf[1], 4);
                memcpy(&newSeparation, &payloadBuf[5], 4);
                
                // Update active params
                WHEEL_DIAMETER_M = newDiameter;
                WHEEL_RADIUS_M = newDiameter / 2.0f;
                if (newSeparation >= 0.100f && newSeparation <= 0.500f) {
                    WHEEL_SEPARATION_M = newSeparation;
                    preferences.putFloat("wheel_sep", newSeparation);
                    Serial.printf("[Config] Dynamic params saved to NVS: diameter=%.4f m, separation=%.4f m\n", newDiameter, newSeparation);
                } else {
                    Serial.printf("[Config ERROR] Rejected out-of-range wheel separation %.4fm (must be 0.100m to 0.500m)\n", newSeparation);
                }
            }
            
            // Send back current active parameters
            uint8_t respData[8];
            memcpy(&respData[0], &WHEEL_DIAMETER_M, 4);
            memcpy(&respData[4], &WHEEL_SEPARATION_M, 4);
            writePacket(0x37, respData, 8);
            break;
        }

        case 0x38: { // CMD_SET_TRIM (Set or Query forward straight trims)
            if (extLen >= 9) {
                float newLeftTrim = 1.00f;
                float newRightTrim = 1.00f;
                memcpy(&newLeftTrim, &payloadBuf[1], 4);
                memcpy(&newRightTrim, &payloadBuf[5], 4);
                
                // Bounds validation [0.80, 1.20]
                if (newLeftTrim >= 0.80f && newLeftTrim <= 1.20f &&
                    newRightTrim >= 0.80f && newRightTrim <= 1.20f) {
                    saveTrimsFwd(newLeftTrim, newRightTrim);
                } else {
                    Serial.printf("[Protocol] Rejected invalid FWD trims: Left=%.4f, Right=%.4f (bounds: [0.8, 1.2])\n", newLeftTrim, newRightTrim);
                }
            }
            
            // Reply back with active forward trims
            uint8_t respData[8];
            memcpy(&respData[0], &LEFT_TRIM_FWD, 4);
            memcpy(&respData[4], &RIGHT_TRIM_FWD, 4);
            writePacket(0x38, respData, 8);
            break;
        }

        case 0x39: { // CMD_SET_TRIM_REV (Set or Query reverse straight trims)
            if (extLen >= 9) {
                float newLeftTrim = 1.00f;
                float newRightTrim = 1.00f;
                memcpy(&newLeftTrim, &payloadBuf[1], 4);
                memcpy(&newRightTrim, &payloadBuf[5], 4);
                
                // Bounds validation [0.80, 1.20]
                if (newLeftTrim >= 0.80f && newLeftTrim <= 1.20f &&
                    newRightTrim >= 0.80f && newRightTrim <= 1.20f) {
                    saveTrimsRev(newLeftTrim, newRightTrim);
                } else {
                    Serial.printf("[Protocol] Rejected invalid REV trims: Left=%.4f, Right=%.4f (bounds: [0.8, 1.2])\n", newLeftTrim, newRightTrim);
                }
            }
            
            // Reply back with active reverse trims
            uint8_t respData[8];
                memcpy(&respData[0], &LEFT_TRIM_REV, 4);
            memcpy(&respData[4], &RIGHT_TRIM_REV, 4);
            writePacket(0x39, respData, 8);
            break;
        }
    }
}

#include "SerialProtocol.h"
#include "RoverConfig.h"
#include "CommandManager.h"
#include "CalibrationManager.h"
#include "SafetyManager.h"
#include "DifferentialDrive.h"
#include "MotorDriver.h"
#include "MaintenanceManager.h"
#include "MotionLimiter.h"
#include "ImuManager.h"

extern MotorDriver motorDriver;

void SerialProtocol::begin() {
    Serial.setTxBufferSize(512); // Must be called BEFORE Serial.begin()
    Serial.begin(115200);
    parserState = WAIT_HEAD;
    lastCharTimeMs = millis();
}

bool SerialProtocol::canWrite(uint8_t wireSize) {
    return (Serial.availableForWrite() >= (int)wireSize);
}

void SerialProtocol::writePacket(uint8_t extType, const uint8_t *data, uint8_t dataLen) {
    constexpr size_t MAX_FRAME_SIZE = 128;
    const uint8_t outExtLen = dataLen + 3;
    const uint8_t frameLen = dataLen + 5;

    if (frameLen > MAX_FRAME_SIZE) {
        return; // Safety guard against buffer overflow
    }

    // Non-blocking TX safety guard: never write if UART ring buffer cannot accommodate entire frame
    if (!canWrite(frameLen)) {
        return; // Drop/defer non-blockingly without partial transmission
    }

    // Construct complete wire frame in contiguous buffer
    uint8_t frameBuf[MAX_FRAME_SIZE];
    frameBuf[0] = 0xFF;
    frameBuf[1] = 0xFB;
    frameBuf[2] = outExtLen;
    frameBuf[3] = extType;

    uint8_t sum = outExtLen + extType;
    for (uint8_t i = 0; i < dataLen; i++) {
        frameBuf[4 + i] = data[i];
        sum += data[i];
    }
    frameBuf[4 + dataLen] = sum & 0xFF;

    // Execute exactly ONE bulk buffer write
    Serial.write(frameBuf, frameLen);
}

uint8_t SerialProtocol::serializeImuTelemetry(
    uint8_t *p,
    uint32_t seq,
    const ImuData &d,
    int64_t snapUs,
    const ImuManager &imuManager
) {
    memset(p, 0, 69);

    p[0] = 0x01; // protocol_version

    uint16_t flags = imuManager.getStatusFlags(snapUs);
    memcpy(&p[1], &flags, 2);

    memcpy(&p[3], &seq, 4);

    uint32_t resetCount = d.resetCount;
    memcpy(&p[7], &resetCount, 4);

    uint64_t snapUs64 = (uint64_t)snapUs;
    memcpy(&p[11], &snapUs64, 8);

    uint16_t rotAge = imuManager.getRotVecAgeMs(snapUs);
    memcpy(&p[19], &rotAge, 2);

    uint16_t gyroAge = imuManager.getGyroAgeMs(snapUs);
    memcpy(&p[21], &gyroAge, 2);

    uint16_t accelAge = imuManager.getAccelAgeMs(snapUs);
    memcpy(&p[23], &accelAge, 2);

    memcpy(&p[25], &d.qw, 4);
    memcpy(&p[29], &d.qx, 4);
    memcpy(&p[33], &d.qy, 4);
    memcpy(&p[37], &d.qz, 4);

    memcpy(&p[41], &d.gx, 4);
    memcpy(&p[45], &d.gy, 4);
    memcpy(&p[49], &d.gz, 4);

    // SH2_ACCELEROMETER (gravity included, REP-145 compliant)
    memcpy(&p[53], &d.raw_ax, 4);
    memcpy(&p[57], &d.raw_ay, 4);
    memcpy(&p[61], &d.raw_az, 4);

    memcpy(&p[65], &d.quatRadAccuracy, 4);

    return 69;
}

static TelemetryTxProf g_txProf;

void SerialProtocol::sendImuTelemetry(const ImuManager &imuManager) {
    uint64_t tStart = esp_timer_get_time();

    // Increment sequence counter for every scheduled 50 Hz attempt BEFORE capacity check
    uint32_t seq = imuSequenceNum++;

    const uint8_t wireSize = 74;
    if (!canWrite(wireSize)) {
        imuTxDropped++;
        return; // Drop non-blockingly
    }

    const ImuData &d = imuManager.getData();
    int64_t snapUs = esp_timer_get_time();

    uint8_t p[69];
    serializeImuTelemetry(p, seq, d, snapUs, imuManager);

    writePacket(0x3A, p, 69);

    uint64_t tEnd = esp_timer_get_time();
    g_txProf.p0x3A_imu.record((uint32_t)(tEnd - tStart));
}

void SerialProtocol::sendTelemetry(
    const int32_t *ticks,
    float batteryVolts,
    float currentYawRate,
    const CalibrationManager &calManager,
    const MaintenanceManager &maintenanceManager,
    const ControlLoopStats &stats,
    uint32_t faultFlags
) {
    // 1. Encoder packet (TYPE_ENCODER = 0x0D)
    uint64_t t0 = esp_timer_get_time();
    uint8_t encoderData[16];
    memcpy(&encoderData[0],  &ticks[0], 4);
    memcpy(&encoderData[4],  &ticks[1], 4);
    memcpy(&encoderData[8],  &ticks[2], 4);
    memcpy(&encoderData[12], &ticks[3], 4);
    writePacket(0x0D, encoderData, 16);
    uint64_t t1 = esp_timer_get_time();
    g_txProf.p0x0D_encoder.record((uint32_t)(t1 - t0));
    
    // 2. Battery packet (TYPE_BATTERY = 0x0A) - Throttled to 5 Hz (every 200ms)
    static unsigned long lastBatteryTxMs = 0;
    unsigned long nowMs = millis();
    if (nowMs - lastBatteryTxMs >= 200) {
        lastBatteryTxMs = nowMs;
        uint8_t batteryData[7] = {0, 0, 0, 0, 0, 0, 0};
        writePacket(0x0A, batteryData, 7);
    }
    uint64_t t2 = esp_timer_get_time();
    g_txProf.p0x0A_battery.record((uint32_t)(t2 - t1));

    // 3b. Maintenance telemetry status (TYPE_MAINTENANCE_STATUS = 0x35)
    uint8_t maintData[15];
    memset(maintData, 0, sizeof(maintData));
    maintData[0] = 1; // Major protocol version
    maintData[1] = 1; // Minor protocol version
    
    uint32_t maintSess = maintenanceManager.getSessionId();
    memcpy(&maintData[2], &maintSess, 4);
    
    maintData[6] = maintenanceManager.isActive() ? 1 : 0;
    maintData[7] = (uint8_t)maintenanceManager.getActiveMotor();
    maintData[8] = (uint8_t)(maintenanceManager.getActiveMotor() + 1); // Human readable motor number
    maintData[9] = (maintenanceManager.getTestPwm() < 0) ? 1 : 0; // Direction (0=FWD, 1=REV)
    maintData[10] = (uint8_t)abs(maintenanceManager.getTestPwm()); // Requested PWM magnitude
    
    int actualPwm = 0;
    if (motorDriver.getMode() == MotorOutputMode::SINGLE_MOTOR_MAINTENANCE && motorDriver.getAuthorizedMotor() == maintenanceManager.getActiveMotor()) {
        actualPwm = abs(maintenanceManager.getTestPwm());
    }
    maintData[11] = (uint8_t)actualPwm; // Actual PWM magnitude
    maintData[12] = maintenanceManager.isDeadmanActive() ? 1 : 0;
    
    uint32_t remTimeout = maintenanceManager.getRemainingTimeoutMs();
    memcpy(&maintData[13], &remTimeout, 2);
    
    writePacket(0x35, maintData, 15);
    uint64_t t3 = esp_timer_get_time();
    g_txProf.p0x35_maint.record((uint32_t)(t3 - t2));

    // 4. Calibration telemetry packet (TYPE_CALIBRATION_STATUS = 0x30)
    uint8_t calData[55];
    memset(calData, 0, sizeof(calData));
    calData[0] = 1; // Major protocol version
    calData[1] = 1; // Minor protocol version
    
    uint32_t sessId = calManager.getSessionId();
    memcpy(&calData[2], &sessId, 4);
    
    calData[6] = calManager.getIsSimulation() ? 1 : 0;
    calData[7] = (uint8_t)calManager.getState();
    
    int activeM = calManager.getActiveMotor();
    calData[8] = (uint8_t)activeM;
    calData[9] = (uint8_t)(activeM + 1);
    calData[10] = (calManager.getState() == CAL_MEASURING_REV) ? 1 : 0;
    calData[11] = (uint8_t)calManager.getCurrentPwm();
    
    int delta = calManager.getSimulatedDelta();
    calData[12] = (uint8_t)delta;
    calData[13] = (delta >= 8) ? 1 : 0; // Movement detected
    
    calData[14] = (motorDriver.getMode() == MotorOutputMode::LOCKED || motorDriver.getMode() == MotorOutputMode::EMERGENCY_STOP || motorDriver.getMode() == MotorOutputMode::FAULTED) ? 1 : 0;
    
    for (int i = 0; i < 4; i++) {
        calData[15 + i] = (uint8_t)calManager.getForwardBreakaway(i);
        calData[19 + i] = (uint8_t)calManager.getReverseBreakaway(i);
    }
    
    strncpy((char*)&calData[23], calManager.getFailureReason(), 32);
    
    writePacket(0x30, calData, 55);
    uint64_t t4 = esp_timer_get_time();
    g_txProf.p0x30_cal.record((uint32_t)(t4 - t3));

    // 5. Control loop timing telemetry packet (TYPE_LOOP_TIMING = 0x33) - 40 bytes (versioned & backward compatible)
    uint8_t timingData[40];
    memcpy(&timingData[0],  &stats.lastDurationUs, 4);
    memcpy(&timingData[4],  &stats.minDurationUs, 4);
    memcpy(&timingData[8],  &stats.avgDurationUs, 4);
    memcpy(&timingData[12], &stats.maxDurationUs, 4);
    memcpy(&timingData[16], &stats.missedDeadlines, 4);
    memcpy(&timingData[20], &stats.totalIterations, 4);

    memcpy(&timingData[24], &stats.lastStartLatenessUs, 4);
    memcpy(&timingData[28], &stats.maxStartLatenessUs, 4);
    memcpy(&timingData[32], &stats.missedControlPeriods, 4);
    memcpy(&timingData[36], &stats.maxConsecutiveMissedPeriods, 4);
    writePacket(0x33, timingData, 40);
    uint64_t t5 = esp_timer_get_time();
    g_txProf.p0x33_timing.record((uint32_t)(t5 - t4));

    // 6. Fault report telemetry packet (TYPE_FAULT_REPORT = 0x34)
    latchFaultReport(faultFlags);
    if (_pendingFaultReport) {
        uint8_t faultData[4];
        memcpy(faultData, &_pendingFaultFlags, 4);
        if (canWrite(9)) {
            writePacket(0x34, faultData, 4);
            _pendingFaultReport = false;
            _pendingFaultFlags = 0;
        }
    }
    uint64_t t6 = esp_timer_get_time();
    g_txProf.p0x34_fault.record((uint32_t)(t6 - t5));

    // 7. Normal drive telemetry packet (TYPE_NORMAL_DRIVE_STATUS = 0x36)
    extern CommandManager commandManager;
    extern MotionLimiter motionLimiter;
    extern MotorDriver motorDriver;
    
    uint8_t normalData[24];
    memset(normalData, 0, sizeof(normalData));
    normalData[0] = commandManager.isNormalDriveArmed() ? 1 : 0;
    normalData[1] = (uint8_t)motorDriver.getMode();
    normalData[2] = (uint8_t)commandManager.getActiveSource();
    
    uint32_t cmdAgeMs = millis() - commandManager.getLastCmdReceivedMs();
    if (commandManager.getActiveSource() == SOURCE_NONE) {
        cmdAgeMs = 999999;
    }
    memcpy(&normalData[3], &cmdAgeMs, 4);
    
    float reqLinear = commandManager.getRequestedLinearVelocity();
    float reqAngular = commandManager.getRequestedAngularVelocity();
    memcpy(&normalData[7], &reqLinear, 4);
    memcpy(&normalData[11], &reqAngular, 4);
    
    float limLinear = motionLimiter.getLinearVelocity();
    float limAngular = motionLimiter.getAngularVelocity();
    memcpy(&normalData[15], &limLinear, 4);
    memcpy(&normalData[19], &limAngular, 4);
    
    normalData[23] = PHASE4A1_NORMAL_DRIVE_OUTPUT_DISABLED ? 1 : 0;
    
    writePacket(0x36, normalData, 24);

#if !PRODUCTION_BINARY_ONLY_SERIAL
    // 10-Second Rate-Limited Packet TX Summary Diagnostic Output (non-blocking, capacity checked)
    if (g_txProf.windowStartMs == 0) {
        g_txProf.windowStartMs = nowMs;
    }
    if (nowMs - g_txProf.windowStartMs >= 10000) {
        char txBuf[384];
        int len = snprintf(txBuf, sizeof(txBuf),
            "[TX PROF 10S] WinMs:%lu | MaxUs(0x0D:%u, 0x0A:%u, 0x35:%u, 0x30:%u, 0x33:%u, 0x34:%u, 0x3A:%u) | >=1ms(0x0D:%u, 0x30:%u, 0x33:%u, 0x3A:%u) | >=5ms(0x30:%u, 0x33:%u, 0x3A:%u) | >=10ms(0x30:%u, 0x33:%u, 0x3A:%u)\n",
            nowMs - g_txProf.windowStartMs,
            g_txProf.p0x0D_encoder.maxUs,
            g_txProf.p0x0A_battery.maxUs,
            g_txProf.p0x35_maint.maxUs,
            g_txProf.p0x30_cal.maxUs,
            g_txProf.p0x33_timing.maxUs,
            g_txProf.p0x34_fault.maxUs,
            g_txProf.p0x3A_imu.maxUs,
            g_txProf.p0x0D_encoder.countGte1ms,
            g_txProf.p0x30_cal.countGte1ms,
            g_txProf.p0x33_timing.countGte1ms,
            g_txProf.p0x3A_imu.countGte1ms,
            g_txProf.p0x30_cal.countGte5ms,
            g_txProf.p0x33_timing.countGte5ms,
            g_txProf.p0x3A_imu.countGte5ms,
            g_txProf.p0x30_cal.countGte10ms,
            g_txProf.p0x33_timing.countGte10ms,
            g_txProf.p0x3A_imu.countGte10ms
        );

        if (len > 0 && len < (int)sizeof(txBuf) && Serial.availableForWrite() >= len) {
            Serial.write((const uint8_t*)txBuf, len);
            g_txProf.reset();
            g_txProf.windowStartMs = nowMs;
        }
    }
#endif
}

void SerialProtocol::sendFirmwareInfo() {
    uint8_t data[112];
    memset(data, 0, sizeof(data));
    
    strncpy((char*)&data[0],  "Maker-ESP32-Unified-Rover", 32);
    strncpy((char*)&data[32], "1.3.0-phase4", 16);
    strncpy((char*)&data[48], "v1.1", 8);
    strncpy((char*)&data[56], "phase4-floor-backtrack", 16);
    strncpy((char*)&data[72], __DATE__ " " __TIME__, 24);
    strncpy((char*)&data[96], "Maker-ESP32-Pro", 16);
    
    writePacket(0x32, data, 112);
}
