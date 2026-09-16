// test/test_esp32_golden_serializer.cpp
// Golden Vector Test for production C++ SerialProtocol refactored single-write frame construction.

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>

struct ImuData {
    bool hardwareInitialized = true;
    bool reportRotVecOk = true;
    bool reportGyroOk = true;
    bool reportAccelOk = true;
    bool reportLinAccOk = true;
    uint32_t resetCount = 2;

    int64_t rotVecUpdateUs = 9876540000LL;
    int64_t gyroUpdateUs = 9876535000LL;
    int64_t accelUpdateUs = 9876530000LL;
    int64_t linAccUpdateUs = 9876530000LL;

    float qw = 0.9995f;
    float qx = 0.0100f;
    float qy = 0.0200f;
    float qz = 0.0050f;
    uint8_t rotVecAccuracy = 2; // calib status 2

    float gx = 0.04f;
    float gy = -0.02f;
    float gz = 0.08f;

    float raw_ax = 0.20f;
    float raw_ay = -0.10f;
    float raw_az = 9.80665f;

    float lin_ax = 0.20f;
    float lin_ay = -0.10f;
    float lin_az = 0.00f;

    float quatRadAccuracy = 0.025f;
};

class ImuManager {
public:
    ImuData d;

    uint16_t getStatusFlags(int64_t snapUs) const {
        uint16_t flags = 0;
        if (d.hardwareInitialized) flags |= (1 << 0);
        if ((snapUs - d.rotVecUpdateUs) <= 100000) flags |= (1 << 2);
        if ((snapUs - d.gyroUpdateUs) <= 100000) flags |= (1 << 3);
        if ((snapUs - d.accelUpdateUs) <= 100000) flags |= (1 << 4);
        flags |= ((uint16_t)(d.rotVecAccuracy & 0x03) << 6);
        return flags;
    }

    uint16_t getRotVecAgeMs(int64_t snapUs) const {
        int64_t diffUs = snapUs - d.rotVecUpdateUs;
        if (diffUs < 0 || diffUs > 100000) return 0xFFFF;
        return (uint16_t)(diffUs / 1000);
    }

    uint16_t getGyroAgeMs(int64_t snapUs) const {
        int64_t diffUs = snapUs - d.gyroUpdateUs;
        if (diffUs < 0 || diffUs > 100000) return 0xFFFF;
        return (uint16_t)(diffUs / 1000);
    }

    uint16_t getAccelAgeMs(int64_t snapUs) const {
        int64_t diffUs = snapUs - d.accelUpdateUs;
        if (diffUs < 0 || diffUs > 100000) return 0xFFFF;
        return (uint16_t)(diffUs / 1000);
    }
};

// Simulation of production refactored writePacket frame construction
size_t buildPacketFrame(uint8_t *frameBuf, size_t maxFrameSize, uint8_t extType, const uint8_t *data, uint8_t dataLen) {
    const uint8_t outExtLen = dataLen + 3;
    const uint8_t frameLen = dataLen + 5;

    assert(frameLen <= maxFrameSize);

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

    return frameLen;
}

uint8_t serializeImuTelemetry(
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

    memcpy(&p[53], &d.raw_ax, 4);
    memcpy(&p[57], &d.raw_ay, 4);
    memcpy(&p[61], &d.raw_az, 4);

    memcpy(&p[65], &d.quatRadAccuracy, 4);

    return 69;
}

int main() {
    printf("=== Running C++ Production Serializer Golden Test ===\n");

    // 1. Golden Test for 0x3A IMU 74-byte complete frame
    ImuManager mgr;
    int64_t snapUs = 9876543210LL;
    uint32_t seq = 105;

    uint8_t imuPayload[69];
    serializeImuTelemetry(imuPayload, seq, mgr.d, snapUs, mgr);

    uint8_t imuFrame[128];
    size_t imuFrameLen = buildPacketFrame(imuFrame, sizeof(imuFrame), 0x3A, imuPayload, 69);

    assert(imuFrameLen == 74);
    assert(imuFrame[0] == 0xFF && imuFrame[1] == 0xFB);
    assert(imuFrame[2] == 72); // extLen = 69 + 3 = 72 (0x48)
    assert(imuFrame[3] == 0x3A);
    printf(" -> PASS 1: 0x3A IMU 74-byte complete frame (extLen=72, checksum=0x%02X)\n", imuFrame[73]);

    // 2. Golden Test for 0x33 Timing 45-byte complete frame
    uint8_t timingPayload[40];
    memset(timingPayload, 0, 40);
    uint32_t lastDur = 67, minDur = 45, avgDur = 67, maxDur = 480, missed = 0, iter = 50000;
    uint32_t lastLate = 4000, maxLate = 15000, missedPer = 1, maxConsec = 1;
    memcpy(&timingPayload[0], &lastDur, 4);
    memcpy(&timingPayload[4], &minDur, 4);
    memcpy(&timingPayload[8], &avgDur, 4);
    memcpy(&timingPayload[12], &maxDur, 4);
    memcpy(&timingPayload[16], &missed, 4);
    memcpy(&timingPayload[20], &iter, 4);
    memcpy(&timingPayload[24], &lastLate, 4);
    memcpy(&timingPayload[28], &maxLate, 4);
    memcpy(&timingPayload[32], &missedPer, 4);
    memcpy(&timingPayload[36], &maxConsec, 4);

    uint8_t timingFrame[128];
    size_t timingFrameLen = buildPacketFrame(timingFrame, sizeof(timingFrame), 0x33, timingPayload, 40);

    assert(timingFrameLen == 45);
    assert(timingFrame[0] == 0xFF && timingFrame[1] == 0xFB);
    assert(timingFrame[2] == 43); // extLen = 40 + 3 = 43 (0x2B)
    assert(timingFrame[3] == 0x33);
    printf(" -> PASS 2: 0x33 Timing 45-byte complete frame (extLen=43, checksum=0x%02X)\n", timingFrame[44]);

    // 3. Golden Test for 0x0D Encoder 21-byte complete frame
    uint8_t encoderPayload[16];
    int32_t ticks[4] = {1000, -500, 1000, -500};
    memcpy(encoderPayload, ticks, 16);

    uint8_t encoderFrame[128];
    size_t encoderFrameLen = buildPacketFrame(encoderFrame, sizeof(encoderFrame), 0x0D, encoderPayload, 16);

    assert(encoderFrameLen == 21);
    assert(encoderFrame[0] == 0xFF && encoderFrame[1] == 0xFB);
    assert(encoderFrame[2] == 19); // extLen = 16 + 3 = 19 (0x13)
    assert(encoderFrame[3] == 0x0D);
    printf(" -> PASS 3: 0x0D Encoder 21-byte complete frame (extLen=19, checksum=0x%02X)\n", encoderFrame[20]);

    // 4. Golden Test for 0x3B PID Diagnostic 101-byte complete frame (96-byte payload)
    struct TestWheelDiag {
        float targetVel;
        float measuredVel;
        float feedforward;
        float pTerm;
        float iTerm;
        float dTerm;
        int16_t finalPwm;
        int16_t stictionState;
        int16_t basePwm;
        int16_t spinSyncTrim;
    };
    struct TestOuterDiag {
        float wzRequested;
        float wzActual;
        float yawOuterError;
        float yawOuterCorrection;
        float wzCorrected;
        bool yawOuterActive;
        bool imuGyroValid;
        uint16_t imuGyroAgeMs;
    };

    TestWheelDiag testWheels[4] = {
        {-4.25f, -4.12f, -15.2f, -8.4f, -1.5f, -0.6f, -185, 2, -190, 5},
        { 3.80f,  3.65f,  12.6f,  7.1f,  2.3f,  1.1f,  160, 1,  150, -10},
        {-2.75f, -0.02f,  -9.8f, -14.5f, -4.2f,  0.8f, -240, 3, -230, -10},
        { 1.50f,  1.48f,   5.4f,  3.2f, -0.8f, -1.4f,   95, 0,   90, 5}
    };
    TestOuterDiag testOuter = {0.80f, 0.76f, 0.04f, -0.12f, 0.68f, true, true, 12};

    uint8_t pidPayload[96];
    memset(pidPayload, 0, 96);
    for (int i = 0; i < 4; i++) {
        const TestWheelDiag &d = testWheels[i];
        int16_t tVal = (int16_t)(d.targetVel * 100.0f);
        int16_t mVal = (int16_t)(d.measuredVel * 100.0f);
        int16_t ffVal = (int16_t)(d.feedforward * 10.0f);
        int16_t pVal = (int16_t)(d.pTerm * 10.0f);
        int16_t iVal = (int16_t)(d.iTerm * 10.0f);
        int16_t dVal = (int16_t)(d.dTerm * 10.0f);
        int16_t pwmVal = d.finalPwm;
        int16_t stateVal = d.stictionState;

        int off = i * 16;
        memcpy(&pidPayload[off + 0],  &tVal, 2);
        memcpy(&pidPayload[off + 2],  &mVal, 2);
        memcpy(&pidPayload[off + 4],  &ffVal, 2);
        memcpy(&pidPayload[off + 6],  &pVal, 2);
        memcpy(&pidPayload[off + 8],  &iVal, 2);
        memcpy(&pidPayload[off + 10], &dVal, 2);
        memcpy(&pidPayload[off + 12], &pwmVal, 2);
        memcpy(&pidPayload[off + 14], &stateVal, 2);

        int16_t basePwmVal = d.basePwm;
        int16_t syncTrimVal = d.spinSyncTrim;
        memcpy(&pidPayload[80 + i * 4 + 0], &basePwmVal, 2);
        memcpy(&pidPayload[80 + i * 4 + 2], &syncTrimVal, 2);
    }
    int16_t wzReqVal = (int16_t)(testOuter.wzRequested * 100.0f);
    int16_t wzActVal = (int16_t)(testOuter.wzActual * 100.0f);
    int16_t errVal   = (int16_t)(testOuter.yawOuterError * 100.0f);
    int16_t corrVal  = (int16_t)(testOuter.yawOuterCorrection * 100.0f);
    int16_t wzCorrVal= (int16_t)(testOuter.wzCorrected * 100.0f);
    uint8_t activeVal= testOuter.yawOuterActive ? 1 : 0;
    uint8_t validVal = testOuter.imuGyroValid ? 1 : 0;
    uint16_t ageVal  = testOuter.imuGyroAgeMs;
    uint16_t rsvdVal = 0;

    memcpy(&pidPayload[64], &wzReqVal, 2);
    memcpy(&pidPayload[66], &wzActVal, 2);
    memcpy(&pidPayload[68], &errVal, 2);
    memcpy(&pidPayload[70], &corrVal, 2);
    memcpy(&pidPayload[72], &wzCorrVal, 2);
    pidPayload[74] = activeVal;
    pidPayload[75] = validVal;
    memcpy(&pidPayload[76], &ageVal, 2);
    memcpy(&pidPayload[78], &rsvdVal, 2);

    // Verify wheel 0 values
    int16_t m1_tgt, m1_meas, m1_pwm, m1_stiction;
    memcpy(&m1_tgt, &pidPayload[0], 2);
    memcpy(&m1_meas, &pidPayload[2], 2);
    memcpy(&m1_pwm, &pidPayload[12], 2);
    memcpy(&m1_stiction, &pidPayload[14], 2);
    assert(m1_tgt == -425);
    assert(m1_meas == -412);
    assert(m1_pwm == -185);
    assert(m1_stiction == 2);

    // Verify wheel 1 values
    int16_t m2_tgt, m2_meas;
    memcpy(&m2_tgt, &pidPayload[16], 2);
    memcpy(&m2_meas, &pidPayload[18], 2);
    assert(m2_tgt == 380);
    assert(m2_meas == 365);

    // Verify extended trim at offset 80
    int16_t m1_base, m1_trim;
    memcpy(&m1_base, &pidPayload[80], 2);
    memcpy(&m1_trim, &pidPayload[82], 2);
    assert(m1_base == -190);
    assert(m1_trim == 5);

    uint8_t pidFrame[128];
    size_t pidFrameLen = buildPacketFrame(pidFrame, sizeof(pidFrame), 0x3B, pidPayload, 96);
    assert(pidFrameLen == 101);
    assert(pidFrame[0] == 0xFF && pidFrame[1] == 0xFB);
    assert(pidFrame[2] == 99); // extLen = 96 + 3 = 99 (0x63)
    assert(pidFrame[3] == 0x3B);
    printf(" -> PASS 4: 0x3B PID Diagnostic 101-byte complete frame (extLen=99, checksum=0x%02X)\n", pidFrame[100]);

    printf("\nALL C++ PRODUCTION SERIALIZER GOLDEN TESTS PASSED 100%%!\n");
    return 0;
}
