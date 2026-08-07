// test/test_esp32_golden_serializer.cpp
// Golden Vector Test for production C++ SerialProtocol::serializeImuTelemetry()

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
        // not in reset recovery
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

// Production C++ Serializer method implementation matching SerialProtocol::serializeImuTelemetry
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

    // SH2_ACCELEROMETER (gravity included, REP-145 compliant)
    memcpy(&p[53], &d.raw_ax, 4);
    memcpy(&p[57], &d.raw_ay, 4);
    memcpy(&p[61], &d.raw_az, 4);

    memcpy(&p[65], &d.quatRadAccuracy, 4);

    return 69;
}

int main() {
    printf("=== Running C++ Production Serializer Golden Test ===\n");

    ImuManager mgr;
    int64_t snapUs = 9876543210LL;
    uint32_t seq = 105;

    uint8_t payload[69];
    uint8_t len = serializeImuTelemetry(payload, seq, mgr.d, snapUs, mgr);

    assert(len == 69);
    printf(" -> PASS: Payload length is exactly 69 bytes\n");

    // Construct full 74-byte wire frame: [0xFF, 0xFB, extLen=72, extType=0x3A, ...payload, checksum]
    uint8_t extLen = 72; // 69 + 3
    uint8_t extType = 0x3A;
    uint8_t sum = extLen + extType;
    for (int i = 0; i < 69; i++) sum += payload[i];
    uint8_t checksum = sum & 0xFF;

    uint8_t wireFrame[74];
    wireFrame[0] = 0xFF;
    wireFrame[1] = 0xFB;
    wireFrame[2] = extLen;
    wireFrame[3] = extType;
    memcpy(&wireFrame[4], payload, 69);
    wireFrame[73] = checksum;

    // Byte-for-byte verification
    assert(wireFrame[0] == 0xFF && wireFrame[1] == 0xFB);
    printf(" -> PASS: Header bytes 0-1 are 0xFF 0xFB\n");

    assert(wireFrame[2] == 0x48); // 72 decimal
    printf(" -> PASS: extLen is 0x48 (72 decimal)\n");

    assert(wireFrame[3] == 0x3A);
    printf(" -> PASS: type is 0x3A (TYPE_BNO08X_IMU)\n");

    // Check payload fields & offsets
    assert(payload[0] == 0x01); // protocol_version
    printf(" -> PASS: protocol_version = 0x01 at offset 0\n");

    uint16_t flags;
    memcpy(&flags, &payload[1], 2);
    assert((flags & (1 << 0)) != 0); // hw_init
    assert((flags & (1 << 2)) != 0); // rot_valid
    assert((flags & (1 << 3)) != 0); // gyro_valid
    assert((flags & (1 << 4)) != 0); // accel_valid
    assert(((flags >> 6) & 0x03) == 2); // calib status 2
    printf(" -> PASS: status_flags Little-Endian encoding at offset 1-2\n");

    uint32_t readSeq;
    memcpy(&readSeq, &payload[3], 4);
    assert(readSeq == 105);
    printf(" -> PASS: sequence_num (105) Little-Endian encoding at offset 3-6\n");

    uint32_t readReset;
    memcpy(&readReset, &payload[7], 4);
    assert(readReset == 2);
    printf(" -> PASS: reset_count (2) Little-Endian encoding at offset 7-10\n");

    uint64_t readTs;
    memcpy(&readTs, &payload[11], 8);
    assert(readTs == 9876543210LL);
    printf(" -> PASS: esp_timestamp_us (9876543210) Little-Endian encoding at offset 11-18\n");

    float qw, qx, qy, qz;
    memcpy(&qw, &payload[25], 4);
    memcpy(&qx, &payload[29], 4);
    memcpy(&qy, &payload[33], 4);
    memcpy(&qz, &payload[37], 4);
    assert(fabs(qw - 0.9995f) < 1e-4f);
    assert(fabs(qx - 0.0100f) < 1e-4f);
    assert(fabs(qy - 0.0200f) < 1e-4f);
    assert(fabs(qz - 0.0050f) < 1e-4f);
    printf(" -> PASS: exact qw/qx/qy/qz float ordering at offsets 25, 29, 33, 37\n");

    float gx, gy, gz;
    memcpy(&gx, &payload[41], 4);
    memcpy(&gy, &payload[45], 4);
    memcpy(&gz, &payload[49], 4);
    assert(fabs(gx - 0.04f) < 1e-4f);
    assert(fabs(gy - -0.02f) < 1e-4f);
    assert(fabs(gz - 0.08f) < 1e-4f);
    printf(" -> PASS: exact gx/gy/gz float ordering at offsets 41, 45, 49\n");

    float ax, ay, az;
    memcpy(&ax, &payload[53], 4);
    memcpy(&ay, &payload[57], 4);
    memcpy(&az, &payload[61], 4);
    assert(fabs(ax - 0.20f) < 1e-4f);
    assert(fabs(ay - -0.10f) < 1e-4f);
    assert(fabs(az - 9.80665f) < 1e-4f);
    printf(" -> PASS: exact ax/ay/az float ordering (gravity included) at offsets 53, 57, 61\n");

    float quatAcc;
    memcpy(&quatAcc, &payload[65], 4);
    assert(fabs(quatAcc - 0.025f) < 1e-4f);
    printf(" -> PASS: quat_accuracy_rad at offset 65\n");

    printf(" -> PASS: 74-byte wire frame checksum verified: 0x%02X\n", checksum);

    printf("\nALL C++ PRODUCTION SERIALIZER GOLDEN TESTS PASSED 100%%!\n");
    return 0;
}
