#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <Arduino.h>
#include <cstdio>
#include <cstring>

struct ControlLoopStats {
    uint32_t lastDurationUs;
    uint32_t minDurationUs;
    uint32_t avgDurationUs;
    uint32_t maxDurationUs;
    uint32_t missedDeadlines;
    uint32_t totalIterations;
};

class SerialProtocol {
public:
    SerialProtocol();
    void begin();
    
    // Parse incoming characters from serial port (non-blocking)
    // returns true if a valid command was processed
    bool update(
        class CommandManager &cmdManager, 
        class CalibrationManager &calManager, 
        class MaintenanceManager &maintenanceManager, 
        class SafetyManager &safetyManager, 
        ControlLoopStats &stats
    );
    
    // Stream production BNO08x 0x3A IMU telemetry packet (50 Hz)
    void sendImuTelemetry(const class ImuManager &imuManager);

    // Stream outgoing telemetry packets (encoders, battery, calibration, maintenance, loop stats, safety faults)
    void sendTelemetry(
        const int32_t *ticks,
        float batteryVolts,
        float currentYawRate,
        const class CalibrationManager &calManager,
        const class MaintenanceManager &maintenanceManager,
        const ControlLoopStats &stats,
        uint32_t faultFlags
    );

    // Send firmware information packet
    void sendFirmwareInfo();

    // Encapsulate production 0x3A 69-byte serialization math for hardware & golden testing
    static uint8_t serializeImuTelemetry(
        uint8_t *p,
        uint32_t seq,
        const struct ImuData &d,
        int64_t snapUs,
        const class ImuManager &imuManager
    );

    // Diagnostics / Metrics & Fault Latch API
    uint32_t getImuTxDropped() const { return imuTxDropped; }
    uint32_t getImuSequenceNum() const { return imuSequenceNum; }
    bool isFaultReportPending() const { return _pendingFaultReport; }
    uint32_t getPendingFaultFlags() const { return _pendingFaultFlags; }
    void latchFaultReport(uint32_t faultFlags) {
        if (faultFlags != 0) {
            _pendingFaultFlags |= faultFlags;
            _pendingFaultReport = true;
        }
    }

private:
    enum ParserState {
        WAIT_HEAD,
        WAIT_DEVICE,
        WAIT_LEN,
        WAIT_PAYLOAD
    };

    ParserState parserState;
    uint8_t extLen;
    uint8_t payloadBuf[128]; // Larger command payloads
    uint8_t payloadIdx;
    uint32_t lastCharTimeMs; // Timer for parser timeout

    uint32_t imuSequenceNum;
    uint32_t imuTxDropped;

    bool _pendingFaultReport;
    uint32_t _pendingFaultFlags;
    
    void processPacket(
        class CommandManager &cmdManager, 
        class CalibrationManager &calManager, 
        class MaintenanceManager &maintenanceManager, 
        class SafetyManager &safetyManager, 
        ControlLoopStats &stats
    );
    bool canWrite(uint8_t wireSize);
    void writePacket(uint8_t extType, const uint8_t *data, uint8_t dataLen);
};

#endif // SERIAL_PROTOCOL_H
