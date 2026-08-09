#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <Arduino.h>
#include <cstdio>
#include <cstring>

// Production Binary-Only Serial Control Flag
// Set to 1 for production binary-only telemetry stream (no plain-text intermingling).
#ifndef PRODUCTION_BINARY_ONLY_SERIAL
#define PRODUCTION_BINARY_ONLY_SERIAL 1
#endif

#if PRODUCTION_BINARY_ONLY_SERIAL
  #define LOG_SERIAL_PRINTF(...)  ((void)0)
  #define LOG_SERIAL_PRINTLN(...) ((void)0)
  #define LOG_SERIAL_PRINT(...)   ((void)0)
#else
  #define LOG_SERIAL_PRINTF(...)  Serial.printf(__VA_ARGS__)
  #define LOG_SERIAL_PRINTLN(...) Serial.println(__VA_ARGS__)
  #define LOG_SERIAL_PRINT(...)   Serial.print(__VA_ARGS__)
#endif

struct ControlLoopStats {
    uint32_t lastDurationUs;
    uint32_t minDurationUs;
    uint32_t avgDurationUs;
    uint32_t maxDurationUs;
    uint32_t missedDeadlines;
    uint32_t totalIterations;

    // Whole-Loop 100 Hz Scheduling & Start Lateness Diagnostics
    uint32_t lastStartLatenessUs;
    uint32_t maxStartLatenessUs;
    uint32_t missedControlPeriods;
    uint32_t maxConsecutiveMissedPeriods;
};

struct PacketProf {
    uint32_t maxUs = 0;
    uint32_t countGte1ms = 0;
    uint32_t countGte5ms = 0;
    uint32_t countGte10ms = 0;

    void record(uint32_t durUs) {
        if (durUs > maxUs) maxUs = durUs;
        if (durUs >= 1000) countGte1ms++;
        if (durUs >= 5000) countGte5ms++;
        if (durUs >= 10000) countGte10ms++;
    }

    void reset() {
        maxUs = 0;
        countGte1ms = 0;
        countGte5ms = 0;
        countGte10ms = 0;
    }
};

struct TelemetryTxProf {
    PacketProf p0x0D_encoder;
    PacketProf p0x0A_battery;
    PacketProf p0x35_maint;
    PacketProf p0x30_cal;
    PacketProf p0x33_timing;
    PacketProf p0x34_fault;
    PacketProf p0x3A_imu;
    uint32_t windowStartMs = 0;

    void reset() {
        p0x0D_encoder.reset();
        p0x0A_battery.reset();
        p0x35_maint.reset();
        p0x30_cal.reset();
        p0x33_timing.reset();
        p0x34_fault.reset();
        p0x3A_imu.reset();
    }
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
