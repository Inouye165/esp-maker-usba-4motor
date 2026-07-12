#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <Arduino.h>

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
    
    // Stream outgoing telemetry packets (encoders, battery, IMU, calibration, maintenance, loop stats, safety faults)
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
    
    void processPacket(
        class CommandManager &cmdManager, 
        class CalibrationManager &calManager, 
        class MaintenanceManager &maintenanceManager, 
        class SafetyManager &safetyManager, 
        ControlLoopStats &stats
    );
    void writePacket(uint8_t extType, const uint8_t *data, uint8_t dataLen);
};

#endif // SERIAL_PROTOCOL_H
