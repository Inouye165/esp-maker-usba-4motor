#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <cmath>

#ifndef IMU_DIAGNOSTIC_MODE
#define IMU_DIAGNOSTIC_MODE 0
#endif

struct ImuData {
  // Quaternion (SI normalized)
  float qw = 1.0f;
  float qx = 0.0f;
  float qy = 0.0f;
  float qz = 0.0f;
  float quatRadAccuracy = 0.0f;

  // Calibrated Gyroscope (rad/s)
  float gx = 0.0f;
  float gy = 0.0f;
  float gz = 0.0f;

  // Raw Accelerometer - GRAVITY INCLUDED (m/s^2, REP-145)
  float raw_ax = 0.0f;
  float raw_ay = 0.0f;
  float raw_az = 0.0f;

  // Linear Acceleration - GRAVITY REMOVED (m/s^2, internal diagnostic use)
  float lin_ax = 0.0f;
  float lin_ay = 0.0f;
  float lin_az = 0.0f;

  // Status & Timestamps
  uint8_t calibrationStatus = 0; // 0=Unreliable, 1=Low, 2=Medium, 3=High (sourced ONLY from SH2_ROTATION_VECTOR)
  uint32_t sampleCount = 0;
  uint32_t resetCount = 0;

  // Independent report timestamps (microseconds via esp_timer_get_time())
  int64_t rotVecUpdateUs = 0;
  int64_t gyroUpdateUs = 0;
  int64_t accelUpdateUs = 0;
  int64_t linAccUpdateUs = 0;

  // Report enablement status flags
  bool reportRotVecOk = false;
  bool reportGyroOk = false;
  bool reportLinAccOk = false;
  bool reportAccelOk = false;
};

class ImuManager {
public:
  ImuManager();
  bool begin(int sdaPin = 21, int sclPin = 22, uint8_t i2cAddr = 0x4B);
  void update();

  bool isInitialized() const { return _initialized; }
  bool inResetRecovery() const { return _inResetRecovery; }
  const ImuData& getData() const { return _data; }

  // Freshness and age helpers
  uint16_t getRotVecAgeMs(int64_t snapUs) const;
  uint16_t getGyroAgeMs(int64_t snapUs) const;
  uint16_t getAccelAgeMs(int64_t snapUs) const;

  // Locked v1 status flags bitfield construction
  uint16_t getStatusFlags(int64_t snapUs) const;

private:
  void setReports();
  void printDiagnostic();
  static uint16_t computeAgeMs(int64_t lastUs, int64_t snapUs);

  Adafruit_BNO08x _bno;
  sh2_SensorValue_t _sensorValue;
  ImuData _data;
  bool _initialized = false;
  bool _inResetRecovery = false;

  // Post-reset reception trackers to clear reset recovery
  bool _rotVecPostReset = false;
  bool _gyroPostReset = false;
  bool _accelPostReset = false;

  uint32_t _reportIntervalUs = 20000; // ~50 Hz
  unsigned long _lastDiagPrintMs = 0;

  static const int MAX_EVENTS_PER_LOOP = 5;
};

#endif // IMU_MANAGER_H
