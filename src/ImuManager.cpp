#include "ImuManager.h"
#include "SerialProtocol.h"

ImuManager::ImuManager() {}

bool ImuManager::begin(int sdaPin, int sclPin, uint8_t i2cAddr) {
  Wire.begin(sdaPin, sclPin);
  Wire.setClock(400000); // 400 kHz I2C Fast Mode

  LOG_SERIAL_PRINTF("[IMU] Initializing BNO08x on SDA GPIO %d, SCL GPIO %d, Address 0x%02X at 400 kHz I2C...\n", sdaPin, sclPin, i2cAddr);

  if (!_bno.begin_I2C(i2cAddr, &Wire)) {
    LOG_SERIAL_PRINTLN("[IMU ERROR] Failed to find BNO08x chip at specified address!");
    _initialized = false;
    _inResetRecovery = true;
    return false;
  }

  // Re-assert 400 kHz in case _bno.begin_I2C / Wire.begin reset clock dividers
  Wire.setClock(400000);

  LOG_SERIAL_PRINTLN("[IMU] BNO08x hardware detected successfully. Enabling sensor reports at ~50 Hz...");
  _initialized = true;
  setReports();
  return true;
}

void ImuManager::setReports() {
  if (!_initialized) return;

  _inResetRecovery = true;
  _rotVecPostReset = false;
  _gyroPostReset = false;
  _accelPostReset = false;

  // Invalidate pre-reset timestamps so old samples become stale immediately
  _data.rotVecUpdateUs = 0;
  _data.gyroUpdateUs = 0;
  _data.accelUpdateUs = 0;
  _data.linAccUpdateUs = 0;
  _data.lin_ax = 0.0f;
  _data.lin_ay = 0.0f;
  _data.lin_az = 0.0f;

  _data.reportRotVecOk = _bno.enableReport(SH2_ROTATION_VECTOR, _reportIntervalUs);
  LOG_SERIAL_PRINTF("  -> SH2_ROTATION_VECTOR: %s\n", _data.reportRotVecOk ? "SUCCESS" : "FAILED");

  _data.reportGyroOk = _bno.enableReport(SH2_GYROSCOPE_CALIBRATED, _reportIntervalUs);
  LOG_SERIAL_PRINTF("  -> SH2_GYROSCOPE_CALIBRATED: %s\n", _data.reportGyroOk ? "SUCCESS" : "FAILED");

  _data.reportAccelOk = _bno.enableReport(SH2_ACCELEROMETER, _reportIntervalUs);
  LOG_SERIAL_PRINTF("  -> SH2_ACCELEROMETER: %s\n", _data.reportAccelOk ? "SUCCESS" : "FAILED");

  // SH2_LINEAR_ACCELERATION explicitly disabled for 1-variable load reduction experiment
  _data.reportLinAccOk = false;
  LOG_SERIAL_PRINTLN("  -> SH2_LINEAR_ACCELERATION: DISABLED (1-variable load reduction test)");
}

void ImuManager::update() {
  if (!_initialized) return;

  int64_t startTimeUs = esp_timer_get_time();

  if (_bno.wasReset()) {
    _data.resetCount++;
    LOG_SERIAL_PRINTLN("[IMU WARNING] BNO08x reset detected. Re-enabling reports...");
    setReports();
  }

  int eventsProcessed = 0;
  int64_t nowUs = esp_timer_get_time();

  while (eventsProcessed < MAX_EVENTS_PER_LOOP && _bno.getSensorEvent(&_sensorValue)) {
    eventsProcessed++;
    _data.sampleCount++;
    _diag.totalEvents++;

    switch (_sensorValue.sensorId) {
      case SH2_ROTATION_VECTOR: {
        _diag.rotVecEvents++;
        if (_diag.lastRotVecEventUs > 0) {
          uint32_t gapMs = (uint32_t)((nowUs - _diag.lastRotVecEventUs) / 1000);
          if (gapMs > _diag.maxRotVecGapMs) _diag.maxRotVecGapMs = gapMs;
        }
        _diag.lastRotVecEventUs = nowUs;

        float r = _sensorValue.un.rotationVector.real;
        float i = _sensorValue.un.rotationVector.i;
        float j = _sensorValue.un.rotationVector.j;
        float k = _sensorValue.un.rotationVector.k;
        float acc = _sensorValue.un.rotationVector.accuracy;

        if (!std::isnan(r) && !std::isinf(r) &&
            !std::isnan(i) && !std::isinf(i) &&
            !std::isnan(j) && !std::isinf(j) &&
            !std::isnan(k) && !std::isinf(k)) {
          _data.qw = r;
          _data.qx = i;
          _data.qy = j;
          _data.qz = k;
          _data.quatRadAccuracy = (std::isnan(acc) || std::isinf(acc)) ? 0.0f : acc;
          _data.calibrationStatus = _sensorValue.status & 0x03; // Sourced ONLY from SH2_ROTATION_VECTOR
          _data.rotVecUpdateUs = nowUs;
          _rotVecPostReset = true;
        }
        break;
      }

      case SH2_GYROSCOPE_CALIBRATED: {
        _diag.gyroEvents++;
        if (_diag.lastGyroEventUs > 0) {
          uint32_t gapMs = (uint32_t)((nowUs - _diag.lastGyroEventUs) / 1000);
          if (gapMs > _diag.maxGyroGapMs) _diag.maxGyroGapMs = gapMs;
        }
        _diag.lastGyroEventUs = nowUs;

        float gx = _sensorValue.un.gyroscope.x;
        float gy = _sensorValue.un.gyroscope.y;
        float gz = _sensorValue.un.gyroscope.z;

        if (!std::isnan(gx) && !std::isinf(gx) &&
            !std::isnan(gy) && !std::isinf(gy) &&
            !std::isnan(gz) && !std::isinf(gz)) {
          _data.gx = gx;
          _data.gy = gy;
          _data.gz = gz;
          _data.gyroUpdateUs = nowUs;
          _gyroPostReset = true;
        }
        break;
      }

      case SH2_ACCELEROMETER: { // Gravity included (REP-145)
        _diag.accelEvents++;
        if (_diag.lastAccelEventUs > 0) {
          uint32_t gapMs = (uint32_t)((nowUs - _diag.lastAccelEventUs) / 1000);
          if (gapMs > _diag.maxAccelGapMs) _diag.maxAccelGapMs = gapMs;
        }
        _diag.lastAccelEventUs = nowUs;

        float ax = _sensorValue.un.accelerometer.x;
        float ay = _sensorValue.un.accelerometer.y;
        float az = _sensorValue.un.accelerometer.z;

        if (!std::isnan(ax) && !std::isinf(ax) &&
            !std::isnan(ay) && !std::isinf(ay) &&
            !std::isnan(az) && !std::isinf(az)) {
          _data.raw_ax = ax;
          _data.raw_ay = ay;
          _data.raw_az = az;
          _data.accelUpdateUs = nowUs;
          _accelPostReset = true;
        }
        break;
      }

      case SH2_LINEAR_ACCELERATION: { // Gravity removed (internal diagnostics)
        _diag.linAccEvents++;
        float lax = _sensorValue.un.linearAcceleration.x;
        float lay = _sensorValue.un.linearAcceleration.y;
        float laz = _sensorValue.un.linearAcceleration.z;

        if (!std::isnan(lax) && !std::isinf(lax) &&
            !std::isnan(lay) && !std::isinf(lay) &&
            !std::isnan(laz) && !std::isinf(laz)) {
          _data.lin_ax = lax;
          _data.lin_ay = lay;
          _data.lin_az = laz;
          _data.linAccUpdateUs = nowUs;
        }
        break;
      }

      default: {
        _diag.unknownEvents++;
        break;
      }
    }
  }

  // Update eventsProcessed loop statistics
  if (eventsProcessed > _diag.maxEventsInSingleUpdate) {
    _diag.maxEventsInSingleUpdate = eventsProcessed;
  }
  if (eventsProcessed == MAX_EVENTS_PER_LOOP) {
    _diag.hitMaxEventsCount++;
  }

  int64_t durUs = esp_timer_get_time() - startTimeUs;
  if (durUs > (int64_t)_diag.maxUpdateDurationUs) {
    _diag.maxUpdateDurationUs = (uint32_t)durUs;
  }

  // Clear reset recovery only after rotation vector, gyro, and accel have ALL produced valid post-reset samples
  if (_inResetRecovery && _rotVecPostReset && _gyroPostReset && _accelPostReset) {
    _inResetRecovery = false;
    LOG_SERIAL_PRINTLN("[IMU] All post-reset sensor reports restored. Exit reset recovery state.");
  }

#if !PRODUCTION_BINARY_ONLY_SERIAL
  // 10-Second Rate-Limited Summary Diagnostic Output (Strictly 1 line per 10s, non-blocking capacity checked)
  unsigned long nowMs = millis();
  if (_diag.windowStartMs == 0) {
    _diag.windowStartMs = nowMs;
  }
  unsigned long elapsedWindowMs = nowMs - _diag.windowStartMs;
  if (elapsedWindowMs >= 10000) {
    char diagBuf[256];
    int len = snprintf(diagBuf, sizeof(diagBuf),
      "[IMU DIAG 10S] WinMs:%lu | Events(Rot:%u, Gyro:%u, Acc:%u, LinAcc:%u, Unk:%u, Tot:%u) | Loop(MaxEvents:%d, HitMaxCount:%u, MaxDurUs:%u) | MaxGapsMs(Rot:%u, Gyro:%u, Acc:%u) | Reports(Rot:%d, Gyro:%d, Acc:%d, LinAcc:%d) | Resets:%u\n",
      elapsedWindowMs,
      _diag.rotVecEvents,
      _diag.gyroEvents,
      _diag.accelEvents,
      _diag.linAccEvents,
      _diag.unknownEvents,
      _diag.totalEvents,
      _diag.maxEventsInSingleUpdate,
      _diag.hitMaxEventsCount,
      _diag.maxUpdateDurationUs,
      _diag.maxRotVecGapMs,
      _diag.maxGyroGapMs,
      _diag.maxAccelGapMs,
      _data.reportRotVecOk ? 1 : 0,
      _data.reportGyroOk ? 1 : 0,
      _data.reportAccelOk ? 1 : 0,
      _data.reportLinAccOk ? 1 : 0,
      _data.resetCount
    );

    // Non-blocking write: require len > 0, len < sizeof(diagBuf) (snprintf safety), and sufficient UART TX capacity
    if (len > 0 && len < (int)sizeof(diagBuf) && Serial.availableForWrite() >= len) {
      Serial.write((const uint8_t*)diagBuf, len);

      // Reset 10-second window counters ONLY after successfully queued
      _diag.rotVecEvents = 0;
      _diag.gyroEvents = 0;
      _diag.accelEvents = 0;
      _diag.linAccEvents = 0;
      _diag.unknownEvents = 0;
      _diag.totalEvents = 0;
      _diag.maxEventsInSingleUpdate = 0;
      _diag.hitMaxEventsCount = 0;
      _diag.maxRotVecGapMs = 0;
      _diag.maxGyroGapMs = 0;
      _diag.maxAccelGapMs = 0;
      _diag.maxUpdateDurationUs = 0;
      _diag.windowStartMs = nowMs;
    }
  }
#endif

#if IMU_DIAGNOSTIC_MODE
  if (nowMs - _lastDiagPrintMs >= 500) {
    _lastDiagPrintMs = nowMs;
    printDiagnostic();
  }
#endif
}

uint16_t ImuManager::computeAgeMs(int64_t lastUs, int64_t snapUs) {
  if (lastUs <= 0 || snapUs < lastUs) {
    return 0xFFFF;
  }
  int64_t diffMs = (snapUs - lastUs) / 1000;
  if (diffMs > 65534) {
    return 0xFFFF;
  }
  return (uint16_t)diffMs;
}

uint16_t ImuManager::getRotVecAgeMs(int64_t snapUs) const {
  return computeAgeMs(_data.rotVecUpdateUs, snapUs);
}

uint16_t ImuManager::getGyroAgeMs(int64_t snapUs) const {
  return computeAgeMs(_data.gyroUpdateUs, snapUs);
}

uint16_t ImuManager::getAccelAgeMs(int64_t snapUs) const {
  return computeAgeMs(_data.accelUpdateUs, snapUs);
}

uint16_t ImuManager::getStatusFlags(int64_t snapUs) const {
  uint16_t flags = 0;

  // Bit 0: hardware_initialized
  if (_initialized) flags |= (1 << 0);

  // Bit 1: in_reset_recovery
  if (_inResetRecovery) flags |= (1 << 1);

  // Bit 2: rotation_vector_valid (fresh if age <= 100ms)
  uint16_t rotAge = getRotVecAgeMs(snapUs);
  if (rotAge <= 100) flags |= (1 << 2);

  // Bit 3: gyro_valid (fresh if age <= 100ms)
  uint16_t gyroAge = getGyroAgeMs(snapUs);
  if (gyroAge <= 100) flags |= (1 << 3);

  // Bit 4: accelerometer_valid (fresh if age <= 100ms)
  uint16_t accelAge = getAccelAgeMs(snapUs);
  if (accelAge <= 100) flags |= (1 << 4);

  // Bit 5: reserved = 0

  // Bits 6-7: calibration status (0..3) sourced ONLY from SH2_ROTATION_VECTOR
  flags |= ((_data.calibrationStatus & 0x03) << 6);

  // Bits 8-15: reserved = 0
  return flags;
}

void ImuManager::printDiagnostic() {
  LOG_SERIAL_PRINTF("[IMU DIAG] Q(w,x,y,z)=(%.3f,%.3f,%.3f,%.3f) Gyro=(%.2f,%.2f,%.2f) Acc=(%.2f,%.2f,%.2f) Status:%d Rec:%d\n",
    _data.qw, _data.qx, _data.qy, _data.qz,
    _data.gx, _data.gy, _data.gz,
    _data.raw_ax, _data.raw_ay, _data.raw_az,
    _data.calibrationStatus, _inResetRecovery ? 1 : 0
  );
}

