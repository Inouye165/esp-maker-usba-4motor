#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <esp_ota_ops.h>
#include <driver/uart.h>
#include <esp_rom_gpio.h>
#include <hal/gpio_hal.h>
#include <soc/uart_periph.h>
#include <Adafruit_NeoPixel.h>
#include <vl53l7cx_class.h>
#include <vl53l4cd_class.h>

#if __has_include("ota_secrets.h")
#include "ota_secrets.h"
#else
#error "Missing esp32s3_status_led/include/ota_secrets.h! Copy ota_secrets.h.example or run deploy_esp32s3.py --provision-secrets."
#endif

#ifndef ESP32S3_FW_VERSION
#define ESP32S3_FW_VERSION "1.2.3-tof-mux"
#endif

// Lonely Binary ESP32-S3 N16R8 onboard WS2812 RGB LED pins across board revisions
static constexpr int LED_PIN_PRIMARY   = 48;
static constexpr int LED_PIN_SECONDARY = 38;
static constexpr int NUM_PIXELS        = 1;
static constexpr uint8_t LED_BRIGHTNESS = 64;

// GPIO UART connection to Raspberry Pi 5 header (Pi GPIO14 TX -> ESP GPIO18 RX, Pi GPIO15 RX <- ESP GPIO17 TX)
static constexpr int UART_RX_PIN = 18;
static constexpr int UART_TX_PIN = 17;
static constexpr unsigned long UART_BAUD = 115200;

// I2C Multiplexer (TCA9548A / PCA9548A) on ESP32-S3 default I2C pins (200 kHz for reliable long-wire rear ToF margin)
static constexpr int I2C_SDA_PIN = 8;
static constexpr int I2C_SCL_PIN = 9;
static constexpr uint32_t I2C_FREQ_HZ = 200000;
static constexpr uint8_t MUX_I2C_ADDR = 0x70;
static constexpr uint8_t TOF_I2C_ADDR = 0x29;
static constexpr uint8_t MUX_NUM_CHANNELS = 8;
static constexpr size_t I2C_CHUNK_BYTES = 128;
static constexpr int I2C_MAX_RETRIES = 5;

// Timeout without RPi5 heartbeat before reverting to Solid Red (RPi5 off/disconnected)
static constexpr unsigned long HEARTBEAT_TIMEOUT_MS = 1500;

// Non-blocking Wi-Fi reconnect check interval
static constexpr unsigned long WIFI_RECONNECT_INTERVAL_MS = 15000;

// ToF polling & hot-plug discovery intervals
static constexpr unsigned long TOF_POLL_INTERVAL_MS = 35;
static constexpr unsigned long TOF_BROADCAST_INTERVAL_MS = 250;
static constexpr unsigned long TOF_HOTPLUG_SCAN_INTERVAL_MS = 2500;

Adafruit_NeoPixel pixel48(NUM_PIXELS, LED_PIN_PRIMARY, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixel38(NUM_PIXELS, LED_PIN_SECONDARY, NEO_GRB + NEO_KHZ800);

enum class LedState : uint8_t {
  RPI_OFF_RED       = 0,
  RPI_ON_GREEN      = 1,
  ROVER_MOVING_BLUE = 2
};

enum class OtaState : uint8_t {
  OTA_DISABLED = 0,
  OTA_READY    = 1,
  OTA_UPDATING = 2,
  OTA_ERROR    = 3
};

enum class TofSensorType : uint8_t {
  NONE     = 0,
  VL53L4CD = 1,
  VL53L7CX = 2,
  UNKNOWN  = 3
};

struct TofChannelState {
  TofSensorType type = TofSensorType::NONE;
  bool present = false;
  bool initialized = false;
  bool initFailed = false;
  uint8_t consecutiveErrors = 0;
  uint16_t modelId = 0;
  unsigned long lastSampleMs = 0;
  uint32_t frameCount = 0;

  // VL53L4CD single-zone state
  VL53L4CD* l4cd = nullptr;
  int16_t distMm = -1;
  uint8_t rangeStatus = 255;
  uint16_t signalKcps = 0;
  uint16_t sigmaMm = 0;

  // VL53L7CX 8x8 multizone state
  VL53L7CX* l7cx = nullptr;
  int16_t minMm = -1;
  int16_t centerMm = -1;
  int16_t meanMm = -1;
  uint8_t validZones = 0;
  int8_t siliconTempC = 0;
  int16_t gridMm[64];
  uint8_t gridStatus[64];
};

static TofChannelState tofChannels[MUX_NUM_CHANNELS];
static VL53L7CX_ResultsData l7cxResultsShared;
static bool muxAvailable = false;
static uint8_t nextPollChannel = 0;
static unsigned long lastTofPollMs = 0;
static unsigned long lastTofBroadcastMs = 0;
static unsigned long lastHotplugScanMs = 0;

static LedState currentState = LedState::RPI_OFF_RED;
static unsigned long lastHeartbeatMs = 0;
static bool hasHeartbeat = false;
static bool uartTxEnabled = false;
static unsigned long lastRefreshMs = 0;

static OtaState currentOtaState = OtaState::OTA_DISABLED;
static bool otaInitialized = false;
static bool wifiForceOffline = false;
static unsigned long wifiForceOfflineUntilMs = 0;
static unsigned long lastWifiCheckMs = 0;
static bool lastWifiConnected = false;
static unsigned int lastOtaPctLogged = 0;

// Safe ESP32 I2C multi-byte read/write helpers for 16-bit register ST ToF sensors (with bounded retry)
static uint8_t esp32TofI2cRead(TwoWire* wire, uint8_t dev7, uint16_t regAddr, uint8_t* pValues, uint32_t size) {
  uint32_t i = 0;
  while (i < size) {
    size_t chunk = (size - i > I2C_CHUNK_BYTES) ? I2C_CHUNK_BYTES : (size_t)(size - i);
    uint16_t curReg = (uint16_t)(regAddr + i);
    bool chunkOk = false;
    for (int attempt = 0; attempt < I2C_MAX_RETRIES; ++attempt) {
      if (attempt > 0) {
        delay(2);
      }
      wire->beginTransmission(dev7);
      wire->write((uint8_t)(curReg >> 8));
      wire->write((uint8_t)(curReg & 0xFF));
      if (wire->endTransmission(false) != 0) {
        continue;
      }
      size_t got = wire->requestFrom(dev7, chunk, true);
      if (got != chunk) {
        continue;
      }
      bool readOk = true;
      for (size_t k = 0; k < chunk; ++k) {
        int b = wire->read();
        if (b < 0) {
          readOk = false;
          break;
        }
        pValues[i + k] = (uint8_t)b;
      }
      if (readOk) {
        chunkOk = true;
        break;
      }
    }
    if (!chunkOk) {
      return 1;
    }
    i += (uint32_t)chunk;
  }
  return 0;
}

static uint8_t esp32TofI2cWrite(TwoWire* wire, uint8_t dev7, uint16_t regAddr, uint8_t* pValues, uint32_t size) {
  uint32_t i = 0;
  uint32_t lastYield = 0;
  while (i < size) {
    size_t chunk = (size - i > I2C_CHUNK_BYTES) ? I2C_CHUNK_BYTES : (size_t)(size - i);
    uint16_t curReg = (uint16_t)(regAddr + i);
    bool chunkOk = false;
    for (int attempt = 0; attempt < I2C_MAX_RETRIES; ++attempt) {
      if (attempt > 0) {
        delay(2);
      }
      wire->beginTransmission(dev7);
      wire->write((uint8_t)(curReg >> 8));
      wire->write((uint8_t)(curReg & 0xFF));
      if (wire->write(pValues + i, chunk) != chunk) {
        wire->endTransmission(true);
        continue;
      }
      if (wire->endTransmission(true) == 0) {
        chunkOk = true;
        break;
      }
    }
    if (!chunkOk) {
      return 1;
    }
    i += (uint32_t)chunk;
    if (i - lastYield >= 2048U) {
      lastYield = i;
      if (hasHeartbeat) {
        lastHeartbeatMs = millis();
      }
      delay(0);
    }
  }
  return 0;
}

// Linker wrappers overriding STM32duino VL53L7CX and VL53L4CD platform I2C transfers on ESP32
extern "C" uint8_t __wrap__ZN8VL53L7CX7RdMultiEP17VL53L7CX_PlatformtPhj(
    VL53L7CX* self, VL53L7CX_Platform* p_platform, uint16_t RegisterAddress, uint8_t* p_values, uint32_t size) {
  (void)self;
  uint8_t dev7 = (uint8_t)((p_platform->address >> 1) & 0x7F);
  return esp32TofI2cRead(p_platform->dev_i2c, dev7, RegisterAddress, p_values, size);
}

extern "C" uint8_t __wrap__ZN8VL53L7CX7WrMultiEP17VL53L7CX_PlatformtPhj(
    VL53L7CX* self, VL53L7CX_Platform* p_platform, uint16_t RegisterAddress, uint8_t* p_values, uint32_t size) {
  (void)self;
  uint8_t dev7 = (uint8_t)((p_platform->address >> 1) & 0x7F);
  return esp32TofI2cWrite(p_platform->dev_i2c, dev7, RegisterAddress, p_values, size);
}

extern "C" uint8_t __wrap__ZN8VL53L4CD16VL53L4CD_I2CReadEhtPhj(
    VL53L4CD* self, uint8_t DeviceAddr, uint16_t RegisterAddress, uint8_t* p_values, uint32_t size) {
  (void)self;
  uint8_t dev7 = (uint8_t)((DeviceAddr >> 1) & 0x7F);
  return esp32TofI2cRead(&Wire, dev7, RegisterAddress, p_values, size);
}

extern "C" uint8_t __wrap__ZN8VL53L4CD17VL53L4CD_I2CWriteEhtPhj(
    VL53L4CD* self, uint8_t DeviceAddr, uint16_t RegisterAddress, uint8_t* p_values, uint32_t size) {
  (void)self;
  uint8_t dev7 = (uint8_t)((DeviceAddr >> 1) & 0x7F);
  return esp32TofI2cWrite(&Wire, dev7, RegisterAddress, p_values, size);
}

static void setUartTxDriven(bool enable) {
  if (enable && !uartTxEnabled) {
    pinMode(UART_TX_PIN, OUTPUT);
    digitalWrite(UART_TX_PIN, HIGH);
    esp_rom_gpio_connect_out_signal(UART_TX_PIN, UART_PERIPH_SIGNAL(1, SOC_UART_TX_PIN_IDX), false, false);
    uartTxEnabled = true;
  } else if (!enable && uartTxEnabled) {
    // Detach UART1 TX matrix signal and place GPIO17 in high-Z INPUT so powered ESP32-S3 never backfeeds an unpowered RPi5
    esp_rom_gpio_connect_out_signal(UART_TX_PIN, SIG_GPIO_OUT_IDX, false, false);
    pinMode(UART_TX_PIN, INPUT);
    uartTxEnabled = false;
  }
}

static void broadcastLog(const char* line) {
  Serial.println(line);
  if (uartTxEnabled) {
    Serial1.println(line);
  }
}

static const char* stateName(LedState state) {
  switch (state) {
    case LedState::RPI_ON_GREEN:      return "GREEN";
    case LedState::ROVER_MOVING_BLUE: return "BLUE";
    case LedState::RPI_OFF_RED:
    default:                          return "RED";
  }
}

static const char* otaStateName(OtaState state) {
  switch (state) {
    case OtaState::OTA_READY:    return "READY";
    case OtaState::OTA_UPDATING: return "UPDATING";
    case OtaState::OTA_ERROR:    return "ERROR";
    case OtaState::OTA_DISABLED:
    default:                     return "DISABLED";
  }
}

static const char* tofTypeName(TofSensorType t) {
  switch (t) {
    case TofSensorType::VL53L4CD: return "VL53L4CD";
    case TofSensorType::VL53L7CX: return "VL53L7CX";
    case TofSensorType::UNKNOWN:  return "UNKNOWN";
    case TofSensorType::NONE:
    default:                      return "NONE";
  }
}

static const char* activePartitionLabel() {
  const esp_partition_t* running = esp_ota_get_running_partition();
  return (running != nullptr && running->label[0] != '\0') ? running->label : "unknown";
}

static void applyLedColor(LedState state) {
  uint8_t r = 0, g = 0, b = 0;
  switch (state) {
    case LedState::RPI_ON_GREEN:
      r = 0; g = 255; b = 0;
      break;
    case LedState::ROVER_MOVING_BLUE:
      r = 0; g = 0; b = 255;
      break;
    case LedState::RPI_OFF_RED:
    default:
      r = 255; g = 0; b = 0;
      break;
  }

  pixel48.setPixelColor(0, pixel48.Color(r, g, b));
  pixel48.show();

  pixel38.setPixelColor(0, pixel38.Color(r, g, b));
  pixel38.show();
}

static void setState(LedState nextState) {
  if (currentState != nextState) {
    currentState = nextState;
    applyLedColor(currentState);
    char buf[96];
    snprintf(buf, sizeof(buf), "[ESP32S3_LED] State -> %s (uptime=%lu ms)", stateName(currentState), millis());
    broadcastLog(buf);
  }
}

static bool probeI2cAddress(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

static bool writeMuxMask(uint8_t mask, uint8_t* readbackOut = nullptr) {
  Wire.beginTransmission(MUX_I2C_ADDR);
  Wire.write(mask);
  if (Wire.endTransmission() != 0) {
    return false;
  }
  if (Wire.requestFrom(MUX_I2C_ADDR, (size_t)1, true) != 1) {
    return false;
  }
  uint8_t rb = (uint8_t)Wire.read();
  if (readbackOut != nullptr) {
    *readbackOut = rb;
  }
  return (rb == mask);
}

static void recoverDownstreamI2cIfStuck() {
  // Re-init ESP32 Wire master if a downstream channel caused an arbitration/timeout error
  Wire.end();
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, OUTPUT);
  for (int i = 0; i < 9; ++i) {
    digitalWrite(I2C_SCL_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(5);
  }
  Wire.setBufferSize(1024);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
  Wire.setTimeOut(150);
}

static bool selectMuxChannel(uint8_t ch) {
  if (ch >= MUX_NUM_CHANNELS) {
    return false;
  }
  writeMuxMask(0x00);
  delayMicroseconds(300);
  bool ok = writeMuxMask((uint8_t)(1U << ch));
  delay(2);
  return ok;
}

static bool probeTofAddressWithRetry() {
  for (int i = 0; i < 3; ++i) {
    if (i > 0) {
      delay(2);
    }
    Wire.beginTransmission(TOF_I2C_ADDR);
    uint8_t rc = Wire.endTransmission();
    if (rc == 0) {
      return true;
    }
    if (rc == 4 || rc == 5) {
      recoverDownstreamI2cIfStuck();
    }
  }
  return false;
}

static void resetChannelState(uint8_t ch) {
  TofChannelState& st = tofChannels[ch];
  st.type = TofSensorType::NONE;
  st.present = false;
  st.initialized = false;
  st.initFailed = false;
  st.consecutiveErrors = 0;
  st.modelId = 0;
  st.lastSampleMs = 0;
  st.frameCount = 0;
  st.distMm = -1;
  st.rangeStatus = 255;
  st.signalKcps = 0;
  st.sigmaMm = 0;
  st.minMm = -1;
  st.centerMm = -1;
  st.meanMm = -1;
  st.validZones = 0;
  st.siliconTempC = 0;
  for (int i = 0; i < 64; ++i) {
    st.gridMm[i] = -1;
    st.gridStatus[i] = 255;
  }
}

// Safely stop VL53L7CX internal MCU if it is already ranging before reading bank 0 or re-initializing
static void safeStopVl53l7cxIfRunning(VL53L7CX* l7cx) {
  if (l7cx == nullptr) {
    return;
  }
  uint8_t zero = 0x00;
  uint8_t two = 0x02;
  uint8_t pwr = 0x00;
  // Switch to bank 0 and check power mode register 0x0009; if sensor is already awake/ranging, stop MCU cleanly
  if (esp32TofI2cWrite(&Wire, TOF_I2C_ADDR, 0x7FFF, &zero, 1) == 0 &&
      esp32TofI2cRead(&Wire, TOF_I2C_ADDR, 0x0009, &pwr, 1) == 0) {
    if (pwr == 0x05) {
      // Sensor is in xshut bypass / active ranging mode (0x09 == 0x05): restore bank 2 and run official stop sequence
      esp32TofI2cWrite(&Wire, TOF_I2C_ADDR, 0x7FFF, &two, 1);
      l7cx->vl53l7cx_stop_ranging();
      delay(20);
    }
  }
}

// Identify whether a 0x29 ToF device on the currently selected mux channel is VL53L4CD (0xEBAA) or VL53L7CX (0xF002)
static TofSensorType identifyTofSensorOnBus(VL53L7CX* l7cxHelper, uint16_t* modelIdOut = nullptr) {
  uint8_t buf[2] = {0, 0};
  if (esp32TofI2cRead(&Wire, TOF_I2C_ADDR, 0x010F, buf, 2) == 0) {
    uint16_t id4 = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    if (id4 == 0xEBAA) {
      if (modelIdOut) *modelIdOut = id4;
      return TofSensorType::VL53L4CD;
    }
  }

  // Ensure VL53L7CX MCU is not actively ranging in bank 2 before switching to bank 0
  safeStopVl53l7cxIfRunning(l7cxHelper);

  // Probe VL53L7CX bank 0 device_id (0xF0) & revision_id (0x02)
  uint8_t zero = 0x00;
  uint8_t two = 0x02;
  uint8_t id7[2] = {0, 0};
  if (esp32TofI2cWrite(&Wire, TOF_I2C_ADDR, 0x7FFF, &zero, 1) == 0 &&
      esp32TofI2cRead(&Wire, TOF_I2C_ADDR, 0x0000, id7, 2) == 0) {
    esp32TofI2cWrite(&Wire, TOF_I2C_ADDR, 0x7FFF, &two, 1);
    uint16_t combined = ((uint16_t)id7[0] << 8) | (uint16_t)id7[1];
    if (modelIdOut) *modelIdOut = combined;
    if (id7[0] == 0xF0) {
      return TofSensorType::VL53L7CX;
    }
  }
  return TofSensorType::UNKNOWN;
}

static bool initChannelSensor(uint8_t ch) {
  if (!selectMuxChannel(ch)) {
    return false;
  }
  TofChannelState& st = tofChannels[ch];
  if (!probeTofAddressWithRetry()) {
    resetChannelState(ch);
    writeMuxMask(0x00);
    return false;
  }

  if (st.l7cx == nullptr) {
    st.l7cx = new VL53L7CX(&Wire, -1);
  }
  st.l7cx->begin();

  st.present = true;
  uint16_t modelId = 0;
  TofSensorType detected = identifyTofSensorOnBus(st.l7cx, &modelId);
  st.modelId = modelId;
  st.type = detected;

  if (detected == TofSensorType::VL53L4CD) {
    if (st.l4cd == nullptr) {
      st.l4cd = new VL53L4CD(&Wire, -1);
    }
    st.l4cd->begin();
    if (st.l4cd->InitSensor() == VL53L4CD_ERROR_NONE &&
        st.l4cd->VL53L4CD_SetRangeTiming(25, 0) == VL53L4CD_ERROR_NONE &&
        st.l4cd->VL53L4CD_StartRanging() == VL53L4CD_ERROR_NONE) {
      st.initialized = true;
      st.initFailed = false;
      st.consecutiveErrors = 0;
      char logBuf[128];
      snprintf(logBuf, sizeof(logBuf), "[ESP32S3_TOF_INIT] CH=%u TYPE=VL53L4CD MODEL=0x%04X STATUS=RANGING", ch, modelId);
      broadcastLog(logBuf);
      writeMuxMask(0x00);
      return true;
    } else {
      st.initialized = false;
      st.initFailed = true;
      char logBuf[128];
      snprintf(logBuf, sizeof(logBuf), "[ESP32S3_TOF_INIT] CH=%u TYPE=VL53L4CD MODEL=0x%04X STATUS=INIT_FAIL", ch, modelId);
      broadcastLog(logBuf);
      writeMuxMask(0x00);
      return false;
    }
  } else if (detected == TofSensorType::VL53L7CX || detected == TofSensorType::UNKNOWN) {
    unsigned long t0 = millis();
    if (hasHeartbeat) {
      lastHeartbeatMs = t0;
    }
    // Call vl53l7cx_init() directly (which executes the full SW reboot sequence) after safeStopVl53l7cxIfRunning()
    int rcInit = (int)st.l7cx->vl53l7cx_init();
    int rcRes = (rcInit == 0) ? (int)st.l7cx->vl53l7cx_set_resolution(VL53L7CX_RESOLUTION_8X8) : -1;
    int rcFreq = (rcRes == 0) ? (int)st.l7cx->vl53l7cx_set_ranging_frequency_hz(10) : -1;
    int rcStart = (rcFreq == 0) ? (int)st.l7cx->vl53l7cx_start_ranging() : -1;
    if (rcInit == 0 && rcRes == 0 && rcFreq == 0 && rcStart == 0) {
      st.type = TofSensorType::VL53L7CX;
      if (modelId == 0) {
        st.modelId = 0xF002;
      }
      st.initialized = true;
      st.initFailed = false;
      st.consecutiveErrors = 0;
      if (hasHeartbeat) {
        lastHeartbeatMs = millis();
      }
      char logBuf[144];
      snprintf(logBuf,
               sizeof(logBuf),
               "[ESP32S3_TOF_INIT] CH=%u TYPE=VL53L7CX MODEL=0x%04X STATUS=RANGING_8X8_10HZ INIT_MS=%lu",
               ch,
               st.modelId,
               millis() - t0);
      broadcastLog(logBuf);
      writeMuxMask(0x00);
      return true;
    } else {
      st.initialized = false;
      st.initFailed = true;
      if (hasHeartbeat) {
        lastHeartbeatMs = millis();
      }
      char logBuf[160];
      snprintf(logBuf,
               sizeof(logBuf),
               "[ESP32S3_TOF_INIT] CH=%u TYPE=%s MODEL=0x%04X STATUS=INIT_FAIL RC=%d/%d/%d/%d",
               ch,
               tofTypeName(detected),
               modelId,
               rcInit,
               rcRes,
               rcFreq,
               rcStart);
      broadcastLog(logBuf);
      writeMuxMask(0x00);
      return false;
    }
  }

  writeMuxMask(0x00);
  return false;
}

static void scanAndInitAllTofChannels(bool forceReinit) {
  muxAvailable = probeI2cAddress(MUX_I2C_ADDR);
  if (!muxAvailable) {
    for (uint8_t ch = 0; ch < MUX_NUM_CHANNELS; ++ch) {
      resetChannelState(ch);
    }
    return;
  }

  for (uint8_t ch = 0; ch < MUX_NUM_CHANNELS; ++ch) {
    if (!selectMuxChannel(ch)) {
      continue;
    }
    bool devHere = probeTofAddressWithRetry();
    if (!devHere) {
      if (tofChannels[ch].present || tofChannels[ch].initialized) {
        resetChannelState(ch);
      }
      continue;
    }
    tofChannels[ch].present = true;
    // Never disrupt an already-initialized and healthy sensor during routine rescan unless forceReinit is explicitly set
    if (forceReinit && tofChannels[ch].initialized) {
      if (tofChannels[ch].type == TofSensorType::VL53L7CX && tofChannels[ch].l7cx != nullptr) {
        tofChannels[ch].l7cx->vl53l7cx_stop_ranging();
        delay(20);
      } else if (tofChannels[ch].type == TofSensorType::VL53L4CD && tofChannels[ch].l4cd != nullptr) {
        tofChannels[ch].l4cd->VL53L4CD_StopRanging();
        delay(10);
      }
      resetChannelState(ch);
      tofChannels[ch].present = true;
    }
    if (!tofChannels[ch].initialized) {
      initChannelSensor(ch);
    }
  }
  writeMuxMask(0x00);
}

static void pollNextActiveTofChannel() {
  if (!muxAvailable || currentOtaState == OtaState::OTA_UPDATING) {
    return;
  }

  for (uint8_t attempt = 0; attempt < MUX_NUM_CHANNELS; ++attempt) {
    uint8_t ch = (nextPollChannel + attempt) % MUX_NUM_CHANNELS;
    TofChannelState& st = tofChannels[ch];
    if (!st.initialized) {
      continue;
    }
    nextPollChannel = (ch + 1) % MUX_NUM_CHANNELS;

    if (!selectMuxChannel(ch)) {
      return;
    }

    if (st.type == TofSensorType::VL53L4CD && st.l4cd != nullptr) {
      uint8_t ready = 0;
      VL53L4CD_ERROR err = st.l4cd->VL53L4CD_CheckForDataReady(&ready);
      if (err != VL53L4CD_ERROR_NONE) {
        if (++st.consecutiveErrors >= 4) {
          resetChannelState(ch);
        }
      } else if (ready) {
        VL53L4CD_Result_t res;
        if (st.l4cd->VL53L4CD_GetResult(&res) == VL53L4CD_ERROR_NONE) {
          st.l4cd->VL53L4CD_ClearInterrupt();
          st.consecutiveErrors = 0;
          st.lastSampleMs = millis();
          st.frameCount++;
          st.rangeStatus = res.range_status;
          st.signalKcps = res.signal_per_spad_kcps;
          st.sigmaMm = res.sigma_mm;
          st.distMm = (res.range_status == 0) ? (int16_t)res.distance_mm : (int16_t)res.distance_mm;
        }
      }
    } else if (st.type == TofSensorType::VL53L7CX && st.l7cx != nullptr) {
      uint8_t ready = 0;
      uint8_t err = st.l7cx->vl53l7cx_check_data_ready(&ready);
      if (err != 0) {
        if (++st.consecutiveErrors >= 4) {
          resetChannelState(ch);
        }
      } else if (ready) {
        if (st.l7cx->vl53l7cx_get_ranging_data(&l7cxResultsShared) == 0) {
          st.consecutiveErrors = 0;
          st.lastSampleMs = millis();
          st.frameCount++;
          st.siliconTempC = l7cxResultsShared.silicon_temp_degc;

          int32_t sumValid = 0;
          uint8_t countValid = 0;
          int16_t minVal = 32767;

          for (int z = 0; z < 64; ++z) {
            uint8_t tstat = l7cxResultsShared.target_status[z];
            uint8_t nTargets = l7cxResultsShared.nb_target_detected[z];
            int16_t d = l7cxResultsShared.distance_mm[z];
            st.gridStatus[z] = tstat;
            // Status 5 (100% valid), 6, 9, or 10 with >=1 target and positive distance
            bool valid = (nTargets > 0) && (d > 0) && (tstat == 5 || tstat == 6 || tstat == 9 || tstat == 10);
            if (valid) {
              st.gridMm[z] = d;
              sumValid += d;
              countValid++;
              if (d < minVal) {
                minVal = d;
              }
            } else {
              st.gridMm[z] = -1;
            }
          }

          st.validZones = countValid;
          st.minMm = (countValid > 0) ? minVal : -1;
          st.meanMm = (countValid > 0) ? (int16_t)(sumValid / countValid) : -1;

          // Center 2x2 zones in 8x8 grid: indices 27 (r3,c3), 28 (r3,c4), 35 (r4,c3), 36 (r4,c4)
          const int centerIdx[4] = {27, 28, 35, 36};
          int32_t centerSum = 0;
          int centerCnt = 0;
          for (int k = 0; k < 4; ++k) {
            int16_t cd = st.gridMm[centerIdx[k]];
            if (cd > 0) {
              centerSum += cd;
              centerCnt++;
            }
          }
          st.centerMm = (centerCnt > 0) ? (int16_t)(centerSum / centerCnt) : st.minMm;
        }
      }
    }

    writeMuxMask(0x00);
    break;
  }
}

static void printTofSnapshotJson(Stream& out) {
  unsigned long now = millis();
  int activeCount = 0;
  for (uint8_t ch = 0; ch < MUX_NUM_CHANNELS; ++ch) {
    if (tofChannels[ch].present) {
      activeCount++;
    }
  }

  out.printf("[ESP32S3_TOF] {\"fw\":\"%s\",\"uptime_ms\":%lu,\"mux_ok\":%d,\"i2c_hz\":%lu,\"active_sensors\":%d,\"channels\":[",
             ESP32S3_FW_VERSION,
             now,
             muxAvailable ? 1 : 0,
             (unsigned long)I2C_FREQ_HZ,
             activeCount);

  for (uint8_t ch = 0; ch < MUX_NUM_CHANNELS; ++ch) {
    const TofChannelState& st = tofChannels[ch];
    if (ch > 0) {
      out.print(",");
    }
    unsigned long ageMs = (st.lastSampleMs > 0 && now >= st.lastSampleMs) ? (now - st.lastSampleMs) : 999999UL;
    out.printf("{\"ch\":%u,\"present\":%d,\"type\":\"%s\",\"init\":%d,\"fail\":%d,\"frames\":%lu,\"age_ms\":%lu",
               ch,
               st.present ? 1 : 0,
               tofTypeName(st.type),
               st.initialized ? 1 : 0,
               st.initFailed ? 1 : 0,
               (unsigned long)st.frameCount,
               ageMs);

    if (st.type == TofSensorType::VL53L4CD && st.initialized) {
      out.printf(",\"dist_mm\":%d,\"status\":%u,\"signal_kcps\":%u,\"sigma_mm\":%u",
                 (int)st.distMm,
                 (unsigned int)st.rangeStatus,
                 (unsigned int)st.signalKcps,
                 (unsigned int)st.sigmaMm);
    } else if (st.type == TofSensorType::VL53L7CX && st.initialized) {
      out.printf(",\"min_mm\":%d,\"center_mm\":%d,\"mean_mm\":%d,\"valid_zones\":%u,\"temp_c\":%d,\"grid_mm\":[",
                 (int)st.minMm,
                 (int)st.centerMm,
                 (int)st.meanMm,
                 (unsigned int)st.validZones,
                 (int)st.siliconTempC);
      for (int z = 0; z < 64; ++z) {
        if (z > 0) out.print(",");
        out.print((int)st.gridMm[z]);
      }
      out.print("]");
    }
    out.print("}");
  }
  out.println("]}");
}

static void runMuxScanReport(Stream& replyStream) {
  muxAvailable = probeI2cAddress(MUX_I2C_ADDR);
  replyStream.printf("[ESP32S3_I2C] SDA=%d SCL=%d FREQ=%lu MUX_ADDR=0x%02X PRESENT=%d\n",
                     I2C_SDA_PIN, I2C_SCL_PIN, (unsigned long)I2C_FREQ_HZ, MUX_I2C_ADDR, muxAvailable ? 1 : 0);
  if (!muxAvailable) {
    replyStream.printf("[ESP32S3_I2C] SUMMARY MUX_0x70=FAIL CHANNELS_OK=0/8 DESELECTED=0 TOTAL_SENSORS=0\n");
    return;
  }

  int channelsOk = 0;
  int totalSensors = 0;

  for (uint8_t ch = 0; ch < MUX_NUM_CHANNELS; ++ch) {
    uint8_t mask = (uint8_t)(1U << ch);
    uint8_t readback = 0xFF;
    bool selOk = selectMuxChannel(ch) && writeMuxMask(mask, &readback);
    if (selOk) {
      channelsOk++;
    }

    Wire.beginTransmission(TOF_I2C_ADDR);
    uint8_t err29 = Wire.endTransmission();

    String devList = "";
    if (selOk) {
      for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
        if (addr == MUX_I2C_ADDR) {
          continue;
        }
        if (probeI2cAddress(addr)) {
          totalSensors++;
          char buf[8];
          snprintf(buf, sizeof(buf), "0x%02X", addr);
          if (devList.length() > 0) {
            devList += ",";
          }
          devList += buf;
        }
      }
    }
    if (devList.length() == 0) {
      devList = "NONE";
    }

    replyStream.printf("[ESP32S3_I2C] CH=%u MASK=0x%02X SELECT_OK=%d READBACK=0x%02X ERR29=%u DEVICES=%s TYPE=%s\n",
                       ch,
                       mask,
                       selOk ? 1 : 0,
                       readback,
                       (unsigned int)err29,
                       devList.c_str(),
                       tofTypeName(tofChannels[ch].type));
  }

  uint8_t deselectReadback = 0xFF;
  bool deselectOk = writeMuxMask(0x00, &deselectReadback);
  replyStream.printf("[ESP32S3_I2C] DESELECT MASK=0x00 DESELECT_OK=%d READBACK=0x%02X\n",
                     deselectOk ? 1 : 0, deselectReadback);
  replyStream.printf("[ESP32S3_I2C] SUMMARY MUX_0x70=OK CHANNELS_OK=%d/8 DESELECTED=%d TOTAL_SENSORS=%d\n",
                     channelsOk, deselectOk ? 1 : 0, totalSensors);
}

static void printOtaStatusReport(Stream& replyStream) {
  bool connected = (WiFi.status() == WL_CONNECTED) && !wifiForceOffline;
  String ipStr = connected ? WiFi.localIP().toString() : "0.0.0.0";
  long rssi = connected ? WiFi.RSSI() : 0;
  const char* wifiState = wifiForceOffline ? "FORCED_OFFLINE" : (connected ? "CONNECTED" : "DISCONNECTED");

  replyStream.printf(
      "[ESP32S3_OTA] FW=%s PART=%s WIFI=%s IP=%s RSSI=%ld HOST=%s PORT=3232 AUTH=MD5_CHALLENGE STATE=%s\n",
      ESP32S3_FW_VERSION,
      activePartitionLabel(),
      wifiState,
      ipStr.c_str(),
      rssi,
      ESP32S3_OTA_HOSTNAME,
      otaStateName(currentOtaState));
}

static void configureArduinoOtaCallbacks() {
  ArduinoOTA.setPort(3232);
  ArduinoOTA.setHostname(ESP32S3_OTA_HOSTNAME);
  ArduinoOTA.setPassword(ESP32S3_OTA_PASSWORD);
  ArduinoOTA.setRebootOnSuccess(false);

  ArduinoOTA.onStart([]() {
    currentOtaState = OtaState::OTA_UPDATING;
    lastOtaPctLogged = 0;
    const char* typeStr = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
    char buf[128];
    snprintf(buf, sizeof(buf), "[ESP32S3_OTA] EVENT=START TYPE=%s TARGET_FW_CURRENT=%s", typeStr, ESP32S3_FW_VERSION);
    broadcastLog(buf);
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    if (total == 0) {
      return;
    }
    unsigned int pct = (progress * 100U) / total;
    if (pct >= lastOtaPctLogged + 25U || pct == 100U) {
      lastOtaPctLogged = pct;
      char buf[96];
      snprintf(buf, sizeof(buf), "[ESP32S3_OTA] EVENT=PROGRESS PCT=%u BYTES=%u/%u", pct, progress, total);
      broadcastLog(buf);
    }
  });

  ArduinoOTA.onEnd([]() {
    broadcastLog("[ESP32S3_OTA] EVENT=END STATUS=SUCCESS REBOOTING=1");
    Serial.flush();
    if (uartTxEnabled) {
      Serial1.flush();
    }
    // Place GPIO17 TX back into high-Z INPUT right before reboot
    setUartTxDriven(false);
    delay(80);
    ESP.restart();
  });

  ArduinoOTA.onError([](ota_error_t error) {
    currentOtaState = OtaState::OTA_ERROR;
    const char* errName = "UNKNOWN";
    if (error == OTA_AUTH_ERROR) errName = "AUTH_FAILED";
    else if (error == OTA_BEGIN_ERROR) errName = "BEGIN_FAILED";
    else if (error == OTA_CONNECT_ERROR) errName = "CONNECT_FAILED";
    else if (error == OTA_RECEIVE_ERROR) errName = "RECEIVE_FAILED";
    else if (error == OTA_END_ERROR) errName = "END_FAILED";
    char buf[96];
    snprintf(buf, sizeof(buf), "[ESP32S3_OTA] EVENT=ERROR CODE=%u REASON=%s", (unsigned int)error, errName);
    broadcastLog(buf);
  });
}

static void startWifiStationNonBlocking() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setHostname(ESP32S3_OTA_HOSTNAME);
  WiFi.begin(ESP32S3_WIFI_SSID, ESP32S3_WIFI_PASSWORD);
  lastWifiCheckMs = millis();
}

static void serviceWifiAndOta(unsigned long now) {
  if (wifiForceOffline) {
    if (now >= wifiForceOfflineUntilMs) {
      wifiForceOffline = false;
      broadcastLog("[ESP32S3_OTA] EVENT=WIFI_TEST_END ACTION=RECONNECTING");
      startWifiStationNonBlocking();
    } else {
      return;
    }
  }

  bool connected = (WiFi.status() == WL_CONNECTED);
  if (connected && !lastWifiConnected) {
    lastWifiConnected = true;
    if (!otaInitialized) {
      configureArduinoOtaCallbacks();
      ArduinoOTA.begin();
      otaInitialized = true;
    }
    currentOtaState = OtaState::OTA_READY;
    char buf[128];
    snprintf(buf,
             sizeof(buf),
             "[ESP32S3_OTA] EVENT=WIFI_READY IP=%s RSSI=%ld HOST=%s",
             WiFi.localIP().toString().c_str(),
             WiFi.RSSI(),
             ESP32S3_OTA_HOSTNAME);
    broadcastLog(buf);
  } else if (!connected && lastWifiConnected) {
    lastWifiConnected = false;
    if (currentOtaState != OtaState::OTA_UPDATING) {
      currentOtaState = OtaState::OTA_DISABLED;
    }
    broadcastLog("[ESP32S3_OTA] EVENT=WIFI_LOST STATE=DISCONNECTED");
  }

  if (connected && otaInitialized) {
    ArduinoOTA.handle();
  } else if (!connected && (now - lastWifiCheckMs >= WIFI_RECONNECT_INTERVAL_MS)) {
    lastWifiCheckMs = now;
    WiFi.reconnect();
  }
}

static void handleCommandLine(const String& rawLine, Stream& replyStream, bool isGpioUart) {
  String cmd = rawLine;
  cmd.trim();
  cmd.toUpperCase();
  if (cmd.length() == 0) {
    return;
  }

  unsigned long now = millis();
  if (cmd == "G" || cmd == "GREEN" || cmd == "ON" || cmd == "IDLE") {
    lastHeartbeatMs = now;
    hasHeartbeat = true;
    if (isGpioUart) {
      setUartTxDriven(true);
    }
    setState(LedState::RPI_ON_GREEN);
  } else if (cmd == "B" || cmd == "BLUE" || cmd == "MOVING" || cmd == "MOVE") {
    lastHeartbeatMs = now;
    hasHeartbeat = true;
    if (isGpioUart) {
      setUartTxDriven(true);
    }
    setState(LedState::ROVER_MOVING_BLUE);
  } else if (cmd == "R" || cmd == "RED" || cmd == "OFF" || cmd == "SHUTDOWN") {
    hasHeartbeat = false;
    setUartTxDriven(false);
    setState(LedState::RPI_OFF_RED);
  } else if (cmd == "?" || cmd == "STATUS") {
    if (!isGpioUart || uartTxEnabled) {
      bool muxOk = probeI2cAddress(MUX_I2C_ADDR);
      bool wifiConn = (WiFi.status() == WL_CONNECTED) && !wifiForceOffline;
      String ipStr = wifiConn ? WiFi.localIP().toString() : "0.0.0.0";
      const char* wifiStr = wifiForceOffline ? "FORCED_OFFLINE" : (wifiConn ? "CONNECTED" : "DISCONNECTED");
      replyStream.printf(
          "[ESP32S3_LED] STATUS=%s HEARTBEAT=%d AGE_MS=%lu MUX_0x70=%s WIFI=%s IP=%s OTA=%s FW=%s PART=%s UPTIME_MS=%lu\n",
          stateName(currentState),
          hasHeartbeat ? 1 : 0,
          hasHeartbeat ? (now - lastHeartbeatMs) : 0UL,
          muxOk ? "OK" : "NONE",
          wifiStr,
          ipStr.c_str(),
          otaStateName(currentOtaState),
          ESP32S3_FW_VERSION,
          activePartitionLabel(),
          now);
    }
  } else if (cmd == "OTA" || cmd == "WIFI") {
    lastHeartbeatMs = now;
    hasHeartbeat = true;
    if (isGpioUart) {
      setUartTxDriven(true);
    }
    printOtaStatusReport(replyStream);
  } else if (cmd.startsWith("WIFI_OFF_TEST")) {
    lastHeartbeatMs = now;
    hasHeartbeat = true;
    if (isGpioUart) {
      setUartTxDriven(true);
    }
    unsigned long durSec = 8;
    int spaceIdx = cmd.indexOf(' ');
    if (spaceIdx > 0) {
      long parsed = cmd.substring(spaceIdx + 1).toInt();
      if (parsed >= 2 && parsed <= 60) {
        durSec = (unsigned long)parsed;
      }
    }
    wifiForceOffline = true;
    wifiForceOfflineUntilMs = now + (durSec * 1000UL);
    lastWifiConnected = false;
    currentOtaState = OtaState::OTA_DISABLED;
    WiFi.disconnect(true, false);
    replyStream.printf("[ESP32S3_OTA] EVENT=WIFI_TEST_START DURATION_S=%lu WIFI=FORCED_OFFLINE\n", durSec);
  } else if (cmd == "WIFI_ON") {
    lastHeartbeatMs = now;
    hasHeartbeat = true;
    if (isGpioUart) {
      setUartTxDriven(true);
    }
    wifiForceOffline = false;
    startWifiStationNonBlocking();
    replyStream.printf("[ESP32S3_OTA] EVENT=WIFI_RECONNECT_TRIGGERED\n");
  } else if (cmd == "I2C" || cmd == "MUX" || cmd == "SCAN") {
    lastHeartbeatMs = now;
    hasHeartbeat = true;
    if (isGpioUart) {
      setUartTxDriven(true);
    }
    scanAndInitAllTofChannels(false);
    runMuxScanReport(replyStream);
  } else if (cmd == "TOF") {
    lastHeartbeatMs = now;
    hasHeartbeat = true;
    if (isGpioUart) {
      setUartTxDriven(true);
    }
    printTofSnapshotJson(replyStream);
  } else if (cmd == "TOF_RESCAN") {
    lastHeartbeatMs = now;
    hasHeartbeat = true;
    if (isGpioUart) {
      setUartTxDriven(true);
    }
    scanAndInitAllTofChannels(false);
    runMuxScanReport(replyStream);
    printTofSnapshotJson(replyStream);
  } else if (cmd == "TOF_FORCE_REINIT") {
    lastHeartbeatMs = now;
    hasHeartbeat = true;
    if (isGpioUart) {
      setUartTxDriven(true);
    }
    scanAndInitAllTofChannels(true);
    runMuxScanReport(replyStream);
    printTofSnapshotJson(replyStream);
  }
}

void setup() {
  // Do not block if USB cable is unplugged (ARDUINO_USB_CDC_ON_BOOT=1)
  Serial.setTxTimeoutMs(0);
  Serial.begin(UART_BAUD);

  // Initialize GPIO17 in high-Z INPUT until RPi5 proves it is powered by sending a heartbeat on GPIO18
  pinMode(UART_TX_PIN, INPUT);
  Serial1.setRxBufferSize(512);
  Serial1.setTxBufferSize(2048);
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, -1);
  uartTxEnabled = false;

  for (uint8_t ch = 0; ch < MUX_NUM_CHANNELS; ++ch) {
    resetChannelState(ch);
  }

  // Initialize I2C bus on SDA=GPIO8, SCL=GPIO9 at 400 kHz with 1024-byte buffer for ST ToF transfers
  Wire.setBufferSize(1024);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
  Wire.setTimeOut(150);
  if (probeI2cAddress(MUX_I2C_ADDR)) {
    muxAvailable = true;
    writeMuxMask(0x00);
  }

  pixel48.begin();
  pixel48.setBrightness(LED_BRIGHTNESS);
  pixel38.begin();
  pixel38.setBrightness(LED_BRIGHTNESS);

  // Default to Solid Red until RPi5 heartbeat arrives
  currentState = LedState::RPI_OFF_RED;
  applyLedColor(currentState);
  lastRefreshMs = millis();

  // Start Wi-Fi in Station Mode non-blocking (never creates an AP)
  startWifiStationNonBlocking();

  // Discover and initialize any ToF sensors connected on boot
  scanAndInitAllTofChannels(false);
}

void loop() {
  static String usbRxBuf;
  static String uartRxBuf;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (usbRxBuf.length() > 0) {
        handleCommandLine(usbRxBuf, Serial, false);
        usbRxBuf = "";
      }
    } else if (usbRxBuf.length() < 64) {
      usbRxBuf += c;
    } else {
      usbRxBuf = "";
    }
  }

  while (Serial1.available() > 0) {
    char c = (char)Serial1.read();
    if (c == '\n' || c == '\r') {
      if (uartRxBuf.length() > 0) {
        handleCommandLine(uartRxBuf, Serial1, true);
        uartRxBuf = "";
      }
    } else if (uartRxBuf.length() < 64) {
      uartRxBuf += c;
    } else {
      uartRxBuf = "";
    }
  }

  unsigned long now = millis();

  // Service non-blocking Wi-Fi station and ArduinoOTA state machine
  serviceWifiAndOta(now);

  // Revert to Solid Red and release GPIO17 TX to high-Z if RPi5 heartbeat stops arriving
  if (hasHeartbeat && (now - lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS)) {
    hasHeartbeat = false;
    setUartTxDriven(false);
    setState(LedState::RPI_OFF_RED);
  }

  // Poll active ToF sensors round-robin
  if (now - lastTofPollMs >= TOF_POLL_INTERVAL_MS) {
    lastTofPollMs = now;
    pollNextActiveTofChannel();
  }

  // Hot-plug scan of uninitialized mux channels every 2.5s so newly plugged sensors auto-initialize
  if (now - lastHotplugScanMs >= TOF_HOTPLUG_SCAN_INTERVAL_MS) {
    lastHotplugScanMs = now;
    scanAndInitAllTofChannels(false);
  }

  // Broadcast live ToF JSON snapshot at 4 Hz when UART TX is active
  if (now - lastTofBroadcastMs >= TOF_BROADCAST_INTERVAL_MS) {
    lastTofBroadcastMs = now;
    if (uartTxEnabled && currentOtaState != OtaState::OTA_UPDATING) {
      printTofSnapshotJson(Serial1);
    }
  }

  // Periodic refresh every 250ms to guard against any power-glitch pixel reset
  if (now - lastRefreshMs >= 250) {
    lastRefreshMs = now;
    applyLedColor(currentState);
  }

  delay(3);
}
