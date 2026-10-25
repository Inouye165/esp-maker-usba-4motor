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

#if __has_include("ota_secrets.h")
#include "ota_secrets.h"
#else
#error "Missing esp32s3_status_led/include/ota_secrets.h! Copy ota_secrets.h.example or run deploy_esp32s3.py --provision-secrets."
#endif

#ifndef ESP32S3_FW_VERSION
#define ESP32S3_FW_VERSION "1.1.0-ota"
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

// I2C Multiplexer (TCA9548A / PCA9548A) on ESP32-S3 default I2C pins
static constexpr int I2C_SDA_PIN = 8;
static constexpr int I2C_SCL_PIN = 9;
static constexpr uint32_t I2C_FREQ_HZ = 100000;
static constexpr uint8_t MUX_I2C_ADDR = 0x70;
static constexpr uint8_t MUX_NUM_CHANNELS = 8;

// Timeout without RPi5 heartbeat before reverting to Solid Red (RPi5 off/disconnected)
static constexpr unsigned long HEARTBEAT_TIMEOUT_MS = 1500;

// Non-blocking Wi-Fi reconnect check interval
static constexpr unsigned long WIFI_RECONNECT_INTERVAL_MS = 15000;

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
  if (Wire.requestFrom(MUX_I2C_ADDR, (uint8_t)1) != 1) {
    return false;
  }
  uint8_t rb = (uint8_t)Wire.read();
  if (readbackOut != nullptr) {
    *readbackOut = rb;
  }
  return (rb == mask);
}

static void runMuxScanReport(Stream& replyStream) {
  bool muxPresent = probeI2cAddress(MUX_I2C_ADDR);
  replyStream.printf("[ESP32S3_I2C] SDA=%d SCL=%d FREQ=%lu MUX_ADDR=0x%02X PRESENT=%d\n",
                     I2C_SDA_PIN, I2C_SCL_PIN, (unsigned long)I2C_FREQ_HZ, MUX_I2C_ADDR, muxPresent ? 1 : 0);
  if (!muxPresent) {
    replyStream.printf("[ESP32S3_I2C] SUMMARY MUX_0x70=FAIL CHANNELS_OK=0/8 DESELECTED=0 TOTAL_SENSORS=0\n");
    return;
  }

  int channelsOk = 0;
  int totalSensors = 0;

  for (uint8_t ch = 0; ch < MUX_NUM_CHANNELS; ++ch) {
    uint8_t mask = (uint8_t)(1U << ch);
    uint8_t readback = 0xFF;
    bool selOk = writeMuxMask(mask, &readback);
    if (selOk) {
      channelsOk++;
    }

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

    replyStream.printf("[ESP32S3_I2C] CH=%u MASK=0x%02X SELECT_OK=%d READBACK=0x%02X DEVICES=%s\n",
                       ch, mask, selOk ? 1 : 0, readback, devList.c_str());
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
    runMuxScanReport(replyStream);
  }
}

void setup() {
  // Do not block if USB cable is unplugged (ARDUINO_USB_CDC_ON_BOOT=1)
  Serial.setTxTimeoutMs(0);
  Serial.begin(UART_BAUD);

  // Initialize GPIO17 in high-Z INPUT until RPi5 proves it is powered by sending a heartbeat on GPIO18
  pinMode(UART_TX_PIN, INPUT);
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, -1);
  uartTxEnabled = false;

  // Initialize I2C bus on SDA=GPIO8, SCL=GPIO9 at 100 kHz and ensure all mux channels start deselected
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
  Wire.setTimeOut(50);
  if (probeI2cAddress(MUX_I2C_ADDR)) {
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

  // Periodic refresh every 250ms to guard against any power-glitch pixel reset
  if (now - lastRefreshMs >= 250) {
    lastRefreshMs = now;
    applyLedColor(currentState);
  }

  delay(5);
}
