# Lonely Binary ESP32-S3 N16R8 — Status LED, I2C Multiplexer & Secure Wi-Fi OTA

Isolated PlatformIO firmware and Raspberry Pi 5 host daemon/deployer for Rover 1's auxiliary **Lonely Binary ESP32-S3 N16R8** (`16MB` Quad Flash, `8MB` PSRAM).

---

## 1. Hardware & Pin Configuration (Preserved Invariants)

- **Power**: Powered by 5V from the **Geekworm X1206 UPS** (`JST 5V`), sharing common ground with the Raspberry Pi 5.
- **Primary Data Connection (RPi5 40-Pin Header GPIO UART0, `115200` baud)**:
  - **RPi5 `GPIO 14 / TXD0` (Pin 8)** $\to$ **ESP32-S3 `GPIO 18 / RX`**
  - **RPi5 `GPIO 15 / RXD0` (Pin 10)** $\leftarrow$ **ESP32-S3 `GPIO 17 / TX`**
  - **Unpowered-RPi5 Backfeed Protection**: `GPIO 17` starts in high-impedance `INPUT` mode and is only connected to the `UART1 TX` output matrix while an active RPi5 heartbeat (`G` / `B` / query) is present on `GPIO 18`. If `"R\n"` is received or heartbeats stop for `> 1.5 s`, `GPIO 17` immediately detaches back to high-Z `INPUT`.
- **I2C Multiplexer (`TCA9548A` / `PCA9548A`, addr `0x70`)**:
  - `SDA = GPIO 8`, `SCL = GPIO 9` at `200 kHz` (`150 ms` bus timeout, `1024-byte` Wire buffer), 8 downstream channels (`CH 0..7`).
  - Currently populated with:
    - **`CH 0`**: Front-Right `VL53L7CX` 8×8 Wide-Field ToF (`0x29`, angled $8^\circ$ outward right)
    - **`CH 1`**: Front-Left `VL53L7CX` 8×8 Wide-Field ToF (`0x29`, angled $8^\circ$ outward left)
    - **`CH 2`**: `VL53L4CD` Single-Zone ToF (`0x29`, wired for testing; not yet mapped to radar/ROS 2)
    - **`CH 3..7`**: Open for remaining `VL53L4CD` cliff / auxiliary sensors (auto-discovered via 2.5s hot-plug scan).
- **Onboard WS2812 RGB Status LED (`GPIO 48` / `GPIO 38`)**:
  - **Solid Green**: RPi5 is ON and Rover 1 is stationary/idle.
  - **Solid Blue**: RPi5 is ON and Rover 1 is actively moving.
  - **Solid Red**: RPi5 is OFF/halted or heartbeats have timed out (`> 1.5 s`).
- **Strict Separation from Motor Controller**:
  - Never touches `/dev/rover-esp32` or `/dev/ttyUSB0` (`1a86:7523`, NULLLAB Maker-ESP32-Pro) or `rover-server.service`.

---

## 2. Flash Layout & Partition Table (`default_16MB.csv`)

Configured in [`platformio.ini`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/esp32s3_status_led/platformio.ini) via `board_build.partitions = default_16MB.csv`:

| Partition | Type / SubType | Offset | Size | Purpose |
| :--- | :--- | :--- | :--- | :--- |
| `nvs` | `data` / `nvs` | `0x009000` | `20 KB` | Non-volatile storage |
| `otadata` | `data` / `ota` | `0x00E000` | `8 KB` | Active OTA slot selector (`app0` $\leftrightarrow$ `app1`) |
| `app0` | `app` / `ota_0` | `0x010000` | `6.25 MB` (`0x640000`) | Dual OTA slot 0 (firmware `1.2.4-tof-mux` uses ~`885 KB` / `13.5%`) |
| `app1` | `app` / `ota_1` | `0x650000` | `6.25 MB` (`0x640000`) | Dual OTA slot 1 |
| `spiffs` | `data` / `spiffs` | `0xC90000` | `3.375 MB` | Filesystem partition |
| `coredump` | `data` / `coredump` | `0xFF0000` | `64 KB` | Crash dump storage |

---

## 3. Wi-Fi & OTA Security Architecture and Limitations

- **Local Home Wi-Fi Station Mode Only (`WIFI_STA`)**:
  - Associates only with the existing WPA2-encrypted home Wi-Fi network (`SSID='Dobby'`).
  - Never creates a soft-AP hotspot and is never exposed outside the local LAN.
- **Authentication & Integrity vs. Encryption / Code Signing**:
  - Uses `ArduinoOTA` (`UDP/TCP port 3232`, hostname `rover1-esp32s3`) with **MD5 challenge-response authentication** (`AUTH=MD5_CHALLENGE`) backed by a 256-bit random secret (`ESP32S3_OTA_PASSWORD`) and full-image MD5 integrity verification prior to switching boot partitions.
  - **Important Security Distinction**:
    - Every update requires challenge-response authentication (unauthenticated or wrong-password requests are rejected with `OTA_AUTH_ERROR`).
    - Wireless transport on the LAN is protected by WPA2 link-layer encryption, **not** application-layer TLS, and the firmware binary is **not** cryptographically signed with an asymmetric private key (ESP32-S3 hardware Secure Boot / Flash Encryption are intentionally not burned into eFuses so USB recovery remains unrestricted).
- **Secret Hygiene**:
  - Wi-Fi and OTA secrets are stored exclusively in git-ignored local files:
    - `esp32s3_status_led/include/ota_secrets.h` (compiled into firmware; template in [`include/ota_secrets.h.example`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/esp32s3_status_led/include/ota_secrets.h.example))
    - `esp32s3_status_led/.ota_secrets.json` (used by `deploy_esp32s3.py`)
  - Secrets are never printed to UART, stdout, build logs, or passed in command-line arguments (`sys.argv`) on the RPi5.

---

## 4. Setup, Routine OTA Update & Diagnostic Commands

### A. Provision Local Secrets (First-Time or Rotation)
Automatically reads the home Wi-Fi SSID/PSK from RPi5 NetworkManager over SSH and generates a 256-bit OTA secret into the git-ignored local files:
```powershell
python esp32s3_status_led/deploy_esp32s3.py --provision-secrets
```

### B. Routine Wireless Firmware Update (`--ota`, USB Unplugged)
Compiles the firmware locally, queries `/dev/ttyAMA0` on the RPi5 to verify target identity (`HOST=rover1-esp32s3`), live IP, and active partition (`app0`/`app1`), pushes the update over LAN while `esp32s3-status-led.service` coordinates the maintenance window, and verifies post-reboot UART reconnection and `0x70` I2C multiplexer health:
```powershell
python esp32s3_status_led/deploy_esp32s3.py --ota
# Optional explicit version string:
python esp32s3_status_led/deploy_esp32s3.py --ota --fw-version 1.2.4-tof-mux
```

### C. Query Status, OTA Readiness, and Multiplexer over RPi5 GPIO UART0
On the RPi5 (`ron@10.0.0.246`, pausing `esp32s3-status-led.service` briefly when running one-shot queries):
```bash
sudo systemctl stop esp32s3-status-led.service
python3 /home/ron/esp32s3_status_led/rpi5_status_led_daemon.py --ota-status
python3 /home/ron/esp32s3_status_led/rpi5_status_led_daemon.py --status
python3 /home/ron/esp32s3_status_led/rpi5_status_led_daemon.py --scan-mux
sudo systemctl start esp32s3-status-led.service
```

### D. Bounded Wi-Fi Unavailable Resilience Test
Forces the ESP32-S3 Wi-Fi interface offline for `N` seconds (default `8 s`) without disrupting the RPi5's network connection, verifying that UART heartbeats, LED control, and I2C multiplexer scans continue unimpeded and Wi-Fi/OTA automatically reconnects afterward:
```bash
sudo systemctl stop esp32s3-status-led.service
python3 /home/ron/esp32s3_status_led/rpi5_status_led_daemon.py --wifi-off-test 8
python3 /home/ron/esp32s3_status_led/rpi5_status_led_daemon.py --status
python3 /home/ron/esp32s3_status_led/rpi5_status_led_daemon.py --scan-mux
sleep 9
python3 /home/ron/esp32s3_status_led/rpi5_status_led_daemon.py --ota-status
sudo systemctl start esp32s3-status-led.service
```

---

## 5. USB Recovery Path (`--usb`)

If Wi-Fi is unreachable, credentials change, or the partition table needs to be re-flashed:
1. Temporarily connect the ESP32-S3 native **USB** port (`303a:1001`, `/dev/ttyACM0`) to the RPi5.
2. Run the USB bootstrap/recovery deployer:
   ```powershell
   python esp32s3_status_led/deploy_esp32s3.py --usb
   ```
3. Unplug the USB cable once flashing and verification complete.
