# Rover 1 — Session Handoff (`HANDOFF.md`)

**Date**: 2026-11-05
**Active Branch (Both Repos)**: `feat/esp32s3-tof-sensor-bench`
**Motor Controller Firmware (`esp-maker-usba-4motor`)**: **`1.3.4-remap-r03`** (`source: rev42-remap-c`, commit `a2e3d43`, untouched & verified)
**Auxiliary ESP32-S3 Firmware (`esp32s3_status_led/`)**: **`1.2.3-tof-mux`** (running on ESP32-S3), **`1.2.4-tof-mux`** (audited & compiled in repo, ready for `--ota` flash when RPi5 is powered on)

---

## 1. Current Hardware & Sensor Architecture

### A. Primary Motor Controller — NULLLAB Maker-ESP32-Pro (`1a86:7523`, `/dev/rover-esp32` → `/dev/ttyUSB0`)
- Owned exclusively by `rover-server.service` (`115200` baud).
- Active firmware **`1.3.4-remap-r03`** with calibrated rebuilt-chassis motor/encoder mapping (`LF=M2/E2`, `RF=M1/E1`, `LR=M4/E4`, `RR=M3/E3`), `0.06695 m` effective wheel diameter, `0.3408575433 m` track width, `0.0135 m` linear stopping advance, and `0.7°` angular stopping advance.
- BNO08x IMU on `SDA=21, SCL=22` (`0x4B`) in non-magnetic `SH2_GAME_ROTATION_VECTOR` (`0x08`) mode at `50 Hz`.

### B. Auxiliary Sensor & Status LED Controller — Lonely Binary ESP32-S3 N16R8 (`/dev/ttyAMA0`, `303a:1001`)
- Connected to RPi5 40-pin header UART0 (`RPi5 GPIO 14 TXD0 -> ESP32-S3 GPIO 18 RX`, `RPi5 GPIO 15 RXD0 <- ESP32-S3 GPIO 17 TX`) at `115200` baud; owned by `esp32s3-status-led.service` ([`esp32s3_status_led/rpi5_status_led_daemon.py`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/esp32s3_status_led/rpi5_status_led_daemon.py)).
- `GPIO 17 (TX)` stays in high-Z `INPUT` whenever the RPi5 heartbeat is absent (`> 1.5 s`) so the UPS-powered ESP32-S3 never backfeeds a powered-down RPi5.
- **8-Channel I2C Multiplexer (`TCA9548A`, `0x70`)** on `SDA = GPIO 8`, `SCL = GPIO 9` at `200 kHz` (`150 ms` timeout):

| Mux Channel | Sensor Model | I2C Addr | Model ID | Logical Slot | Role & Status |
| :---: | :--- | :---: | :---: | :--- | :--- |
| **`CH 0`** | **`VL53L7CX`** (8×8 multizone) | `0x29` | `0xF002` | `front_right_l7cx` | Front-Right corner, angled **$-8^\circ$** outward (`rot="90_ccw"`). Live in `/api/tof-sensors`, `/sensors.html`, ROS 2, and HUD Radar. |
| **`CH 1`** | **`VL53L7CX`** (8×8 multizone) | `0x29` | `0xF002` | `front_left_l7cx` | Front-Left corner, angled **$+8^\circ$** outward (`rot="90_ccw"`). Live in `/api/tof-sensors`, `/sensors.html`, ROS 2, and HUD Radar. |
| **`CH 2`** | **`VL53L4CD`** (single-zone) | `0x29` | `0xEBAA` | `unassigned` | Attached and live in `/api/tof-sensors` & `/sensors.html`; intentionally excluded from ROS 2 and HUD Radar until physically mounted/mapped. |
| `CH 3..7` | *(Empty)* | — | — | — | Auto-discovered every `2.5 s` via hot-plug scan. |

---

## 2. ROS 2 & HUD Radar Integration Summary (2026-11-05)

### A. ROS 2 Bridge (`rover_tof_bridge.py` & `foundation.launch.py`)
- **[`rover_tof_bridge.py`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/ros2/ros2_ws/src/rover_bringup/rover_bringup/rover_tof_bridge.py)** polls `http://127.0.0.1:3000/api/tof-sensors` at `10 Hz` using a persistent `requests.Session()` and publishes:
  - `/tof/front_left/points` (`sensor_msgs/msg/PointCloud2`) & `/tof/front_left/range` (`sensor_msgs/msg/Range`, frame `tof_front_left_link`)
  - `/tof/front_right/points` (`sensor_msgs/msg/PointCloud2`) & `/tof/front_right/range` (`sensor_msgs/msg/Range`, frame `tof_front_right_link`)
- **[`foundation.launch.py`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/ros2/ros2_ws/src/rover_bringup/launch/foundation.launch.py)** publishes static TFs from `base_link`:
  - `base_link -> tof_front_left_link`: `x = +0.14 m, y = +0.08 m, z = +0.08 m, yaw = +0.139626 rad (+8°)`
  - `base_link -> tof_front_right_link`: `x = +0.14 m, y = -0.08 m, z = +0.08 m, yaw = -0.139626 rad (-8°)`
  *(Note: `x=0.14, y=±0.08, z=0.08` are initial placeholders; update both `foundation.launch.py` and `TOF_MOUNTS` in `public/hud-core.js` once physically measured with a ruler.)*

### B. Cockpit HUD Radar Overlay & Micro-Zoom (`public/hud-core.js`, `public/hud.html`, `public/hud-mobile.html`)
- `tofToRobotFrame()` projects the closest valid zone in each of the 8 display columns per front `VL53L7CX` sensor into `base_link` coordinates (`x` forward, `y` left).
- `drawRadar()` renders 360° RPLiDAR points alongside front-corner 8×8 ToF points (drawn as `#ff44ee` magenta squares) and reports `{ live, lidarLive, tofLive, nearest }`.
- Added close-in zoom steps to both Desktop and Mobile HUDs so obstacles inches from the front corners are clearly visible around the `34 cm × 30 cm` rover footprint box:
  - **`0.25 m (10 in)`** — rings at `10cm`, `20cm`, `25cm`
  - **`0.5 m (20 in)`** — rings at `10cm`, `25cm`, `50cm`
  - **`1.0 m (3.3 ft)`** — rings at `0.25m`, `0.5m`, `1m`
  - Plus existing `2 m`, `3 m`, `5 m`, `8 m` ranges.

---

## 3. Code & Firmware Audit Fixes Applied (2026-11-05)

1. **[`esp32s3_status_led/src/main.cpp`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/esp32s3_status_led/src/main.cpp) (`1.2.4-tof-mux`)**:
   - **Line 645 (`VL53L4CD` invalid status bug)**: Fixed `(res.range_status == 0) ? (int16_t)res.distance_mm : (int16_t)res.distance_mm` (identical branches) to `(res.range_status == 0 && res.distance_mm > 0) ? (int16_t)res.distance_mm : -1`.
   - **Line 700 (`VL53L7CX` center 2×2 fallback bug)**: Fixed `(cValid > 0) ? (int16_t)(cSum / cValid) : st.minMm` to `: -1` so `distMm` reports `-1` (`-- mm`) when center zones `27, 28, 35, 36` have no valid target instead of masquerading edge targets as center distance.
2. **[`esp32s3_status_led/rpi5_status_led_daemon.py`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/esp32s3_status_led/rpi5_status_led_daemon.py) & [`esp32s3_status_led/deploy_esp32s3.py`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/esp32s3_status_led/deploy_esp32s3.py)**:
   - Updated USB fallback MAC from old replaced board `58:E6:C5:72:3F:FC` to active unit `58:E6:C5:71:6A:98` with dynamic `/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_*-if00` lookup gated by `303a:1001` verification.
   - Added periodic `STATUS\nOTA\n` refresh every `10.0 s` and on LED state changes so `esp_status` and `ota_status` in `/tmp/esp32s3_tof_state.json` stay current.
3. **[`public/hud-core.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/public/hud-core.js), [`public/hud-desktop.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/public/hud-desktop.js), [`public/hud-mobile.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/public/hud-mobile.js)**:
   - Fixed `drawRadar()` so `res.live` is `true` when either LiDAR or Front ToF is active (`FRONT TOF LIVE` / `LIDAR + TOF LIVE`), and updated `NEAREST` / `MIN` badge to display both `cm` and `in` even when RPLiDAR C1 is disconnected.
4. **Tests & Deployment Verification**:
   - Added [`test_tof_bridge.py`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/ros2/ros2_ws/src/rover_bringup/test/test_tof_bridge.py) (`5/5` passed) and `Core 5` in [`test_hud_cockpit.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/test/test_hud_cockpit.js) (`ALL PASSED`).
   - Updated [`test_launch.py`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/ros2/ros2_ws/src/rover_bringup/test/test_launch.py) and [`deploy_yahboom.py`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/deploy_yahboom.py) (`EXPECTED_ROS_NODES`) to include `rover_tof_bridge` and the two ToF static TF publishers.

---

## 4. Next-Session Startup Commands (When Rover 1 is Powered On)

```powershell
# 1. Flash audited ESP32-S3 firmware (1.2.4-tof-mux) and updated daemon via Wi-Fi OTA
cd c:\Users\Ron\electronic_projects\esp\esp-maker-usba-4motor
python esp32s3_status_led/deploy_esp32s3.py --ota

# 2. Deploy audited HUD radar & ROS 2 rover_tof_bridge updates to RPi5
cd c:\Users\Ron\electronic_projects\yahboom-encoder
$env:ROVER_PI_HOST="10.0.0.246"; $env:ROVER_PI_USER="ron"; python deploy_yahboom.py
```

