# Rover 1 — Session Handoff (`HANDOFF.md`)

**Date**: 2026-11-07
**Active Branches**:
- Firmware (`esp-maker-usba-4motor`): `fix/chassis-supervisor-pure-spin-motion-evidence` (commit `012e641`)
- Host (`yahboom-encoder`): `feat/smooth-progressive-manual-turning` (commit `caad519`)
**Motor Controller Firmware (`esp-maker-usba-4motor`)**: **`1.3.4-remap-r03`** (`source: rev42-remap-c`, commit `012e641`, flashed SHA-256 `7f808b16df0e6d8ec6e7206b307178453b514ebf67bf22057207c873cc17bba0`)
**Auxiliary ESP32-S3 Firmware (`esp32s3_status_led/`)**: **`1.2.3-tof-mux`** (running on ESP32-S3), **`1.2.5-tof-mux`** (audited & compiled in repo with `0 mm` close-range contact fix, ready for `--ota` flash when RPi5 is powered on)

---

## 1. Current Hardware & Sensor Architecture

### A. Primary Motor Controller — NULLLAB Maker-ESP32-Pro (`1a86:7523`, `/dev/rover-esp32` → `/dev/ttyUSB0`)
- Owned exclusively by `rover-server.service` (`115200` baud).
- Active firmware **`1.3.4-remap-r03`** with calibrated rebuilt-chassis motor/encoder mapping (`LF=M2/E2`, `RF=M1/E1`, `LR=M4/E4`, `RR=M3/E3`), `0.06695 m` effective wheel diameter, `0.3408575433 m` track width, `0.0135 m` linear stopping advance, and `0.7°` angular stopping advance.
- Two-tier pure-spin motion evidence implemented in [`src/ChassisSupervisor.h`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/src/ChassisSupervisor.h):
  - **Tier 1 (Healthy IMU)**: strictly requires gyro yaw rate ($|gz| \ge 0.04\text{ rad/s}$) to preserve obstacle stall detection.
  - **Tier 2 (Degraded / Offline IMU)**: falls back to opposing wheel encoder movement (`movingLeft >= 1 && movingRight >= 1`).
- BNO08x IMU on `SDA=21, SCL=22` (`0x4B`/`0x4A` probe) in non-magnetic `SH2_GAME_ROTATION_VECTOR` (`0x08`) mode at `50 Hz`. *Hardware note: BNO08x does not currently ACK on I2C bus; Tier 2 encoder fallback enables manual driving and bench verification.*

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
  - `base_link -> tof_front_left_link`: `x = +0.14 m, y = +0.0625 m, z = +0.08 m, yaw = +0.13962634 rad (+8°)`
  - `base_link -> tof_front_right_link`: `x = +0.14 m, y = -0.0625 m, z = +0.08 m, yaw = -0.13962634 rad (-8°)`
  *(Note: Lateral separation physically measured at 125 mm [y = ±0.0625 m, ±8° outward]; forward position x = +0.14 m and height z = +0.08 m remain unverified placeholder measurements.)*

### B. Cockpit HUD Radar Overlay & Micro-Zoom (`public/hud-core.js`, `public/hud.html`, `public/hud-mobile.html`)
- `tofToRobotFrame()` projects the closest valid zone in each of the 8 display columns per front `VL53L7CX` sensor into `base_link` coordinates (`x` forward, `y` left).
- `drawRadar()` renders 360° RPLiDAR points alongside front-corner 8×8 ToF points (drawn as `#ff44ee` magenta squares) and reports `{ live, lidarLive, tofLive, nearest }`.
- Added close-in zoom steps to both Desktop and Mobile HUDs so obstacles inches from the front corners are clearly visible around the `34 cm × 30 cm` rover footprint box:
  - **`0.25 m (10 in)`** — rings at `10cm`, `20cm`, `25cm`
  - **`0.5 m (20 in)`** — rings at `10cm`, `25cm`, `50cm`
  - **`1.0 m (3.3 ft)`** — rings at `0.25m`, `0.5m`, `1m`
  - Plus existing `2 m`, `3 m`, `5 m`, `8 m` ranges.

### C. Front Obstacle Safety Zone (`<= 75 mm` Crawl / `<= 30 mm` Stop) (`front_safety_zone.js` & `server.js`)
- **[`front_safety_zone.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/front_safety_zone.js)** evaluates the closest valid reading across the front-mounted ToF sensors (`front_left_l7cx` on `CH 1` and `front_right_l7cx` on `CH 0`):
  - **Crawl Zone (`30 mm < minMm <= 75 mm`)**: Clamps forward velocity (`linear > 0`) to `0.03 m/s`, scales `angular` proportionally to preserve curvature, and caps in-place rotation to `±0.35 rad/s`.
  - **Stop Zone (`0 mm <= minMm <= 30 mm`)**: Immediately zeroes all forward and in-place motion (`linear = 0.0, angular = 0.0` for `linear >= 0`), while allowing **reverse motion (`linear < 0`)** (`REVERSE_ALLOWED`, angular capped to `±0.35 rad/s`) so the operator or autonomy can back away from the obstacle.
- Enforced in [`server.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/server.js) for **both manual driving (`BROWSER`/`GAMEPAD`) and autonomous driving (`ROS_AUTONOMY`/`CALIBRATION_TEST`)**, including post-slew-limiter clamping inside `startDriveKeepaliveLoop()` (`20 Hz`) so speed drops on the immediate `50 ms` tick without waiting for the `0.60 m/s²` autonomy deceleration ramp.
- **Radar Box Proximity Indicator (`#hud-radar-drive-ind`, `evaluateRadarDriveIndicator`)**: Placed **inside `.radar-box` (`bottom: 3px`) and strictly outside the radar circles** (`cy = 0.445 * size`, `outer = 0.335 * size`, with no zone boxes drawn inside the circles) to show:
  - **Proximity Clear**: `PROXIMITY — CLEAR` (> 75 mm)
  - **Proximity Crawl / Slowing**: `PROXIMITY — CRAWL ZONE` / `PROXIMITY — SLOWING` (`31..75 mm`)
  - **Proximity Stop Zone / Stop**: `PROXIMITY — STOP ZONE` / `PROXIMITY — STOP` (`0..30 mm`)
  - **Proximity Offline**: `PROXIMITY — OFFLINE` (missing, disconnected, or stale > 2500 ms)
- **Operator Verification (2026-11-07)**:
  - Ron physically verified actual wheel response on a raised platform with wheels spinning: slowdown occurs at $\le 75\text{ mm}$ and full stop occurs at $\le 30\text{ mm}$, with reverse motion preserved.
  - Floor-driving stopping distance under rover momentum was not tested.
  - Front sensor lateral spacing corrected to 125 mm (`y = ±0.0625 m`), retaining existing $\pm 8^\circ$ angles.
  - Further physical alignment calibration deferred as unnecessary for the current proximity task. Remaining mounting placeholders ($x = +0.14\text{ m}$, $z = +0.08\text{ m}$) remain identified. Task closed out as complete.

---

## 3. Code & Firmware Audit Fixes Applied (2026-11-05 – 2026-11-06)

1. **[`esp32s3_status_led/src/main.cpp`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/esp32s3_status_led/src/main.cpp) (`1.2.5-tof-mux`)**:
   - **Line 645 (`VL53L4CD` invalid status bug)**: Fixed `(res.range_status == 0) ? (int16_t)res.distance_mm : (int16_t)res.distance_mm` (identical branches) to `(res.range_status == 0 && res.distance_mm >= 0) ? (int16_t)res.distance_mm : -1`.
   - **Line 700 (`VL53L7CX` center 2×2 fallback bug)**: Fixed `(cValid > 0) ? (int16_t)(cSum / cValid) : st.minMm` to `: -1` so `distMm` reports `-1` (`-- mm`) when center zones `27, 28, 35, 36` have no valid target instead of masquerading edge targets as center distance.
   - **Close-Range Contact Fix (`0..19 mm`)**: Changed `VL53L7CX` zone filter from `d > 0` to `d >= 0` (and accepted `tstat == 4` when `d <= 60`) so `< 4 mm` contact readings (which `vl53l7cx_api.cpp` integer-divides `distance_mm[i] /= 4` to `0`) are preserved as `0 mm` instead of dropped to `-1`. Lowered `range_min_m` to `0.0` in [`rover_tof_bridge.py`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/ros2/ros2_ws/src/rover_bringup/rover_bringup/rover_tof_bridge.py) and `d >= 0` in [`public/hud-core.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/public/hud-core.js).
2. **[`esp32s3_status_led/rpi5_status_led_daemon.py`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/esp32s3_status_led/rpi5_status_led_daemon.py) & [`esp32s3_status_led/deploy_esp32s3.py`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/esp32s3_status_led/deploy_esp32s3.py)**:
   - Updated USB fallback MAC from old replaced board `58:E6:C5:72:3F:FC` to active unit `58:E6:C5:71:6A:98` with dynamic `/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_*-if00` lookup gated by `303a:1001` verification.
   - Added periodic `STATUS\nOTA\n` refresh every `10.0 s` and on LED state changes so `esp_status` and `ota_status` in `/tmp/esp32s3_tof_state.json` stay current.
3. **[`public/hud-core.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/public/hud-core.js), [`public/hud-desktop.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/public/hud-desktop.js), [`public/hud-mobile.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/public/hud-mobile.js)**:
   - Fixed `drawRadar()` so `res.live` is `true` when either LiDAR or Front ToF is active (`FRONT TOF LIVE` / `LIDAR + TOF LIVE`), and updated `NEAREST` / `MIN` badge to display both `cm` and `in` even when RPLiDAR C1 is disconnected.
4. **Tests & Deployment Verification**:
   - Added [`test_tof_bridge.py`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/ros2/ros2_ws/src/rover_bringup/test/test_tof_bridge.py) (`6/6` passed) and `Core 5` & `Core 6` in [`test_hud_cockpit.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/test/test_hud_cockpit.js) (`ALL PASSED`).
   - Updated [`test_launch.py`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/ros2/ros2_ws/src/rover_bringup/test/test_launch.py) and [`deploy_yahboom.py`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/deploy_yahboom.py) (`EXPECTED_ROS_NODES`) to include `rover_tof_bridge` and the two ToF static TF publishers.

---

## 4. Smooth Manual Turning & Chassis Supervisor Closeout (2026-11-07)

- **Detailed Closeout Report**: See [`docs/CLOSEOUT_SMOOTH_TURNING_AND_CHASSIS_SUPERVISOR_20261107.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/docs/CLOSEOUT_SMOOTH_TURNING_AND_CHASSIS_SUPERVISOR_20261107.md).
- **Core Problem**:
  - Abrupt turn initiation caused by compound double deadband (`0.12` gamepad + `0.10` server = `20.8%` physical stick dead travel) and discontinuous 35% speed jump.
  - Unexpected sustained turn cutout caused by offline IMU ($gz = 0.0\text{ rad/s}$): [`ChassisSupervisor.h`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/src/ChassisSupervisor.h) strictly required gyro yaw rate for pure-spin motion evidence, timing out after $1,000\text{ ms}$ (at $+960\text{ ms}$ active turn) and cutting motor PWM to 0 via `forceBlocked()`.
- **Delivered Changes**:
  - **Host (`yahboom-encoder`)**: Continuous quadratic turn shaping from $0.0\text{ rad/s}$, host angular slew limiter ($2.0\text{ rad/s}^2$ accel / $3.0\text{ rad/s}^2$ decel) with immediate safety bypass, and scoping recovery fault handling to autonomy.
  - **Firmware (`esp-maker-usba-4motor`)**: Two-tier motion evidence architecture (Tier 1 healthy gyro yaw vs. Tier 2 degraded encoder fallback), I2C bus clear sequence, fast presence checks, and disarmed runtime retries.
  - **Evidence**: Verified by 23/23 native tests, 311 pytest tests, full host test suite, and 5-second raised-wheel diagnostic runs at both $0.40$ and $0.15\text{ rad/s}$ with zero stops.
  - **Operator Acceptance**: Ron confirmed: **"its working."**

---

## 5. Next-Session Startup Commands (When Rover 1 is Powered On)

```powershell
# 1. Flash audited ESP32-S3 firmware (1.2.5-tof-mux) and updated daemon via Wi-Fi OTA
cd c:\Users\Ron\electronic_projects\esp\esp-maker-usba-4motor
python esp32s3_status_led/deploy_esp32s3.py --ota

# 2. Deploy Front Safety Zone, HUD radar & ROS 2 rover_tof_bridge updates to RPi5
cd c:\Users\Ron\electronic_projects\yahboom-encoder
$env:ROVER_PI_HOST="10.0.0.246"; $env:ROVER_PI_USER="ron"; python deploy_yahboom.py
```

