# Rover 1 — Prioritized Action List (`TODO.md`)

**Last Updated**: 2026-11-07
**Active Branches**:
- Firmware (`esp-maker-usba-4motor`): `fix/chassis-supervisor-pure-spin-motion-evidence` (commit `012e641`)
- Host (`yahboom-encoder`): `feat/smooth-progressive-manual-turning` (commit `caad519`)
**Motor Controller Firmware**: `1.3.4-remap-r03` (Maker-ESP32-Pro, flashed SHA-256 `7f808b16df0e6d8ec6e7206b307178453b514ebf67bf22057207c873cc17bba0`)
**ESP32-S3 ToF/LED Firmware**: `1.2.3-tof-mux` (live on RPi5), `1.2.5-tof-mux` (audited in repo with `0 mm` close-range contact fix, ready for `--ota` flash)

---

## 1. Immediate Next-Session Startup (When Rover 1 / RPi5 is Powered On)

- [x] **1C. Smooth Progressive Manual Turning & Two-Tier Chassis Supervisor (Completed 2026-11-07)**:
  - Eliminated 35% speed jump with continuous quadratic curve starting from $0.0\text{ rad/s}$ and host angular slew rate limiter ($2.0\text{ rad/s}^2$ accel / $3.0\text{ rad/s}^2$ decel).
  - Resolved $+960\text{ ms}$ sustained turn stall cutout via two-tier motion evidence in [`ChassisSupervisor.h`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/src/ChassisSupervisor.h) (Tier 1 healthy gyro yaw vs. Tier 2 degraded encoder fallback).
  - Added I2C bus clear, non-blocking presence probe, and disarmed runtime retries.
  - Verified on bench blocks: 5.0-second sustained turns at both $0.40$ and $0.15\text{ rad/s}$ completed with zero stall cutouts.
  - Ron tested and confirmed: **"its working."** Detailed closeout in [`docs/CLOSEOUT_SMOOTH_TURNING_AND_CHASSIS_SUPERVISOR_20261107.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/docs/CLOSEOUT_SMOOTH_TURNING_AND_CHASSIS_SUPERVISOR_20261107.md).

- [ ] **1A. Flash Audited ESP32-S3 Firmware (`1.2.5-tof-mux`) & Updated Daemon via Wireless OTA**:
  ```powershell
  cd c:\Users\Ron\electronic_projects\esp\esp-maker-usba-4motor
  python esp32s3_status_led/deploy_esp32s3.py --ota
  ```
  *What this updates on RPi5 / ESP32-S3*:
  - Flashes [`esp32s3_status_led/src/main.cpp`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/esp32s3_status_led/src/main.cpp) (`1.2.5-tof-mux`) preserving `0 mm` (`< 4 mm` contact) `VL53L7CX`/`VL53L4CD` readings (`d >= 0` and `tstat == 4 && d <= 60`), fixing the `VL53L4CD` invalid-range-status ternary bug, and fixing the `VL53L7CX` center 2×2 fallback (`-1` when center 4 zones have no target).
  - Updates `/home/ron/esp32s3_status_led/rpi5_status_led_daemon.py` so `esp_status` and `ota_status` in `/tmp/esp32s3_tof_state.json` refresh every `10 s` and on LED state transitions (`GREEN` / `BLUE` / `RED`).

- [x] **1B. Sync Front Safety Zone, Cockpit HUD & ROS 2 `rover_tof_bridge` Updates to RPi5**:
  ```powershell
  cd c:\Users\Ron\electronic_projects\yahboom-encoder
  $env:ROVER_PI_HOST="10.0.0.246"; $env:ROVER_PI_USER="ron"; python deploy_yahboom.py --allow-dirty
  ```
  *What this updates on RPi5*:
  - Deploys [`front_safety_zone.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/front_safety_zone.js) and [`server.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/server.js) enforcing the **Front Obstacle Safety Zone** (`31..75 mm` slows forward speed to a `0.03 m/s` crawl; `0..30 mm` stops all forward/in-place motion while allowing reverse) for both manual and autonomous driving.
  - Updates [`public/hud.html`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/public/hud.html), [`public/hud.css`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/public/hud.css), [`public/hud-core.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/public/hud-core.js), [`public/hud-desktop.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/public/hud-desktop.js), and [`public/hud-mobile.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/public/hud-mobile.js) with the `#hud-radar-drive-ind` indicator inside `.radar-box` (outside the radar circles), updated thresholds (`30 mm` stop / `75 mm` crawl), and `d >= 0` close-range ToF retention.
  - Updates [`rover_tof_bridge.py`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/ros2/ros2_ws/src/rover_bringup/rover_bringup/rover_tof_bridge.py) inside `rover-ros2` to use `range_min_m = 0.0`, a persistent `requests.Session()`, and REP-117 `Range` bounds checking.

---

## 2. Time-of-Flight (`TCA9548A` + `VL53L7CX` / `VL53L4CD`) Tasks

- [x] **2A. Measure Exact Physical Lateral Separation & Front Proximity Safety Zone**:
  - Measured 125 mm lateral separation symmetrically centered across the front (`y = ±0.0625 m`), retaining existing `±8°` outward angles.
  - Updated both [`foundation.launch.py`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/ros2/ros2_ws/src/rover_bringup/launch/foundation.launch.py) and `TOF_MOUNTS` in [`public/hud-core.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/public/hud-core.js).
  - Forward position (`x = +0.14 m`) and height (`z = +0.08 m`) remain unverified placeholder measurements; further physical alignment calibration deferred as unnecessary for the current proximity task.
  - Implemented authoritative Front Obstacle Safety Zone: yellow/crawl at $\le 75\text{ mm}$, red/stop at $\le 30\text{ mm}$, with reverse escape preserved.
  - Operator verified actual wheel slowdown and stop response on a raised platform with spinning wheels (floor-driving stopping distance not tested). Proximity indicator and sensor-spacing task complete.

- [ ] **2B. Decide Physical Role & Mapping for `CH 2` (`VL53L4CD`)**:
  - `CH 2` (`VL53L4CD`, `0x29`, `MODEL=0xEBAA`) is initialized and live in `/api/tof-sensors` and `http://10.0.0.246:3000/sensors.html`, but intentionally unassigned (`unassigned`) and excluded from ROS 2 / HUD Radar.
  - Once physically mounted (e.g., front-left cliff, front-right cliff, or close-range bumper), assign its slot in `sensors.html` (`/api/tof-sensors/mapping`) and add its TF / ROS 2 `Range` topic if desired.

- [ ] **2C. Wire Remaining `VL53L4CD` Cliff Sensors (`CH 3..5`)**:
  - Connect and verify any additional downward-facing `VL53L4CD` cliff sensors on `TCA9548A` channels `CH 3..7` (hot-plug scan automatically detects new sensors every `2.5 s`).

- [ ] **2D. Nav2 Costmap / CollisionMonitor Integration (Future)**:
  - Once the front `VL53L7CX` physical offsets are finalized, optionally subscribe Nav2 `voxel_layer` / `obstacle_layer` or `CollisionMonitor` to `/tof/front_left/points` and `/tof/front_right/points` for low-obstacle and close-corner protection.

---

## 3. Autonomous Navigation Roadmap Tasks

- [ ] **3A. Reconnect & Verify RPLiDAR C1 (`/dev/rover-lidar`)**:
  - Verify USB cable connection for RPLiDAR C1 (`10c4:ea60`) and confirm `/api/lidar/status` reports `connected: true` before running SLAM, AMCL localization, or Nav2 missions.

- [ ] **3B. Roadmap Phase 3 — Spin-to-Arc & Curved Transition Verification**:
  - Run Suite A Case **8** (`Transition_PureSpin_To_Arc`) and Case **1** (`FailedArc_Baseline`) on RPi5:
    ```bash
    cd /home/ron/yahboom-encoder && python3 scripts/characterize_floor_speeds.py --case 8
    cd /home/ron/yahboom-encoder && python3 scripts/characterize_floor_speeds.py --case 1
    ```

- [ ] **3C. Roadmap Phase 5 — Gold Standard Autonomous HOME Test (`gold_standard_home_test`)**:
  - Run preflight dry-run, safety watchdog test, and full autonomous execution (`2.0 ft` outbound $\to 180^\circ$ CW turn $\to$ return HOME within $\le 40\text{ mm}$ and $\le 3.0^\circ$):
    ```bash
    ./tools/run_gold_standard_home_test.sh --dry-run
    ./tools/run_gold_standard_home_test.sh --test-safety
    ./tools/run_gold_standard_home_test.sh --execute
    ```

- [ ] **3D. Physical BNO08x IMU Hardware & Power Rail Inspection**:
  - Telemetry confirmed the BNO08x does not acknowledge on I2C address `0x4B` or `0x4A`. Check physical Dupont wiring (`SDA=GPIO 21`, `SCL=GPIO 22`, `3V3/VIN`, `GND`) and board peripheral rail before attempting autonomous navigation runs that rely on gyro yaw dead-reckoning.

---

## 4. Repository Maintenance

- [ ] **4A. Merge `feat/esp32s3-tof-sensor-bench` to `main` (When Authorized by Ron)**:
  - Both `esp-maker-usba-4motor` and `yahboom-encoder` are working on branch `feat/esp32s3-tof-sensor-bench`. Merge to `main` once physical verification of the close-in radar and `1.2.4-tof-mux` is complete.

