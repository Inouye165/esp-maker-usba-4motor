# Rover 1 — Prioritized Action List (`TODO.md`)

**Last Updated**: 2026-11-05
**Active Branch (Both Repos)**: `feat/esp32s3-tof-sensor-bench`
**Motor Controller Firmware**: `1.3.4-remap-r03` (Maker-ESP32-Pro, verified & locked)
**ESP32-S3 ToF/LED Firmware**: `1.2.3-tof-mux` (live on RPi5), `1.2.4-tof-mux` (audited in repo, ready for `--ota` flash)

---

## 1. Immediate Next-Session Startup (When Rover 1 / RPi5 is Powered On)

Because the post-session code & firmware audit fixes were committed while Rover 1 (`10.0.0.246`) was powered off for the night, run these two sync steps first thing on next boot:

- [ ] **1A. Flash Audited ESP32-S3 Firmware (`1.2.4-tof-mux`) & Updated Daemon via Wireless OTA**:
  ```powershell
  cd c:\Users\Ron\electronic_projects\esp\esp-maker-usba-4motor
  python esp32s3_status_led/deploy_esp32s3.py --ota
  ```
  *What this updates on RPi5 / ESP32-S3*:
  - Flashes [`esp32s3_status_led/src/main.cpp`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/esp32s3_status_led/src/main.cpp) (`1.2.4-tof-mux`) fixing the `VL53L4CD` invalid-range-status ternary bug and `VL53L7CX` center 2×2 fallback (`-1` when center 4 zones have no target).
  - Updates `/home/ron/esp32s3_status_led/rpi5_status_led_daemon.py` so `esp_status` and `ota_status` in `/tmp/esp32s3_tof_state.json` refresh every `10 s` and on LED state transitions (`GREEN` / `BLUE` / `RED`).

- [ ] **1B. Sync Audited Cockpit HUD & ROS 2 `rover_tof_bridge` Fixes to RPi5**:
  ```powershell
  cd c:\Users\Ron\electronic_projects\yahboom-encoder
  $env:ROVER_PI_HOST="10.0.0.246"; $env:ROVER_PI_USER="ron"; python deploy_yahboom.py
  ```
  *What this updates on RPi5*:
  - Updates [`public/hud-core.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/public/hud-core.js), [`public/hud-desktop.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/public/hud-desktop.js), and [`public/hud-mobile.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/public/hud-mobile.js) so the radar displays `FRONT TOF LIVE` and live `NEAREST` / `MIN: XX cm (X.X in)` even when the 360° RPLiDAR C1 is unplugged.
  - Updates [`rover_tof_bridge.py`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/ros2/ros2_ws/src/rover_bringup/rover_bringup/rover_tof_bridge.py) inside `rover-ros2` to use a persistent `requests.Session()` and REP-117 `Range` bounds checking.

---

## 2. Time-of-Flight (`TCA9548A` + `VL53L7CX` / `VL53L4CD`) Tasks

- [ ] **2A. Measure Exact Physical Mount Offsets for Front-Corner `VL53L7CX` Sensors**:
  - Currently, `base_link -> tof_front_left_link` and `base_link -> tof_front_right_link` use placeholder offsets (`x = +0.14 m`, `y = ±0.08 m`, `z = +0.08 m`, `yaw = ±8°` [`±0.139626 rad`]).
  - Measure the exact physical `(x, y, z)` distance from `base_link` (chassis center) to each front corner sensor lens and update both:
    - [`foundation.launch.py`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/ros2/ros2_ws/src/rover_bringup/launch/foundation.launch.py)
    - `TOF_MOUNTS` in [`public/hud-core.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/public/hud-core.js)

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

---

## 4. Repository Maintenance

- [ ] **4A. Merge `feat/esp32s3-tof-sensor-bench` to `main` (When Authorized by Ron)**:
  - Both `esp-maker-usba-4motor` and `yahboom-encoder` are working on branch `feat/esp32s3-tof-sensor-bench`. Merge to `main` once physical verification of the close-in radar and `1.2.4-tof-mux` is complete.
