# Rover Project Master Handoff: Nav2 Lifecycle Restoration & Obstacle Stopping Plan

---

## 1. Executive Summary & Commissioning Status
- **Clear-Floor Nav2 Autonomous Goal**: **PASSED** (0.9 m run).
- **Disarmed Collision-Monitor Verification (Phase 1)**: **PASSED**.
  - Level 1 (Obstacle Detection at Scan Height): **VERIFIED** (20 laser scan returns at $10.4\text{ cm}$ from LiDAR, $\approx 1.5\text{ cm}$ ahead of bumper).
  - Level 2 (Zero Output / Command Clamping): **VERIFIED** (`[collision_monitor]: Robot to stop due to PolygonStop polygon`, output clamped to $0.0\text{ m/s}$).
- **Level 3 (Open-Floor Stopping Measurement from 0.05 m/s)**: **PASSED & VERIFIED**.
- **Current Hardware State**:
  - Drivetrain: **DISARMED** (`armed: false`).
  - Firmware Mode: **Mode 0** (Locked).
  - Autonomy: **DISABLED** (`autonomyState: "DISABLED"`, `cmdSource: "NONE"`).
  - Physical Rover: Stationary at rest.

---

## 2. Disarmed Test Results Breakdown

1. **Safety Enforcement**:
   - Pre-check and post-check verified hardware strictly `armed: false`, in Mode 0. No motor power was enabled at any time.
2. **Level 1 (Obstacle Detection)**:
   - LiDAR scan height verified at $z = 0.1715\text{ m}$ ($\approx 19\text{ cm}$ above floor level).
   - Box detected at $0.104\text{ m}$ ($10.4\text{ cm}$) from LiDAR optical center.
   - Front bumper clearance: $\approx 1.5\text{ cm}$.
   - 20 scan returns located squarely inside `PolygonStop` ($< 0.15\text{ m}$ from `base_link`).
3. **Level 2 (Command Path & Zero Output)**:
   - Injected $v_x = 0.10\text{ m/s}$ at $10\text{ Hz}$ into `/cmd_vel_smoothed`.
   - Production `collision_monitor` logged:
     `[collision_monitor]: Robot to stop due to PolygonStop polygon`
   - Output on `/cmd_vel` clamped to $0.0\text{ m/s}$ (zero velocity published for `stop_pub_timeout = 1.0s`, then command transmission suppressed).
   - Zero motion commands reached the drivetrain.

---

## 3. Corrected Findings from Prior Nav2 Run

### A. Position Tolerance Reconciliation ($4.9\text{ cm}$ vs. $4.0\text{ cm}$)
- External telemetry polled at 2 Hz logged $0.049\text{ m}$ ($4.9\text{ cm}$) from $t = 12.1\text{s}$ through $13.7\text{s}$.
- At $t = 13.72\text{s}$, `controller_server` logged: `[controller_server]: Reached the goal!`.
- **Finding**: Entering the 4.0 cm tolerance between telemetry samples is **an unconfirmed hypothesis**. There is no recorded sample $\le 0.040\text{ m}$ in the dataset. The records establish that `controller_server` declared success while external sampled telemetry measured $4.9\text{ cm}$, but the records **cannot conclusively establish why** this difference occurred.

### B. Heading Discrepancy Correction ($-30.7^\circ$ at $13.7\text{s}$ to $-12.83^\circ$ at $13.87\text{s}$)
- **Mathematical Implication**: Shifting $17.87^\circ$ in $0.17\text{s}$ equates to an apparent rate of **$\approx 105.1^\circ/\text{s}$ ($1.83\text{ rad/s}$)**, exceeding the configured controller limit (`max_vel_theta: 0.50 rad/s` / $28.6^\circ/\text{s}$) by $>3.6\times$.
- **Finding**: Without independent high-rate IMU and wheel encoder telemetry logged to disk during that sub-second interval, the exact distribution between **physical rotation, AMCL particle filter resampling, and asynchronous telemetry reporting latency remains UNRESOLVED**.

---

## 4. Open-Floor Stopping Measurement from 0.05 m/s - COMPLETED
- **Test Stimulus**: $0.05\text{ m/s}$ forward command via `/cmd_vel_smoothed` $\to$ `collision_monitor` $\to$ `/cmd_vel` $\to$ `rover_cmd_vel_bridge` $\to$ ESP32 Mode 3 for 3.5s on open floor.
- **Measured Metrics**:
  1. **Delay from zero output command to drivetrain response**: **$67\text{ ms}$** end-to-end command pipeline latency; $\le 50\text{ ms}$ bridge loop.
  2. **Distance traveled after zero output**: **$0.000000\text{ m}$ ($0.0\text{ mm}$)** post-zero travel (cumulative drive pulse motion was $1.96\text{ mm}$).
  3. **Time and distance until confirmed standstill**: **$\le 33\text{ ms}$** time; **$0.0\text{ mm}$** distance.
  4. **Dynamic braking activation**: **NO**. Firmware sets `writeLEDC(0)` (high-impedance coast). High-ratio gearbox mechanical backdrive friction halts the rover virtually instantaneously at $0.05\text{ m/s}$.
- **Reconciliation with 29 mm Stop Margin**:
  - Worst-case stopping envelope: LiDAR delay ($7.70\text{ mm}$) + pipeline delay ($3.35\text{ mm}$) + post-zero stop ($\le 2.0\text{ mm}$) = **$13.05\text{ mm}$**.
  - Bumper margin in `PolygonStop`: **$29.0\text{ mm}$**.
  - **Verdict**: **SAFE PASS** ($+15.95\text{ mm}$ safety margin remaining).

---

## 5. Fixed-Obstacle Physical Gate: PolygonSlow → PolygonStop → zero clamp PASSED

- **Test Objective**: Verify production Collision Monitor end-to-end multi-stage safety on physical rover: autonomous crawl approaching a fixed obstacle directly ahead.
- **Physical Test Configuration**:
  - Obstacle: Heavy box placed directly in the centerline path at $12.0\text{ inches}$ ($30.5\text{ cm}$) forward clearance from the rover front bumper.
  - Motion parameters: Commanded forward velocity $v_x = 0.100\text{ m/s}$, angular $v_z = 0.0\text{ rad/s}$.
  - Independent Staged Guards: Guard 1 ($7.50\text{ in}$ SLOWDOWN limit), Guard 2 (Failure to slow watchdog), Guard 3 ($11.25\text{ in}$ hard STOP cutoff), staged time bounds ($3.5\text{ s}$ post-slowdown / $6.5\text{ s}$ total), and continuous sensor liveness (`/scan`, `/cmd_vel` output, `/odom`, localization).
- **Measured Response Sequence**:
  1. **Cruise**: Accelerated to $0.100\text{ m/s}$ nominal speed.
  2. **`PolygonSlow` Transition**: Triggered at $t = 3.402\text{ s}$ ($6.39\text{ inches}$ / $16.2\text{ cm}$ travel) upon obstacle crossing $x_{\text{base}} \le 0.250\text{ m}$.
  3. **50% Speed Reduction**: Output velocity `/cmd_vel` attenuated immediately to **$0.050\text{ m/s}$**, initiating controlled creep mode.
  4. **`PolygonStop` Transition**: Triggered at $t = 6.165\text{ s}$ ($10.65\text{ inches}$ / $27.1\text{ cm}$ travel) upon obstacle crossing $x_{\text{base}} \le 0.150\text{ m}$.
  5. **Zero Output Clamping**: `/cmd_vel` clamped immediately to **$0.000\text{ m/s}$** (`action = 1 (STOP)`, `polygon = 'PolygonStop'`).
  6. **Standstill & Disarm**: Standstill confirmed ($v_x = 0.0000\text{ m/s}$) within $0.420\text{ s}$. Drivetrain immediately disarmed to Mode 0.
- **Key Metrics**:
  - **Total Forward Travel**: **$27.30\text{ cm}$ ($10.75\text{ inches}$)**.
  - **Encoder-Odometry-Estimated Post-Zero Displacement**: **$0.0\text{ mm}$** (below measurement resolution).
  - **Final Bumper Clearance**: **$1.33\text{ inches}$ ($3.4\text{ cm}$)**.
  - **Physical Contact**: **FALSE (ZERO CONTACT)**.
  - **Data Retention**: Preserved in durable directory [`reports/obstacle_stop_test_results.json`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/reports/obstacle_stop_test_results.json) (599 samples).
  - **Note on Margin**: While the 1.33-inch clearance observed in this run was safe and contact-free, single-run empirical clearance does not constitute an unconditional guarantee under all boundary conditions.

---

## 6. Dynamic Nav2 Obstacle Gate: PolygonSlow → DWB/local-planner commanded stop before PolygonStop PASSED

- **Test Objective**: Verify that the production navigation stack and Collision Monitor intercept, attenuate, and halt autonomous Nav2 navigation when an obstacle is dynamically introduced into the planned route while underway.
- **Test Configuration**:
  - Start Pose: $x = 1.191\text{ m}, y = -0.052\text{ m}, \theta = -4.13^\circ$.
  - Target Goal: $x = 2.154\text{ m}, y = -0.235\text{ m}$ (unobstructed corridor route).
  - Operator Safety Procedure: Zero personnel in path; obstacle introduced from the side via handle/tether after observing forward motion underway.
  - Operator Observation / Geometry Fact: The box was placed nearly directly in front of the moving rover rather than $12\text{--}16\text{ inches}$ ahead. Therefore, the brief $1.13\text{-inch}$ ($2.87\text{ cm}$) transition from initial slowdown detection to zero command is fully physically expected and consistent with obstacle entry directly into `PolygonSlow`.
  - Independent Guards: $1.05\text{ m}$ maximum travel cutoff, $10.0\text{ s}$ hard runtime timeout, sensor freshness watchdogs, immediate disarm on stop.
- **Measured Response Sequence**:
  1. **Autonomous Acceleration**: Nav2 planned clear path and dispatched speed commands: $0.05 \to 0.10 \to 0.15\text{ m/s}$ (reaching $0.150\text{ m/s}$ commanded cruise speed; measured wheel odometry reached $0.12\text{ m/s}$).
  2. **Dynamic Obstacle Introduction**: Operator introduced the box into the path from the side at $\approx 7.5\text{ inches}$ ($19.1\text{ cm}$) of travel.
  3. **`PolygonSlow` Interception**: At $t = 2.665\text{ s}$ ($x_{\text{odom}} = 0.6895\text{ m}$, $7.52\text{ in}$ travel), `collision_monitor` transitioned to `action = SLOWDOWN (2)`, `polygon = 'PolygonSlow'`. Output command `/cmd_vel` scaled immediately by 50% from $0.150\text{ m/s} \to \mathbf{0.075\text{ m/s}}$.
  4. **DWB Local Planner Obstacle Reaction & Stopping**: DWB / local costmap reacted to the obstacle appearing directly ahead (eliminating collision-free forward trajectories), commanding velocity down through $0.050\text{ m/s} \to 0.025\text{ m/s} \to \mathbf{0.000\text{ m/s}}$ at $t = 3.017\text{ s}$ (at $x_{\text{odom}} = 0.7162\text{ m}$, travel = $8.65\text{ in}$). The velocity smoother processed the resulting command, passing the $0.000\text{ m/s}$ output through.
  5. **Standstill & Disarm**: Full mechanical standstill ($v_x = 0.0000\text{ m/s}$) confirmed at $t = 3.409\text{ s}$. Nav2 goal was automatically cancelled and drivetrain disarmed to Mode 0.
- **Key Metrics & Forensic Reconciliation**:
  - **Component That Issued First Zero Command**: **DWB Local Planner** (`controller_server`). When the dynamic obstacle populated the local costmap, DWB detected trajectory obstruction and commanded $0.000\text{ m/s}$, which the velocity smoother and collision monitor forwarded.
  - **Collision Monitor State Verification**: Across all 310 telemetry samples, raw `cm_action == 2 (SLOWDOWN)` during the obstacle encounter. **Zero samples had `cm_action == 1 (STOP)`**. `PolygonStop` was **not** entered or activated during this dynamic run.
  - **Total Forward Travel**: **$22.09\text{ cm}$ ($8.70\text{ inches}$)** (from start $x=0.4987\text{ m}$ to final $x=0.7193\text{ m}$).
  - **Transition Distance (Slowdown $\to$ Zero Command)**: **$1.13\text{ inches}$ ($2.87\text{ cm}$)** ($7.52\text{ in} \to 8.65\text{ in}$).
  - **Final Bumper Clearance**: **$3.81\text{ inches}$ ($9.68\text{ cm}$)** ahead of front bumper.
    - Clearance calculation: Measured closest LiDAR ray $x_{\text{lidar}} = 0.1857\text{ m}$ minus LiDAR-to-bumper offset ($0.0889\text{ m}$) = $0.0968\text{ m}$ ($3.81\text{ in}$).
    - Geometry Reconciliation: Front bumper is at $x_{\text{base}} = +0.121\text{ m}$. In `base_link`, obstacle came to rest at $x_{\text{base}} = 0.218\text{ m}$. This placed the obstacle squarely inside `PolygonSlow` ($x \le 0.250\text{ m}$, front margin $12.9\text{ cm}$ / $5.08\text{ in}$ ahead of bumper) but ahead of `PolygonStop` ($x \le 0.150\text{ m}$, front margin $2.9\text{ cm}$ / $1.14\text{ in}$ ahead of bumper).
  - **Encoder-Odometry-Estimated Post-Zero Displacement (Raw Unrounded)**: **$3.14\text{ mm}$ ($0.124\text{ inches}$)**.
    - Raw odometry at zero command ($t = 3.017\text{ s}$): $x = 0.716166594\text{ m}, y = 0.018759547\text{ m}, v_x = 0.0552\text{ m/s}$.
    - Raw odometry at final rest ($t = 3.409\text{ s}$): $x = 0.719301489\text{ m}, y = 0.018984186\text{ m}, v_x = 0.0000\text{ m/s}$.
    - Vector displacement from wheel encoder odometry $\sqrt{\Delta x^2 + \Delta y^2} = 0.003143\text{ m} = 3.14\text{ mm}$ ($0.124\text{ in}$).
  - **Physical Contact Occurred**: **FALSE (ZERO CONTACT)**.
  - **Stop Reason**: DWB local planner reacted to obstacle in local costmap by commanding $0.000\text{ m/s}$ while under `PolygonSlow` 50% velocity scaling, stopping the rover safely before reaching `PolygonStop`.
  - **Data Retention**: Preserved in durable project directory [`reports/dynamic_obstacle_test_results.json`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/reports/dynamic_obstacle_test_results.json) (310 samples) and [`reports/obstacle_stop_test_results.json`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/reports/obstacle_stop_test_results.json) (599 samples).

---

## 8. Authoritative Permanent HOME Pose Definition
- **Physical Settling & Verification**: Rover settled at rest on the floor. 100-sample mean AMCL tracking recorded and validated:
  - **Coordinates**: $x = 1.193853\text{ m},\; y = -0.045221\text{ m},\; \theta = -5.047^\circ$ ($-0.088087\text{ rad}$).
  - **Orientation Quaternions**: $q_z = -0.044029,\; q_w = 0.999030$.
  - **Configuration Sync**: Updated in `/home/ron/yahboom-encoder/ros2/volumes/maps/home_pose_slam_2026-08-23_final.json`, synced inside container at `/ros2_ws/home_pose_slam.json`, and served live via Cockpit `/api/navigation/home`.
  - **Cockpit UI**: Preset button and legend updated to **`🏠 Saved HOME (1.19, -0.05)`**.
  - **Tolerance Reconciliation**: Corrected previous $-8.34^\circ$ heading target discrepancy; setting target to $-5.05^\circ$ ensures Nav2's $5.0^\circ$ yaw tolerance (`general_goal_checker.yaw_goal_tolerance = 0.087 rad`) settles directly on the rover's physical floor alignment.

---

## 9. Commissioning Gates Passed & Authoritative Baseline
The following gates are verified and recorded as **PASSED** (avoid repeating):
1. **Cold-Start Service Recovery**: Verified clean container initialization, service daemon bringup, and sensor discovery.
2. **Foxglove Bridge & Sensor Streaming**: Verified `/scan` ($10\text{ Hz}$), `/odom` ($20\text{ Hz}$), `/imu/data` ($100\text{ Hz}$), `/tf` dynamic transforms.
3. **AMCL Localization at Physical HOME**: Reseeded and converged via 10 no-motion updates ($1.5\text{ cm}$ radial error, $0.32^\circ$ heading deviation, 80.1% scan alignment within $10\text{ cm}$).
4. **Saved HOME Route Preview**: Smac2D A* trajectory generation verified feasible and obstacle-free.
5. **Corrected Dynamic Relative-Goal Navigation**: Relative presets (`+0.50m`, `+0.60m`, `+0.70m`) dynamically compute ahead along live approach heading.
6. **Fixed and Dynamic Obstacle Response**: Verified `PolygonSlow` ($50\%$ speed attenuation) and `PolygonStop` / DWB local planner obstacle stopping with contact-free safety margins.

---

## 10. Nav2 Plan Preview Telemetry & Latency Optimization
- **Six-Stage Latency Instrumentation**:
  1. Browser click to HTTP request: $\approx 1.0\text{ ms}$
  2. Backend validation & localization check: $< 0.5\text{ ms}$
  3. Waiting for Nav2 planner action server: $0.1\text{ ms}$ (pre-warmed client, non-blocking `server_is_ready()` check)
  4. `ComputePathToPose` Smac2D planning: $1.5\text{--}3.0\text{ ms}$
  5. Backend response network transmission: $11\text{--}12\text{ ms}$
  6. Browser JSON parsing & 2D canvas rendering: $1.5\text{--}2.0\text{ ms}$
  - **Total End-to-End Click-to-Render**: **`17 – 18 ms`** (down from $500\text{+ ms}$).
- **Root Cause Fixes**:
  - Eliminated unprimed `rclpy` `ActionClient.wait_for_server()` 500 ms blocking delay by adding background client pre-warming and non-blocking `server_is_ready()` checks.
  - Reduced polling quantization in `rover_nav_bridge.py` from 20 ms to 1 ms (`time.sleep(0.001)`).
  - Streamlined UI click handlers in `app.js` to eliminate redundant sequential `/api/localization/status` calls.
  - Enforced immediate visual feedback (`⏳ Planning...`) and disabled button state to prevent duplicate concurrent planner requests.

---

## 11. Autonomy Dispatch & Controller Startup Stabilization
- **Root Cause of Dispatch Stall**: 
  - Nav2 global path calculation and DWB trajectory initialization require $5.5\text{--}7.0\text{ s}$ before emitting the first `/cmd_vel` packet.
  - Cockpit's watchdog had a premature $5.0\text{ s}$ timeout (`Awaiting initial Nav2 cmd_vel timed out (5000ms)`), switching autonomy to `DISABLED` and rejecting incoming velocity commands with HTTP 403 (`lastRejectionReason: "Autonomy is disabled"`).
  - In addition, setting `autonomyState.state = 'READY_ARMED'` was conditioned on the rover not being previously armed, causing already-armed dispatches to skip arming the autonomy intake.
- **Fixes Applied & Deployed**:
  - Unconditionally set `autonomyState.state = 'READY_ARMED'` and `autonomyState.enabled = true` on every successful dispatch.
  - Expanded the initial command timeout window from $5,000\text{ ms}$ to $20,000\text{ ms}$ ($20\text{ s}$) to provide ample headroom for Nav2 to initialize before velocity streaming begins.

---

---

## 12. Authoritative Map Persistence & Guarded Cold-Start Localization Policy (2026-09-26)
- **Active Production Map Promoted**: **`house_slam_2026-09-23_candidate_hallway.yaml`**
  - **Resolution & Size**: $340 \times 132$ grid cells ($17.0\text{ m} \times 6.6\text{ m}$) @ $0.050\text{ m/px}$.
  - **Map Origin**: `[-4.202, -5.100, 0.0]`.
  - **Coordinate Alignment**: Fixed `map` frame identical to original SLAM origin; shared across Cockpit, Foxglove Studio, Nav2, and saved HOME.
- **Canonical Version-Controlled Map Assets Directory**:
  - Tracked in `ros2/maps/production/` with `MANIFEST.json` SHA-256 checksums:
    1. `house_slam_2026-09-23_candidate_hallway.yaml`
    2. `house_slam_2026-09-23_candidate_hallway.pgm` (44,895 bytes)
    3. `house_slam_2026-09-23_candidate_hallway.data` (349,769 bytes)
    4. `house_slam_2026-09-23_candidate_hallway.posegraph` (7,119,855 bytes, durable SLAM graph preserved in Git)
    5. `active_map_path.txt` (points to `/ros2_ws/maps/house_slam_2026-09-23_candidate_hallway.yaml`)
    6. `home_pose_slam_2026-09-23_candidate_hallway.json`
    7. `home_pose_slam.json`
- **Guarded Cold-Start Localization Policy (No Unconditional Injection)**:
  - **Unconditional injection removed**: Rover NEVER injects `/initialpose` blindly on startup.
  - **Independent Scan-to-Map Validation**: At boot, `rover_nav_bridge.py` independently transforms live LiDAR scan beams into map coordinates using the saved HOME hypothesis $(1.194, -0.045, -5.05^\circ)$ and static sensor offset $(0.03175, 0.0, 0.0)$ before touching `/initialpose`.
  - **Strict Acceptance Gate**:
    1. **Scan Freshness**: $\text{age} \le 0.75\text{ s}$ (rejects `STALE_SCAN`).
    2. **Beam Sufficiency**: $\ge 100$ valid range returns (rejects `INSUFFICIENT_BEAMS`).
    3. **Scan-to-Map Overlap**: $\ge 75.0\%$ of scan beams must fall within $\pm 5\text{ cm}$ ($\pm 1$ grid cell) of known occupied map walls (rejects `SCAN_MISMATCH`).
  - **Fail-Safe Behavior**: If validation does not strongly confirm HOME, the rover remains `NOT_LOCALIZED`, `/initialpose` is never sent, Nav2 goal dispatch remains strictly blocked (HTTP 409), and initialization requires explicit Cockpit "Initialize at HOME" action or Foxglove operator pose.
- **Automated Regression Suites**:
  - `test/test_production_map_assets.py`: **11/11 PASSED** (manifest checksums, asset existence, PGM/YAML consistency).
  - `test/test_cold_start_localization_policy.py`: **7/7 PASSED**:
    - `test_verified_home_case`: Confirms `VERIFIED_HOME` and overlap $\ge 75\%$ ($81.6\%$ measured).
    - `test_scan_mismatch_displaced_y`: Confirms rejection on $+25\text{ cm}$ lateral displacement ($44.5\%$ overlap).
    - `test_scan_mismatch_rotated_yaw`: Confirms rejection on $+25^\circ$ heading displacement ($46.9\%$ overlap).
    - `test_scan_mismatch_different_room`: Confirms rejection in wrong room coordinates ($30.8\%$ overlap).
    - `test_missing_scan_case`: Confirms `MISSING_SCAN` rejection when scan is None or empty.
    - `test_stale_scan_case`: Confirms `STALE_SCAN` rejection when scan age exceeds $0.75\text{ s}$.
    - `test_insufficient_beams_case`: Confirms `INSUFFICIENT_BEAMS` rejection when returns $< 100$.
- **Genuine Stationary Reboot Verification (Rover Physically at HOME)**: **PASSED**
  - Performed physical `sudo reboot` on Raspberry Pi 5 with rover resting at HOME tape mark.
  - Zero manual startup intervention:
    1. Systemd services (`rover-server`, `rover-lidar`, `rover-health`, `docker`): all **`active`**.
    2. Docker container `rover-ros2`: **`running`**.
    3. Boot Scan Validation: **`PASSED: VERIFIED_HOME (overlap = 85.7%, hits = 263/307)`**.
    4. Nav2 Lifecycle: Autonomously initialized HOME, 5 no-motion updates converged, all nodes transitioned to **`active`**.
    5. Localization State: **`LOCALIZED`**, `freshValidationOk: true`, $x = 1.209\text{ m}, y = -0.053\text{ m}, \theta = -4.86^\circ$.
    6. Drivetrain Hardware Safety: Strictly **DISARMED** (`armed: false`), Mode 0 (Locked), zero velocities, `cmdSource: "NONE"`.
    7. Stationary Regression Suite: `test/test_dispatch_localization_refresh.py` **7/7 CHECKS PASSED**.
- **Cockpit Operator Controls**:
  - Added `📍 Initialize at HOME` button in Cockpit UI (`public/index.html`, `public/app.js`).
  - Added `/api/navigation/init_home` and `/api/navigation/validate_home` endpoints (`server.js`, `rover_nav_bridge.py`).

---

## 13. Current System State & Next Physical Gate
- **Hardware State**: DISARMED (`armed: false`), Mode 0 (Locked), `cmdSource: "NONE"`.
- **Navigation State**: `LOCALIZED` at HOME ($x \approx 1.209, y \approx -0.053, \theta \approx -4.86^\circ$), all Nav2 nodes `active` against `house_slam_2026-09-23_candidate_hallway.yaml`.
- **Safety Policy**: Cold-start localization is guarded by independent scan-to-map validation. Drivetrain remains disarmed and navigation blocked if rover is not verified at HOME.
- **Next Physical Gate**: Out-and-back autonomous hallway waypoint navigation under exact motion contract with operator approval.

