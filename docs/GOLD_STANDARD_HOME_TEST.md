# Rover One Gold Standard HOME Acceptance Test (`gold_standard_home_test`)

This document defines the permanent, version-controlled CLI acceptance test for Rover One navigation and return-to-HOME performance.

Governed strictly by [`docs/EXACT_MOTION_CONTRACT.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/docs/EXACT_MOTION_CONTRACT.md) and the `rover-exact-motion` skill.

---

## 1. Objective & Invariants

The `gold_standard_home_test` executes one deterministic production-stack mission to validate end-to-end translation, rotation, and return-to-HOME accuracy without agent interpretation:

1. **Authoritative HOME Resolution**: Loads the active map's saved HOME pose from `active_map_path.txt` and its corresponding JSON file (`home_pose_slam.json`). Coordinates and yaws are never hardcoded.
2. **Pre-Arm Gate (Fail-Closed)**: Queries live AMCL pose. Refuses to arm unless AMCL is localized (`state: LOCALIZED`, `localized: true`) and the initial pose is within **$0.05\text{ m}$ ($50\text{ mm}$)** and **$5.0^\circ$** of saved HOME. If live AMCL cannot be queried, localization is not `LOCALIZED`, or required fields are missing/stale, execution halts immediately and never substitutes HOME coordinates.
3. **Leg 1 (Outbound Translation via Nav2)**: Nav2 drives forward exactly **$2.000\text{ ft}$ ($0.6096\text{ m}$)** along the saved HOME heading.
4. **Leg 2 (Explicit 180° CW In-Place Rotation via Exact-Motion)**: After Leg 1 completes and zero output is verified, exact-motion controller executes an explicit $180.0^\circ$ **CLOCKWISE** in-place rotation ($\omega_z < 0$, cruise ceiling $0.40\text{ rad/s}$, creep ceiling $0.18\text{ rad/s}$).
5. **Leg 3 (Return to HOME Coordinates via Nav2)**: After Leg 2 finishes, exact-motion ownership is released, and Nav2 navigates back to HOME coordinates while **retaining the return-facing heading** (no direct velocity override competing with Nav2).
6. **Leg 4 (Final HOME Yaw Alignment & Settle via Exact-Motion)**: After reaching HOME coordinates, exact-motion controller performs an in-place rotation to exact saved HOME yaw using signed shortest-angle error (capped at **$0.18\text{ rad/s}$**), stops within $3.0^\circ$, and settles for 1.5 seconds. A Leg 4 timeout fails the test.
7. **Safe Stop & Lock**: Vehicle cancels remaining goals, releases command ownership, and disarms to Mode 0.

---

## 2. Frozen Kinematic Settings & Speed Distinctions

The following limits are permanently frozen across the production stack and acceptance test:

| Parameter | Value | Scope | Description |
| :--- | :--- | :--- | :--- |
| **Nav2 Linear Speed Cap** | $0.20\text{ m/s}$ | Legs 1 & 3 | Active production Nav2 cap (`max_vel_x`, `max_speed_xy`, and `velocity_smoother`) |
| **Nav2 Autonomous Yaw Limit** | $0.50\text{ rad/s}$ | Nav2 stack | Active production Nav2 angular limit (`max_vel_theta` and `rotate_to_heading_angular_vel`) |
| **Explicit 180° Rotation Speed** | $0.40\text{ rad/s}$ | Leg 2 | Governed approach controller cruise limit (tapers to $0.18\text{ rad/s}$ creep when $<30^\circ$) |
| **Final HOME Yaw Alignment Speed** | $0.18\text{ rad/s}$ | Leg 4 | Maximum allowable yaw rate during final signed shortest-angle alignment |
| **Final Position Correction Ceiling** | $0.10\text{ m/s}$ | Final approach | Governed approach ceiling when within $0.25\text{ m}$ of HOME |
| **DWB Minimum Velocity Floor** | $0.00\text{ m/s}$ | Nav2 DWB | Zero minimum velocity floor (never impose nonzero crawling) |
| **Navigation Tolerances** | $0.04\text{ m}$ / $3.0^\circ$ | Mission pass | Target tolerances for position and orientation |

---

## 3. Synchronized 20+ Hz Telemetry Recorder

Every trial automatically records at $\ge 20\text{ Hz}$ into a new timestamped report:
`reports/gold_standard_home_test/gold_standard_run_<timestamp>.json`

Captured telemetry channels:
- Nav2 raw, smoothed, and final `cmd_vel` ($v_x, \omega_z$) during Legs 1 & 3; exact-motion commands during Legs 2 & 4.
- Command source attribution (`ROS_AUTONOMY`, `EXACT_MOTION`, `NONE`).
- AMCL pose $(x, y, \theta)$ and full covariance matrix ($cov_x, cov_y, cov_\theta$).
- Wheel Odometry pose $(x, y, \theta)$, twist ($v_x, \omega_z$), and wheel travel.
- IMU Game Rotation Vector yaw, relative yaw, gyro rate Z, and non-magnetic validation flag.
- Drive status: `armed`, `mode`, `reqLinear`, `reqAngular`, `limLinear`, `limAngular`, `lockStatus`.
- Safety: Collision monitor state, polygon clearance masks, and minimum distances.
- ESP32 hardware health: `bootCount`, `resetReason`, `rtcResetReason`, `dataAgeMs`, `sequence`.
- Mission transition timestamps and verified achieved sample rate.

---

## 4. Fail-Safe Abort Invariants

The runner monitors invariants on every tick. It immediately zeroes velocity, cancels navigation goals, and disarms the drivetrain to Mode 0 if:
1. **Localization Lost**: Cockpit reports `state !== 'LOCALIZED'` or `localized === false`.
2. **Telemetry Stale**: Underlying ESP32 IMU packet age exceeds $500\text{ ms}$ (`dataAgeMs > 500`), serial packet is marked stale, or packet sequence freezes for $> 500\text{ ms}$.
3. **ESP32 Hardware Reboot**: ESP32 `bootCount` changes from its initial armed baseline.
4. **IMU Discontinuity**: Raw IMU yaw jumps $> 45^\circ$ in a single step without high gyro rate.
5. **Serial Disconnect**: Serial communication to the ESP32 disconnects.
6. **Leg Timeout**: Any individual mission leg exceeds its timeout limit (Leg 4 timeout fails the mission).
7. **Simultaneous Ownership Violation**: Attempting direct velocity commands while a Nav2 goal is active immediately aborts.
8. **No Automatic Retry**: The test exits immediately on failure and never retries automatically.

---

## 5. Acceptance Grading & Pass Criteria

| Metric | Pass Threshold | Description |
| :--- | :--- | :--- |
| **Final HOME Position Error** | $\le 0.040\text{ m}$ ($40\text{ mm}$) | Euclidean distance from settled AMCL pose to saved HOME |
| **Final HOME Yaw Error** | $\le 3.0^\circ$ | Angular error from settled AMCL yaw to saved HOME yaw |
| **Hardware Stability** | $0$ resets / discontinuities | Zero change in `bootCount`, zero packet loss discontinuities |
| **Direction Reversals** | $\le 1$ corrective reversal | Maximum of one direction change during final HOME settling |
| **Crawling Window** | $\le 2.0\text{ s}$ continuous | No prolonged crawl below $0.05\text{ m/s}$ or $0.12\text{ rad/s}$ |
| **Sample Rate** | $\ge 20.0\text{ Hz}$ achieved | Actual recorded telemetry rate verified by grader |
| **Final Safety State** | `armed=false`, `mode=0` | Vehicle finishes stopped, disarmed, and hardware locked |

---

## 6. Execution Commands

### A. Dry-Run / Pre-Flight Verification (No Motor Motion)
Simulates all state transitions, geometry calculations, and telemetry recording without arming:
```bash
./tools/run_gold_standard_home_test.sh --dry-run
```

### B. Automated Safety Watchdog Test
Validates that simulated stale telemetry and ESP32 reboot events trigger immediate fail-safe aborts:
```bash
./tools/run_gold_standard_home_test.sh --test-safety
```

### C. Physical Acceptance Test Execution (Requires Confirmation)
Prompts with the preview table and requires typing `CONFIRM` before arming:
```bash
./tools/run_gold_standard_home_test.sh --execute
```
Credentials are read dynamically via `--env-file /home/ron/yahboom-encoder/.env`. No tokens are hardcoded or copied.
