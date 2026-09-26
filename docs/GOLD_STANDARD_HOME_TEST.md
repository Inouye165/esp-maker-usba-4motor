# Rover One Gold Standard HOME Acceptance Test (`gold_standard_home_test`)

This document defines the permanent, version-controlled CLI acceptance test for Rover One navigation and return-to-HOME performance.

Governed strictly by [`docs/EXACT_MOTION_CONTRACT.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/docs/EXACT_MOTION_CONTRACT.md) and the `rover-exact-motion` skill.

---

## 1. Objective & Invariants

The `gold_standard_home_test` executes one deterministic production-stack mission to validate end-to-end translation, rotation, and return-to-HOME accuracy without agent interpretation:

1. **Authoritative HOME Resolution**: Loads the active map's saved HOME pose from `active_map_path.txt` and its corresponding JSON file (`home_pose_slam.json`). Coordinates and yaws are never hardcoded.
2. **Pre-Arm Gate**: Refuses to arm unless AMCL is localized (`state: LOCALIZED`) and the initial pose is within **$0.05\text{ m}$ ($50\text{ mm}$)** and **$5.0^\circ$** of saved HOME.
3. **Leg 1 (Outbound Translation)**: Drives forward exactly **$2.000\text{ ft}$ ($0.6096\text{ m}$)** along the saved HOME heading.
4. **Leg 2 (Explicit 180° CW In-Place Rotation)**: Performs an explicit $180.0^\circ$ **CLOCKWISE** in-place rotation at the turnaround point ($\omega_z < 0$).
5. **Leg 3 (Return to HOME Coordinates)**: Returns to the authoritative saved HOME coordinates.
6. **Leg 4 (Final HOME Yaw Alignment & Settle)**: Aligns to the authoritative saved HOME yaw (not an assumed $0^\circ$), stops, cancels active goals, and disarms to Mode 0.

---

## 2. Frozen Kinematic Settings

The following limits are permanently frozen across the production stack:

| Parameter | Value | Scope |
| :--- | :--- | :--- |
| **Normal Linear Speed** | $0.20\text{ m/s}$ | Cruise translation across Legs 1 & 3 |
| **Normal Angular Speed** | $0.40\text{ rad/s}$ | Cruise rotation across Leg 2 |
| **Final Position Correction Ceiling** | $0.10\text{ m/s}$ | Maximum allowable velocity during final approach |
| **Final Yaw Correction Ceiling** | $0.18\text{ rad/s}$ | Maximum allowable yaw rate during final approach |
| **DWB Minimum Velocity Floor** | $0.00\text{ m/s}$ | Zero minimum velocity floor (never impose nonzero crawling) |
| **Tolerances** | $0.04\text{ m}$ / $3.0^\circ$ | Existing navigation tolerances preserved strictly |

---

## 3. Synchronized 20+ Hz Telemetry Recorder

Every trial automatically records at $\ge 20\text{ Hz}$ into a new timestamped report:
`reports/gold_standard_home_test/gold_standard_run_<timestamp>.json`

Captured telemetry channels:
- Raw, smoothed, and final `cmd_vel` ($v_x, \omega_z$).
- Command source (`ROS_AUTONOMY`, `EXACT_MOTION`, `NONE`).
- AMCL pose $(x, y, \theta)$ and full covariance matrix ($cov_x, cov_y, cov_\theta$).
- Wheel Odometry pose $(x, y, \theta)$, twist ($v_x, \omega_z$), and wheel travel.
- IMU Game Rotation Vector yaw, relative yaw, gyro rate Z, and non-magnetic validation flag.
- Drive status: `armed`, `mode`, `reqLinear`, `reqAngular`, `limLinear`, `limAngular`, `lockStatus`.
- Safety: Collision monitor state and polygon name.
- ESP32 hardware health: `bootCount`, `resetReason`, `rtcResetReason`.
- Mission transition timestamps.

---

## 4. Fail-Safe Abort Invariants

The runner monitors invariants on every 20ms tick. It immediately zeroes velocity, cancels navigation goals, and disarms the drivetrain to Mode 0 if:
1. **Localization Lost**: Cockpit reports `state !== 'LOCALIZED'` or AMCL covariance $> 0.20$.
2. **Telemetry Stale**: No telemetry packet received for $> 500\text{ ms}$.
3. **ESP32 Hardware Reboot**: ESP32 `bootCount` changes from its initial armed baseline.
4. **IMU Discontinuity**: Raw IMU yaw jumps $> 45^\circ$ in a single step without high gyro rate.
5. **Leg Timeout**: Any individual mission leg exceeds its timeout limit.
6. **No Automatic Retry**: The test exits immediately on failure and never retries automatically.

---

## 5. Acceptance Grading & Pass Criteria

| Metric | Pass Threshold | Description |
| :--- | :--- | :--- |
| **Final HOME Position Error** | $\le 0.040\text{ m}$ ($40\text{ mm}$) | Euclidean distance from settled AMCL pose to saved HOME |
| **Final HOME Yaw Error** | $\le 3.0^\circ$ | Angular error from settled AMCL yaw to saved HOME yaw |
| **Hardware Stability** | $0$ resets / discontinuities | Zero change in `bootCount`, zero packet loss discontinuities |
| **Direction Reversals** | $\le 1$ corrective reversal | Maximum of one direction change during final HOME settling |
| **Crawling Window** | $\le 2.0\text{ s}$ continuous | No prolonged crawl below $0.05\text{ m/s}$ or $0.12\text{ rad/s}$ |
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
