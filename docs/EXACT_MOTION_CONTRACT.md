# Canonical Exact Motion Contract

This document defines the mandatory, non-negotiable rules for commanding physical rover motion across all scripts, test harnesses, autonomous routines, and AI agent workflows (Antigravity and VS Code / GitHub Copilot).

---

## 1. Exact Linear Movement

Whenever an operator or agent commands the rover to:
* move, drive, travel, advance, or reverse an exact distance;
* go forward/backward a specified number of meters, millimeters, feet, or inches;
* perform a scripted out-and-back or linear translation trial;

The agent and codebase **MUST**:
1. **Use the Established Controller**: Reuse the canonical `LinearApproachController`. Never write ad-hoc open-loop loops or constant-velocity-until-target scripts.
2. **Calibrated Effective Kinematics**: Use effective wheel diameter **`0.06695 m`** (radius `0.033475 m`) and calibrated ROS odometry (`/rover_encoder_odometry`).
3. **Established Approach Behavior**:
   * **Cruise Phase**: Drive at established cruise speed ($v_x = 0.20\text{ m/s}$).
   * **Slowdown / Creep Phase**: Automatically step down to creep speed ($v_x = 0.05\text{ m/s}$) upon entering the $0.150\text{ m}$ ($150\text{ mm}$) approach zone.
   * **Calibrated Stopping Advance**: Command zero velocity ($v_x = 0.00\text{ m/s}$) at $\text{target\_distance} - 0.0135\text{ m}$ ($13.5\text{ mm}$ stopping advance) so dynamic brake coast lands on target.
4. **Never Cut From Full Speed**: Never drive at constant cruise speed ($0.20\text{ m/s}$) until the target and abruptly command zero.
5. **Continuous Telemetry & Actuation Monitoring**: Continuously monitor M1–M4 speed targets, PWM, measured wheel speeds, and raw encoder ticks.
6. **Capture Required Milestones**: Record slowdown entry, target crossing, zero command, post-zero coast movement, and final settled odometry.
7. **Motor Unresponsiveness Abort**: Abort, stop, and disarm immediately if any commanded wheel fails to respond or shows stall/divergence.
8. **Final State**: Always finish stopped, **DISARMED**, and **LOCKED**.

---

## 2. Exact In-Place Rotation

Whenever an operator or agent commands the rover to:
* turn or rotate an exact angle;
* pivot in place;
* turn 90°, 180°, 360°, or any specified angle;

The agent and codebase **MUST**:
1. **Use the Established Controller**: Reuse the canonical `AngularApproachController`.
2. **Non-Magnetic Orientation**: Use **`SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC`** (via `/imu/game_rotation_vector` or `/api/imu`) for relative-yaw feedback.
3. **Independent Gyro Verification**: Use calibrated gyro integration (trapezoidal gyro Z integration) as an independent verification source.
4. **Never Use Magnetic Yaw**: Never use or enable magnetic `SH2_ROTATION_VECTOR` for indoor angular control due to local magnetic distortion.
5. **Fresh Relative Reference**: Establish a new relative-yaw reference ($yaw_0$) at the beginning of each turn maneuver.
6. **Established Angular Slowdown**:
   * **Cruise Phase**: Rotate at cruise speed ($\omega_z = 0.80\text{ rad/s}$).
   * **Slowdown / Creep Phase**: Automatically step down to creep speed ($\omega_z = 0.20\text{ rad/s}$) within the final $30.0^\circ$ approach zone.
   * **Calibrated Stopping Advance**: Command zero angular velocity ($\omega_z = 0.00\text{ rad/s}$) at $\text{target\_angle} - 0.7^\circ$ stopping advance.
7. **Wheel Polarity Verification for Pivots** *(logical telemetry channels `m1..m4`; on rebuilt chassis `1.3.4-remap-r03`, `m1/LF`=Port M2, `m2/RF`=Port M1, `m3/LR`=Port M4, `m4/RR`=Port M3)*:
   * **Clockwise (CW / +Yaw delta in `rover-test`)**: `m1` (LF) and `m3` (LR) positive; `m2` (RF) and `m4` (RR) negative.
   * **Counter-Clockwise (CCW / -Yaw delta in `rover-test`)**: `m1` (LF) and `m3` (LR) negative; `m2` (RF) and `m4` (RR) positive.
8. **Symmetric Target Magnitudes**: Verify opposite sides receive opposite directions with equal intended speed magnitudes.
9. **Telemetry Distinction**: Distinguish speed targets from differing PID/PWM outputs.
10. **Final State**: Always finish stopped, **DISARMED**, and **LOCKED**.

---

## 3. Nav2 vs. Exact Motion Distinction

* **Nav2 Autonomous Navigation**:
  * If Ron explicitly requests a Nav2 goal, action, or waypoint navigation, use the Nav2 action/controller path (`/navigate_to_pose`, `/follow_path`).
  * Allow Nav2's controller (DWB/MPPI) and Goal Checker to manage approach deceleration and pose convergence.
  * Clearly report that Nav2 accuracy depends on global localization (AMCL/SLAM) and configured goal tolerances.
* **Scripted Exact Motion**:
  * If Ron requests an exact scripted distance, translation, or rotation angle without specifically requesting Nav2 autonomy, agents **MUST** use the canonical exact-motion controllers (`LinearApproachController` / `AngularApproachController`).
  * Direct scripted motions must never masquerade as Nav2 pipelines.

---

## 4. Universal Motion Requirements

1. **Operator Presence & Approval**: Ron must explicitly confirm physical presence and authorize motion before any physical movement begins.
2. **No Redundant Preflights**: Do not repeat full preflight audits unless physical setup/hardware changed or a fault occurred.
3. **Single Process Execution**: Use one motion process only; never launch concurrent or duplicate background test routines.
4. **No Ad-Hoc Motion Algorithms**: Never create new motion algorithms or fork tuning constants when proven controllers already exist.
5. **Failure Protocol**: Do not tune or retry automatically after a failure. Preserve telemetry from the first failure for forensic review.
6. **Disarm & Lock Invariant**: Drivetrain must always be stopped, disarmed, and locked on normal completion, timeouts, and all abort/error paths.
