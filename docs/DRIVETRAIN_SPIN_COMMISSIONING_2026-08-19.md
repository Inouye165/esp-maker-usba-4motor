# Drivetrain Spin Commissioning & Closed-Loop Control Forensics
**Date**: August 19, 2026  
**Target Board**: NULLLAB Maker-ESP32 / Maker-ESP32-Pro  
**Architecture**: 4-Wheel Skid-Steer Mobile Robot Platform  

---

## Executive Summary & Status Overview

Over a multi-day empirical commissioning sequence, the drivetrain pure-spin in-place turning behavior ($v_x = 0.0\text{ m/s}$, $|\omega_z| > 0$) was systematically investigated, isolated, modeled, and upgraded with a closed-loop assistance layer. 

As of this checkpoint:
- **Canonical Hardware & Encoder Mapping**: Authoritatively verified 1-to-1 ($M1\rightarrow\text{LF}$, $M2\rightarrow\text{RF}$, $M3\rightarrow\text{LR}$, $M4\rightarrow\text{RR}$).
- **Rear-Pair Breakout Boost**: 102 PWM startup boost with conservative breakout guard (100 ms min dwell, dual-rear velocity early exit, 200 ms max timeout).
- **Forward-Rear Kinetic Floor**: 94 PWM floor on the forward-driving rear wheel ($M4/\text{RR}$ in CCW, $M3/\text{LR}$ in CW).
- **Pure-Spin Proportional Gain**: `SPIN_PID_KP = 6.0f` (active strictly during pure spin; normal driving retains $K_p = 2.2$).
- **Pure-Spin Kinetic Feedforward Base Candidate**: `SPIN_KINETIC_KS_PWM = 75.0f` ($\approx 100.2\text{ PWM}$ before PID at nominal $4.195\text{ rad/s}$ target). Compiled & deployed over SSH to Maker ESP32. **Awaiting first physical floor validation.**

---

## Chronological Engineering & Forensic History

### 1. Initial Symptoms & Suspicion of Hardware Defect
- **Symptom**: During autonomous and manual in-place spin maneuvers ($v_x = 0.0\text{ m/s}$, $\omega_z = +0.8\text{ rad/s}$), the rover exhibited severe yaw stall. Physical $RR/M4$ stopped rotating completely after initial motion.
- **Initial Hypothesis**: Defective DC motor or damaged gearbox on Physical $RR/M4$.

### 2. Unloaded Bench Testing & Motor Replacement
- **Unloaded Test**: Off-ground suspension tests showed all 4 motors rotating freely with normal current draw.
- **Hardware Swap**: Physical $RR/M4$ motor assembly was replaced with new hardware. Upon placing the chassis back on the floor, the spin stall persisted.

### 3. Temporary Channel Swap Experiment ($M2 \leftrightarrow M4$)
- **Method**: RF ($M2$) and RR ($M4$) motor wiring and encoders were temporarily swapped.
- **Result**: The spin stall remained fixed at the **physical RR wheel position**, rather than following the ESP32 hardware channel $M2$.
- **Finding**: **PROVEN**. The stall was driven by physical axle loading, lateral tire scrub friction, and differential weight distribution, NOT an electrical driver or channel fault.

### 4. Canonical Hardware & Encoder Verification
- Restored canonical wiring and verified 1-to-1 mapping via manual wheel rotation and low-power channel pulses:
  - `M1 / Slot 1 -> Physical LF -> Encoder m1` (Index 0)
  - `M2 / Slot 2 -> Physical RF -> Encoder m2` (Index 1)
  - `M3 / Slot 3 -> Physical LR -> Encoder m3` (Index 2)
  - `M4 / Slot 4 -> Physical RR -> Encoder m4` (Index 3)
  - Positive PWM on all four channels moves all four wheels **chassis-forward**.

### 5. Open-Loop Loaded Maneuver Characterization & Mirrored Re-Stall
- **Straight Drive**: $1.00$ trim baseline confirmed clean straight driving forward and reverse.
- **Pure Spin Characterization**:
  - In CCW spin, $M4/\text{RR}$ is the forward-driving rear wheel and re-stalled under nominal 85 PWM.
  - In CW spin, $M3/\text{LR}$ is the forward-driving rear wheel and re-stalled under nominal 85 PWM.
- **Finding**: **PROVEN**. Static breakaway stiction and kinetic scrub resistance are mirrored across turn directions and concentrate on the forward-driving rear wheel.

### 6. Transient Breakout Kick & 3/3 Reproducibility
- **Strategy**: Applied 102 PWM startup boost across the rear axle pair ($M3, M4$) for ~200 ms.
- **Reproducibility**: Passed 3/3 CCW trials and 3/3 CW trials. Static stiction was overcome reproducibly in both turn directions.

### 7. Post-Breakout Kinetic Floor (94 PWM)
- Returning to nominal 85 PWM after breakout allowed the forward-driving rear wheel to re-stall.
- Applying a modest kinetic floor of **94 PWM** on ONLY the forward-driving rear wheel ($M4/\text{RR}$ in CCW, $M3/\text{LR}$ in CW) eliminated prolonged re-stalls (near-zero interval fell from hundreds of ms to ~31 ms) without transferring stall to other wheels.

### 8. Telemetry Analysis Artifact & Fresh-Packet Correction
- **Artifact**: Initial analysis scripts reported a 323 ms near-zero interval for $M4/\text{RR}$ during closed-loop CCW spin.
- **Forensic Diagnosis**: HTTP REST `/api/encoders` was polled at 50–100 Hz, while ESP32 serial telemetry streamed at 25 Hz. Polling duplicate REST snapshots produced $0\text{ tick}$ sub-interval deltas.
- **Correction**: Telemetry analysis scripts were updated to filter strictly for **fresh telemetry packets**. Windowed tick analysis confirmed $M4/\text{RR}$ accumulated +189 ticks continuously with **zero physical re-stall**.

### 9. Closed-Loop Assistance Layer Architecture & Bounded Dwell Guard
Firmware logic was added to `WheelController.cpp`:
- **Breakout Dwell Guard**: Minimum 100 ms dwell (10 cycles @ 100 Hz); early exit to kinetic mode requires both rear wheels ($M3, M4$) to maintain measured velocity $\ge 0.10\text{ rad/s}$ for 3 consecutive cycles (30 ms); hard maximum timeout at 200 ms.
- **9/9 Unit Tests**: Host unit test suite verified dwell guard, twitch protection, hard timeout, kinetic floor, and disarm cancellation.

### 10. Bounded +90° CCW Physical Turn Validation
- **Command**: $v_x = 0.0\text{ m/s}$, $\omega_z = +0.8\text{ rad/s}$ CCW. Hard timeout: 3.0s.
- **Result**: Rover rotated smoothly to **$+55.94^\circ$** in 3.008s before the 3.0s hard timeout disarmed the drivetrain.
- **Performance**: Zero wheel stalls, zero oscillations, clean disarm, but actual yaw rate ($\sim 0.326\text{ rad/s}$) undershot commanded $\omega_z = +0.80\text{ rad/s}$.

### 11. Wheel-Speed-Control Audit & $K_p = 6.0$ Tuning
- **Audit Finding**: Under $K_p = 2.2$, a 2.66 rad/s velocity error produced only $+5.9\text{ PWM}$ of P-term correction. Wheels operated at ~1.5–3.0 rad/s under lateral scrub.
- **Firmware Update**: Added parameterized pure-spin proportional gain `SPIN_PID_KP = 6.0f` (active strictly during pure spin; normal driving retains $K_p = 2.2$).
- **Physical Test Result**: 1.5s CCW spin was stable with zero wheel stalls, zero oscillations, and improved velocity tracking ($M4/\text{RR}$ avg velocity increased to $1.813\text{ rad/s}$).

### 12. Feedforward Model & Integral State Audits
- **Feedforward Model Audit**: Analyzed `ff_raw = SPIN_KS_PWM + kV * targetVel` ($58 + 6 \cdot 4.195 = 83.2\text{ PWM}$). Confirmed that 83.2 PWM feedforward systematically underpredicts the PWM required (~110–130 PWM) to maintain 4.195 rad/s under 4-wheel lateral scrub drag.
- **Integral State Audit**: Audited `SingleWheelController::setTargetVelocity()`. Confirmed `errorSum` is **NOT** reset when target velocity is unchanged during 100 Hz loop iterations; integral accumulator functions correctly.

### 13. Current Candidate: `SPIN_KINETIC_KS_PWM = 75.0f`
- **Implementation**: Added dedicated pure-spin kinetic feedforward base `SPIN_KINETIC_KS_PWM = 75.0f`.
- **Feedforward Prediction**: At target $|\omega_{\text{target}}| = 4.195\text{ rad/s}$:
  $$\text{ff\_raw} = 75.0 + (6.0 \times 4.195) = 100.17\text{ PWM} \approx \mathbf{100.2\text{ PWM before PID}}$$
- **Deployment**: Compiled cleanly and flashed over SSH to Maker ESP32 (`task-1308` PASS).

---

## Categorized Forensic Conclusions

### PROVEN
1. Canonical hardware mapping is 1-to-1 ($M1\rightarrow\text{LF}$, $M2\rightarrow\text{RF}$, $M3\rightarrow\text{LR}$, $M4\rightarrow\text{RR}$).
2. In-place spin stalls on hard floors are caused by lateral tire scrub drag concentrating torque load on the forward-driving rear wheel ($M4/\text{RR}$ in CCW, $M3/\text{LR}$ in CW).
3. 102 PWM rear-pair breakout boost for 100–140 ms reliably overcomes static stiction in both CCW and CW turns (3/3 reproducibility in both directions).
4. A 94 PWM kinetic floor on the forward-driving rear wheel prevents static re-stalls after breakout.
5. The 323 ms near-zero interval report was a telemetry sampling artifact caused by HTTP REST polling exceeding 25 Hz serial packet updates; fresh-packet filtering resolved it.
6. `SingleWheelController::setTargetVelocity()` does not reset `errorSum` when target velocity is unchanged.

### STRONGLY SUPPORTED
1. Pure-spin proportional gain $K_p = 6.0$ operates stably without oscillation or output saturation while improving velocity loop authority.
2. The original pure-spin feedforward base (`SPIN_KS_PWM = 58.0`) underpredicts the feedforward PWM required during continuous 4-wheel lateral scrub sliding.

### HYPOTHESIS
1. Raising pure-spin kinetic feedforward intercept to `SPIN_KINETIC_KS_PWM = 75.0f` (~100.2 PWM FF before PID at 4.195 rad/s) will bring measured wheel speeds close to the 4.195 rad/s target, closing the chassis yaw-rate tracking gap.

### NOT YET VALIDATED
1. Physical floor performance of `SPIN_KINETIC_KS_PWM = 75.0f` (compiled and flashed, but no physical motion performed yet).

---

## Current Next Step

**Action Item**: Execute ONE bounded 1.5-second physical floor test ($v_x = 0.0\text{ m/s}$, $\omega_z = +0.8\text{ rad/s}$ CCW) to evaluate wheel velocity tracking and feedforward performance under `SPIN_KINETIC_KS_PWM = 75.0f` before any further tuning.
