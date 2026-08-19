# ESP Maker Actuator Controller (`esp-maker-usba-4motor`)

This repository contains the embedded C++ firmware (PlatformIO) for the main 4-wheel skid-steer actuator controller board of the Robot Tank, running on the **Maker-ESP32 / Maker-ESP32-Pro** (Espressif ESP32-WROOM-32E module).

---

## 📍 Authoritative Canonical Hardware & Encoder Mapping

The physical wiring, motor terminals, and quadrature encoders map strictly 1-to-1:

| Board Terminal | Physical Location | Quadrature Encoder | Direction Normalization |
| :--- | :--- | :--- | :--- |
| **M1 / Slot 1** | **Physical Left Front (LF)** | **`m1` (Index 0)** | Positive PWM moves chassis **FORWARD** |
| **M2 / Slot 2** | **Physical Right Front (RF)** | **`m2` (Index 1)** | Positive PWM moves chassis **FORWARD** |
| **M3 / Slot 3** | **Physical Left Rear (LR)** | **`m3` (Index 2)** | Positive PWM moves chassis **FORWARD** |
| **M4 / Slot 4** | **Physical Right Rear (RR)** | **`m4` (Index 3)** | Positive PWM moves chassis **FORWARD** |

> [!IMPORTANT]
> **Canonical Axis Convention**: Positive PWM on all four channels drives all four wheels in the **chassis-forward** direction. Skid-steer turns reverse the left or right track pair automatically.

---

## ⚙️ Closed-Loop Pure-Spin Assistance Architecture

To overcome high static breakaway friction and kinetic lateral scrub on hard floors during pure in-place turns ($v_x = 0.0\text{ m/s}$, $|\omega_z| > 0$), the firmware executes a specialized multi-stage assistance layer:

1. **Startup Rear-Pair Breakout Boost (Phase A)**:
   - Command magnitude: **102 PWM** applied across the rear axle pair ($M3, M4$).
   - **Breakout Dwell Guard**: Minimum 100 ms dwell; early exit triggered only when both rear wheels maintain sustained velocity $\ge 0.10\text{ rad/s}$ for 3 consecutive control cycles; hard timeout fallback at 200 ms.
2. **Kinetic Feedforward Base (Phase B)**:
   - Pure-spin kinetic feedforward base: `SPIN_KINETIC_KS_PWM = 75.0f` ($\approx 100.2\text{ PWM}$ before PID at nominal $4.195\text{ rad/s}$ wheel target).
   - Forward-driving rear wheel kinetic floor: `SPIN_FORWARD_REAR_KINETIC_FLOOR = 94.0f`.
   - Ordinary spin wheel kinetic floor: `MIN_SPIN_KINETIC_FF_FLOOR = 80.0f`.
3. **Pure-Spin Velocity Loop Control**:
   - Pure-spin proportional gain: `SPIN_PID_KP = 6.0f` (active strictly during pure spin).
   - Normal driving proportional gain: `KP_SPEED = 2.2f` (reverts immediately upon exiting pure spin).
   - Gains: $K_i = 1.2$, $K_d = 0.05$.

---

## 🛠️ Remote Compilation & SSH Deployment

Flashing is executed remotely through the connected Raspberry Pi 5 (`10.0.0.246` over `/dev/rover-esp32`):

```powershell
# Compile locally and flash remotely over SSH
python deploy_firmware.py
```

---

## 📄 Commissioning & Diagnostics History

Detailed engineering forensics, physical characterizations, telemetry audits, and multi-day diagnostic test results are documented in [`docs/DRIVETRAIN_SPIN_COMMISSIONING_2026-08-19.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/docs/DRIVETRAIN_SPIN_COMMISSIONING_2026-08-19.md).
