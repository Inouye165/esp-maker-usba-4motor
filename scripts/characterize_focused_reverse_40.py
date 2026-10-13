#!/usr/bin/env python3
"""
scripts/characterize_focused_reverse_40.py
Rover 1 Focused Reverse Sustained Crawl Test (-40 PWM for 1.5s with -64 Dynamic Breakout).

Objective:
- Establish reverse motion cleanly via an initial -64 PWM straight-crawl breakout impulse
  matching the firmware's STRAIGHT_STICTION_BOOST_PWM, bounded by a short timeout (350 ms).
- Require fresh encoder evidence of reverse rotation on all four wheels (displacement <= -6 ticks
  and speed <= -0.15 rad/s) with advancing encoder sequence and fresh PID telemetry.
- To prevent speed overshoot caused by continuing -64 PWM after rolling is already established,
  transition immediately to Sustained -40 PWM on the first qualified frame (CONSECUTIVE_STARTUP_FRAMES = 1).
  If multi-frame qualification is configured, immediately drop PWM from -64 to -40 once motion is
  detected so -64 is never kept active merely while waiting for verification or slower PID telemetry.
- Promptly dispatch -40 PWM using the validated snapshot to transition into hold.
  (Note: while prompt transition avoids unnecessary breakout duration, this does not guarantee avoidance
  of the speed abort if physical wheel acceleration is sufficiently rapid).
- Record encoder samples through cleanup to capture deceleration and final stopped displacement.
- If startup is not established within timeout, abort immediately and classify hold as INCONCLUSIVE.
- If established, step directly to -40 PWM and hold for approximately 1.50 seconds.
- Evaluate whether -40 PWM maintains continuous wheel rotation at equilibrium near the
  calibrated reverse crawl velocity (-0.015 m/s <=> -0.462 rad/s), or whether it
  progressively decelerates into a stall.
- Inherits directly from the reviewed TrialAHarness to preserve all safety protections:
  - Telemetry validation: rejects missing, malformed, non-finite, or stale fields.
  - Fresh firmware verification: requires 1.3.2-olp-safe confirmed fresh via systemd cursor.
  - Arming inside cleanup-protected try block.
  - Independent stream advancement tracking (drive, PID, encoders, IMU).
  - Pre-dispatch rechecks of monotonic deadlines, effective age, and armed status.
  - Common exit path: fail-closed zero PWM, disarm, and standstill verification.

Profile:
  Phase 1 (Breakout Startup):   -64 PWM, bounded by 350 ms timeout (transitions on 4-wheel rotation)
  Phase 2 (Sustained Hold):     -40 PWM for 1500 ms (1.50 s)
  Total Nominal Duration:       ~1.70 - 1.85 s (Hard cutoff: 2.00 s)
"""

import sys
import os
import time
import json
import math
import argparse
from typing import Dict, Any, List, Optional

# Add parent directory of scripts to path if needed
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from characterize_trial_a_openloop import (
    TrialAHarness,
    TelemetryValidationError,
    ContinuousYawTracker,
    AccumulatedTravelTracker,
    StreamAdvancementTracker,
    WHEEL_DIAMETER_M,
    WHEEL_RADIUS_M,
    TICKS_PER_REV,
    METERS_PER_TICK,
    MAX_MEAN_ABS_SPEED_MPS,
    MAX_ACCUMULATED_YAW_DEG,
    MAX_TELEMETRY_AGE_S,
    HTTP_REQUEST_TIMEOUT_S,
    REQUIRED_SAFETY_FIRMWARE
)

# Focused Safety Thresholds & Parameters
MAX_ABS_WHEEL_TRAVEL_M = 0.100   # 100 mm maximum accumulated absolute travel on any single wheel (expected ~35-50 mm)
MAX_TOTAL_DURATION_S = 2.00      # 2.00 s hard safety cutoff (previously reviewed value)
TARGET_CRAWL_SPEED_MPS = -0.015                                # -15 mm/s
TARGET_CRAWL_RADPS = TARGET_CRAWL_SPEED_MPS / WHEEL_RADIUS_M   # -0.4615 rad/s

# Dynamic Startup Parameters (-64 PWM Breakout)
STARTUP_PWM = -64
STARTUP_TIMEOUT_S = 0.350         # 350 ms maximum timeout for startup breakout
MIN_BREAKOUT_TICKS = 6            # >= 6 ticks negative displacement on each wheel (~0.62 mm)
MIN_BREAKOUT_SPEED_RADPS = -0.15  # <= -0.15 rad/s negative angular velocity on each wheel
CONSECUTIVE_STARTUP_FRAMES = 1    # transitions immediately on first verified 4-wheel breakout frame

# Sustained Hold Parameters
SUSTAINED_PWM = -40
SUSTAINED_DURATION_S = 1.50


class FocusedReverse40Harness(TrialAHarness):
    def __init__(self, host="127.0.0.1", port=3000, report_dir="reports", timeout_s=HTTP_REQUEST_TIMEOUT_S, ssh_host=None, ssh_user="ron"):
        super().__init__(host=host, port=port, report_dir=report_dir, timeout_s=timeout_s, ssh_host=ssh_host, ssh_user=ssh_user)

    def _record_telemetry_sample(self, now_mono: float, start_mono: float, commanded_pwm: int, step_idx: int, snap: Dict[str, Any], step_name: str = "Unknown", in_cleanup: bool = False):
        raw_yaw = snap["yaw_deg"]
        unwrapped_yaw, net_yaw_drift, accum_yaw_excursion = self.yaw_tracker.update(raw_yaw)

        ticks = snap["ticks"]
        accum_travel_per_wheel, max_accum_travel_m, max_net_disp_m = self.travel_tracker.update(ticks)

        speeds_mps = snap["speeds_mps"]
        mean_abs_speed_mps = sum(abs(s) for s in speeds_mps) / 4.0 if speeds_mps else 0.0

        time_offset_s = now_mono - start_mono

        sample = {
            "time_offset_s": round(time_offset_s, 4),
            "timestamp_mono": now_mono,
            "epoch_time_ms": snap["receipt_epoch_ms"],
            "step_index": step_idx,
            "step_name": step_name,
            "commanded_pwm": commanded_pwm,
            "delivered_pwms": list(snap["pwms"]),
            "measured_speeds_radps": list(snap["speeds_radps"]),
            "measured_speeds_mps": [round(s, 4) for s in speeds_mps],
            "mean_abs_speed_mps": round(mean_abs_speed_mps, 4),
            "raw_ticks": list(ticks),
            "accum_travel_per_wheel_m": [round(t, 5) for t in accum_travel_per_wheel],
            "max_accum_abs_travel_m": round(max_accum_travel_m, 5),
            "max_net_displacement_m": round(max_net_disp_m, 5),
            "raw_yaw_deg": round(raw_yaw, 2),
            "unwrapped_yaw_deg": round(unwrapped_yaw, 2),
            "net_yaw_drift_deg": round(net_yaw_drift, 2),
            "accum_yaw_excursion_deg": round(accum_yaw_excursion, 2),
            "armed": snap["armed"],
            "mode": snap["mode"],
            "source_metadata": {
                "seq_drive": snap["seq_drive"],
                "seq_pid": snap["seq_pid"],
                "seq_encoders": snap["seq_encoders"],
                "seq_imu": snap["seq_imu"],
                "age_drive_ms": snap["age_drive_ms"],
                "age_pid_ms": snap["age_pid_ms"],
                "age_encoders_ms": snap["age_encoders_ms"],
                "age_imu_ms": snap["age_imu_ms"],
                "data_age_ms": snap["data_age_ms"]
            }
        }
        self.samples.append(sample)

        # Do not evaluate trial safety abort limits during cleanup
        if in_cleanup:
            return

        # Evaluate Safety Limits
        if max_accum_travel_m > MAX_ABS_WHEEL_TRAVEL_M:
            self.abort_reason = f"EXCEEDED_MAX_ABS_TRAVEL ({max_accum_travel_m*1000:.1f} mm > {MAX_ABS_WHEEL_TRAVEL_M*1000:.1f} mm)"
            return

        if mean_abs_speed_mps > MAX_MEAN_ABS_SPEED_MPS:
            self.abort_reason = f"EXCEEDED_MAX_SPEED ({mean_abs_speed_mps:.4f} m/s > {MAX_MEAN_ABS_SPEED_MPS:.4f} m/s)"
            return

        if abs(net_yaw_drift) > MAX_ACCUMULATED_YAW_DEG:
            self.abort_reason = f"EXCEEDED_MAX_YAW_DRIFT ({abs(net_yaw_drift):.2f} deg > {MAX_ACCUMULATED_YAW_DEG:.2f} deg)"
            return

    def _execute_cleanup(self) -> bool:
        """
        Fail-closed drivetrain cleanup with continuous encoder sampling through standstill.
        Dispatches zero PWM and disarm IMMEDIATELY without delay, then samples telemetry
        during verification/settling to record deceleration and final stopped displacement.
        """
        print("\n=== Executing Fail-Closed Drivetrain Cleanup ===")

        # 1. Independent Zero PWM Dispatch (IMMEDIATE, NO DELAY)
        try:
            res = self.send_open_loop_pwm(0, 0, 0, 0)
            if res and res.get("ok"):
                print("1. Sent zero open-loop PWM command [0, 0, 0, 0]: SUCCESS")
            else:
                print(f"1. Zero PWM dispatch returned ok=false: {res}", file=sys.stderr)
        except Exception as e:
            print(f"1. Zero PWM dispatch exception (attempting disarm anyway): {e}", file=sys.stderr)

        # 2. Independent Drive Disarm Dispatch (IMMEDIATE, NO DELAY)
        try:
            res = self.disarm_drive()
            if res and res.get("ok"):
                print("2. Disarmed normal drive (transition to LOCKED): SUCCESS")
            else:
                print(f"2. Drive disarm returned: {res}", file=sys.stderr)
        except Exception as e:
            print(f"2. Drive disarm exception: {e}", file=sys.stderr)

        # 3. Telemetry Verification & Deceleration Recording Loop
        start_mono = self.samples[0]["timestamp_mono"] if self.samples else time.monotonic()
        deadline = time.monotonic() + 1.50
        verified = False
        last_failure_reason = "TIMEOUT"

        while time.monotonic() < deadline:
            t_sample = time.monotonic()
            try:
                snap = self.fetch_telemetry_snapshot()

                is_standstill = all(abs(w) < 0.05 for w in snap["speeds_radps"])
                step_name = "Cleanup_Standstill" if (is_standstill and snap["mode"] == 0 and not snap["armed"]) else "Cleanup_Cutoff"
                self._record_telemetry_sample(t_sample, start_mono, 0, 99, snap, step_name=step_name, in_cleanup=True)

                # Freshness
                if snap["data_age_ms"] > (MAX_TELEMETRY_AGE_S * 1000.0):
                    last_failure_reason = f"Stale telemetry ({snap['data_age_ms']:.1f} ms > 250 ms)"
                    time.sleep(0.05)
                    continue

                # Disarmed state
                if snap["armed"] is not False:
                    last_failure_reason = "Drive reports armed=True"
                    time.sleep(0.05)
                    continue

                # Mode 0 (LOCKED)
                if snap["mode"] != 0:
                    last_failure_reason = f"Drive mode is {snap['mode']}, expected 0 (LOCKED)"
                    time.sleep(0.05)
                    continue

                # Zero motor output
                if any(p != 0 for p in snap["pwms"]):
                    last_failure_reason = f"Non-zero motor PWM outputs: {snap['pwms']}"
                    time.sleep(0.05)
                    continue

                # Standstill confirmed (< 0.05 rad/s)
                if not is_standstill:
                    last_failure_reason = f"Residual wheel motion detected: {snap['speeds_radps']} rad/s"
                    time.sleep(0.05)
                    continue

                verified = True
                print("3. Verification PASSED: Confirmed zero output (PWM=0), standstill (<0.05 rad/s), disarmed (armed=False), and Mode 0 (LOCKED) in fresh telemetry.")
                break
            except Exception as e:
                last_failure_reason = f"Verification query exception: {e}"
            time.sleep(0.05)

        if not verified:
            print(f"3. Verification FAILED: {last_failure_reason}", file=sys.stderr)

        return verified

    def _finalize_report(self, aborted: bool = False, cleanup_ok: bool = True) -> Dict[str, Any]:
        os.makedirs(self.report_dir, exist_ok=True)
        timestamp_str = time.strftime("%Y%m%d_%H%M%S")
        report_filename = f"focused_reverse_40_report_{timestamp_str}.json"
        report_path = os.path.join(self.report_dir, report_filename)

        if not cleanup_ok:
            overall_status = "CLEANUP_FAILED"
        elif aborted:
            overall_status = "ABORTED"
        else:
            overall_status = "COMPLETED"

        # Focused Hold Analysis on Phase 2 (-40 PWM)
        sustained_samples = [s for s in self.samples if s["step_name"] == "Sustained_40"]
        hold_analysis = {}
        if sustained_samples:
            n_hold = len(sustained_samples)
            # Evaluate final 1.0 second of hold (samples >= 0.5s into the step)
            t0_step = sustained_samples[0]["time_offset_s"]
            tail_samples = [s for s in sustained_samples if (s["time_offset_s"] - t0_step) >= 0.50]
            if not tail_samples:
                tail_samples = sustained_samples[n_hold // 2:]

            wheel_speeds = [[s["measured_speeds_radps"][m] for s in tail_samples] for m in range(4)]
            mean_speeds = [sum(w) / len(w) for w in wheel_speeds]
            std_speeds = [math.sqrt(sum((x - m)**2 for x in w) / len(w)) for w, m in zip(wheel_speeds, mean_speeds)]
            stalled_wheels = [m+1 for m, w in enumerate(wheel_speeds) if abs(w[-1]) < 0.05]

            # Calculate dω/dt trend over tail
            dt = tail_samples[-1]["time_offset_s"] - tail_samples[0]["time_offset_s"]
            trend_slopes = [((w[-1] - w[0]) / dt) if dt > 0.01 else 0.0 for w in wheel_speeds]

            hold_analysis = {
                "status": "EVALUATED",
                "sample_count": n_hold,
                "tail_sample_count": len(tail_samples),
                "tail_duration_s": round(dt, 3),
                "tail_mean_speeds_radps": [round(x, 4) for x in mean_speeds],
                "tail_speed_stdevs_radps": [round(x, 4) for x in std_speeds],
                "tail_trend_slopes_radps2": [round(x, 4) for x in trend_slopes],
                "stalled_wheels": stalled_wheels,
                "overall_tail_mean_radps": round(sum(mean_speeds) / 4.0, 4),
                "target_crawl_radps": round(TARGET_CRAWL_RADPS, 4),
                "near_crawl_target": abs(sum(mean_speeds) / 4.0 - TARGET_CRAWL_RADPS) < 0.15,
                "is_rolling_sustained": (len(stalled_wheels) == 0 and abs(sum(mean_speeds)/4.0) >= 0.20)
            }
        else:
            # If startup failed or hold was never reached, classify hold as INCONCLUSIVE
            hold_analysis = {
                "status": "INCONCLUSIVE",
                "reason": "Startup breakout not established on all 4 wheels; hold phase was bypassed.",
                "startup_timeout_s": STARTUP_TIMEOUT_S,
                "startup_pwm": STARTUP_PWM,
                "target_crawl_radps": round(TARGET_CRAWL_RADPS, 4)
            }

        max_travel = max(s["max_accum_abs_travel_m"] for s in self.samples) if self.samples else 0.0
        max_speed = max(s["mean_abs_speed_mps"] for s in self.samples) if self.samples else 0.0
        max_yaw = max(abs(s["net_yaw_drift_deg"]) for s in self.samples) if self.samples else 0.0

        report_data = {
            "test_type": "FOCUSED_REVERSE_SUSTAINED_40",
            "timestamp": timestamp_str,
            "aborted": aborted,
            "abort_reason": self.abort_reason,
            "cleanup_confirmed": cleanup_ok,
            "overall_status": overall_status,
            "summary_metrics": {
                "max_accum_abs_travel_m": round(max_travel, 4),
                "max_mean_abs_speed_mps": round(max_speed, 4),
                "max_net_yaw_drift_deg": round(max_yaw, 2),
                "total_samples": len(self.samples)
            },
            "hold_analysis": hold_analysis,
            "telemetry_samples": self.samples
        }

        with open(report_path, "w", encoding="utf-8") as f:
            json.dump(report_data, f, indent=2)

        print(f"\nReport generated successfully: {report_path}")
        print(f"Overall Status: {overall_status}")
        if self.abort_reason:
            print(f"Abort Reason: {self.abort_reason}")
        print(f"Max Accumulated Absolute Wheel Travel: {max_travel*1000:.1f} mm (Limit: {MAX_ABS_WHEEL_TRAVEL_M*1000:.1f} mm)")
        print(f"Max Mean Absolute Speed:             {max_speed:.4f} m/s (Limit: {MAX_MEAN_ABS_SPEED_MPS:.4f} m/s)")
        print(f"Max Net Yaw Drift:                   {max_yaw:.2f} deg (Limit: {MAX_ACCUMULATED_YAW_DEG:.2f} deg)")
        return report_data

    def execute_trial(self, dry_run: bool = False, skip_firmware_check: bool = False) -> Dict[str, Any]:
        if dry_run:
            print("[DRY-RUN] Simulating Focused Reverse -40 execution without physical arming or motion.")
            return {"overall_status": "DRY_RUN_SUCCESS", "status": "DRY_RUN_SUCCESS", "startup_pwm": STARTUP_PWM, "hold_pwm": SUSTAINED_PWM}

        print("\n=== Executing Focused Reverse Test (-64 PWM Breakout -> -40 PWM Hold for 1.5s) ===")
        print(f"Startup Timeout: {STARTUP_TIMEOUT_S:.2f} s | Hold Duration: {SUSTAINED_DURATION_S:.2f} s | Hard Ceiling: {MAX_TOTAL_DURATION_S:.2f} s")

        self.samples.clear()
        self.abort_reason = None
        self.cleanup_failed = False
        cleanup_ok = False

        try:
            # 0. Safety Firmware Verification
            if not skip_firmware_check:
                fw_ok = False
                try:
                    fw_ok = self.verify_safety_firmware(REQUIRED_SAFETY_FIRMWARE)
                except Exception as e:
                    self.abort_reason = f"FIRMWARE_VERIFICATION_FAILED ({e})"
                    print(f"\n[ABORT TRIGGERED] {self.abort_reason}", file=sys.stderr)

                if not fw_ok and not self.abort_reason:
                    self.abort_reason = "FIRMWARE_VERIFICATION_FAILED (Verification returned False)"
                    print(f"\n[ABORT TRIGGERED] {self.abort_reason}", file=sys.stderr)

            if not self.abort_reason:
                # 1. Arming is INSIDE cleanup-protected try block
                print("Arming normal drive...")
                arm_resp = self.arm_drive()
                if not arm_resp.get("ok"):
                    raise RuntimeError(f"Arming request rejected: {arm_resp}")

                # 2. Confirm armed state in fresh telemetry
                t_arm_deadline = time.monotonic() + 1.0
                armed_confirmed = False
                while time.monotonic() < t_arm_deadline:
                    drv = self.fetch_drive_status()
                    if drv["armed"] is True and drv["mode"] != 0:
                        armed_confirmed = True
                        break
                    time.sleep(0.02)

                if not armed_confirmed:
                    raise RuntimeError("Arming confirmation failed: drive did not confirm armed=True within 1.0s")
                print("Drive armed confirmation: PASS (Mode active, armed=True)")

                # 3. Establish travel/yaw baselines BEFORE sending nonzero PWM
                baseline_snap = self.fetch_telemetry_snapshot()
                if any(abs(s) > 0.02 for s in baseline_snap["speeds_mps"]):
                    raise RuntimeError(f"Motion detected during baseline capture: {baseline_snap['speeds_mps']}")

                self.travel_tracker.reset(baseline_snap["ticks"])
                self.yaw_tracker.reset(baseline_snap["yaw_deg"])
                start_mono = time.monotonic()
                self.stream_tracker.reset(start_mono)
                baseline_ticks = list(baseline_snap["ticks"])
                baseline_seq_enc = baseline_snap["seq_encoders"]
                baseline_seq_pid = baseline_snap["seq_pid"]

                # Record baseline sample
                self._record_telemetry_sample(start_mono, start_mono, 0, -1, baseline_snap, "Baseline")

                # 4. Phase 1: Dynamic Breakout Startup (-64 PWM)
                print(f"\n[Phase 1] Startup Breakout: PWM={STARTUP_PWM} (Timeout: {STARTUP_TIMEOUT_S:.2f}s, Criterion: all 4 wheels >= {MIN_BREAKOUT_TICKS} ticks & <= {MIN_BREAKOUT_SPEED_RADPS} rad/s with advancing sequences and negative progression over {CONSECUTIVE_STARTUP_FRAMES} frames)")
                startup_established = False
                startup_qualifying_count = 0
                last_qualifying_ticks = None
                last_qualifying_seq_enc = None
                last_qualifying_seq_pid = None
                phase1_start_mono = time.monotonic()

                while (time.monotonic() - phase1_start_mono) < STARTUP_TIMEOUT_S:
                    t_loop_start = time.monotonic()
                    elapsed_total = t_loop_start - start_mono

                    # Check 1: Monotonic duration deadline
                    if elapsed_total > MAX_TOTAL_DURATION_S:
                        self.abort_reason = f"EXCEEDED_MAX_TOTAL_DURATION ({elapsed_total:.3f}s > {MAX_TOTAL_DURATION_S}s)"
                        print(f"\n[ABORT TRIGGERED] {self.abort_reason}", file=sys.stderr)
                        break

                    # Check 2: Ingest telemetry snapshot
                    snap = self.fetch_telemetry_snapshot()
                    t_pre_dispatch = time.monotonic()

                    # Check 1: Monotonic duration deadline post-fetch
                    if (t_pre_dispatch - start_mono) > MAX_TOTAL_DURATION_S:
                        self.abort_reason = f"EXCEEDED_MAX_TOTAL_DURATION_POST_FETCH ({t_pre_dispatch - start_mono:.3f}s > {MAX_TOTAL_DURATION_S}s)"
                        print(f"\n[ABORT TRIGGERED] {self.abort_reason}", file=sys.stderr)
                        break

                    # Check 2: Recheck startup deadline after telemetry collection and before any -64 PWM dispatch
                    elapsed_startup = t_pre_dispatch - phase1_start_mono
                    if elapsed_startup > STARTUP_TIMEOUT_S:
                        self.abort_reason = f"STARTUP_TIMEOUT_POST_FETCH (Startup deadline exceeded post-fetch: {elapsed_startup:.3f}s > {STARTUP_TIMEOUT_S:.3f}s)"
                        print(f"\n[ABORT TRIGGERED] {self.abort_reason}", file=sys.stderr)
                        break

                    # Check 3: Effective telemetry age
                    fetch_start = snap.get("fetch_start_mono", snap["receipt_mono"])
                    fetch_delay_s = t_pre_dispatch - fetch_start
                    effective_age_ms = snap["data_age_ms"] + (fetch_delay_s * 1000.0)
                    if effective_age_ms > (MAX_TELEMETRY_AGE_S * 1000.0):
                        self.abort_reason = f"STALE_TELEMETRY_POST_FETCH (Effective age {effective_age_ms:.1f}ms > {MAX_TELEMETRY_AGE_S*1000}ms)"
                        print(f"\n[ABORT TRIGGERED] {self.abort_reason}", file=sys.stderr)
                        break

                    # Check 4: Stream advancement
                    seq_map = {
                        "drive": snap["seq_drive"],
                        "pid": snap["seq_pid"],
                        "encoders": snap["seq_encoders"],
                        "imu": snap["seq_imu"]
                    }
                    stream_abort = self.stream_tracker.update(t_pre_dispatch, seq_map)
                    if stream_abort:
                        self.abort_reason = stream_abort
                        print(f"\n[ABORT TRIGGERED] {self.abort_reason}", file=sys.stderr)
                        break

                    # Check 5: Armed state
                    if snap["armed"] is not True or snap["mode"] == 0:
                        self.abort_reason = f"DRIVE_DISARMED_OR_LOCKED (armed={snap['armed']}, mode={snap['mode']})"
                        print(f"\n[ABORT TRIGGERED] {self.abort_reason}", file=sys.stderr)
                        break

                    # Check 6: Physical motion bounds
                    self._record_telemetry_sample(t_pre_dispatch, start_mono, STARTUP_PWM, 0, snap, "Startup_Breakout_64")
                    if self.abort_reason:
                        print(f"\n[ABORT TRIGGERED] {self.abort_reason}", file=sys.stderr)
                        break

                    # Evaluate Startup Breakout Criterion on current snapshot
                    # Count confirmation from TWO DISTINCT ENCODER OBSERVATIONS with additional reverse tick progression on every wheel
                    # Require PID telemetry to have advanced since baseline and remain fresh (do not require new PID packet on every encoder frame)
                    cur_ticks = snap["ticks"]
                    cur_speeds = snap["speeds_radps"]
                    cur_seq_enc = snap["seq_encoders"]
                    cur_seq_pid = snap["seq_pid"]
                    tick_deltas = [cur_ticks[m] - baseline_ticks[m] for m in range(4)]

                    all_wheels_displaced = all(d <= -MIN_BREAKOUT_TICKS for d in tick_deltas)
                    all_wheels_speed_ok = all(w <= MIN_BREAKOUT_SPEED_RADPS for w in cur_speeds)
                    pid_advanced = (cur_seq_pid > baseline_seq_pid)
                    pid_fresh = (snap["age_pid_ms"] <= (MAX_TELEMETRY_AGE_S * 1000.0)) and (last_qualifying_seq_pid is None or cur_seq_pid >= last_qualifying_seq_pid)

                    if all_wheels_displaced and all_wheels_speed_ok and pid_advanced and pid_fresh:
                        if startup_qualifying_count == 0:
                            # Frame 1: encoder sequence must have advanced from baseline
                            if cur_seq_enc > baseline_seq_enc:
                                startup_qualifying_count = 1
                                last_qualifying_ticks = list(cur_ticks)
                                last_qualifying_seq_enc = cur_seq_enc
                                last_qualifying_seq_pid = cur_seq_pid
                        else:
                            # Frame 2: requires distinct advancing encoder sequence and additional negative tick progression on EVERY wheel
                            enc_advancing = (cur_seq_enc > last_qualifying_seq_enc)
                            additional_neg_ticks = all(cur_ticks[m] < last_qualifying_ticks[m] for m in range(4))
                            if enc_advancing and additional_neg_ticks:
                                startup_qualifying_count += 1
                                last_qualifying_ticks = list(cur_ticks)
                                last_qualifying_seq_enc = cur_seq_enc
                                last_qualifying_seq_pid = cur_seq_pid
                            else:
                                # Repeated encoder sequence or stall without progression resets qualifier
                                startup_qualifying_count = 0
                    else:
                        startup_qualifying_count = 0

                    if startup_qualifying_count >= CONSECUTIVE_STARTUP_FRAMES:
                        startup_established = True
                        print(f"Startup breakout confirmed on all 4 wheels (deltas={tick_deltas}, speeds={cur_speeds} across {startup_qualifying_count} qualifying frame(s) with advancing sequences).")
                        print(f"Promptly dispatching Sustained PWM ({SUSTAINED_PWM}) using validated snapshot.")
                        self.send_open_loop_pwm(SUSTAINED_PWM, SUSTAINED_PWM, SUSTAINED_PWM, SUSTAINED_PWM)
                        cycle_elapsed = time.monotonic() - t_loop_start
                        sleep_time = max(0.005, 0.050 - cycle_elapsed)
                        time.sleep(sleep_time)
                        break

                    # Dispatch startup PWM (-64 before motion detected; drop to -40 once motion is observed to avoid overshoot)
                    next_startup_pwm = SUSTAINED_PWM if startup_qualifying_count > 0 else STARTUP_PWM
                    self.send_open_loop_pwm(next_startup_pwm, next_startup_pwm, next_startup_pwm, next_startup_pwm)

                    cycle_elapsed = time.monotonic() - t_loop_start
                    sleep_time = max(0.005, 0.050 - cycle_elapsed)
                    time.sleep(sleep_time)

                # Check if startup succeeded or failed
                if not startup_established and not self.abort_reason:
                    self.abort_reason = f"STARTUP_BREAKAWAY_FAILED (All 4 wheels did not establish sustained reverse rotation within {STARTUP_TIMEOUT_S:.2f}s)"
                    print(f"\n[ABORT TRIGGERED] {self.abort_reason}", file=sys.stderr)

                # 5. Phase 2: Sustained Hold (-40 PWM for 1.50 s)
                if startup_established and not self.abort_reason:
                    print(f"\n[Phase 2] Sustained Hold: PWM={SUSTAINED_PWM} for {SUSTAINED_DURATION_S:.2f}s")
                    phase2_start_mono = time.monotonic()

                    while (time.monotonic() - phase2_start_mono) < SUSTAINED_DURATION_S:
                        t_loop_start = time.monotonic()
                        elapsed_total = t_loop_start - start_mono

                        if elapsed_total > MAX_TOTAL_DURATION_S:
                            self.abort_reason = f"EXCEEDED_MAX_TOTAL_DURATION ({elapsed_total:.3f}s > {MAX_TOTAL_DURATION_S}s)"
                            print(f"\n[ABORT TRIGGERED] {self.abort_reason}", file=sys.stderr)
                            break

                        snap = self.fetch_telemetry_snapshot()
                        t_pre_dispatch = time.monotonic()

                        if (t_pre_dispatch - start_mono) > MAX_TOTAL_DURATION_S:
                            self.abort_reason = f"EXCEEDED_MAX_TOTAL_DURATION_POST_FETCH ({t_pre_dispatch - start_mono:.3f}s > {MAX_TOTAL_DURATION_S}s)"
                            print(f"\n[ABORT TRIGGERED] {self.abort_reason}", file=sys.stderr)
                            break

                        fetch_start = snap.get("fetch_start_mono", snap["receipt_mono"])
                        fetch_delay_s = t_pre_dispatch - fetch_start
                        effective_age_ms = snap["data_age_ms"] + (fetch_delay_s * 1000.0)
                        if effective_age_ms > (MAX_TELEMETRY_AGE_S * 1000.0):
                            self.abort_reason = f"STALE_TELEMETRY_POST_FETCH (Effective age {effective_age_ms:.1f}ms > {MAX_TELEMETRY_AGE_S*1000}ms)"
                            print(f"\n[ABORT TRIGGERED] {self.abort_reason}", file=sys.stderr)
                            break

                        seq_map = {
                            "drive": snap["seq_drive"],
                            "pid": snap["seq_pid"],
                            "encoders": snap["seq_encoders"],
                            "imu": snap["seq_imu"]
                        }
                        stream_abort = self.stream_tracker.update(t_pre_dispatch, seq_map)
                        if stream_abort:
                            self.abort_reason = stream_abort
                            print(f"\n[ABORT TRIGGERED] {self.abort_reason}", file=sys.stderr)
                            break

                        if snap["armed"] is not True or snap["mode"] == 0:
                            self.abort_reason = f"DRIVE_DISARMED_OR_LOCKED (armed={snap['armed']}, mode={snap['mode']})"
                            print(f"\n[ABORT TRIGGERED] {self.abort_reason}", file=sys.stderr)
                            break

                        self._record_telemetry_sample(t_pre_dispatch, start_mono, SUSTAINED_PWM, 1, snap, "Sustained_40")
                        if self.abort_reason:
                            print(f"\n[ABORT TRIGGERED] {self.abort_reason}", file=sys.stderr)
                            break

                        self.send_open_loop_pwm(SUSTAINED_PWM, SUSTAINED_PWM, SUSTAINED_PWM, SUSTAINED_PWM)

                        cycle_elapsed = time.monotonic() - t_loop_start
                        sleep_time = max(0.005, 0.050 - cycle_elapsed)
                        time.sleep(sleep_time)

                if not self.abort_reason:
                    print("\nProfile completed cleanly. Initiating controlled zero cutoff...")

        except Exception as e:
            if not self.abort_reason:
                self.abort_reason = f"EXCEPTION: {e}"
            print(f"\n[EXECUTION ERROR] {self.abort_reason}", file=sys.stderr)
        finally:
            cleanup_ok = self._execute_cleanup()
            report = self._finalize_report(aborted=(self.abort_reason is not None), cleanup_ok=cleanup_ok)
            if not cleanup_ok:
                self.cleanup_failed = True
                print("\n[FATAL] Unconfirmed cleanup! Drivetrain state unconfirmed.", file=sys.stderr)

        return report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Rover 1 Focused Reverse Sustained Crawl Test (-40 PWM for 1.5s with -64 Dynamic Breakout)")
    parser.add_argument("--dry-run", action="store_true", help="Simulate execution without physical arming or motion")
    parser.add_argument("--host", default="127.0.0.1", help="Cockpit server host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=3000, help="Cockpit server port (default: 3000)")
    parser.add_argument("--report-dir", default="reports", help="Directory for JSON reports")
    parser.add_argument("--skip-firmware-check", action="store_true", help="Skip SSH safety firmware check (for local testing)")
    args = parser.parse_args(argv)

    harness = FocusedReverse40Harness(host=args.host, port=args.port, report_dir=args.report_dir)

    if not args.dry_run:
        try:
            harness.run_preflight_checks()
        except Exception as e:
            print(f"[FATAL] Preflight check failed: {e}", file=sys.stderr)
            return 1

        print("\n" + "="*60)
        print("OPERATOR CONFIRMATION REQUIRED BEFORE PHYSICAL MOTION")
        print("Test Profile: -64 PWM dynamic breakout (max 350 ms), then hold -40 PWM for 1500 ms.")
        print(f"Safety limits: {MAX_ABS_WHEEL_TRAVEL_M*1000:.0f} mm accumulated travel, {MAX_MEAN_ABS_SPEED_MPS} m/s speed, {MAX_ACCUMULATED_YAW_DEG} deg yaw drift.")
        print(f"Nominal duration: ~1.70 - 1.85 s | Hard safety cutoff: {MAX_TOTAL_DURATION_S} s.")
        print("="*60)
        confirm = input("Type 'CONFIRM' to execute Focused Reverse Test: ").strip()
        if confirm != "CONFIRM":
            print("Execution cancelled by operator. No motion commanded.")
            return 0

    try:
        report = harness.execute_trial(dry_run=args.dry_run, skip_firmware_check=args.skip_firmware_check)
    except Exception as e:
        print(f"[FATAL] Trial execution error: {e}", file=sys.stderr)
        return 1

    overall_status = report.get("overall_status", "UNKNOWN") if report else "UNKNOWN"
    if overall_status == "CLEANUP_FAILED":
        return 2
    elif overall_status in ("ABORTED", "UNKNOWN"):
        return 1
    elif overall_status in ("COMPLETED", "DRY_RUN_SUCCESS"):
        return 0
    else:
        return 1


if __name__ == "__main__":
    sys.exit(main())
