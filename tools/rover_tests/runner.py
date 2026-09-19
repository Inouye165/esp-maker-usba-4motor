"""
tools.rover_tests.runner - Multi-Trial Physical Test Execution Engine

Governs the complete execution lifecycle for Rover One tests:
- Obtains fresh IMU and odometry samples before each trial.
- Verifies non-magnetic BNO08x Game Rotation Vector orientation report (fails closed).
- Completes the 3-consecutive-zero autonomy handshake.
- Enforces explicit operator confirmation (inter-trial approval) and Ron physical angle estimate ingestion.
- Commands symmetric opposite wheel directions for rotate-in-place turns.
- Continuously logs all four wheels, gyro angles, creep milestone, zero command, post-zero coast, and settle.
- Disarms and locks drivetrain on normal completion, timeout, and all abort paths.
"""

import sys
import time
import math
import statistics
import traceback
from typing import Optional, Dict, Any, List, Callable, Tuple, Union

if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

from .turn import TurnParameters, compute_wheel_speed_targets, verify_wheel_command_symmetry
from .linear import (
    LinearParameters,
    LinearConfigurationException,
    compute_linear_wheel_speed_targets,
    verify_linear_wheel_command_symmetry,
    ticks_to_meters,
    meters_to_ticks
)
from .transport import (
    NativeWSClient,
    CockpitClient,
    perform_zero_handshake,
    complete_zero_handshake,
    arm_and_verify_ready_armed,
    disarm_and_stop,
    TransportException,
    HandshakeException
)
from .sensors import (
    quat_to_yaw,
    normalize_angle_deg,
    YawUnwrapper,
    BiasCorrectedGyroIntegrator,
    verify_non_magnetic_imu,
    check_imu_freshness,
    wait_for_advancing_imu_sample,
    check_encoder_freshness,
    wait_for_advancing_encoder_sample,
    extract_encoder_ticks,
    MagneticImuException,
    SensorException
)
from .controllers import AngularApproachController, LinearApproachController, ApproachPhase
from .reporting import (
    TrialReport,
    WheelTrialMetrics,
    MultiTrialSuiteReport,
    ReportGenerator,
    compute_phase_balancing_metrics
)


class TestAbortException(Exception):
    """Raised when a safety guard or watchdog aborts a trial."""
    pass


def ingest_physical_estimate(
    prompt_fn: Callable[[str], str],
    target_degrees: float,
    direction: str,
    final_settled_yaw: Optional[float]
) -> Tuple[Optional[float], Optional[float]]:
    """
    Ingests the operator's physical ground-truth angle estimate.
    
    Rules:
    - Operator can enter an unsigned magnitude (e.g. 90 or 89.8 for either direction).
    - The program automatically applies the CW/CCW sign:
        * 'cw'  -> positive (+abs(val))
        * 'ccw' -> negative (-abs(val))
    - An obvious typo (such as 9.05 for a 90° maneuver, where ratio < 0.5 or > 2.0)
      requires explicit confirmation before proceeding.
    - Empty input skips collection.
    
    Returns:
        (signed_estimate_deg, estimate_vs_gyro_delta_deg)
    """
    target_mag = abs(target_degrees)
    dir_norm = direction.lower()

    while True:
        est_input = prompt_fn("Enter Ron's physical ground-truth angle estimate in degrees (or Enter to skip): ").strip()
        if not est_input:
            return None, None

        try:
            val = float(est_input)
        except ValueError:
            print("  Invalid numeric input for physical estimate. Please enter a number or press Enter to skip.")
            continue

        mag = abs(val)
        signed_val = mag if dir_norm == "cw" else -mag

        # Check for obvious typo: e.g. 9.05 for a 90° maneuver
        is_typo = False
        if target_mag > 0.0:
            ratio = mag / target_mag
            if ratio < 0.5 or ratio > 2.0:
                is_typo = True

        if is_typo:
            confirm = prompt_fn(
                f"  Estimate {mag:.2f}° differs significantly from requested {target_mag:.1f}° maneuver (typo?). Confirm this value? [y/N]: "
            ).strip().lower()
            if confirm not in ("y", "yes"):
                print("  Re-enter physical estimate:")
                continue

        delta = round(final_settled_yaw - signed_val, 4) if final_settled_yaw is not None else None
        print(f"  Recorded Ron's physical estimate: {signed_val:+.2f}°" + (f" (Delta vs Gyro: {delta:+.2f}°)" if delta is not None else ""))
        return signed_val, delta


class PhysicalTestRunner:
    """
    Orchestrates physical test execution, safety monitoring, and report compilation.
    """
    def __init__(
        self,
        params: Union[TurnParameters, LinearParameters],
        prompt_fn: Optional[Callable[[str], str]] = None,
        cockpit_client: Optional[CockpitClient] = None,
        ws_client: Optional[NativeWSClient] = None
    ):
        params.validate()
        self.params = params
        self.prompt_fn = prompt_fn or input
        self.cockpit = cockpit_client or CockpitClient(host=params.host, port=params.port)
        self.ws = ws_client or NativeWSClient(host=params.host, port=params.port)
        self._cleaned_up = False
        self._last_cleanup_state: Dict[str, Any] = {"armed": None, "autonomyState": None, "cmdSource": None}

    def cleanup(self, verbose: bool = True) -> Dict[str, Any]:
        """
        Idempotent cleanup for test runner.
        - If cleanup was previously confirmed (self._cleaned_up is True), safely reverifies
          the final state against the live endpoint. If still confirmed safe (armed=False,
          autonomyState=DISABLED, cmdSource in ("NONE", None)), returns the verified state.
        - If reverification reveals an unconfirmed/unsafe state or query fails, clears
          self._cleaned_up and actively executes the disarm and stop routine.
        - Does NOT mark self._cleaned_up = True unless confirmed:
          armed=False, autonomyState=DISABLED, cmdSource=NONE.
        - Does not suppress a retry after a failed or unverified cleanup.
        """
        if self.params.dry_run:
            self._cleaned_up = True
            cleanup_st = {"armed": False, "autonomyState": "DISABLED", "cmdSource": "NONE"}
            self._last_cleanup_state = cleanup_st
            if verbose:
                print(f"[CLEANUP VERIFIED] Final safety state: armed=False autonomyState=DISABLED cmdSource=NONE")
            return cleanup_st

        if self._cleaned_up:
            try:
                st = self.cockpit.get_status()
                is_safe = (
                    st.get("armed") is False
                    and st.get("autonomyState") == "DISABLED"
                    and st.get("cmdSource") in ("NONE", None)
                )
                if is_safe:
                    self._last_cleanup_state = {
                        "armed": False,
                        "autonomyState": "DISABLED",
                        "cmdSource": st.get("cmdSource") or "NONE"
                    }
                    if verbose:
                        print(f"[CLEANUP REVERIFIED] Safety state confirmed: armed={self._last_cleanup_state['armed']} autonomyState={self._last_cleanup_state['autonomyState']} cmdSource={self._last_cleanup_state['cmdSource']}")
                    return self._last_cleanup_state
                else:
                    self._cleaned_up = False
            except Exception:
                self._cleaned_up = False

        cleanup_st = disarm_and_stop(self.cockpit, self.ws, verbose=verbose)
        self._last_cleanup_state = cleanup_st

        # Disarm, zero, and disable autonomy only (do not touch persistent drive configuration)

        confirmed = (
            cleanup_st.get("armed") is False
            and cleanup_st.get("autonomyState") == "DISABLED"
            and cleanup_st.get("cmdSource") in ("NONE", None)
        )
        self._cleaned_up = confirmed
        return cleanup_st

    def execute_suite(self) -> MultiTrialSuiteReport:
        """Executes the full suite of trials (dispatches to turn or linear)."""
        if isinstance(self.params, LinearParameters):
            return self.execute_linear_suite()
        return self.execute_turn_suite()

    def execute_turn_suite(self) -> MultiTrialSuiteReport:
        """Executes the full suite of turn trials."""
        suite_id = str(int(time.time()))
        cmd_parts = [
            f"rover-test turn --degrees {self.params.degrees} --direction {self.params.direction} --trials {self.params.trials}"
        ]
        if self.params.enable_balancing:
            cmd_parts.append("--enable-balancing")
        if self.params.enable_braking:
            cmd_parts.append("--enable-braking")
        if self.params.stopping_advance_deg is not None:
            cmd_parts.append(f"--stopping-advance-deg {self.params.stopping_advance_deg}")
        cmd_str = " ".join(cmd_parts)

        # Query live config so unspecified parameters inherit production settings
        current_cfg = {}
        if not self.params.dry_run:
            try:
                cfg_resp = self.cockpit.get_drive_config()
                if cfg_resp.get("ok"):
                    current_cfg = cfg_resp.get("config", {})
            except Exception:
                pass
        else:
            current_cfg = {"wheelBalancing": False, "dynamicBraking": True, "brakeDurationMs": 100, "maxTriggerSpeed": 0.35, "maxTriggerSpeedMps": 0.35}

        effective_bal = bool(self.params.enable_balancing) if self.params.enable_balancing is not None else bool(current_cfg.get("wheelBalancing", False))
        effective_brk = bool(self.params.enable_braking) if self.params.enable_braking is not None else bool(current_cfg.get("dynamicBraking", True))
        effective_advance = self.params.stopping_advance_deg if self.params.stopping_advance_deg is not None else (0.7 if effective_brk else 0.0)
        brake_dur = int(current_cfg.get("brakeDurationMs", 100))
        brake_max_spd = float(current_cfg.get("maxTriggerSpeed", 0.35))
        brake_max_spd_mps = float(current_cfg.get("maxTriggerSpeedMps", brake_max_spd))

        suite_report = MultiTrialSuiteReport(
            suite_id=suite_id,
            test_type="turn",
            command_line=cmd_str,
            timestamp_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            target_degrees=self.params.degrees,
            direction=self.params.direction,
            total_trials=self.params.trials,
            repetitions=self.params.trials,
            is_dry_run=self.params.dry_run,
            wheel_balancing_enabled=effective_bal,
            dynamic_braking_enabled=effective_brk,
            dynamic_brake_duration_ms=brake_dur,
            dynamic_brake_max_speed=brake_max_spd,
            dynamic_brake_max_speed_mps=brake_max_spd_mps,
            stopping_advance_deg=effective_advance
        )

        print("=" * 80)
        print(f"ROVER ONE REUSABLE PHYSICAL TEST FRAMEWORK: TURN {self.params.degrees:.1f}° {self.params.direction.upper()}")
        print(f"Repetitions: {self.params.trials} | Max Speed: {self.params.max_angular_speed:.2f} rad/s | Creep: {self.params.creep_angular_speed:.2f} rad/s")
        print(f"Balancing: {'ENABLED' if effective_bal else 'DISABLED'} | Braking: {'ENABLED' if effective_brk else 'DISABLED'} | Stopping Advance: {effective_advance:.2f}°")
        print(f"Mode: {'DRY RUN (STATIONARY - NO MOTOR POWER)' if self.params.dry_run else 'PHYSICAL MOTION EXECUTION'}")
        print("=" * 80)

        # Connect WebSocket unless in pure offline mock mode
        if not self.params.dry_run:
            if not self.ws.connect():
                print(f"[TRANSPORT ERROR] Failed to connect to Cockpit WebSocket on {self.params.host}:{self.params.port}")
                raise TransportException("Cannot connect to Cockpit WebSocket")
            if not self.ws.authenticate(self.cockpit.token):
                print("[AUTH ERROR] Failed to authenticate with Cockpit using operator token")
                raise TransportException("Operator authentication rejected")
            print("[OK] Connected and authenticated with Cockpit WebSocket.")

        cumulative_continuous_heading: Optional[float] = None
        total_cmd_deg = 0.0
        total_meas_deg = 0.0

        if not self.params.dry_run:
            try:
                init_imu = self.cockpit.get_imu()
                if init_imu and init_imu.get("ok", False):
                    suite_start_raw = math.degrees(quat_to_yaw(init_imu.get("orientation", {})))
                    suite_report.suite_start_raw_heading_deg = round(suite_start_raw, 4)
                    suite_report.suite_start_continuous_heading_deg = round(suite_start_raw, 4)
                    cumulative_continuous_heading = suite_start_raw
            except Exception:
                pass
        else:
            suite_report.suite_start_raw_heading_deg = 0.0
            suite_report.suite_start_continuous_heading_deg = 0.0
            cumulative_continuous_heading = 0.0

        try:
            for trial_idx in range(1, self.params.trials + 1):
                trial_report = self.execute_single_trial(trial_idx, cumulative_continuous_heading=cumulative_continuous_heading)
                suite_report.trials.append(trial_report)
                if trial_report.status in ("SUCCESS", "DRY_RUN_PASSED"):
                    suite_report.successful_trials += 1
                else:
                    suite_report.aborted_trials += 1

                if suite_report.suite_start_raw_heading_deg is None and trial_report.start_heading_raw_deg is not None:
                    suite_report.suite_start_raw_heading_deg = trial_report.start_heading_raw_deg
                    suite_report.suite_start_continuous_heading_deg = trial_report.start_heading_continuous_deg

                if trial_report.final_settled_raw_heading_deg is not None:
                    suite_report.suite_end_raw_heading_deg = trial_report.final_settled_raw_heading_deg
                    suite_report.suite_end_continuous_heading_deg = trial_report.final_settled_continuous_heading_deg

                step_target = self.params.signed_target_deg
                step_meas = trial_report.final_settled_gyro_angle_deg
                if step_meas is not None:
                    total_cmd_deg += step_target
                    total_meas_deg += step_meas
                    if cumulative_continuous_heading is not None:
                        cumulative_continuous_heading += step_meas

                suite_report.repetition_trajectory.append({
                    "repetition": trial_idx,
                    "start_raw_heading_deg": trial_report.start_heading_raw_deg,
                    "end_raw_heading_deg": trial_report.final_settled_raw_heading_deg,
                    "start_continuous_deg": trial_report.start_heading_continuous_deg,
                    "end_continuous_deg": trial_report.final_settled_continuous_heading_deg,
                    "target_deg": step_target,
                    "measured_deg": step_meas,
                    "error_deg": trial_report.settled_heading_error_deg,
                    "post_zero_coast_deg": trial_report.post_zero_rotation_deg,
                    "status": trial_report.status
                })

                # If aborted and not dry run, do not automatically retry without operator intervention
                if trial_report.status == "ABORTED" and not self.params.dry_run:
                    print(f"\n[SAFETY STOP] Step/Trial {trial_idx} aborted. Routine halted for forensic preservation.")
                    break
        finally:
            final_cleanup = self.cleanup(verbose=False)
            if self.ws.connected:
                self.ws.close()
            final_disarmed = (
                final_cleanup.get("armed") is False
                and final_cleanup.get("autonomyState") == "DISABLED"
                and final_cleanup.get("cmdSource") in ("NONE", None)
            )
            for t in suite_report.trials:
                if not final_disarmed:
                    t.confirmed_final_disarmed_state = False

        suite_report.total_cumulative_commanded_deg = round(total_cmd_deg, 4)
        suite_report.total_cumulative_measured_deg = round(total_meas_deg, 4)
        suite_report.total_cumulative_error_deg = round(total_meas_deg - total_cmd_deg, 4)

        # Compute suite statistics across successful trials with valid final measurements
        successful_trials = [t for t in suite_report.trials if t.status in ("SUCCESS", "DRY_RUN_PASSED")]
        valid_error_trials = [t for t in successful_trials if t.final_measurement_valid and t.settled_heading_error_deg is not None]
        if valid_error_trials:
            errors = [t.settled_heading_error_deg for t in valid_error_trials]
            suite_report.mean_settled_error_deg = round(statistics.mean(errors), 4)
            suite_report.std_dev_settled_error_deg = round(statistics.stdev(errors), 4) if len(errors) > 1 else 0.0
            suite_report.repeatability_deg = round(max(errors) - min(errors), 4)
        else:
            suite_report.mean_settled_error_deg = None
            suite_report.std_dev_settled_error_deg = None
            suite_report.repeatability_deg = None

        estimate_deltas = [t.estimate_vs_gyro_delta_deg for t in successful_trials if t.estimate_vs_gyro_delta_deg is not None]
        if estimate_deltas:
            suite_report.mean_estimate_delta_deg = round(statistics.mean(estimate_deltas), 4)

        # Console totals summary for multi-repetition runs
        if len(suite_report.repetition_trajectory) > 1:
            print("\n" + "=" * 80)
            print(f"MULTI-REPETITION TOTALS SUMMARY ({len(suite_report.repetition_trajectory)} STEPS)")
            print("=" * 80)
            st_s = f"{suite_report.suite_start_raw_heading_deg:+.2f}°" if suite_report.suite_start_raw_heading_deg is not None else "N/A"
            end_s = f"{suite_report.suite_end_raw_heading_deg:+.2f}°" if suite_report.suite_end_raw_heading_deg is not None else "N/A"
            print(f"  • Initial Heading (Step 1 Start):  {st_s}")
            print(f"  • Final Settled (Step {len(suite_report.repetition_trajectory)} End):   {end_s}")
            print(f"  • Cumulative Commanded Rotation:   {suite_report.total_cumulative_commanded_deg:+.2f}°")
            print(f"  • Cumulative Measured Rotation:    {suite_report.total_cumulative_measured_deg:+.2f}°")
            print(f"  • Total Net Trajectory Error:      {suite_report.total_cumulative_error_deg:+.2f}°")
            print("=" * 80)

        # Generate output files
        json_path = ReportGenerator.save_json(suite_report, self.params.report_directory)
        md_summary = ReportGenerator.format_markdown_summary(suite_report)
        print("\n" + md_summary)
        print(f"\n[REPORT SAVED] Full JSON telemetry report: {json_path}")
        return suite_report

    def execute_linear_suite(self) -> MultiTrialSuiteReport:
        """Executes the full suite of linear trials."""
        suite_id = str(int(time.time()))
        cmd_parts = [
            f"rover-test linear --distance {self.params.distance} --direction {self.params.direction} --trials {self.params.trials}"
        ]
        if self.params.enable_balancing:
            cmd_parts.append("--enable-balancing")
        if self.params.enable_braking:
            cmd_parts.append("--enable-braking")
        cmd_str = " ".join(cmd_parts)

        suite_report = MultiTrialSuiteReport(
            suite_id=suite_id,
            test_type="linear",
            command_line=cmd_str,
            timestamp_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            target_distance_m=self.params.distance,
            target_linear_speed_mps=self.params.max_linear_speed,
            direction=self.params.direction,
            total_trials=self.params.trials,
            repetitions=self.params.trials,
            is_dry_run=self.params.dry_run,
            wheel_balancing_enabled=bool(self.params.enable_balancing),
            dynamic_braking_enabled=bool(self.params.enable_braking) if self.params.enable_braking is not None else True
        )

        effective_brk = self.params.enable_braking if self.params.enable_braking is not None else True
        print("=" * 80)
        print(f"ROVER ONE REUSABLE PHYSICAL TEST FRAMEWORK: LINEAR {self.params.distance:.3f}m {self.params.direction.upper()}")
        print(f"Repetitions: {self.params.trials} | Requested Velocity: {self.params.max_linear_speed:.2f} m/s (Constant Production Speed)")
        print(f"Balancing: {'ENABLED' if self.params.enable_balancing else 'DISABLED'} | Braking: {'ENABLED' if effective_brk else 'DISABLED'}")
        print(f"Mode: {'DRY RUN (STATIONARY - NO MOTOR POWER)' if self.params.dry_run else 'PHYSICAL MOTION EXECUTION'}")
        print("=" * 80)

        # Connect WebSocket unless in pure offline mock mode
        if not self.params.dry_run:
            if not self.ws.connect():
                print(f"[TRANSPORT ERROR] Failed to connect to Cockpit WebSocket on {self.params.host}:{self.params.port}")
                raise TransportException("Cannot connect to Cockpit WebSocket")
            if not self.ws.authenticate(self.cockpit.token):
                print("[AUTH ERROR] Failed to authenticate with Cockpit using operator token")
                raise TransportException("Operator authentication rejected")
            print("[OK] Connected and authenticated with Cockpit WebSocket.")

        try:
            for trial_idx in range(1, self.params.trials + 1):
                trial_report = self.execute_single_linear_trial(trial_idx)
                suite_report.trials.append(trial_report)
                if trial_report.status in ("SUCCESS", "DRY_RUN_PASSED"):
                    suite_report.successful_trials += 1
                else:
                    suite_report.aborted_trials += 1
                    print(f"\n[SUITE ABORTED] Trial {trial_idx} aborted: {trial_report.abort_reason}")
                    break

            # Propagate confirmed live configuration from executed trials
            if suite_report.trials:
                first_t = suite_report.trials[0]
                suite_report.wheel_balancing_enabled = first_t.wheel_balancing_enabled
                suite_report.dynamic_braking_enabled = first_t.dynamic_braking_enabled
                suite_report.dynamic_brake_duration_ms = first_t.dynamic_brake_duration_ms
                suite_report.dynamic_brake_max_speed = first_t.dynamic_brake_max_speed
                suite_report.anti_stall_confirmed = first_t.anti_stall_confirmed

            # Aggregate linear metrics across successful trials
            successful_trials = [t for t in suite_report.trials if t.status in ("SUCCESS", "DRY_RUN_PASSED")]
            errors = [t.settled_distance_error_m for t in successful_trials if t.settled_distance_error_m is not None]
            if errors:
                suite_report.mean_settled_distance_error_m = round(statistics.mean(errors), 5)
                suite_report.std_dev_settled_distance_error_m = round(statistics.stdev(errors), 5) if len(errors) > 1 else 0.0
                suite_report.repeatability_distance_m = round(max(errors) - min(errors), 5) if len(errors) > 1 else 0.0

            hdg_changes = [t.imu_heading_change_deg for t in successful_trials if t.imu_heading_change_deg is not None]
            if hdg_changes:
                suite_report.mean_heading_change_deg = round(statistics.mean(hdg_changes), 3)

            cum_cmd = 0.0
            cum_meas = 0.0
            for t in suite_report.trials:
                step_cmd = t.requested_distance_m if t.requested_distance_m is not None else self.params.distance
                step_meas = t.measured_distance_m if t.measured_distance_m is not None else 0.0
                step_err = t.settled_distance_error_m if t.settled_distance_error_m is not None else 0.0
                step_coast = t.post_zero_coast_m if t.post_zero_coast_m is not None else 0.0
                step_hdg = t.imu_heading_change_deg if t.imu_heading_change_deg is not None else 0.0
                cum_cmd += step_cmd
                cum_meas += step_meas
                suite_report.repetition_trajectory.append({
                    "repetition": t.trial_index,
                    "target_m": round(step_cmd, 4),
                    "measured_m": round(step_meas, 4),
                    "error_m": round(step_err, 4),
                    "post_zero_coast_m": round(step_coast, 4),
                    "heading_change_deg": round(step_hdg, 2),
                    "status": t.status
                })

            suite_report.total_cumulative_commanded_m = round(cum_cmd, 4)
            suite_report.total_cumulative_measured_m = round(cum_meas, 4)
            suite_report.total_cumulative_error_m = round(cum_meas - cum_cmd, 4)

            # Generate output files
            json_path = ReportGenerator.save_json(suite_report, self.params.report_directory)
            md_summary = ReportGenerator.format_markdown_summary(suite_report)
            print("\n" + md_summary)
            print(f"\n[REPORT SAVED] Full JSON telemetry report: {json_path}")
            return suite_report
        finally:
            self.cleanup()

    def execute_single_linear_trial(self, trial_idx: int) -> TrialReport:
        """Executes a single forward or reverse linear translation trial."""
        trial_report = TrialReport(
            trial_index=trial_idx,
            test_type="linear",
            direction=self.params.direction,
            requested_distance_m=self.params.distance,
            target_signed_distance_m=self.params.signed_target_m,
            status="INITIALIZING"
        )

        t_trial_start = time.time()
        wheel_metrics = {w_id: WheelTrialMetrics(wheel_id=w_id) for w_id in ["m1", "m2", "m3", "m4"]}
        trial_report.wheel_metrics = wheel_metrics

        # Step 0: Read back & confirm live production controller configuration BEFORE motion
        live_cfg = {}
        anti_stall_confirmed = False
        if not self.params.dry_run:
            # If user explicitly requested drive parameter overrides via CLI flags, apply them
            if self.params.enable_balancing is not None or self.params.enable_braking is not None:
                # Query current live config first so unspecified flags remain untouched
                current_cfg = {}
                try:
                    cfg_pre = self.cockpit.get_drive_config()
                    if cfg_pre.get("ok"):
                        current_cfg = cfg_pre.get("config", {})
                except Exception:
                    pass

                target_bal = bool(self.params.enable_balancing) if self.params.enable_balancing is not None else current_cfg.get("wheelBalancing", False)
                target_brk = bool(self.params.enable_braking) if self.params.enable_braking is not None else current_cfg.get("dynamicBraking", True)
                try:
                    self.cockpit.configure_drive(
                        wheel_balancing=target_bal,
                        dynamic_braking=target_brk
                    )
                    time.sleep(0.05)
                except Exception as e:
                    print(f"[WARN] Failed to configure drive parameters: {e}")

            # Read back actual live drive config from cockpit server & ESP32
            try:
                cfg_resp = self.cockpit.get_drive_config()
                if cfg_resp.get("ok"):
                    live_cfg = cfg_resp.get("config", {})
            except Exception as e:
                print(f"[WARN] Failed to read live drive configuration: {e}")

            # Query live PID telemetry to confirm anti-stall / stiction state machine
            try:
                pid_resp = self.cockpit.get_pid_telemetry()
                if pid_resp.get("ok") and pid_resp.get("telemetry"):
                    telem = pid_resp.get("telemetry", {})
                    if "m1" in telem and "stictionState" in telem["m1"]:
                        anti_stall_confirmed = True
            except Exception as e:
                print(f"[WARN] Failed to read live anti-stall telemetry: {e}")
        else:
            # Stationary dry run: reflect parameters / mock state
            live_cfg = {
                "wheelBalancing": bool(self.params.enable_balancing) if self.params.enable_balancing is not None else False,
                "dynamicBraking": bool(self.params.enable_braking) if self.params.enable_braking is not None else True,
                "brakeDurationMs": 100,
                "maxTriggerSpeedMps": 0.35,
                "maxTriggerSpeed": 0.35,
            }
            anti_stall_confirmed = True

        confirmed_bal = bool(live_cfg.get("wheelBalancing", False))
        confirmed_brk = bool(live_cfg.get("dynamicBraking", True))
        confirmed_dur = int(live_cfg.get("brakeDurationMs", 100))
        confirmed_max_spd = float(live_cfg.get("maxTriggerSpeedMps", live_cfg.get("maxTriggerSpeed", 0.35)))

        trial_report.wheel_balancing_enabled = confirmed_bal
        trial_report.dynamic_braking_enabled = confirmed_brk
        trial_report.dynamic_brake_duration_ms = confirmed_dur
        trial_report.dynamic_brake_max_speed = confirmed_max_spd
        trial_report.dynamic_brake_max_speed_mps = confirmed_max_spd
        trial_report.anti_stall_confirmed = anti_stall_confirmed

        # Step 1: Inter-Trial Approval
        if self.params.inter_trial_approval or (trial_idx == 1 and not self.params.dry_run):
            print("\n" + "-" * 60)
            print(f"OPERATOR INTER-TRIAL APPROVAL REQUIRED FOR TRIAL {trial_idx}/{self.params.trials}")
            print(f"  • Distance:            {self.params.distance:.3f} m ({self.params.direction.upper()})")
            print(f"  • Requested Speed:     {self.params.max_linear_speed:.2f} m/s (Constant Production Speed)")
            print(f"  • Confirmed Balancing: {'ENABLED' if confirmed_bal else 'DISABLED'} (Live controller readback)")
            print(f"  • Confirmed Braking:   {'ENABLED' if confirmed_brk else 'DISABLED'} ({confirmed_dur}ms pulse, max {confirmed_max_spd:.2f} rad/s)")
            print(f"  • Confirmed Anti-Stall: {'ACTIVE' if anti_stall_confirmed else 'UNVERIFIED'} (Live PID telemetry)")
            print(f"  • Execution Mode:      {'DRY RUN (STATIONARY)' if self.params.dry_run else 'PHYSICAL MOTION'}")
            print("-" * 60)
            ans = self.prompt_fn("Authorize motion for this trial? [y/N]: ").strip().lower()
            if ans not in ("y", "yes"):
                trial_report.status = "ABORTED"
                trial_report.abort_reason = "Operator declined authorization"
                return trial_report

        # Step 2: Clear faults if requested
        if self.params.clear_faults and not self.params.dry_run:
            self.cockpit.clear_faults()
            time.sleep(0.1)

        # Step 3: Fresh sensor baselines & pre-arm encoder validation
        start_yaw_deg = 0.0
        start_ticks = {w: 0 for w in ["m1", "m2", "m3", "m4"]}
        if not self.params.dry_run:
            try:
                init_imu = self.cockpit.get_imu()
                is_non_mag, imu_msg = verify_non_magnetic_imu(init_imu)
                if not is_non_mag:
                    trial_report.status = "ABORTED"
                    trial_report.abort_reason = f"IMU Safety Failure: {imu_msg}"
                    return trial_report
                start_yaw_deg = math.degrees(quat_to_yaw(init_imu.get("orientation", {})))
                trial_report.start_heading_raw_deg = round(start_yaw_deg, 4)

                # Require fresh, advancing encoder telemetry before arming
                enc_ok, enc_res, enc_err = wait_for_advancing_encoder_sample(
                    self.cockpit,
                    max_wait_sec=0.50,
                    max_acceptable_age_ms=150.0,
                    watchdog_max_age_ms=250.0
                )
                if not enc_ok or not enc_res:
                    trial_report.status = "ABORTED"
                    trial_report.abort_reason = f"Pre-arm encoder telemetry verification failed: {enc_err}"
                    print(f"[FAIL-CLOSED ERROR] {trial_report.abort_reason}")
                    cleanup_st = self.cleanup()
                    trial_report.confirmed_final_zero_command = True
                    trial_report.confirmed_final_disarmed_state = cleanup_st.get("armed") is False
                    return trial_report

                extracted_start = extract_encoder_ticks(enc_res)
                if extracted_start is None:
                    trial_report.status = "ABORTED"
                    trial_report.abort_reason = "Pre-arm encoder telemetry malformed (cannot extract m1..m4 ticks)"
                    print(f"[FAIL-CLOSED ERROR] {trial_report.abort_reason}")
                    cleanup_st = self.cleanup()
                    trial_report.confirmed_final_zero_command = True
                    trial_report.confirmed_final_disarmed_state = cleanup_st.get("armed") is False
                    return trial_report

                start_ticks = extracted_start
                for w in ["m1", "m2", "m3", "m4"]:
                    wheel_metrics[w].encoder_start_ticks = start_ticks[w]
            except Exception as e:
                trial_report.status = "ABORTED"
                trial_report.abort_reason = f"Preflight sensor verification fault: {e}"
                cleanup_st = self.cleanup()
                trial_report.confirmed_final_zero_command = True
                trial_report.confirmed_final_disarmed_state = cleanup_st.get("armed") is False
                return trial_report
        else:
            trial_report.start_heading_raw_deg = 0.0

        # Step 5: Zero Handshake
        if not self.params.dry_run:
            try:
                complete_zero_handshake(self.cockpit, self.ws, transitions=trial_report.autonomy_state_transitions)
            except HandshakeException as e:
                trial_report.status = "ABORTED"
                trial_report.abort_reason = f"Handshake failure: {e}"
                cleanup_st = self.cleanup()
                trial_report.confirmed_final_zero_command = True
                trial_report.confirmed_final_disarmed_state = cleanup_st.get("armed") is False
                return trial_report
        else:
            trial_report.autonomy_state_transitions.append(
                {"timestamp": time.time(), "state": "READY_DISARMED", "trigger": "zero_handshake (dry run)"}
            )

        # Step 6: Arm Drivetrain
        if not self.params.dry_run:
            try:
                arm_and_verify_ready_armed(self.cockpit, transitions=trial_report.autonomy_state_transitions)
            except HandshakeException as e:
                trial_report.status = "ABORTED"
                trial_report.abort_reason = f"Arming failure: {e}"
                cleanup_st = self.cleanup()
                trial_report.confirmed_final_zero_command = True
                trial_report.confirmed_final_disarmed_state = cleanup_st.get("armed") is False
                return trial_report
        else:
            trial_report.autonomy_state_transitions.append(
                {"timestamp": time.time(), "state": "READY_ARMED", "trigger": "arm_drive (dry run)"}
            )

        # Step 7: Pre-motion assertions
        if not self.params.dry_run:
            inv_err = self._assert_pre_motion_invariants()
            if inv_err:
                trial_report.status = "ABORTED"
                trial_report.abort_reason = inv_err
                cleanup_st = self.cleanup()
                trial_report.confirmed_final_zero_command = True
                trial_report.confirmed_final_disarmed_state = cleanup_st.get("armed") is False
                return trial_report

        # Step 8: Setup LinearApproachController
        controller = LinearApproachController(
            cruise_speed_mps=self.params.max_linear_speed,
            creep_speed_mps=self.params.creep_linear_speed,
            approach_zone_m=self.params.creep_threshold_m
        )
        controller.reset(target=self.params.signed_target_m, start_time=time.time())

        # Step 9: Execution Loop (Dry Run or Physical)
        wheel_samples = {
            w_id: {
                "targets": [],
                "measured": [],
                "pwms": [],
                "stopped_events": 0,
                "stopped_duration_s": 0.0,
                "is_currently_stopped": False,
                "stopped_entry_time": None,
            }
            for w_id in ["m1", "m2", "m3", "m4"]
        }
        total_telemetry_samples = 0
        active_pid_packets: List[Dict[str, Any]] = []

        try:
            if self.params.dry_run:
                sim_target = self.params.signed_target_m
                sim_target = self.params.signed_target_m
                sim_progress_stages = [
                    0.0,
                    sim_target * 0.10,
                    sim_target * 0.50,
                    sim_target * 0.85,
                    sim_target
                ]
                for sim_dist in sim_progress_stages:
                    cmd_vx = controller.update(current_progress=sim_dist, current_time=time.time())
                    wheel_cmds = compute_linear_wheel_speed_targets(cmd_vx)
                    sym_ok, sym_msg = verify_linear_wheel_command_symmetry(wheel_cmds)
                    if not sym_ok:
                        raise TestAbortException(f"Symmetry check failed: {sym_msg}")
                    if controller.phase == ApproachPhase.CRUISE:
                        for w_id in ["m1", "m2", "m3", "m4"]:
                            cmd_val = wheel_cmds[w_id]
                            wheel_samples[w_id]["targets"].append(cmd_val)
                            wheel_samples[w_id]["measured"].append(cmd_val)
                            wheel_samples[w_id]["pwms"].append(55)
                        total_telemetry_samples += 1
                    time.sleep(0.01)

                t_dry_mono = time.monotonic()
                trial_report.zero_command_send_time_monotonic = t_dry_mono
                trial_report.zero_command_response_time_monotonic = t_dry_mono
                trial_report.zero_command_latency_ms = 0.0
                trial_report.zero_command_response = {"ok": True, "dry_run": True}
                if self.params.enable_braking:
                    trial_report.actuation_states_observed = ["DRIVE", "BRAKE", "COAST"]
                    trial_report.brake_active_duration_ms = 100.0
                else:
                    trial_report.actuation_states_observed = ["DRIVE", "COAST"]
                    trial_report.brake_active_duration_ms = 0.0

                trial_report.status = "DRY_RUN_PASSED"
                trial_report.measured_distance_m = sim_target
                trial_report.final_settled_distance_m = sim_target
                trial_report.settled_distance_error_m = 0.0
                trial_report.post_zero_coast_m = 0.0
                trial_report.imu_heading_change_deg = 0.0
                trial_report.front_to_rear_diff_left_radps = 0.0
                trial_report.front_to_rear_diff_right_radps = 0.0

                # Simulated Phase Breakdown
                dir_sign = 1.0 if sim_target >= 0.0 else -1.0
                acc_dur = 0.20
                acc_dist = round(0.025 * dir_sign, 4)
                acc_spd = round(0.125 * dir_sign, 3)
                trial_report.acceleration_phase = {
                    "duration_s": acc_dur,
                    "distance_m": acc_dist,
                    "mean_speed_mps": acc_spd,
                    "peak_pwm": 65
                }

                std_dist = round((abs(sim_target) - 0.08) * dir_sign, 4)
                std_dur = round(abs(std_dist) / self.params.max_linear_speed, 2)
                std_spd = round(self.params.max_linear_speed * dir_sign, 3)
                trial_report.steady_speed_phase = {
                    "duration_s": std_dur,
                    "distance_m": std_dist,
                    "mean_speed_mps": std_spd,
                    "left_to_right_speed_diff_radps": 0.0,
                    "left_to_right_pwm_diff": 0.0
                }

                stp_dur = 0.35
                stp_dist = round(0.055 * dir_sign, 4)
                trial_report.stopping_phase = {
                    "duration_s": stp_dur,
                    "distance_m": stp_dist
                }
                trial_report.stopping_distance_m = stp_dist
                trial_report.stopping_duration_s = stp_dur
                trial_report.steady_speed_front_to_rear_left_radps = 0.0
                trial_report.steady_speed_front_to_rear_right_radps = 0.0
                trial_report.steady_speed_left_to_right_speed_diff_radps = 0.0
                trial_report.steady_speed_left_to_right_pwm_diff = 0.0

                delta_ticks_sim = int(meters_to_ticks(sim_target))
                for w_id in ["m1", "m2", "m3", "m4"]:
                    w_metric = wheel_metrics[w_id]
                    w_metric.encoder_delta_ticks = delta_ticks_sim
                    w_metric.encoder_final_ticks = w_metric.encoder_start_ticks + delta_ticks_sim
                    w_metric.stopped_while_commanded = False
                    w_metric.commanded_speed_source = "calculated_expected"
                    speed_sign = 1.0 if sim_target >= 0.0 else -1.0
                    w_metric.commanded_speed_radps_mean = round(speed_sign * (self.params.max_linear_speed / 0.033475), 2)
                    w_metric.measured_speed_radps_mean = w_metric.commanded_speed_radps_mean
                    w_metric.commanded_speed_radps_max = w_metric.commanded_speed_radps_mean
                    w_metric.measured_speed_radps_max = w_metric.commanded_speed_radps_mean
                    w_metric.measured_speed_radps_min = w_metric.commanded_speed_radps_mean
                    w_metric.measured_speed_radps_abs_mean = abs(w_metric.commanded_speed_radps_mean)
                    w_metric.pwm_mean = 55.0
                    w_metric.pwm_max = 65
                    w_metric.active_samples_count = len(sim_progress_stages)
                trial_report.steady_speed_wheel_metrics = dict(wheel_metrics)
            else:
                # Physical motion loop
                theoretical_sec = abs(self.params.distance) / self.params.max_linear_speed
                max_duration = max(3.0, min(15.0, round(theoretical_sec * 1.5 + 2.0, 2)))

                # Drain stale WebSocket frames before motion so telemetry strictly reflects active driving
                if self.ws and self.ws.connected:
                    self.ws.recv_frames()

                first_cmd_vx = controller.update(current_progress=0.0, current_time=time.time())
                first_res = self.cockpit.send_cmd_vel(vx=first_cmd_vx, wz=0.0)
                trial_report.first_command_response = first_res
                if not first_res.get("ok", False):
                    raise TestAbortException(f"First command rejected: {first_res.get('error')}")

                # Step 10: Bounded startup verification (max 800ms)
                # Verify autonomy ACTIVE, nonzero wheel targets confirmed, and distance begins advancing
                startup_active = False
                startup_has_targets = False
                startup_has_advancement = False
                t_chk = time.time()
                last_enc_ticks = dict(start_ticks)

                while time.time() - t_chk < 0.80:
                    a_st = self.cockpit.get_autonomy_status()
                    if a_st.get("state") == "ACTIVE" or abs(a_st.get("clampedLinear", 0.0)) > 1e-4:
                        startup_active = True

                    pid_st = self.cockpit.get_pid_telemetry()
                    if pid_st and pid_st.get("ok") and pid_st.get("telemetry"):
                        telem = pid_st["telemetry"]
                        m1_tgt = abs(float(telem.get("m1", {}).get("targetRadps", 0.0)))
                        if m1_tgt > 0.05 or abs(a_st.get("clampedLinear", 0.0)) > 1e-4:
                            startup_has_targets = True
                    elif startup_active:
                        startup_has_targets = True

                    enc_st = self.cockpit.get_encoders()
                    if enc_st:
                        ticks_now = extract_encoder_ticks(enc_st)
                        if ticks_now:
                            last_enc_ticks = ticks_now
                            tick_deltas = [abs(ticks_now[w] - start_ticks[w]) for w in ["m1", "m2", "m3", "m4"]]
                            if max(tick_deltas) >= 15:  # At least 15 ticks (~1.5 mm)
                                startup_has_advancement = True
                                break
                    time.sleep(0.03)

                if not startup_active:
                    self.cockpit.send_cmd_vel(vx=0.0, wz=0.0)
                    raise TestAbortException("Startup safety guard tripped: Autonomy ACTIVE state not achieved within 800ms")

                if not startup_has_targets:
                    self.cockpit.send_cmd_vel(vx=0.0, wz=0.0)
                    raise TestAbortException("Startup safety guard tripped: Non-zero wheel PID targets not confirmed within 800ms")

                if not startup_has_advancement:
                    self.cockpit.send_cmd_vel(vx=0.0, wz=0.0)
                    raise TestAbortException(
                        "Startup safety guard tripped: Commanded motion but encoder distance stream did not advance within 800ms (reported progress remained zero)"
                    )

                trial_report.autonomy_state_transitions.append({
                    "timestamp": time.time(),
                    "state": "ACTIVE",
                    "trigger": "first_nonzero_cmd_accepted_and_advancing"
                })

                t_start_motion = time.time()
                t_steady_start: Optional[float] = None
                t_zero_issued: Optional[float] = None
                steady_start_ticks: Optional[Dict[str, int]] = None
                steady_end_ticks: Optional[Dict[str, int]] = None
                dist_at_steady_start = 0.0
                dist_at_zero_issued = 0.0
                curr_dist_m = 0.0
                last_adv_dist = 0.0
                last_dist_advance_time = t_start_motion
                last_encoder_time = t_start_motion
                last_yaw_deg = start_yaw_deg
                breakout_start_time = None
                breakout_count = 0
                total_breakout_dwell_ms = 0.0
                cur_ticks = dict(last_enc_ticks)

                steady_wheel_samples: Dict[str, Dict[str, List[float]]] = {
                    w: {"targets": [], "measured": [], "pwms": []} for w in ["m1", "m2", "m3", "m4"]
                }
                accel_pwms: List[int] = []

                while True:
                    now = time.time()
                    if now - t_start_motion > max_duration:
                        self.cockpit.send_cmd_vel(vx=0.0, wz=0.0)
                        raise TestAbortException(
                            f"Linear trial exceeded bounded duration limit ({max_duration:.1f}s, theoretical: {theoretical_sec:.1f}s)"
                        )

                    # 1. Ingest telemetry from WebSocket
                    got_encoder_frame = False
                    if self.ws and self.ws.connected:
                        frames = self.ws.recv_frames()
                        for f in frames:
                            ftype = f.get("type")
                            if ftype in ("odom", "encoders", "encoder_status"):
                                extracted = extract_encoder_ticks(f)
                                if extracted:
                                    cur_ticks = extracted
                                    last_encoder_time = now
                                    got_encoder_frame = True
                                if "yaw" in f:
                                    last_yaw_deg = math.degrees(float(f["yaw"]))
                            elif ftype == "imu":
                                ori = f.get("orientation", {})
                                if ori:
                                    last_yaw_deg = math.degrees(quat_to_yaw(ori))
                            elif ftype == "pid_diagnostic":
                                total_telemetry_samples += 1
                                active_pid_packets.append(f)
                                for w_id in ["m1", "m2", "m3", "m4"]:
                                    w_f = f.get(w_id, {})
                                    tgt = w_f.get("targetRadps")
                                    meas = w_f.get("measuredRadps")
                                    pwm = w_f.get("finalPwm", w_f.get("basePwm"))
                                    stict = w_f.get("stictionState")
                                    if tgt is not None:
                                        wheel_samples[w_id]["targets"].append(float(tgt))
                                    if meas is not None:
                                        wheel_samples[w_id]["measured"].append(float(meas))
                                    if pwm is not None:
                                        pwm_int = int(pwm)
                                        wheel_samples[w_id]["pwms"].append(pwm_int)
                                        if t_steady_start is None:
                                            accel_pwms.append(pwm_int)
                                        else:
                                            steady_wheel_samples[w_id]["pwms"].append(pwm_int)
                                    if t_steady_start is not None and t_zero_issued is None:
                                        if tgt is not None:
                                            steady_wheel_samples[w_id]["targets"].append(float(tgt))
                                        if meas is not None:
                                            steady_wheel_samples[w_id]["measured"].append(float(meas))
                                    if stict == "STICTION_BOOST":
                                        wheel_metrics[w_id].stiction_boost_events += 1
                                    elif stict == "BLOCKED":
                                        wheel_metrics[w_id].blocked_state_events += 1

                    # 2. Authoritative HTTP polling fallback if no WebSocket encoder frame within 40ms
                    if not got_encoder_frame or (now - last_encoder_time > 0.040):
                        try:
                            enc_resp = self.cockpit.get_encoders(timeout=0.10)
                            is_fresh, fresh_msg = check_encoder_freshness(enc_resp, max_age_ms=250.0)
                            if not is_fresh:
                                self.cockpit.send_cmd_vel(vx=0.0, wz=0.0)
                                raise TestAbortException(f"Encoder telemetry stale or dropped during motion: {fresh_msg}")
                            extracted = extract_encoder_ticks(enc_resp)
                            if extracted:
                                cur_ticks = extracted
                                last_encoder_time = now
                        except Exception as e:
                            if isinstance(e, TestAbortException):
                                raise
                            self.cockpit.send_cmd_vel(vx=0.0, wz=0.0)
                            raise TestAbortException(f"Failed to query encoder telemetry during motion: {e}")

                    # 3. Compute current linear displacement from encoder ticks
                    deltas = [cur_ticks[w] - start_ticks[w] for w in ["m1", "m2", "m3", "m4"]]
                    avg_delta = sum(deltas) / 4.0
                    curr_dist_m = ticks_to_meters(avg_delta)

                    # 4. Check progress advancement (must advance at least 1 mm)
                    if abs(curr_dist_m - last_adv_dist) >= 0.001:
                        last_adv_dist = curr_dist_m
                        last_dist_advance_time = now

                    # 5. Stall watchdog: progress MUST advance while motion is commanded
                    if now - last_dist_advance_time > 0.60:
                        self.cockpit.send_cmd_vel(vx=0.0, wz=0.0)
                        raise TestAbortException(
                            f"Distance telemetry stalled while commanded: progress remained at {curr_dist_m:.4f}m for > 600ms"
                        )

                    # 6. Check steady-speed phase transition (>= 85% of target speed)
                    if t_steady_start is None and active_pid_packets:
                        recent_pkt = active_pid_packets[-1]
                        speeds = [abs(float(recent_pkt.get(w, {}).get("measuredRadps", 0.0))) for w in ["m1", "m2", "m3", "m4"]]
                        expected_radps = abs(self.params.max_linear_speed / 0.033475)
                        if speeds and (sum(speeds) / len(speeds)) >= 0.85 * expected_radps:
                            t_steady_start = now
                            dist_at_steady_start = curr_dist_m
                            steady_start_ticks = dict(cur_ticks)

                    # 7. Update approach controller
                    cmd_vx = controller.update(current_progress=curr_dist_m, current_time=now)
                    wheel_cmds = compute_linear_wheel_speed_targets(cmd_vx)
                    sym_ok, sym_msg = verify_linear_wheel_command_symmetry(wheel_cmds)
                    if not sym_ok:
                        self.cockpit.send_cmd_vel(vx=0.0, wz=0.0)
                        raise TestAbortException(f"Command symmetry fault: {sym_msg}")

                    if controller.phase == ApproachPhase.ZERO:
                        # Target reached: issue zero command immediately
                        t_zero_issued = now
                        dist_at_zero_issued = curr_dist_m
                        steady_end_ticks = dict(cur_ticks)
                        t_zero_start = time.monotonic()
                        z_res = self.cockpit.send_cmd_vel(vx=0.0, wz=0.0)
                        t_zero_end = time.monotonic()
                        trial_report.zero_command_send_time_monotonic = t_zero_start
                        trial_report.zero_command_response_time_monotonic = t_zero_end
                        trial_report.zero_command_latency_ms = (t_zero_end - t_zero_start) * 1000.0
                        trial_report.zero_command_response = z_res
                        break

                    self.cockpit.send_cmd_vel(vx=cmd_vx, wz=0.0)
                    time.sleep(0.02)

                if steady_end_ticks is None:
                    steady_end_ticks = dict(cur_ticks)

                # Standstill settling period
                time.sleep(self.params.settle_seconds)
                t_settled_time = time.time()
                final_enc = self.cockpit.get_encoders()
                final_imu = self.cockpit.get_imu()
                final_yaw = math.degrees(quat_to_yaw(final_imu.get("orientation", {}))) if final_imu else last_yaw_deg
                extracted_final = extract_encoder_ticks(final_enc) or cur_ticks
                settled_ticks = [extracted_final[w] - start_ticks[w] for w in ["m1", "m2", "m3", "m4"]]
                settled_dist = ticks_to_meters(sum(settled_ticks) / 4.0)

                controller.mark_settled(settled_dist, current_time=t_settled_time)
                trial_report.status = "SUCCESS"
                trial_report.measured_distance_m = round(settled_dist, 5)
                trial_report.final_settled_distance_m = round(settled_dist, 5)
                trial_report.settled_distance_error_m = round(settled_dist - self.params.signed_target_m, 5)
                trial_report.post_zero_coast_m = round(settled_dist - dist_at_zero_issued, 5)
                trial_report.imu_heading_change_deg = round(normalize_angle_deg(final_yaw - start_yaw_deg), 3)

                # Construct Phase Breakdown for Physical Run
                t_accel_end = t_steady_start if t_steady_start is not None else (t_zero_issued or t_settled_time)
                accel_dur = max(0.0, round(t_accel_end - t_start_motion, 3))
                accel_dist = round(dist_at_steady_start, 4)
                accel_spd = round(accel_dist / accel_dur, 3) if accel_dur > 0 else 0.0
                trial_report.acceleration_phase = {
                    "duration_s": accel_dur,
                    "distance_m": accel_dist,
                    "mean_speed_mps": accel_spd,
                    "peak_pwm": max(accel_pwms) if accel_pwms else 0
                }

                if t_zero_issued is not None and t_steady_start is not None and t_zero_issued > t_steady_start:
                    std_dur = round(t_zero_issued - t_steady_start, 3)
                    std_dist = round(dist_at_zero_issued - dist_at_steady_start, 4)
                    std_spd = round(std_dist / std_dur, 3) if std_dur > 0 else 0.0
                else:
                    std_dur = 0.0
                    std_dist = 0.0
                    std_spd = 0.0

                # Compute steady-speed wheel metrics
                steady_metrics: Dict[str, WheelTrialMetrics] = {}
                for w_id in ["m1", "m2", "m3", "m4"]:
                    sm = WheelTrialMetrics(wheel_id=w_id)
                    sw = steady_wheel_samples[w_id]
                    if sw["targets"]:
                        sm.commanded_speed_radps_mean = round(sum(sw["targets"]) / len(sw["targets"]), 3)
                    if sw["measured"]:
                        sm.measured_speed_radps_mean = round(sum(sw["measured"]) / len(sw["measured"]), 3)
                        sm.measured_speed_radps_max = round(max(sw["measured"]), 3)
                        sm.measured_speed_radps_min = round(min(sw["measured"]), 3)
                    if sw["pwms"]:
                        sm.pwm_mean = round(sum(sw["pwms"]) / len(sw["pwms"]), 1)
                        sm.pwm_max = max(sw["pwms"])

                    if steady_start_ticks and steady_end_ticks and (steady_end_ticks[w_id] != steady_start_ticks[w_id]):
                        sm.encoder_start_ticks = steady_start_ticks[w_id]
                        sm.encoder_final_ticks = steady_end_ticks[w_id]
                        sm.encoder_delta_ticks = steady_end_ticks[w_id] - steady_start_ticks[w_id]
                    else:
                        sm.encoder_start_ticks = start_ticks[w_id]
                        sm.encoder_final_ticks = extracted_final[w_id]
                        sm.encoder_delta_ticks = extracted_final[w_id] - start_ticks[w_id]

                    steady_metrics[w_id] = sm
                trial_report.steady_speed_wheel_metrics = steady_metrics

                # Left vs Right Steady Disparities
                left_meas = [steady_metrics[w].measured_speed_radps_mean for w in ["m1", "m3"] if steady_metrics[w].measured_speed_radps_mean is not None]
                right_meas = [steady_metrics[w].measured_speed_radps_mean for w in ["m2", "m4"] if steady_metrics[w].measured_speed_radps_mean is not None]
                left_pwms = [steady_metrics[w].pwm_mean for w in ["m1", "m3"] if steady_metrics[w].pwm_mean is not None]
                right_pwms = [steady_metrics[w].pwm_mean for w in ["m2", "m4"] if steady_metrics[w].pwm_mean is not None]

                lr_spd_diff = round((sum(left_meas)/len(left_meas)) - (sum(right_meas)/len(right_meas)), 3) if left_meas and right_meas else 0.0
                lr_pwm_diff = round((sum(left_pwms)/len(left_pwms)) - (sum(right_pwms)/len(right_pwms)), 1) if left_pwms and right_pwms else 0.0

                trial_report.steady_speed_phase = {
                    "duration_s": std_dur,
                    "distance_m": std_dist,
                    "mean_speed_mps": std_spd,
                    "left_to_right_speed_diff_radps": lr_spd_diff,
                    "left_to_right_pwm_diff": lr_pwm_diff
                }
                trial_report.steady_speed_left_to_right_speed_diff_radps = lr_spd_diff
                trial_report.steady_speed_left_to_right_pwm_diff = lr_pwm_diff

                if steady_metrics["m1"].measured_speed_radps_mean is not None and steady_metrics["m3"].measured_speed_radps_mean is not None:
                    trial_report.steady_speed_front_to_rear_left_radps = round(
                        steady_metrics["m1"].measured_speed_radps_mean - steady_metrics["m3"].measured_speed_radps_mean, 3
                    )
                if steady_metrics["m2"].measured_speed_radps_mean is not None and steady_metrics["m4"].measured_speed_radps_mean is not None:
                    trial_report.steady_speed_front_to_rear_right_radps = round(
                        steady_metrics["m2"].measured_speed_radps_mean - steady_metrics["m4"].measured_speed_radps_mean, 3
                    )

                # Stopping Phase
                stp_dur = round(t_settled_time - (t_zero_issued or t_settled_time), 3)
                stp_dist = round(settled_dist - dist_at_zero_issued, 4)
                trial_report.stopping_phase = {
                    "duration_s": stp_dur,
                    "distance_m": stp_dist
                }
                trial_report.stopping_distance_m = stp_dist
                trial_report.stopping_duration_s = stp_dur

                # Overall wheel metrics
                for w_id in ["m1", "m2", "m3", "m4"]:
                    w_metric = wheel_metrics[w_id]
                    ws = wheel_samples[w_id]
                    w_metric.encoder_final_ticks = extracted_final[w_id]
                    w_metric.encoder_delta_ticks = w_metric.encoder_final_ticks - w_metric.encoder_start_ticks
                    if ws["targets"]:
                        w_metric.commanded_speed_radps_mean = round(sum(ws["targets"]) / len(ws["targets"]), 3)
                        w_metric.commanded_speed_radps_max = round(max(ws["targets"], key=abs), 3)
                    if ws["measured"]:
                        w_metric.measured_speed_radps_mean = round(sum(ws["measured"]) / len(ws["measured"]), 3)
                        w_metric.measured_speed_radps_max = round(max(ws["measured"]), 3)
                        w_metric.measured_speed_radps_min = round(min(ws["measured"]), 3)
                        w_metric.measured_speed_radps_abs_mean = round(sum(abs(x) for x in ws["measured"]) / len(ws["measured"]), 3)
                    if ws["pwms"]:
                        w_metric.pwm_mean = round(sum(ws["pwms"]) / len(ws["pwms"]), 1)
                        w_metric.pwm_max = max(ws["pwms"])

                # Front-to-rear differences
                if active_pid_packets:
                    bal = compute_phase_balancing_metrics(active_pid_packets)
                    if bal:
                        trial_report.front_to_rear_diff_left_radps = round(bal[0]["pair_error_left"], 3)
                        trial_report.front_to_rear_diff_right_radps = round(bal[0]["pair_error_right"], 3)

        except Exception as e:
            trial_report.status = "ABORTED"
            trial_report.abort_reason = str(e)
            print(f"[TRIAL ABORTED] {e}")
        finally:
            # Capture final encoder ticks and distance EVEN IF ABORTED
            if not self.params.dry_run:
                try:
                    final_enc = self.cockpit.get_encoders()
                    final_ticks = extract_encoder_ticks(final_enc)
                    if final_ticks:
                        for w in ["m1", "m2", "m3", "m4"]:
                            wheel_metrics[w].encoder_final_ticks = final_ticks[w]
                            wheel_metrics[w].encoder_delta_ticks = final_ticks[w] - start_ticks[w]
                        final_avg_delta = sum(wheel_metrics[w].encoder_delta_ticks for w in ["m1", "m2", "m3", "m4"]) / 4.0
                        if trial_report.measured_distance_m is None:
                            trial_report.measured_distance_m = round(ticks_to_meters(final_avg_delta), 5)
                            trial_report.final_settled_distance_m = trial_report.measured_distance_m
                        if trial_report.steady_speed_wheel_metrics:
                            for w in ["m1", "m2", "m3", "m4"]:
                                sm = trial_report.steady_speed_wheel_metrics.get(w)
                                if sm and sm.encoder_delta_ticks == 0 and wheel_metrics[w].encoder_delta_ticks != 0:
                                    sm.encoder_start_ticks = wheel_metrics[w].encoder_start_ticks
                                    sm.encoder_final_ticks = wheel_metrics[w].encoder_final_ticks
                                    sm.encoder_delta_ticks = wheel_metrics[w].encoder_delta_ticks
                except Exception:
                    pass

            cleanup_st = self.cleanup()
            trial_report.confirmed_final_zero_command = True
            trial_report.confirmed_final_disarmed_state = cleanup_st.get("armed") is False

        trial_report.approach_milestones = controller.get_telemetry_summary()
        return trial_report

    def _assert_pre_motion_invariants(self) -> Optional[str]:
        """
        Immediately before the first nonzero command, assert:

        - armed == true
        - autonomyState == READY_ARMED
        - expected command ownership is valid (NONE or ROS_AUTONOMY)
        - IMU and encoder telemetry are fresh
        Returns None if valid, or a descriptive failure message if invalid.
        """
        drv_stat = self.cockpit.get_status()
        auto_stat = self.cockpit.get_autonomy_status()
        imu_stat = self.cockpit.get_imu()
        enc_stat = self.cockpit.get_encoders()

        if drv_stat.get("armed") is not True:
            return f"Pre-motion invariant violated: armed is {drv_stat.get('armed')} (expected True)"

        cur_state = auto_stat.get("state")
        if cur_state != "READY_ARMED":
            return f"Pre-motion invariant violated: autonomyState is '{cur_state}' (expected READY_ARMED)"

        cmd_source = auto_stat.get("cmdSource") or drv_stat.get("cmdSource")
        if cmd_source not in ("NONE", "ROS_AUTONOMY", None):
            return f"Pre-motion invariant violated: unexpected cmdSource '{cmd_source}' (expected NONE or ROS_AUTONOMY)"

        is_fresh, freshness_msg = check_imu_freshness(imu_stat, max_age_ms=250.0)
        if not is_fresh:
            return f"Pre-motion invariant violated: {freshness_msg}"

        is_enc_fresh, enc_msg = check_encoder_freshness(enc_stat, max_age_ms=250.0)
        if not is_enc_fresh:
            return f"Pre-motion invariant violated: {enc_msg}"

        return None

    @staticmethod
    def _validate_wheel_forensics_consistency(
        trial_report: TrialReport,
        wheel_metrics: Dict[str, WheelTrialMetrics],
        wheel_samples: Dict[str, Dict[str, Any]],
    ) -> None:
        """
        Validates telemetry consistency against physical encoder feedback.
        If encoder deltas are nonzero (indicating physical motion) but every active
        target and measured speed is zero, mark wheel forensics INVALID rather than
        reporting misleading valid zero values.
        """
        any_nonzero_encoder = any(abs(wheel_metrics[w].encoder_delta_ticks) > 50 for w in ["m1", "m2", "m3", "m4"])

        for w_id in ["m1", "m2", "m3", "m4"]:
            w_metric = wheel_metrics[w_id]
            ws_buf = wheel_samples.get(w_id, {})
            targets = ws_buf.get("targets", [])
            measured = ws_buf.get("measured", [])

            has_encoder_motion = (abs(w_metric.encoder_delta_ticks) > 50) or (any_nonzero_encoder and abs(w_metric.encoder_delta_ticks) > 0)
            all_targets_zero = len(targets) > 0 and all(abs(t) < 1e-4 for t in targets)
            all_measured_zero = len(measured) > 0 and all(abs(m) < 1e-4 for m in measured)

            if has_encoder_motion and all_targets_zero and all_measured_zero:
                w_metric.validity = "INVALID"
                w_metric.commanded_speed_source = "INVALID_ZERO_TELEMETRY"
                w_metric.commanded_speed_radps_mean = None
                w_metric.commanded_speed_radps_max = None
                w_metric.measured_speed_radps_mean = None
                w_metric.measured_speed_radps_abs_mean = None
                w_metric.measured_speed_radps_min = None
                w_metric.measured_speed_radps_max = None
                w_metric.stopped_while_commanded = None
                trial_report.wheel_forensics_valid = False

    def execute_single_trial(self, trial_idx: int, cumulative_continuous_heading: Optional[float] = None) -> TrialReport:
        """Executes one physical turn trial adhering to all exact-motion invariants."""
        print(f"\n--- Starting Trial/Step {trial_idx}/{self.params.trials} ---")

        # 1. Inter-Trial Operator Approval
        if self.params.inter_trial_approval or not self.params.dry_run:
            print("\n[OPERATOR APPROVAL REQUIRED]")
            print(f"  • Maneuver (Step {trial_idx}/{self.params.trials}): {self.params.degrees:.1f}° {self.params.direction.upper()}")
            print(f"  • Target Signed Yaw: {self.params.signed_target_deg:+.1f}°")
            print(f"  • Speeds: Cruise = {self.params.max_angular_speed:.2f} rad/s | Creep = {self.params.creep_angular_speed:.2f} rad/s")
            print("  Please confirm rover area is clear and authorize physical motion.")
            resp = self.prompt_fn("Authorize motion? [y/N]: ").strip().lower()
            if resp not in ("y", "yes"):
                print("[OPERATOR ABORT] Motion not approved by Ron. Aborting trial.")
                return TrialReport(
                    trial_index=trial_idx,
                    requested_turn_deg=self.params.degrees,
                    direction=self.params.direction,
                    target_signed_yaw_deg=self.params.signed_target_deg,
                    status="ABORTED",
                    abort_reason="Operator authorization withheld"
                )

        trial_report = TrialReport(
            trial_index=trial_idx,
            requested_turn_deg=self.params.degrees,
            direction=self.params.direction,
            target_signed_yaw_deg=self.params.signed_target_deg,
            status="INITIALIZING",
            stopping_advance_deg=self.params.stopping_advance_deg or 0.0
        )

        t_trial_start = time.time()

        # Step 2b: Configure drive parameters (wheel balancing & dynamic braking) via Cockpit
        anti_stall_confirmed = False
        if not self.params.dry_run:
            # If user explicitly requested drive parameter overrides via CLI flags, apply them
            if self.params.enable_balancing is not None or self.params.enable_braking is not None:
                current_cfg = {}
                try:
                    cfg_pre = self.cockpit.get_drive_config()
                    if cfg_pre.get("ok"):
                        current_cfg = cfg_pre.get("config", {})
                except Exception:
                    pass

                target_bal = bool(self.params.enable_balancing) if self.params.enable_balancing is not None else current_cfg.get("wheelBalancing", False)
                target_brk = bool(self.params.enable_braking) if self.params.enable_braking is not None else current_cfg.get("dynamicBraking", True)
                try:
                    self.cockpit.configure_drive(
                        wheel_balancing=target_bal,
                        dynamic_braking=target_brk
                    )
                    time.sleep(0.05)
                except Exception as e:
                    print(f"[WARN] Failed to configure drive parameters: {e}")

            # Read back actual live drive config from cockpit server & ESP32
            live_cfg = {}
            try:
                cfg_resp = self.cockpit.get_drive_config()
                if cfg_resp.get("ok"):
                    live_cfg = cfg_resp.get("config", {})
            except Exception as e:
                print(f"[WARN] Failed to read live drive configuration: {e}")

            # Query live PID telemetry to confirm anti-stall / stiction state machine
            try:
                pid_resp = self.cockpit.get_pid_telemetry()
                if pid_resp.get("ok") and pid_resp.get("telemetry"):
                    telem = pid_resp.get("telemetry", {})
                    if "m1" in telem and "stictionState" in telem["m1"]:
                        anti_stall_confirmed = True
            except Exception as e:
                print(f"[WARN] Failed to read live anti-stall telemetry: {e}")
        else:
            live_cfg = {
                "wheelBalancing": bool(self.params.enable_balancing) if self.params.enable_balancing is not None else False,
                "dynamicBraking": bool(self.params.enable_braking) if self.params.enable_braking is not None else True,
                "brakeDurationMs": 100,
                "maxTriggerSpeedMps": 0.35,
                "maxTriggerSpeed": 0.35,
            }
            anti_stall_confirmed = True

        confirmed_bal = bool(live_cfg.get("wheelBalancing", False))
        confirmed_brk = bool(live_cfg.get("dynamicBraking", True))
        confirmed_dur = int(live_cfg.get("brakeDurationMs", 100))
        confirmed_max_spd = float(live_cfg.get("maxTriggerSpeed", 0.35))
        confirmed_max_spd_mps = float(live_cfg.get("maxTriggerSpeedMps", confirmed_max_spd))

        trial_report.wheel_balancing_enabled = confirmed_bal
        trial_report.dynamic_braking_enabled = confirmed_brk
        trial_report.dynamic_brake_duration_ms = confirmed_dur
        trial_report.dynamic_brake_max_speed = confirmed_max_spd
        trial_report.dynamic_brake_max_speed_mps = confirmed_max_spd_mps
        trial_report.anti_stall_confirmed = anti_stall_confirmed
        trial_report.stopping_advance_deg = self.params.stopping_advance_deg if self.params.stopping_advance_deg is not None else (0.7 if confirmed_brk else 0.0)

        # Step 3: Ensure stationary IMU/gyro calibration and settling while DISARMED
        if not self.params.dry_run:
            cur_st = self.cockpit.get_status()
            if cur_st.get("armed") is True or cur_st.get("autonomyState") != "DISABLED":
                self.cleanup(verbose=False)

        # 2. Fresh IMU Snapshot & Non-Magnetic Validation
        imu_snap = self.cockpit.get_imu()
        if not self.params.dry_run:
            valid_non_mag, reason = verify_non_magnetic_imu(imu_snap)
            if not valid_non_mag:
                trial_report.status = "ABORTED"
                trial_report.abort_reason = f"IMU Non-Magnetic Check FAILED: {reason}"
                print(f"[FAIL-CLOSED ERROR] {trial_report.abort_reason}")
                cleanup_st = self.cleanup()
                trial_report.confirmed_final_zero_command = True
                trial_report.confirmed_final_disarmed_state = (
                    cleanup_st.get("armed") is False
                    and cleanup_st.get("autonomyState") == "DISABLED"
                    and cleanup_st.get("cmdSource") in ("NONE", None)
                )
                return trial_report
            print(f"[OK] BNO08x Orientation Verified: {reason}")
        else:
            print("[OK] [DRY RUN] BNO08x non-magnetic check simulated.")

        # 3. Capture Initial Encoder State
        enc_start = self.cockpit.get_encoders().get("encoders", {"m1": 0, "m2": 0, "m3": 0, "m4": 0})
        wheel_metrics = {
            "m1": WheelTrialMetrics(wheel_id="m1", encoder_start_ticks=enc_start.get("m1", 0)),
            "m2": WheelTrialMetrics(wheel_id="m2", encoder_start_ticks=enc_start.get("m2", 0)),
            "m3": WheelTrialMetrics(wheel_id="m3", encoder_start_ticks=enc_start.get("m3", 0)),
            "m4": WheelTrialMetrics(wheel_id="m4", encoder_start_ticks=enc_start.get("m4", 0)),
        }

        # 4. Relative Orientation Initialization
        raw_yaw_0 = quat_to_yaw(imu_snap.get("orientation", {"w": 1, "x": 0, "y": 0, "z": 0}))
        unwrapper = YawUnwrapper(initial_raw_yaw=raw_yaw_0)
        gyro_integrator = BiasCorrectedGyroIntegrator()

        initial_raw_yaw_deg = round(math.degrees(raw_yaw_0), 4)
        if self.params.dry_run and cumulative_continuous_heading is not None:
            trial_report.start_heading_raw_deg = round(normalize_angle_deg(cumulative_continuous_heading), 4)
            trial_report.start_heading_continuous_deg = round(cumulative_continuous_heading, 4)
        else:
            trial_report.start_heading_raw_deg = initial_raw_yaw_deg
            trial_report.start_heading_continuous_deg = round(cumulative_continuous_heading, 4) if cumulative_continuous_heading is not None else initial_raw_yaw_deg

        # Gather pre-turn stationary samples for gyro bias calibration while DISARMED
        pre_turn_samples = [imu_snap.get("gyro", {}).get("z", 0.0)]
        for _ in range(5):
            time.sleep(0.03)
            sample_snap = self.cockpit.get_imu()
            pre_turn_samples.append(sample_snap.get("gyro", {}).get("z", 0.0))

        try:
            gyro_integrator.calibrate_bias(pre_turn_samples)
        except SensorException as e:
            if not self.params.dry_run:
                trial_report.status = "ABORTED"
                trial_report.abort_reason = f"Stationary gyro bias calibration failed: {e}"
                print(f"[SENSOR ABORT] {trial_report.abort_reason}")
                cleanup_st = self.cleanup()
                trial_report.confirmed_final_zero_command = True
                trial_report.confirmed_final_disarmed_state = (
                    cleanup_st.get("armed") is False
                    and cleanup_st.get("autonomyState") == "DISABLED"
                    and cleanup_st.get("cmdSource") in ("NONE", None)
                )
                return trial_report

        # Step 4b: Controlled Operator-Authorized Fault Clear Path (if requested)
        if self.params.clear_faults and not self.params.dry_run:
            stat = self.cockpit.get_status()
            if stat.get("armed", False):
                print("[FAULT CLEAR REJECTED] Cannot clear faults while rover is armed.")
            else:
                clear_res = self.cockpit.clear_faults()
                if clear_res.get("ok", False):
                    print("[OK] Controlled fault clear executed: rover confirmed stationary and disarmed.")
                else:
                    print(f"[FAULT CLEAR WARNING] Clear faults response: {clear_res.get('error', 'unknown')}")

        # Step 5: Autonomy Enable and Three-Consecutive-Zero Handshake -> READY_DISARMED
        if not self.params.dry_run:
            try:
                print("  Executing 3-consecutive-zero autonomy handshake...")
                perform_zero_handshake(self.cockpit, ws=self.ws, transitions=trial_report.autonomy_state_transitions, arm=False)
                print("[OK] Autonomy Handshake complete: Drivetrain READY_DISARMED.")
            except HandshakeException as e:
                tb_str = traceback.format_exc()
                trial_report.status = "ABORTED"
                trial_report.abort_reason = (
                    f"[{type(e).__name__}] {e} | Stage: {e.stage} | State: {e.state} "
                    f"| ZeroCount: {e.zero_count} | CmdSource: {e.cmd_source}"
                )
                print("\n[HANDSHAKE ABORT]")
                print(f"  • Exception Type:    {type(e).__name__}")
                print(f"  • Exception Message: {e}")
                print(f"  • Handshake Stage:   {e.stage}")
                print(f"  • Handshake State:   state={e.state}, zeroCount={e.zero_count}, cmdSource={e.cmd_source}")
                if e.last_rejection_reason:
                    print(f"  • Last Rejection:    {e.last_rejection_reason}")
                if e.underlying_error:
                    print(f"  • Underlying Error:  {e.underlying_error}")
                print("  • Traceback:")
                for line in tb_str.strip().splitlines():
                    print(f"      {line}")
                cleanup_st = self.cleanup()
                trial_report.confirmed_final_zero_command = True
                trial_report.confirmed_final_disarmed_state = (
                    cleanup_st.get("armed") is False
                    and cleanup_st.get("autonomyState") == "DISABLED"
                    and cleanup_st.get("cmdSource") in ("NONE", None)
                )
                return trial_report
            except Exception as e:
                tb_str = traceback.format_exc()
                trial_report.status = "ABORTED"
                trial_report.abort_reason = f"[{type(e).__name__}] Unexpected fault in zero handshake: {e}"
                print(f"\n[HANDSHAKE UNEXPECTED FAULT]")
                print(f"  • Exception Type:    {type(e).__name__}")
                print(f"  • Exception Message: {e}")
                print("  • Traceback:")
                for line in tb_str.strip().splitlines():
                    print(f"      {line}")
                cleanup_st = self.cleanup()
                trial_report.confirmed_final_zero_command = True
                trial_report.confirmed_final_disarmed_state = (
                    cleanup_st.get("armed") is False
                    and cleanup_st.get("autonomyState") == "DISABLED"
                    and cleanup_st.get("cmdSource") in ("NONE", None)
                )
                return trial_report
        else:
            print("[OK] [DRY RUN] 3-consecutive-zero autonomy handshake simulated.")
            trial_report.autonomy_state_transitions.extend([
                {"timestamp": time.time(), "state": "WAITING_FOR_ZERO", "trigger": "enable_autonomy (dry run)"},
                {"timestamp": time.time(), "state": "READY_DISARMED", "trigger": "zero_handshake (dry run)"}
            ])

        # Step 6: Acquire an advancing fresh IMU orientation sample after zero handshake (while safely READY_DISARMED)
        if not self.params.dry_run:
            baseline_imu = self.cockpit.get_imu()
            baseline_seq = baseline_imu.get("sequence") if baseline_imu else None
            baseline_esp_ts = baseline_imu.get("espTimestampUs") if baseline_imu else None

            adv_ok, fresh_imu, adv_reason = wait_for_advancing_imu_sample(
                self.cockpit,
                baseline_seq=baseline_seq,
                baseline_esp_ts_us=baseline_esp_ts,
                max_wait_sec=0.50,
                max_acceptable_age_ms=100.0,
                watchdog_max_age_ms=250.0
            )
            if not adv_ok:
                fail_reason = f"Pre-motion IMU synchronization failed in READY_DISARMED: {adv_reason}"
                print(f"[PRE-MOTION ABORT] {fail_reason}")
                trial_report.status = "ABORTED"
                trial_report.abort_reason = fail_reason
                trial_report.telemetry_samples_count = 0
                trial_report.wheel_metrics = wheel_metrics
                cleanup_st = self.cleanup()
                trial_report.confirmed_final_zero_command = True
                trial_report.confirmed_final_disarmed_state = (
                    cleanup_st.get("armed") is False
                    and cleanup_st.get("autonomyState") == "DISABLED"
                    and cleanup_st.get("cmdSource") in ("NONE", None)
                )
                trial_report.autonomy_state_transitions.append({
                    "timestamp": time.time(),
                    "state": cleanup_st.get("autonomyState", "DISABLED"),
                    "trigger": "advancing_imu_sample_timeout"
                })
                return trial_report

            if fresh_imu:
                fresh_raw_yaw = quat_to_yaw(fresh_imu.get("orientation", {}))
                unwrapper.update_orientation_yaw(fresh_raw_yaw)
                fresh_raw_deg = round(math.degrees(fresh_raw_yaw), 4)
                trial_report.start_heading_raw_deg = fresh_raw_deg
                trial_report.start_heading_continuous_deg = round(cumulative_continuous_heading, 4) if cumulative_continuous_heading is not None else fresh_raw_deg
            print(f"[OK] Advancing fresh IMU sample synchronized in READY_DISARMED: {adv_reason}")

        # Step 7: Arm Drivetrain and Verify READY_ARMED
        if not self.params.dry_run:
            try:
                arm_and_verify_ready_armed(self.cockpit, transitions=trial_report.autonomy_state_transitions)
                print("[OK] Drivetrain armed: READY_ARMED verified.")
            except HandshakeException as e:
                tb_str = traceback.format_exc()
                trial_report.status = "ABORTED"
                trial_report.abort_reason = (
                    f"[{type(e).__name__}] {e} | Stage: {e.stage} | State: {e.state} "
                    f"| ZeroCount: {e.zero_count} | CmdSource: {e.cmd_source}"
                )
                print("\n[ARMING ABORT]")
                print(f"  • Exception Type:    {type(e).__name__}")
                print(f"  • Exception Message: {e}")
                print(f"  • Handshake Stage:   {e.stage}")
                print(f"  • Handshake State:   state={e.state}, zeroCount={e.zero_count}, cmdSource={e.cmd_source}")
                if e.last_rejection_reason:
                    print(f"  • Last Rejection:    {e.last_rejection_reason}")
                if e.underlying_error:
                    print(f"  • Underlying Error:  {e.underlying_error}")
                print("  • Traceback:")
                for line in tb_str.strip().splitlines():
                    print(f"      {line}")
                cleanup_st = self.cleanup()
                trial_report.confirmed_final_zero_command = True
                trial_report.confirmed_final_disarmed_state = (
                    cleanup_st.get("armed") is False
                    and cleanup_st.get("autonomyState") == "DISABLED"
                    and cleanup_st.get("cmdSource") in ("NONE", None)
                )
                return trial_report
            except Exception as e:
                tb_str = traceback.format_exc()
                trial_report.status = "ABORTED"
                trial_report.abort_reason = f"[{type(e).__name__}] Unexpected fault in arming: {e}"
                print(f"\n[ARMING UNEXPECTED FAULT]")
                print(f"  • Exception Type:    {type(e).__name__}")
                print(f"  • Exception Message: {e}")
                print("  • Traceback:")
                for line in tb_str.strip().splitlines():
                    print(f"      {line}")
                cleanup_st = self.cleanup()
                trial_report.confirmed_final_zero_command = True
                trial_report.confirmed_final_disarmed_state = (
                    cleanup_st.get("armed") is False
                    and cleanup_st.get("autonomyState") == "DISABLED"
                    and cleanup_st.get("cmdSource") in ("NONE", None)
                )
                return trial_report
        else:
            trial_report.autonomy_state_transitions.append(
                {"timestamp": time.time(), "state": "READY_ARMED", "trigger": "arm_drive (dry run)"}
            )

        # Step 8: Immediately before first nonzero command, assert pre-motion invariants
        if not self.params.dry_run:
            inv_error = self._assert_pre_motion_invariants()
            if inv_error:
                trial_report.status = "ABORTED"
                trial_report.abort_reason = inv_error
                trial_report.telemetry_samples_count = 0
                trial_report.wheel_metrics = wheel_metrics
                print(f"[FAIL-CLOSED ASSERTION FAILED] {inv_error}")
                cleanup_st = self.cleanup()
                trial_report.confirmed_final_zero_command = True
                trial_report.confirmed_final_disarmed_state = (
                    cleanup_st.get("armed") is False
                    and cleanup_st.get("autonomyState") == "DISABLED"
                    and cleanup_st.get("cmdSource") in ("NONE", None)
                )
                trial_report.autonomy_state_transitions.append({
                    "timestamp": time.time(),
                    "state": cleanup_st.get("autonomyState", "DISABLED"),
                    "trigger": "pre_motion_assertion_failed"
                })
                return trial_report
            print("[OK] Pre-motion invariants verified: armed=True, autonomyState=READY_ARMED, valid cmdSource, fresh IMU & encoders.")

        # Step 9: Immediately send first command (no dwell/sleep while armed)
        stopping_advance = self.params.stopping_advance_deg if self.params.enable_braking else 0.0
        controller = AngularApproachController(
            cruise_wz_radps=self.params.max_angular_speed,
            creep_wz_radps=self.params.creep_angular_speed,
            approach_zone_deg=self.params.creep_threshold_deg,
            stopping_advance_deg=stopping_advance
        )
        controller.reset(target=self.params.signed_target_deg, start_time=time.time())

        first_cmd_wz = controller.update(current_progress=0.0, current_time=time.time())
        if abs(first_cmd_wz) < 1e-4:
            first_cmd_wz = math.copysign(self.params.creep_angular_speed, self.params.signed_target_deg)

        if not self.params.dry_run:
            first_res = self.cockpit.send_cmd_vel(vx=0.0, wz=first_cmd_wz)
            trial_report.first_command_response = first_res

            # Verify the first nonzero command was accepted
            if not first_res.get("ok", False):
                err = first_res.get("error", "Unknown command rejection")
                rej = self.cockpit.get_autonomy_status().get("lastRejectionReason")
                fail_reason = f"First nonzero command rejected: {err}" + (f" ({rej})" if rej else "")
                print(f"[FAIL-FAST ABORT] {fail_reason}")
                trial_report.status = "ABORTED"
                trial_report.abort_reason = fail_reason
                trial_report.telemetry_samples_count = 0
                trial_report.wheel_metrics = wheel_metrics
                cleanup_st = self.cleanup()
                trial_report.confirmed_final_zero_command = True
                trial_report.confirmed_final_disarmed_state = (
                    cleanup_st.get("armed") is False
                    and cleanup_st.get("autonomyState") == "DISABLED"
                    and cleanup_st.get("cmdSource") in ("NONE", None)
                )
                trial_report.autonomy_state_transitions.append({
                    "timestamp": time.time(),
                    "state": cleanup_st.get("autonomyState", "DISABLED"),
                    "trigger": "first_cmd_rejected_cleanup"
                })
                return trial_report

            # Step 10: Within bounded interval (max 500ms), verify targets become nonzero / state becomes ACTIVE
            cmd_accepted_active = False
            t_check_start = time.time()
            while time.time() - t_check_start < 0.5:
                a_stat = self.cockpit.get_autonomy_status()
                if a_stat.get("state") == "ACTIVE" or abs(a_stat.get("clampedAngular", 0.0)) > 1e-4:
                    cmd_accepted_active = True
                    trial_report.autonomy_state_transitions.append({
                        "timestamp": time.time(),
                        "state": "ACTIVE",
                        "trigger": "first_nonzero_cmd_accepted"
                    })
                    break
                time.sleep(0.04)

            # Step 11: If command rejected or targets remain zero, abort immediately (do not wait 15s)
            if not cmd_accepted_active:
                a_stat = self.cockpit.get_autonomy_status()
                rejection = a_stat.get("lastRejectionReason") or "Commanded wheel targets remained zero / ACTIVE state not achieved within 500ms"
                fail_reason = f"Motion startup failed within bounded window: {rejection} (state={a_stat.get('state')})"
                print(f"[FAIL-FAST ABORT] {fail_reason}")
                trial_report.status = "ABORTED"
                trial_report.abort_reason = fail_reason
                trial_report.telemetry_samples_count = 0
                trial_report.wheel_metrics = wheel_metrics
                cleanup_st = self.cleanup()
                trial_report.confirmed_final_zero_command = True
                trial_report.confirmed_final_disarmed_state = (
                    cleanup_st.get("armed") is False
                    and cleanup_st.get("autonomyState") == "DISABLED"
                    and cleanup_st.get("cmdSource") in ("NONE", None)
                )
                trial_report.autonomy_state_transitions.append({
                    "timestamp": time.time(),
                    "state": cleanup_st.get("autonomyState", "DISABLED"),
                    "trigger": "startup_timeout_cleanup"
                })
                return trial_report

            # Step 11b: Drain any stale pre-motion WebSocket frames so telemetry collection starts strictly on ACTIVE motion
            if self.ws and self.ws.connected:
                self.ws.recv_frames()
        else:
            trial_report.first_command_response = {"ok": True, "linear": 0.0, "angular": first_cmd_wz, "state": "ACTIVE"}
            trial_report.autonomy_state_transitions.append({
                "timestamp": time.time(),
                "state": "ACTIVE",
                "trigger": "first_nonzero_cmd_accepted (dry run)"
            })

        # Safety thresholds
        max_duration_sec = 15.0
        max_angle_limit_deg = abs(self.params.degrees) + 15.0
        angle_at_zero_cmd = 0.0
        stalls_detected = {w: False for w in ["m1", "m2", "m3", "m4"]}
        breakout_count = 0
        breakout_start_time = None
        total_breakout_dwell_ms = 0.0

        # Active-motion telemetry collection buffers (strictly excluding preflight, settling, and final-zero)
        wheel_samples = {
            w_id: {
                "targets": [],
                "measured": [],
                "stopped_events": 0,
                "stopped_duration_s": 0.0,
                "is_currently_stopped": False,
                "stopped_entry_time": None,
            }
            for w_id in ["m1", "m2", "m3", "m4"]
        }
        total_telemetry_samples = 0
        angle_at_zero_cmd = 0.0
        zero_send_wall_ms: Optional[int] = None

        abort_reason = None
        t_motion_start = time.time()
        active_pid_packets: List[Dict[str, Any]] = []

        try:
            if self.params.dry_run:
                # In dry-run mode, simulate steps through approach controller without moving motors
                advance = self.params.stopping_advance_deg if self.params.enable_braking else 0.0
                sim_target_stop = self.params.signed_target_deg - math.copysign(advance, self.params.signed_target_deg)
                sim_angles = [
                    0.0,
                    self.params.signed_target_deg * 0.5,
                    self.params.signed_target_deg * 0.85,
                    sim_target_stop
                ]
                for sim_yaw in sim_angles:
                    cmd_wz = controller.update(current_progress=sim_yaw, current_time=time.time())
                    wheel_cmds = compute_wheel_speed_targets(cmd_wz)
                    sym_ok, sym_msg = verify_wheel_command_symmetry(wheel_cmds)
                    if not sym_ok:
                        raise TestAbortException(f"Symmetry check failed: {sym_msg}")
                    if controller.phase in (ApproachPhase.CRUISE, ApproachPhase.CREEP):
                        for w_id in ["m1", "m2", "m3", "m4"]:
                            cmd_val = wheel_cmds[w_id]
                            wheel_samples[w_id]["targets"].append(cmd_val)
                            wheel_samples[w_id]["measured"].append(cmd_val)
                        total_telemetry_samples += 1
                    time.sleep(0.05)
                angle_at_zero_cmd = sim_target_stop
                t_dry_mono = time.monotonic()
                trial_report.zero_command_send_time_monotonic = t_dry_mono
                trial_report.zero_command_response_time_monotonic = t_dry_mono
                trial_report.zero_command_latency_ms = 0.0
                trial_report.zero_command_response = {"ok": True, "dry_run": True}
                if self.params.enable_braking:
                    trial_report.actuation_states_observed = ["DRIVE", "BRAKE", "COAST"]
                    trial_report.brake_active_duration_ms = 100.0
                else:
                    trial_report.actuation_states_observed = ["DRIVE", "COAST"]
                    trial_report.brake_active_duration_ms = 0.0
            else:
                # Physical motion loop
                last_motion_imu_seq: Optional[int] = None
                last_motion_esp_ts: Optional[int] = None
                last_motion_reset_count: Optional[int] = None
                last_motion_imu_advance_time: float = t_motion_start
                latest_imu: Optional[Dict[str, Any]] = None
                wheel_cmds: Dict[str, float] = {w: 0.0 for w in ["m1", "m2", "m3", "m4"]}

                while True:
                    now = time.time()
                    dt_motion = now - t_motion_start

                    # Duration watchdog
                    if dt_motion > max_duration_sec:
                        abort_reason = f"Maximum trial duration exceeded ({max_duration_sec}s)"
                        trial_report.watchdog_trips.append("MAX_DURATION_EXCEEDED")
                        break

                    # Ingest WebSocket frames (IMU and PID diagnostics)
                    ws_frames = []
                    latest_ws_imu = None
                    if self.ws and self.ws.connected:
                        ws_frames = self.ws.recv_frames()
                        for f in ws_frames:
                            if f.get("type") == "bno08x_imu":
                                latest_ws_imu = f

                    if latest_ws_imu is not None:
                        latest_imu = latest_ws_imu
                        # Frame freshly delivered via low-latency WebSocket push
                        if "dataAgeMs" not in latest_imu or latest_imu["dataAgeMs"] is None:
                            latest_imu["dataAgeMs"] = 0.0
                    else:
                        # Fallback to HTTP endpoint if no WebSocket IMU frame was received
                        latest_imu = self.cockpit.get_imu()

                    cur_seq = latest_imu.get("sequence") if latest_imu else None
                    cur_esp_ts = latest_imu.get("espTimestampUs") if latest_imu else None
                    cur_reset_cnt = latest_imu.get("resetCount") if latest_imu else None

                    # Check for explicit ESP32 hardware reset during motion
                    is_hardware_reset = False
                    reset_msg = ""
                    if cur_seq is not None and last_motion_imu_seq is not None and cur_seq < last_motion_imu_seq:
                        is_hardware_reset = True
                        reset_msg = f"ESP32 sequence dropped ({last_motion_imu_seq} -> {cur_seq})"
                    elif cur_esp_ts is not None and last_motion_esp_ts is not None and cur_esp_ts < last_motion_esp_ts:
                        is_hardware_reset = True
                        reset_msg = f"ESP32 uptime reset ({last_motion_esp_ts}us -> {cur_esp_ts}us)"
                    elif cur_reset_cnt is not None and last_motion_reset_count is not None and cur_reset_cnt > last_motion_reset_count:
                        is_hardware_reset = True
                        reset_msg = f"ESP32 resetCount incremented ({last_motion_reset_count} -> {cur_reset_cnt})"

                    is_fresh, freshness_msg = check_imu_freshness(latest_imu, max_age_ms=250.0)

                    if cur_seq is not None:
                        if last_motion_imu_seq is None or cur_seq != last_motion_imu_seq:
                            last_motion_imu_seq = cur_seq
                            last_motion_imu_advance_time = now
                    elif cur_esp_ts is not None:
                        if last_motion_esp_ts is None or cur_esp_ts != last_motion_esp_ts:
                            last_motion_imu_advance_time = now
                    else:
                        if is_fresh:
                            last_motion_imu_advance_time = now

                    if cur_esp_ts is not None:
                        last_motion_esp_ts = cur_esp_ts
                    if cur_reset_cnt is not None:
                        last_motion_reset_count = cur_reset_cnt

                    is_non_advancing = (now - last_motion_imu_advance_time) > 0.250

                    if is_hardware_reset or not is_fresh or is_non_advancing:
                        # CRITICAL SAFETY INVARIANT:
                        # 1. Immediately command zero to rover over both ROS2 bridge and WebSocket.
                        # Never leave a nonzero velocity command active while sensor integrity is suspect.
                        self.cockpit.send_cmd_vel(vx=0.0, wz=0.0)
                        if self.ws and self.ws.connected:
                            try:
                                self.ws.send_drive(0.0, 0.0)
                            except Exception:
                                pass

                        # 2. Perform bounded confirmation check strictly while zero motion is commanded
                        time.sleep(0.04)
                        confirm_imu = self.cockpit.get_imu()
                        is_confirm_fresh, confirm_msg = check_imu_freshness(confirm_imu, max_age_ms=250.0)
                        confirm_seq = confirm_imu.get("sequence") if confirm_imu else None
                        confirm_esp_ts = confirm_imu.get("espTimestampUs") if confirm_imu else None
                        confirm_reset_cnt = confirm_imu.get("resetCount") if confirm_imu else None

                        if not is_hardware_reset:
                            if confirm_seq is not None and last_motion_imu_seq is not None and confirm_seq < last_motion_imu_seq:
                                is_hardware_reset = True
                                reset_msg = f"ESP32 sequence dropped ({last_motion_imu_seq} -> {confirm_seq})"
                            elif confirm_esp_ts is not None and last_motion_esp_ts is not None and confirm_esp_ts < last_motion_esp_ts:
                                is_hardware_reset = True
                                reset_msg = f"ESP32 uptime reset ({last_motion_esp_ts}us -> {confirm_esp_ts}us)"
                            elif confirm_reset_cnt is not None and last_motion_reset_count is not None and confirm_reset_cnt > last_motion_reset_count:
                                is_hardware_reset = True
                                reset_msg = f"ESP32 resetCount incremented ({last_motion_reset_count} -> {confirm_reset_cnt})"

                        confirm_advancing = True
                        if confirm_seq is not None and last_motion_imu_seq is not None:
                            if confirm_seq == last_motion_imu_seq and (time.time() - last_motion_imu_advance_time) > 0.250:
                                confirm_advancing = False

                        # 3. Always abort rather than automatically resume once zero motion is commanded
                        if is_hardware_reset:
                            abort_reason = f"ESP32 hardware reset detected during motion: {reset_msg}"
                            trial_report.watchdog_trips.append("ESP32_HARDWARE_RESET")
                        elif (not is_confirm_fresh) or (not confirm_advancing):
                            fail_detail = freshness_msg if not is_fresh else (confirm_msg if not is_confirm_fresh else f"Non-advancing IMU sequence ({last_motion_imu_seq}) for {(time.time() - last_motion_imu_advance_time)*1000:.0f}ms")
                            abort_reason = f"Confirmed stale/non-advancing IMU data: {fail_detail}"
                            trial_report.watchdog_trips.append("STALE_IMU_DATA")
                        else:
                            abort_reason = f"Transient stale IMU reading ({freshness_msg}); stopped immediately for safety, aborted without auto-resume"
                            trial_report.watchdog_trips.append("TRANSIENT_IMU_SAFE_STOP")
                        break

                    cur_raw_yaw = quat_to_yaw(latest_imu.get("orientation", {}))
                    unwrapper.update_orientation_yaw(cur_raw_yaw)
                    cur_rel_yaw = unwrapper.relative_yaw_deg

                    # Max angle safety guard
                    if abs(cur_rel_yaw) > max_angle_limit_deg:
                        abort_reason = f"Hard angle limit exceeded: {abs(cur_rel_yaw):.1f}° > {max_angle_limit_deg:.1f}°"
                        trial_report.watchdog_trips.append("ANGLE_LIMIT_EXCEEDED")
                        break

                    # Controller update
                    cmd_wz = controller.update(current_progress=cur_rel_yaw, current_time=now)
                    wheel_cmds = compute_wheel_speed_targets(cmd_wz)

                    # Verify symmetric opposite commands
                    sym_ok, sym_msg = verify_wheel_command_symmetry(wheel_cmds)
                    if not sym_ok:
                        trial_report.opposite_polarity_maintained = False
                        abort_reason = f"Command symmetry violation: {sym_msg}"
                        break

                    # Check completion (zero command issued)
                    # When target is reached, command zero and exit active motion immediately.
                    # This ensures zero-command and settling samples are NEVER mixed into active motion statistics.
                    if controller.phase == ApproachPhase.ZERO:
                        angle_at_zero_cmd = cur_rel_yaw
                        t_send_mono = time.monotonic()
                        zero_send_wall_ms = int(time.time() * 1000)
                        zero_cmd_res = self.cockpit.send_cmd_vel(vx=0.0, wz=0.0)
                        t_recv_mono = time.monotonic()

                        trial_report.zero_command_send_time_monotonic = t_send_mono
                        trial_report.zero_command_response_time_monotonic = t_recv_mono
                        trial_report.zero_command_latency_ms = (t_recv_mono - t_send_mono) * 1000.0
                        trial_report.zero_command_response = zero_cmd_res if isinstance(zero_cmd_res, dict) else {"ok": False, "raw": str(zero_cmd_res)}
                        break

                    # Stream command to rover via ROS2 bridge
                    cmd_res = self.cockpit.send_cmd_vel(vx=0.0, wz=cmd_wz)
                    if not cmd_res.get("ok", False):
                        abort_reason = f"cmd_vel transmission failed: {cmd_res.get('error')}"
                        trial_report.communication_faults.append("CMD_VEL_SEND_FAILED")
                        break

                    # Ingest PID diagnostics from received WebSocket frames and monitor wheel stalls / breakouts
                    # ONLY collected while controller is in active motion (CRUISE or CREEP)
                    for f in ws_frames:
                        if f.get("type") == "pid_diagnostic":
                            total_telemetry_samples += 1
                            act_st = f.get("actuationState")
                            if act_st and act_st not in trial_report.actuation_states_observed:
                                trial_report.actuation_states_observed.append(act_st)

                            current_phase = controller.phase.name if hasattr(controller, "phase") else "ACTIVE"
                            m1_f = f.get("m1", {})
                            m2_f = f.get("m2", {})
                            m3_f = f.get("m3", {})
                            m4_f = f.get("m4", {})

                            v1_val = m1_f.get("measuredRadps")
                            v2_val = m2_f.get("measuredRadps")
                            v3_val = m3_f.get("measuredRadps")
                            v4_val = m4_f.get("measuredRadps")

                            p_err_l = (abs(float(v1_val)) - abs(float(v3_val))) if (v1_val is not None and v3_val is not None) else None
                            p_err_r = (abs(float(v2_val)) - abs(float(v4_val))) if (v2_val is not None and v4_val is not None) else None

                            active_pid_packets.append({
                                "host_receipt_time_monotonic": round(time.monotonic(), 6),
                                "t_rel_s": round(now - t_motion_start, 4),
                                "source_timestamp_ms": f.get("timestamp"),
                                "source_sequence": f.get("sequence"),
                                "phase": current_phase,
                                "actuationState": act_st,
                                "pair_error_left_radps": round(p_err_l, 3) if p_err_l is not None else None,
                                "pair_error_right_radps": round(p_err_r, 3) if p_err_r is not None else None,
                                "m1": m1_f,
                                "m2": m2_f,
                                "m3": m3_f,
                                "m4": m4_f,
                                "outerYaw": f.get("outerYaw")
                            })
                            for w_id in ["m1", "m2", "m3", "m4"]:
                                w_data = f.get(w_id, {})
                                tgt = w_data.get("targetRadps")
                                meas = w_data.get("measuredRadps")
                                stiction_state = w_data.get("stictionState", "IDLE")

                                if tgt is not None and isinstance(tgt, (int, float)):
                                    wheel_samples[w_id]["targets"].append(float(tgt))
                                if meas is not None and isinstance(meas, (int, float)):
                                    wheel_samples[w_id]["measured"].append(float(meas))

                                if stiction_state == "STICTION_BOOST":
                                    wheel_metrics[w_id].stiction_boost_events += 1
                                    if breakout_start_time is None:
                                        breakout_start_time = now
                                        breakout_count += 1
                                elif stiction_state == "BLOCKED":
                                    wheel_metrics[w_id].blocked_state_events += 1
                                    stalls_detected[w_id] = True

                                # Stopped-while-commanded monitoring:
                                tgt_val = float(tgt) if (tgt is not None and isinstance(tgt, (int, float))) else abs(wheel_cmds.get(w_id, 0.0))
                                meas_val = float(meas) if (meas is not None and isinstance(meas, (int, float))) else 0.0
                                is_stopped = (abs(tgt_val) > 0.10) and (abs(meas_val) < 0.05 or stiction_state == "BLOCKED")

                                ws_buf = wheel_samples[w_id]
                                if is_stopped:
                                    stalls_detected[w_id] = True
                                    if not ws_buf["is_currently_stopped"]:
                                        ws_buf["is_currently_stopped"] = True
                                        ws_buf["stopped_entry_time"] = now
                                        ws_buf["stopped_events"] += 1
                                    else:
                                        # Continuous stopped-while-commanded dwell check (fail-fast safeguard)
                                        stopped_dwell = now - (ws_buf["stopped_entry_time"] or now)
                                        if stopped_dwell >= 1.5:  # Reacts in 1.5s, well before 8 seconds
                                            abort_reason = (
                                                f"Fail-fast safeguard triggered: wheel {w_id} stopped while commanded "
                                                f"for {stopped_dwell:.2f}s (target={tgt_val:.2f} rad/s, meas={meas_val:.2f} rad/s)"
                                            )
                                            trial_report.watchdog_trips.append("WHEEL_STALL_FAILSAFE")
                                            break
                                else:
                                    if ws_buf["is_currently_stopped"]:
                                        dwell = max(0.02, now - (ws_buf["stopped_entry_time"] or now))
                                        ws_buf["stopped_duration_s"] += dwell
                                        ws_buf["is_currently_stopped"] = False
                                        ws_buf["stopped_entry_time"] = None

                            if abort_reason:
                                break

                    if abort_reason:
                        break

                    if breakout_start_time and any(f.get(w, {}).get("stictionState") != "STICTION_BOOST" for w in ["m1", "m2", "m3", "m4"] for f in ws_frames if f.get("type") == "pid_diagnostic"):
                        total_breakout_dwell_ms += (now - breakout_start_time) * 1000.0
                        breakout_start_time = None

                    time.sleep(0.02)  # 50Hz control loop

        except Exception as e:
            abort_reason = f"[{type(e).__name__}] Exception during motion execution: {e}"

        # Close any open stopped_while_commanded periods
        now_post = time.time()
        for w_id in ["m1", "m2", "m3", "m4"]:
            ws_buf = wheel_samples[w_id]
            if ws_buf["is_currently_stopped"] and ws_buf["stopped_entry_time"]:
                dwell = max(0.02, now_post - ws_buf["stopped_entry_time"])
                ws_buf["stopped_duration_s"] += dwell
                ws_buf["is_currently_stopped"] = False

        # Aggregate active-motion wheel metrics (strictly before settle period)
        for w_id in ["m1", "m2", "m3", "m4"]:
            w_metric = wheel_metrics[w_id]
            ws_buf = wheel_samples[w_id]
            targets = ws_buf["targets"]
            measured = ws_buf["measured"]
            w_metric.active_samples_count = len(measured)

            if len(targets) > 0:
                w_metric.commanded_speed_radps_mean = sum(targets) / len(targets)
                w_metric.commanded_speed_radps_max = max(targets, key=abs)
                w_metric.commanded_speed_source = "calculated_expected" if self.params.dry_run else "firmware_pid"
            else:
                w_metric.commanded_speed_radps_mean = None
                w_metric.commanded_speed_radps_max = None
                w_metric.commanded_speed_source = "unavailable"

            if len(measured) > 0:
                w_metric.measured_speed_radps_mean = sum(measured) / len(measured)
                w_metric.measured_speed_radps_abs_mean = sum(abs(v) for v in measured) / len(measured)
                w_metric.measured_speed_radps_min = min(measured)
                w_metric.measured_speed_radps_max = max(measured)
            else:
                w_metric.measured_speed_radps_mean = None
                w_metric.measured_speed_radps_abs_mean = None
                w_metric.measured_speed_radps_min = None
                w_metric.measured_speed_radps_max = None

            observed_nonzero_target = any(abs(t) >= 0.05 for t in targets)
            if observed_nonzero_target:
                w_metric.stopped_while_commanded = bool(stalls_detected[w_id] or (ws_buf["stopped_events"] > 0))
                w_metric.stopped_while_commanded_count = ws_buf["stopped_events"]
                w_metric.stopped_while_commanded_duration_s = ws_buf["stopped_duration_s"]
            else:
                w_metric.stopped_while_commanded = None
                w_metric.stopped_while_commanded_count = 0
                w_metric.stopped_while_commanded_duration_s = 0.0

        trial_report.telemetry_samples_count = total_telemetry_samples
        trial_report.wheel_metrics = wheel_metrics

        # If motion loop aborted, execute self.cleanup() immediately and return
        if abort_reason:
            enc_final = self.cockpit.get_encoders().get("encoders", enc_start)
            for w_id in ["m1", "m2", "m3", "m4"]:
                w_metric = wheel_metrics[w_id]
                w_metric.encoder_final_ticks = enc_final.get(w_id, w_metric.encoder_start_ticks)
                w_metric.encoder_delta_ticks = w_metric.encoder_final_ticks - w_metric.encoder_start_ticks
            self._validate_wheel_forensics_consistency(trial_report, wheel_metrics, wheel_samples)
            trial_report.status = "ABORTED"
            trial_report.abort_reason = abort_reason
            trial_report.breakout_event_count = breakout_count
            trial_report.breakout_dwell_time_ms_total = total_breakout_dwell_ms
            trial_report.active_pid_packets = active_pid_packets
            print(f"[TRIAL ABORTED] {abort_reason}")
            cleanup_st = self.cleanup()
            trial_report.confirmed_final_zero_command = True
            trial_report.confirmed_final_disarmed_state = (
                cleanup_st.get("armed") is False
                and cleanup_st.get("autonomyState") == "DISABLED"
                and cleanup_st.get("cmdSource") in ("NONE", None)
            )
            trial_report.autonomy_state_transitions.append({
                "timestamp": time.time(),
                "state": cleanup_st.get("autonomyState", "DISABLED"),
                "trigger": "motion_abort_cleanup"
            })
            return trial_report

        # Step 12: Settle Period - zero command was issued upon target arrival
        print(f"  Settling for {self.params.settle_seconds:.1f}s...")
        t_settle_start_mono = time.monotonic()
        t_settle_target_end_mono = t_settle_start_mono + max(0.0, self.params.settle_seconds)

        settle_imu_samples: List[Dict[str, Any]] = []
        settle_pid_packets: List[Dict[str, Any]] = []
        last_advancing_imu_ts: Optional[int] = None
        last_advancing_imu_seq: Optional[int] = None

        final_measurement_valid = False
        final_settled_yaw: Optional[float] = None
        post_zero_coast: Optional[float] = None
        settled_heading_error: Optional[float] = None

        if self.params.dry_run:
            # Restore full requested dry-run settling duration
            time.sleep(self.params.settle_seconds)
            actual_settle_duration_s = max(0.001, time.monotonic() - t_settle_start_mono)
            final_settled_yaw = angle_at_zero_cmd + (math.copysign(self.params.stopping_advance_deg or 0.0, self.params.signed_target_deg) if self.params.enable_braking else 0.0)
            post_zero_coast = final_settled_yaw - angle_at_zero_cmd
            settled_heading_error = final_settled_yaw - self.params.signed_target_deg
            final_measurement_valid = True
            controller.mark_settled(final_settled_yaw)
            trial_report.settle_duration_s = round(actual_settle_duration_s, 4)
        else:
            # Active bounded sampling loop throughout the settling interval
            while True:
                remaining_budget = t_settle_target_end_mono - time.monotonic()
                if remaining_budget <= 0.0:
                    break

                t_iter_start_mono = time.monotonic()

                # 1. Bounded WebSocket ingestion: preserve each PID diagnostic packet individually
                if self.ws and self.ws.connected:
                    ws_timeout = min(0.005, remaining_budget)
                    try:
                        ws_frames = self.ws.recv_frames(timeout=ws_timeout)
                    except StopIteration:
                        ws_frames = []
                    except Exception:
                        ws_frames = []

                    for f in ws_frames:
                        # Enforce time budget during backlog frame processing
                        if time.monotonic() >= t_settle_target_end_mono:
                            break

                        if f.get("type") == "pid_diagnostic":
                            t_recv_mono = time.monotonic()
                            src_ts = f.get("timestamp")
                            act_st = f.get("actuationState")
                            if act_st and act_st not in trial_report.actuation_states_observed:
                                trial_report.actuation_states_observed.append(act_st)
                            if src_ts is None:
                                is_backlog = None
                                backlog_status = "UNKNOWN"
                            elif zero_send_wall_ms is not None:
                                is_backlog = (src_ts < zero_send_wall_ms)
                                backlog_status = "BACKLOG" if is_backlog else "POST_ZERO"
                            else:
                                is_backlog = None
                                backlog_status = "UNKNOWN"

                            packet_record = {
                                "host_receipt_time_monotonic": round(t_recv_mono, 6),
                                "host_clock_domain": "host_monotonic",
                                "t_rel_s": round(t_recv_mono - t_settle_start_mono, 4),
                                "source_timestamp_ms": src_ts,
                                "source_timestamp_unit": "ms" if src_ts is not None else None,
                                "source_clock_domain": "bridge_timestamp_ms" if src_ts is not None else None,
                                "source_sequence": f.get("sequence"),
                                "actuationState": act_st,
                                "is_pre_zero_backlog": is_backlog,
                                "backlog_status": backlog_status,
                                "m1": f.get("m1", {}),
                                "m2": f.get("m2", {}),
                                "m3": f.get("m3", {}),
                                "m4": f.get("m4", {}),
                                "outerYaw": f.get("outerYaw")
                            }
                            settle_pid_packets.append(packet_record)

                # 2. Bounded IMU sample: check time budget before and within call
                remaining_budget = t_settle_target_end_mono - time.monotonic()
                if remaining_budget <= 0.0:
                    break

                imu_timeout = min(0.05, remaining_budget)
                t_req_start_mono = time.monotonic()
                try:
                    imu_snap = self.cockpit.get_imu(timeout=imu_timeout)
                except StopIteration:
                    break
                except Exception as e:
                    imu_snap = {"ok": False, "error": str(e)}
                t_imu_recv_mono = time.monotonic()

                imu_record: Dict[str, Any] = {
                    "host_request_start_time_monotonic": round(t_req_start_mono, 6),
                    "host_receipt_time_monotonic": round(t_imu_recv_mono, 6),
                    "host_clock_domain": "host_monotonic",
                    "t_rel_s": round(t_imu_recv_mono - t_settle_start_mono, 4),
                    "valid": False,
                    "status": "MISSING",
                    "advancement": "UNKNOWN",
                    "yaw_deg": None,
                    "yaw_units": "deg",
                    "raw_yaw_deg": None,
                    "age_ms": None,
                    "source_timestamp": None,
                    "source_timestamp_unit": None,
                    "source_clock_domain": None,
                    "source_sequence": None,
                    "error": None
                }

                if not imu_snap or not imu_snap.get("ok", False):
                    imu_record["status"] = "MISSING"
                    imu_record["error"] = imu_snap.get("error") if isinstance(imu_snap, dict) and imu_snap.get("error") else "IMU endpoint returned not ok or null"
                else:
                    orientation = imu_snap.get("orientation")
                    age_ms = imu_snap.get("dataAgeMs") if "dataAgeMs" in imu_snap else imu_snap.get("age_ms")
                    src_ts = imu_snap.get("timestamp_ms") if "timestamp_ms" in imu_snap else (imu_snap.get("espTimestampUs") or imu_snap.get("timestamp"))
                    src_ts_unit = "us" if "espTimestampUs" in imu_snap else ("ms" if src_ts is not None else None)
                    src_clock = "esp32_boot_us" if "espTimestampUs" in imu_snap else ("bridge_timestamp_ms" if "timestamp_ms" in imu_snap else ("unknown" if src_ts is not None else None))
                    src_seq = imu_snap.get("sequence")

                    imu_record["age_ms"] = age_ms
                    imu_record["source_timestamp"] = src_ts
                    imu_record["source_timestamp_unit"] = src_ts_unit
                    imu_record["source_clock_domain"] = src_clock
                    imu_record["source_sequence"] = src_seq

                    if not orientation or not isinstance(orientation, dict) or not all(k in orientation for k in ("x", "y", "z", "w")):
                        imu_record["status"] = "INVALID_ORIENTATION"
                        imu_record["error"] = "Orientation missing x, y, z, or w"
                    else:
                        is_fresh, freshness_msg = check_imu_freshness(imu_snap, max_age_ms=250.0)
                        if not is_fresh:
                            imu_record["status"] = "STALE"
                            imu_record["error"] = freshness_msg
                        else:
                            # Freshness verified. Check sample advancement:
                            has_sample_id = (src_seq is not None or src_ts is not None)
                            raw_yaw_rad = quat_to_yaw(orientation)
                            imu_record["raw_yaw_deg"] = round(math.degrees(raw_yaw_rad), 4)

                            if not has_sample_id:
                                imu_record["status"] = "UNKNOWN_ADVANCEMENT"
                                imu_record["advancement"] = "UNKNOWN"
                                imu_record["valid"] = False
                                imu_record["error"] = "Missing sequence and timestamp identifiers; advancement unknown"
                                imu_record["yaw_deg"] = None
                            else:
                                is_advanced = True
                                if src_seq is not None and last_advancing_imu_seq is not None:
                                    is_advanced = (src_seq > last_advancing_imu_seq)
                                elif src_ts is not None and last_advancing_imu_ts is not None:
                                    is_advanced = (src_ts > last_advancing_imu_ts)

                                if not is_advanced:
                                    imu_record["status"] = "NOT_ADVANCING"
                                    imu_record["advancement"] = "NOT_ADVANCING"
                                    imu_record["valid"] = False
                                    imu_record["yaw_deg"] = None
                                else:
                                    imu_record["status"] = "VALID_ADVANCING"
                                    imu_record["advancement"] = "ADVANCING"
                                    imu_record["valid"] = True
                                    unwrapper.update_orientation_yaw(raw_yaw_rad)
                                    imu_record["yaw_deg"] = round(unwrapper.relative_yaw_deg, 4)
                                    if src_ts is not None:
                                        last_advancing_imu_ts = src_ts
                                    if src_seq is not None:
                                        last_advancing_imu_seq = src_seq

                settle_imu_samples.append(imu_record)

                # Bounded iteration pacing targeting ~20ms cadence
                remaining_after = t_settle_target_end_mono - time.monotonic()
                if remaining_after <= 0.0:
                    break
                iteration_elapsed = time.monotonic() - t_iter_start_mono
                time.sleep(min(remaining_after, max(0.002, 0.02 - iteration_elapsed)))

            actual_settle_duration_s = max(0.001, time.monotonic() - t_settle_start_mono)
            imu_poll_count = len(settle_imu_samples)
            imu_valid_advancing_count = len([s for s in settle_imu_samples if s.get("status") == "VALID_ADVANCING"])
            pid_packets_count = len(settle_pid_packets)
            pid_post_zero_count = len([p for p in settle_pid_packets if p.get("backlog_status") == "POST_ZERO"])
            pid_backlog_count = len([p for p in settle_pid_packets if p.get("backlog_status") == "BACKLOG"])
            pid_unknown_count = len([p for p in settle_pid_packets if p.get("backlog_status") == "UNKNOWN"])

            trial_report.settle_duration_s = round(actual_settle_duration_s, 4)
            trial_report.settle_imu_poll_count = imu_poll_count
            trial_report.settle_imu_valid_advancing_count = imu_valid_advancing_count
            trial_report.settle_imu_poll_rate_hz = round(imu_poll_count / actual_settle_duration_s, 2)
            trial_report.settle_imu_valid_advancing_rate_hz = round(imu_valid_advancing_count / actual_settle_duration_s, 2)
            trial_report.settle_achieved_imu_rate_hz = trial_report.settle_imu_valid_advancing_rate_hz

            trial_report.settle_pid_packets_count = pid_packets_count
            trial_report.settle_pid_post_zero_packets_count = pid_post_zero_count
            trial_report.settle_pid_backlog_packets_count = pid_backlog_count
            trial_report.settle_pid_unknown_packets_count = pid_unknown_count
            trial_report.settle_pid_packet_rate_hz = round(pid_packets_count / actual_settle_duration_s, 2)
            trial_report.settle_pid_post_zero_packet_rate_hz = round(pid_post_zero_count / actual_settle_duration_s, 2)
            trial_report.settle_achieved_pid_rate_hz = trial_report.settle_pid_post_zero_packet_rate_hz
            trial_report.settle_imu_samples = settle_imu_samples
            trial_report.settle_pid_packets = settle_pid_packets
            trial_report.active_pid_packets = active_pid_packets

            # Calculate brake pulse duration if observed
            brake_packets = [p for p in settle_pid_packets if p.get("actuationState") == "BRAKE"]
            if brake_packets:
                first_ts = brake_packets[0].get("source_timestamp_ms")
                last_ts = brake_packets[-1].get("source_timestamp_ms")
                if first_ts is not None and last_ts is not None and last_ts >= first_ts:
                    trial_report.brake_active_duration_ms = round(float(last_ts - first_ts + 20.0), 1)
                else:
                    dt = (brake_packets[-1]["host_receipt_time_monotonic"] - brake_packets[0]["host_receipt_time_monotonic"]) * 1000.0
                    trial_report.brake_active_duration_ms = round(dt + 20.0, 1)

            # Determine last settle sample identifiers
            last_settle_seq = None
            last_settle_ts = None
            for s in reversed(settle_imu_samples):
                if last_settle_seq is None and s.get("source_sequence") is not None:
                    last_settle_seq = s.get("source_sequence")
                if last_settle_ts is None and s.get("source_timestamp") is not None:
                    last_settle_ts = s.get("source_timestamp")
                if last_settle_seq is not None and last_settle_ts is not None:
                    break

            # Bounded final endpoint measurement
            # Must advance beyond the last settling sample within the bounded read (timeout budget up to 0.25s)
            t_endpoint_start = time.monotonic()
            endpoint_deadline = t_endpoint_start + 0.25
            while time.monotonic() < endpoint_deadline:
                rem_endpoint = endpoint_deadline - time.monotonic()
                if rem_endpoint <= 0.0:
                    break
                try:
                    final_endpoint_imu = self.cockpit.get_imu(timeout=min(0.05, max(0.01, rem_endpoint)))
                except Exception as e:
                    final_endpoint_imu = {"ok": False, "error": str(e)}
                    break

                if final_endpoint_imu and final_endpoint_imu.get("ok", False):
                    ori = final_endpoint_imu.get("orientation")
                    if ori and isinstance(ori, dict) and all(k in ori for k in ("x", "y", "z", "w")):
                        is_fresh, _ = check_imu_freshness(final_endpoint_imu, max_age_ms=250.0)
                        final_seq = final_endpoint_imu.get("sequence")
                        final_ts = final_endpoint_imu.get("timestamp_ms") if "timestamp_ms" in final_endpoint_imu else (final_endpoint_imu.get("espTimestampUs") or final_endpoint_imu.get("timestamp"))

                        is_endpoint_advancing = False
                        if final_seq is not None and last_settle_seq is not None:
                            is_endpoint_advancing = (final_seq > last_settle_seq)
                        elif final_ts is not None and last_settle_ts is not None:
                            is_endpoint_advancing = (final_ts > last_settle_ts)
                        elif last_settle_seq is None and last_settle_ts is None and (final_seq is not None or final_ts is not None):
                            is_endpoint_advancing = True

                        if is_fresh and is_endpoint_advancing:
                            final_raw_yaw = quat_to_yaw(ori)
                            unwrapper.update_orientation_yaw(final_raw_yaw)
                            final_settled_yaw = unwrapper.relative_yaw_deg
                            post_zero_coast = final_settled_yaw - angle_at_zero_cmd
                            settled_heading_error = final_settled_yaw - self.params.signed_target_deg
                            final_measurement_valid = True
                            controller.mark_settled(final_settled_yaw)
                            break
                time.sleep(0.01)

        trial_report.final_measurement_valid = final_measurement_valid
        trial_report.gyro_angle_at_zero_cmd_deg = round(angle_at_zero_cmd, 4)
        trial_report.final_settled_gyro_angle_deg = round(final_settled_yaw, 4) if final_settled_yaw is not None else None
        trial_report.post_zero_rotation_deg = round(post_zero_coast, 4) if post_zero_coast is not None else None
        trial_report.settled_heading_error_deg = round(settled_heading_error, 4) if settled_heading_error is not None else None

        raw_start = trial_report.start_heading_raw_deg if trial_report.start_heading_raw_deg is not None else 0.0
        cont_start = trial_report.start_heading_continuous_deg if trial_report.start_heading_continuous_deg is not None else raw_start

        if final_settled_yaw is not None:
            trial_report.final_settled_raw_heading_deg = round(normalize_angle_deg(raw_start + final_settled_yaw), 4)
            trial_report.final_settled_continuous_heading_deg = round(cont_start + final_settled_yaw, 4)

        # Step 13b: Construct phase heading breakdown
        phase_records = []
        history = [h for h in controller.phase_history]
        for i in range(len(history)):
            entry = history[i]
            p_name = entry.get("phase", "").replace("ApproachPhase.", "")
            if p_name == "SETTLED":
                continue

            t_entry = entry.get("t_rel", 0.0)
            yaw_entry = entry.get("yaw_deg", 0.0)

            if i + 1 < len(history):
                next_entry = history[i + 1]
                t_exit = next_entry.get("t_rel", t_entry)
                yaw_exit = next_entry.get("yaw_deg", yaw_entry)
            else:
                t_exit = time.time() - t_trial_start
                yaw_exit = final_settled_yaw if final_settled_yaw is not None else angle_at_zero_cmd

            dur = max(0.0, t_exit - t_entry)
            delta = yaw_exit - yaw_entry
            s_abs = normalize_angle_deg(raw_start + yaw_entry)
            e_abs = normalize_angle_deg(raw_start + yaw_exit)
            s_cont = cont_start + yaw_entry
            e_cont = cont_start + yaw_exit

            phase_label = p_name
            if p_name == "ZERO":
                phase_label = "BRAKE_COAST"

            phase_records.append({
                "phase_name": phase_label,
                "start_abs_deg": round(s_abs, 4),
                "end_abs_deg": round(e_abs, 4),
                "start_rel_deg": round(yaw_entry, 4),
                "end_rel_deg": round(yaw_exit, 4),
                "start_continuous_deg": round(s_cont, 4),
                "end_continuous_deg": round(e_cont, 4),
                "delta_deg": round(delta, 4),
                "duration_s": round(dur, 4)
            })
        trial_report.phase_headings = phase_records

        # Capture final encoders
        enc_final = self.cockpit.get_encoders().get("encoders", enc_start)
        for w_id in ["m1", "m2", "m3", "m4"]:
            w_metric = wheel_metrics[w_id]
            w_metric.encoder_final_ticks = enc_final.get(w_id, w_metric.encoder_start_ticks)
            w_metric.encoder_delta_ticks = w_metric.encoder_final_ticks - w_metric.encoder_start_ticks

        self._validate_wheel_forensics_consistency(trial_report, wheel_metrics, wheel_samples)

        # Execute self.cleanup() ONLY after the target and settling sequence completes!
        cleanup_st = self.cleanup()
        trial_report.confirmed_final_zero_command = True
        trial_report.confirmed_final_disarmed_state = (
            cleanup_st.get("armed") is False
            and cleanup_st.get("autonomyState") == "DISABLED"
            and cleanup_st.get("cmdSource") in ("NONE", None)
        )
        trial_report.autonomy_state_transitions.append({
            "timestamp": time.time(),
            "state": cleanup_st.get("autonomyState", "DISABLED"),
            "trigger": "trial_settle_complete_cleanup"
        })

        # Populate report metrics
        trial_report.duration_s = time.time() - t_trial_start
        trial_report.wheel_metrics = wheel_metrics
        trial_report.breakout_event_count = breakout_count
        trial_report.breakout_dwell_time_ms_total = total_breakout_dwell_ms
        trial_report.approach_milestones = controller.get_telemetry_summary()
        trial_report.active_pid_packets = active_pid_packets
        trial_report.status = "DRY_RUN_PASSED" if self.params.dry_run else "SUCCESS"

        print(f"[OK] Step {trial_idx}/{self.params.trials} Completed ({trial_report.status})")
        st_head_str = f"{trial_report.start_heading_raw_deg:+.2f}°" if trial_report.start_heading_raw_deg is not None else "N/A"
        end_head_str = f"{trial_report.final_settled_raw_heading_deg:+.2f}°" if trial_report.final_settled_raw_heading_deg is not None else "N/A"
        final_str = f"{final_settled_yaw:+.2f}°" if final_settled_yaw is not None else "Unavailable"
        coast_str = f"{post_zero_coast:+.2f}°" if post_zero_coast is not None else "Unavailable"
        err_str = f"{settled_heading_error:+.2f}°" if settled_heading_error is not None else "Unavailable"
        print(f"  • Step Heading:      Start: {st_head_str} -> End: {end_head_str} (Net: {final_str}, Target: {self.params.signed_target_deg:+.2f}°, Error: {err_str})")
        if trial_report.phase_headings:
            print("  • Phase Breakdown:")
            for ph in trial_report.phase_headings:
                ph_name = ph.get("phase_name", "")
                p_st = f"{ph.get('start_abs_deg', 0.0):+.2f}°"
                p_end = f"{ph.get('end_abs_deg', 0.0):+.2f}°"
                p_del = f"{ph.get('delta_deg', 0.0):+.2f}°"
                print(f"    - {ph_name:<11}: {p_st} -> {p_end} (Delta: {p_del}, {ph.get('duration_s', 0.0):.2f}s)")
        print(f"  • Angle at Zero Cmd: {angle_at_zero_cmd:+.2f}°")
        print(f"  • Post-Zero Coast:   {coast_str}")
        if trial_report.final_settled_continuous_heading_deg is not None:
            print(f"  • Continuous Angle:  {trial_report.final_settled_continuous_heading_deg:+.2f}°")

        # Ingest Ron's Physical Estimate
        if self.params.inter_trial_approval and not self.params.dry_run:
            print("\n[PHYSICAL ESTIMATE COLLECTION]")
            print(f"  Gyro Measured Settled Angle: {final_str}")
            est_val, est_delta = ingest_physical_estimate(
                prompt_fn=self.prompt_fn,
                target_degrees=self.params.degrees,
                direction=self.params.direction,
                final_settled_yaw=final_settled_yaw
            )
            trial_report.rons_physical_angle_estimate_deg = est_val
            trial_report.estimate_vs_gyro_delta_deg = est_delta

        return trial_report
