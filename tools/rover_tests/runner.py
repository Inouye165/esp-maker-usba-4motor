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
from typing import Optional, Dict, Any, List, Callable

if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

from .turn import TurnParameters, compute_wheel_speed_targets, verify_wheel_command_symmetry
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
    YawUnwrapper,
    BiasCorrectedGyroIntegrator,
    verify_non_magnetic_imu,
    check_imu_freshness,
    wait_for_advancing_imu_sample,
    MagneticImuException,
    SensorException
)
from .controllers import AngularApproachController, ApproachPhase
from .reporting import (
    TrialReport,
    WheelTrialMetrics,
    MultiTrialSuiteReport,
    ReportGenerator
)


class TestAbortException(Exception):
    """Raised when a safety guard or watchdog aborts a trial."""
    pass


class PhysicalTestRunner:
    """
    Orchestrates physical test execution, safety monitoring, and report compilation.
    """
    def __init__(
        self,
        params: TurnParameters,
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

        # Always restore drive configuration to baseline (false/false) on disarm/cleanup
        if not self.params.dry_run:
            try:
                self.cockpit.configure_drive(wheel_balancing=False, dynamic_braking=False)
            except Exception:
                pass

        confirmed = (
            cleanup_st.get("armed") is False
            and cleanup_st.get("autonomyState") == "DISABLED"
            and cleanup_st.get("cmdSource") in ("NONE", None)
        )
        self._cleaned_up = confirmed
        return cleanup_st

    def execute_suite(self) -> MultiTrialSuiteReport:
        """Executes the full suite of trials."""
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

        suite_report = MultiTrialSuiteReport(
            suite_id=suite_id,
            test_type="turn",
            command_line=cmd_str,
            timestamp_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            target_degrees=self.params.degrees,
            direction=self.params.direction,
            total_trials=self.params.trials,
            is_dry_run=self.params.dry_run,
            wheel_balancing_enabled=self.params.enable_balancing,
            dynamic_braking_enabled=self.params.enable_braking,
            stopping_advance_deg=self.params.stopping_advance_deg or 0.0
        )

        print("=" * 80)
        print(f"ROVER ONE REUSABLE PHYSICAL TEST FRAMEWORK: TURN {self.params.degrees:.1f}° {self.params.direction.upper()}")
        print(f"Trials: {self.params.trials} | Max Speed: {self.params.max_angular_speed:.2f} rad/s | Creep: {self.params.creep_angular_speed:.2f} rad/s")
        print(f"Balancing: {'ENABLED' if self.params.enable_balancing else 'DISABLED'} | Braking: {'ENABLED' if self.params.enable_braking else 'DISABLED'} | Stopping Advance: {self.params.stopping_advance_deg or 0.0:.2f}°")
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
                trial_report = self.execute_single_trial(trial_idx)
                suite_report.trials.append(trial_report)
                if trial_report.status in ("SUCCESS", "DRY_RUN_PASSED"):
                    suite_report.successful_trials += 1
                else:
                    suite_report.aborted_trials += 1

                # If aborted and not dry run, do not automatically retry without operator intervention
                if trial_report.status == "ABORTED" and not self.params.dry_run:
                    print(f"\n[SAFETY STOP] Trial {trial_idx} aborted. Routine halted for forensic preservation.")
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

        # Generate output files
        json_path = ReportGenerator.save_json(suite_report, self.params.report_directory)
        md_summary = ReportGenerator.format_markdown_summary(suite_report)
        print("\n" + md_summary)
        print(f"\n[REPORT SAVED] Full JSON telemetry report: {json_path}")
        return suite_report

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

        if not enc_stat or (not enc_stat.get("ok", False) and "encoders" not in enc_stat):
            return "Pre-motion invariant violated: encoder telemetry dropped or unreadable"

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

    def execute_single_trial(self, trial_idx: int) -> TrialReport:
        """Executes one physical turn trial adhering to all exact-motion invariants."""
        print(f"\n--- Starting Trial {trial_idx}/{self.params.trials} ---")

        # 1. Inter-Trial Operator Approval
        if self.params.inter_trial_approval or not self.params.dry_run:
            print("\n[OPERATOR APPROVAL REQUIRED]")
            print(f"  • Target Maneuver: {self.params.degrees:.1f}° {self.params.direction.upper()}")
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
            wheel_balancing_enabled=self.params.enable_balancing,
            dynamic_braking_enabled=self.params.enable_braking,
            stopping_advance_deg=self.params.stopping_advance_deg or 0.0
        )

        t_trial_start = time.time()

        # Step 2b: Configure drive parameters (wheel balancing & dynamic braking) via Cockpit
        if not self.params.dry_run:
            try:
                self.cockpit.configure_drive(
                    wheel_balancing=self.params.enable_balancing,
                    dynamic_braking=self.params.enable_braking
                )
            except Exception as e:
                print(f"[WARN] Failed to configure drive options via Cockpit: {e}")

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

                while True:
                    now = time.time()
                    dt_motion = now - t_motion_start

                    # Duration watchdog
                    if dt_motion > max_duration_sec:
                        abort_reason = f"Maximum trial duration exceeded ({max_duration_sec}s)"
                        trial_report.watchdog_trips.append("MAX_DURATION_EXCEEDED")
                        break

                    # Ingest latest IMU data
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

                    # Ingest PID diagnostics and monitor wheel stalls / breakouts
                    # ONLY collected while controller is in active motion (CRUISE or CREEP)
                    if self.ws and self.ws.connected:
                        frames = self.ws.recv_frames()
                        for f in frames:
                            if f.get("type") == "pid_diagnostic":
                                total_telemetry_samples += 1
                                act_st = f.get("actuationState")
                                if act_st and act_st not in trial_report.actuation_states_observed:
                                    trial_report.actuation_states_observed.append(act_st)
                                active_pid_packets.append({
                                    "host_receipt_time_monotonic": round(time.monotonic(), 6),
                                    "t_rel_s": round(now - t_motion_start, 4),
                                    "source_timestamp_ms": f.get("timestamp"),
                                    "source_sequence": f.get("sequence"),
                                    "actuationState": act_st,
                                    "m1": f.get("m1", {}),
                                    "m2": f.get("m2", {}),
                                    "m3": f.get("m3", {}),
                                    "m4": f.get("m4", {}),
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

                        if breakout_start_time and any(f.get(w, {}).get("stictionState") != "STICTION_BOOST" for w in ["m1", "m2", "m3", "m4"] for f in frames if f.get("type") == "pid_diagnostic"):
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

        print(f"[OK] Trial Completed ({trial_report.status})")
        print(f"  • Angle at Zero Cmd: {angle_at_zero_cmd:+.2f}°")
        final_str = f"{final_settled_yaw:+.2f}°" if final_settled_yaw is not None else "Unavailable"
        print(f"  • Final Settled Yaw: {final_str} (Target: {self.params.signed_target_deg:+.2f}°)")
        coast_str = f"{post_zero_coast:+.2f}°" if post_zero_coast is not None else "Unavailable"
        print(f"  • Post-Zero Coast:   {coast_str}")
        err_str = f"{settled_heading_error:+.2f}°" if settled_heading_error is not None else "Unavailable"
        print(f"  • Settled Error:     {err_str}")

        # Ingest Ron's Physical Estimate
        if self.params.inter_trial_approval and not self.params.dry_run:
            print("\n[PHYSICAL ESTIMATE COLLECTION]")
            print(f"  Gyro Measured Settled Angle: {final_str}")
            est_input = self.prompt_fn("Enter Ron's physical ground-truth angle estimate in degrees (or Enter to skip): ").strip()
            if est_input:
                try:
                    est_val = float(est_input)
                    trial_report.rons_physical_angle_estimate_deg = est_val
                    if final_settled_yaw is not None:
                        trial_report.estimate_vs_gyro_delta_deg = round(final_settled_yaw - est_val, 4)
                except ValueError:
                    print("  Invalid numeric input for physical estimate. Skipped.")

        return trial_report
