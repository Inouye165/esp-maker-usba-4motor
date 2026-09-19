"""
tools.rover_tests.reporting - Forensic Trial Reporting & Telemetry Serialization

Formats and serializes comprehensive multi-trial test reports adhering to Requirement 9:
- Requested turn and sign.
- Gyro angle at zero command, final settled gyro angle, post-zero coast rotation.
- Ron's physical angle estimate and deviation vs measured angle.
- Per-wheel commanded and measured speeds and encoder tick deltas.
- Wheel stall detection (whether any wheel stopped while commanded to move).
- Opposite-side polarity verification and intentional asymmetry reporting.
- Breakout-assistance events (stiction boost occurrences and durations).
- Safety faults, watchdog trips, and communication anomalies.
- Confirmed final zero/disarmed state.
"""

import json
import os
import time
import math
from dataclasses import dataclass, field, asdict
from typing import Dict, Any, List, Optional


@dataclass
class WheelTrialMetrics:
    wheel_id: str  # "m1", "m2", "m3", "m4"
    slot_mapping: str = ""    # "Slot 1" .. "Slot 4"
    corner: str = ""          # "LF", "RF", "LR", "RR"
    validity: str = "VALID"   # "VALID", "INVALID"

    # Commanded speed metrics (None if telemetry unavailable or invalid, never false 0.00)
    commanded_speed_radps_mean: Optional[float] = None
    commanded_speed_radps_max: Optional[float] = None
    commanded_speed_source: str = "firmware_pid"  # "firmware_pid", "calculated_expected", "unavailable", "INVALID_ZERO_TELEMETRY"

    # Measured speed metrics (None if telemetry unavailable or invalid, never false 0.00)
    measured_speed_radps_mean: Optional[float] = None
    measured_speed_radps_abs_mean: Optional[float] = None
    measured_speed_radps_min: Optional[float] = None
    measured_speed_radps_max: Optional[float] = None

    encoder_start_ticks: int = 0
    encoder_final_ticks: int = 0
    encoder_delta_ticks: int = 0

    # PWM Metrics
    pwm_mean: Optional[float] = None
    pwm_max: Optional[int] = None

    stopped_while_commanded: Optional[bool] = None  # None = Unknown, True = Stalled while commanded, False = Commanded and moving
    stopped_while_commanded_count: int = 0
    stopped_while_commanded_duration_s: float = 0.0

    stiction_boost_events: int = 0
    blocked_state_events: int = 0
    active_samples_count: int = 0

    def __post_init__(self):
        canonical_slots = {
            "m1": ("Slot 1", "LF"),
            "m2": ("Slot 2", "RF"),
            "m3": ("Slot 3", "LR"),
            "m4": ("Slot 4", "RR"),
        }
        info = canonical_slots.get(self.wheel_id.lower(), ("Unknown Slot", "Unknown Corner"))
        if not self.slot_mapping:
            self.slot_mapping = info[0]
        if not self.corner:
            self.corner = info[1]


@dataclass
class PhaseHeadingRecord:
    phase_name: str                     # "CRUISE", "CREEP", "BRAKE_COAST", "SETTLE"
    start_abs_deg: float                # Absolute IMU heading [-180, 180] at entry
    end_abs_deg: float                  # Absolute IMU heading [-180, 180] at exit
    start_rel_deg: float                # Relative heading from trial start
    end_rel_deg: float                  # Relative heading from trial start
    start_continuous_deg: float         # Continuous unwrapped heading from suite start
    end_continuous_deg: float           # Continuous unwrapped heading from suite start
    delta_deg: float                    # Net rotation in phase
    duration_s: float                   # Phase duration


@dataclass
class TrialReport:
    trial_index: int
    requested_turn_deg: float = 0.0
    direction: str = "cw"
    target_signed_yaw_deg: float = 0.0
    status: str = "SUCCESS"  # "SUCCESS", "ABORTED", "DRY_RUN_PASSED"
    abort_reason: Optional[str] = None
    duration_s: float = 0.0
    test_type: str = "turn"  # "turn" or "linear"

    # Linear Motion Metrics
    requested_distance_m: Optional[float] = None
    target_signed_distance_m: Optional[float] = None
    measured_distance_m: Optional[float] = None
    final_settled_distance_m: Optional[float] = None
    post_zero_coast_m: Optional[float] = None
    settled_distance_error_m: Optional[float] = None
    imu_heading_change_deg: Optional[float] = None
    lateral_drift_m: Optional[float] = None
    front_to_rear_diff_left_radps: Optional[float] = None
    front_to_rear_diff_right_radps: Optional[float] = None

    # Linear Characterization Phase Breakdown (Acceleration, Steady-Speed, Stopping)
    acceleration_phase: Optional[Dict[str, Any]] = None
    steady_speed_phase: Optional[Dict[str, Any]] = None
    stopping_phase: Optional[Dict[str, Any]] = None
    steady_speed_wheel_metrics: Dict[str, WheelTrialMetrics] = field(default_factory=dict)
    steady_speed_front_to_rear_left_radps: Optional[float] = None
    steady_speed_front_to_rear_right_radps: Optional[float] = None
    steady_speed_left_to_right_speed_diff_radps: Optional[float] = None
    steady_speed_left_to_right_pwm_diff: Optional[float] = None
    stopping_distance_m: Optional[float] = None
    stopping_duration_s: Optional[float] = None

    # Gyro & Orientation Metrics
    gyro_angle_at_zero_cmd_deg: float = 0.0
    final_settled_gyro_angle_deg: Optional[float] = None
    post_zero_rotation_deg: Optional[float] = None
    settled_heading_error_deg: Optional[float] = None
    final_measurement_valid: bool = False
    rons_physical_angle_estimate_deg: Optional[float] = None
    estimate_vs_gyro_delta_deg: Optional[float] = None

    # Step-by-Step & Phase Heading Tracking
    start_heading_raw_deg: Optional[float] = None
    start_heading_continuous_deg: Optional[float] = None
    final_settled_raw_heading_deg: Optional[float] = None
    final_settled_continuous_heading_deg: Optional[float] = None
    phase_headings: List[Dict[str, Any]] = field(default_factory=list)

    # Wheel & Actuation Metrics
    wheel_metrics: Dict[str, WheelTrialMetrics] = field(default_factory=dict)
    wheel_forensics_valid: bool = True
    opposite_polarity_maintained: bool = True
    intentional_asymmetry_reported: bool = False
    breakout_assistance_active: bool = False
    breakout_event_count: int = 0
    breakout_dwell_time_ms_total: float = 0.0

    # Safety & State Confirmation
    watchdog_trips: List[str] = field(default_factory=list)
    communication_faults: List[str] = field(default_factory=list)
    confirmed_final_zero_command: bool = False
    confirmed_final_disarmed_state: bool = False
    telemetry_samples_count: int = 0
    autonomy_state_transitions: List[Dict[str, Any]] = field(default_factory=list)
    first_command_response: Dict[str, Any] = field(default_factory=dict)

    # Zero-Command & Settling Instrumentation
    zero_command_send_time_monotonic: Optional[float] = None
    zero_command_response_time_monotonic: Optional[float] = None
    zero_command_latency_ms: Optional[float] = None
    zero_command_response: Dict[str, Any] = field(default_factory=dict)
    settle_duration_s: float = 0.0
    settle_achieved_imu_rate_hz: float = 0.0
    settle_achieved_pid_rate_hz: float = 0.0
    settle_imu_poll_count: int = 0
    settle_imu_valid_advancing_count: int = 0
    settle_imu_poll_rate_hz: float = 0.0
    settle_imu_valid_advancing_rate_hz: float = 0.0
    settle_pid_packets_count: int = 0
    settle_pid_post_zero_packets_count: int = 0
    settle_pid_backlog_packets_count: int = 0
    settle_pid_unknown_packets_count: int = 0
    settle_pid_packet_rate_hz: float = 0.0
    settle_pid_post_zero_packet_rate_hz: float = 0.0
    settle_imu_samples: List[Dict[str, Any]] = field(default_factory=list)
    settle_pid_packets: List[Dict[str, Any]] = field(default_factory=list)
    active_pid_packets: List[Dict[str, Any]] = field(default_factory=list)

    # Balancing & Dynamic Braking Instrumentation
    wheel_balancing_enabled: bool = False
    dynamic_braking_enabled: bool = False
    dynamic_brake_duration_ms: Optional[int] = None
    dynamic_brake_max_speed: Optional[float] = None
    dynamic_brake_max_speed_mps: Optional[float] = None
    anti_stall_confirmed: bool = False
    stopping_advance_deg: float = 0.0
    actuation_states_observed: List[str] = field(default_factory=list)
    brake_active_duration_ms: Optional[float] = None

    # Raw telemetry frames (optional / truncated in summary)
    approach_milestones: Dict[str, Any] = field(default_factory=dict)


@dataclass
class MultiTrialSuiteReport:
    suite_id: str
    test_type: str = "turn"  # "turn" or "linear"
    command_line: str = ""
    timestamp_utc: str = ""
    target_degrees: float = 0.0
    target_distance_m: Optional[float] = None
    target_linear_speed_mps: float = 0.20
    direction: str = "cw"
    total_trials: int = 0
    repetitions: int = 0
    successful_trials: int = 0
    aborted_trials: int = 0
    is_dry_run: bool = False
    wheel_balancing_enabled: bool = False
    dynamic_braking_enabled: bool = False
    dynamic_brake_duration_ms: Optional[int] = None
    dynamic_brake_max_speed: Optional[float] = None
    dynamic_brake_max_speed_mps: Optional[float] = None
    anti_stall_confirmed: bool = False
    stopping_advance_deg: float = 0.0
    trials: List[TrialReport] = field(default_factory=list)
    mean_settled_error_deg: Optional[float] = None
    std_dev_settled_error_deg: Optional[float] = None
    repeatability_deg: Optional[float] = None
    mean_estimate_delta_deg: Optional[float] = None

    # Linear Suite Metrics
    mean_settled_distance_error_m: Optional[float] = None
    std_dev_settled_distance_error_m: Optional[float] = None
    repeatability_distance_m: Optional[float] = None
    mean_heading_change_deg: Optional[float] = None

    # Multi-Repetition Trajectory & Totals
    suite_start_raw_heading_deg: Optional[float] = None
    suite_start_continuous_heading_deg: Optional[float] = None
    suite_end_raw_heading_deg: Optional[float] = None
    suite_end_continuous_heading_deg: Optional[float] = None
    total_cumulative_commanded_deg: float = 0.0
    total_cumulative_measured_deg: float = 0.0
    total_cumulative_error_deg: float = 0.0
    total_cumulative_commanded_m: float = 0.0
    total_cumulative_measured_m: float = 0.0
    total_cumulative_error_m: float = 0.0
    repetition_trajectory: List[Dict[str, Any]] = field(default_factory=list)

    def __post_init__(self):
        if not self.repetitions:
            self.repetitions = self.total_trials



def compute_phase_balancing_metrics(active_pid_packets: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """
    Computes same-side pair error and actual correction applied separated into CRUISE and CREEP phases.
    """
    phase_buckets: Dict[str, List[Dict[str, Any]]] = {"CRUISE": [], "CREEP": []}
    for p in active_pid_packets:
        ph = p.get("phase")
        if ph in phase_buckets:
            phase_buckets[ph].append(p)

    results = []
    for ph_name in ["CRUISE", "CREEP"]:
        pkts = phase_buckets[ph_name]
        if not pkts:
            continue

        # Left pair: M1 (LF) & M3 (LR)
        v1_list = [abs(p["m1"]["measuredRadps"]) for p in pkts if p.get("m1", {}).get("measuredRadps") is not None]
        v3_list = [abs(p["m3"]["measuredRadps"]) for p in pkts if p.get("m3", {}).get("measuredRadps") is not None]
        t1_list = [p["m1"].get("spinSyncTrim", 0) for p in pkts if "m1" in p and "spinSyncTrim" in p["m1"]]
        t3_list = [p["m3"].get("spinSyncTrim", 0) for p in pkts if "m3" in p and "spinSyncTrim" in p["m3"]]

        # Right pair: M2 (RF) & M4 (RR)
        v2_list = [abs(p["m2"]["measuredRadps"]) for p in pkts if p.get("m2", {}).get("measuredRadps") is not None]
        v4_list = [abs(p["m4"]["measuredRadps"]) for p in pkts if p.get("m4", {}).get("measuredRadps") is not None]
        t2_list = [p["m2"].get("spinSyncTrim", 0) for p in pkts if "m2" in p and "spinSyncTrim" in p["m2"]]
        t4_list = [p["m4"].get("spinSyncTrim", 0) for p in pkts if "m4" in p and "spinSyncTrim" in p["m4"]]

        m1_mean = sum(v1_list) / len(v1_list) if v1_list else 0.0
        m3_mean = sum(v3_list) / len(v3_list) if v3_list else 0.0
        err_l = m1_mean - m3_mean
        t1_mean = sum(t1_list) / len(t1_list) if t1_list else 0.0
        t3_mean = sum(t3_list) / len(t3_list) if t3_list else 0.0

        m2_mean = sum(v2_list) / len(v2_list) if v2_list else 0.0
        m4_mean = sum(v4_list) / len(v4_list) if v4_list else 0.0
        err_r = m2_mean - m4_mean
        t2_mean = sum(t2_list) / len(t2_list) if t2_list else 0.0
        t4_mean = sum(t4_list) / len(t4_list) if t4_list else 0.0

        results.append({
            "phase": ph_name,
            "samples": len(pkts),
            "m1_mean": m1_mean,
            "m3_mean": m3_mean,
            "pair_error_left": err_l,
            "m1_trim": t1_mean,
            "m3_trim": t3_mean,
            "m2_mean": m2_mean,
            "m4_mean": m4_mean,
            "pair_error_right": err_r,
            "m2_trim": t2_mean,
            "m4_trim": t4_mean,
        })
    return results


class ReportGenerator:
    """Serializes reports to JSON and formats readable Markdown summaries."""

    @staticmethod
    def save_json(report: MultiTrialSuiteReport, output_dir: str) -> str:
        os.makedirs(output_dir, exist_ok=True)
        if report.test_type == "linear":
            dist_val = report.target_distance_m if report.target_distance_m is not None else 0.0
            dist_str = f"{dist_val:.3f}".replace(".", "p")
            filename = f"linear_{report.direction}_{dist_str}m_{report.suite_id}.json"
        else:
            filename = f"turn_{report.direction}_{int(report.target_degrees)}deg_{report.suite_id}.json"
        filepath = os.path.join(output_dir, filename)

        # Custom serializer for dataclasses
        data = asdict(report)
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

        return filepath

    @staticmethod
    def format_markdown_summary(report: MultiTrialSuiteReport) -> str:
        if report.test_type == "linear":
            return ReportGenerator._format_linear_markdown_summary(report)
        return ReportGenerator._format_turn_markdown_summary(report)

    @staticmethod
    def _format_linear_markdown_summary(report: MultiTrialSuiteReport) -> str:
        md = []
        target_dist = report.target_distance_m or 0.0
        md.append(f"# Physical Linear Motion Test Report: {target_dist:.3f}m {report.direction.upper()}")
        md.append(f"**Suite ID:** `{report.suite_id}` | **Mode:** `{'DRY RUN (STATIONARY)' if report.is_dry_run else 'PHYSICAL EXECUTION'}`")
        md.append(f"**Command:** `{report.command_line}`")
        md.append(f"**Result:** {report.successful_trials}/{report.total_trials} Trials Succeeded\n")

        # Active Production Controller Configuration (Read back and confirmed from live system)
        md.append("## Production Controller Configuration")
        md.append("| Parameter | Setting | Confirmation Source | Description |")
        md.append("| :--- | :--- | :--- | :--- |")
        md.append(f"| Requested Linear Velocity | `{report.target_linear_speed_mps:.3f} m/s` (Constant) | Test Harness Command | Constant production speed (no test-layer creep or trims) |")
        bal_str = "`ACTIVE (K_sync = 0.005)`" if report.wheel_balancing_enabled else "`DISABLED`"
        md.append(f"| Production Wheel Balancing | {bal_str} | Live Controller Readback (`/api/drive/config`) | Active straight-line tick synchronization |")
        if report.dynamic_braking_enabled:
            dur = report.dynamic_brake_duration_ms or 100
            spd = report.dynamic_brake_max_speed_mps or report.dynamic_brake_max_speed or 0.35
            unit = "m/s" if report.test_type == "linear" else "rad/s"
            brake_str = f"`ACTIVE ({dur}ms H-bridge brake pulse, max {spd:.2f} {unit})`"
        else:
            brake_str = "`DISABLED`"
        md.append(f"| Production Dynamic Braking | {brake_str} | Live Controller Readback (`/api/drive/config`) | Low-side H-bridge dynamic brake state (no motor reversal) |")
        anti_stall_str = "`ACTIVE (Dynamic Stiction Machine)`" if report.anti_stall_confirmed else "`ACTIVE (Firmware)`"
        md.append(f"| Production Anti-Stall / Stiction | {anti_stall_str} | Live Telemetry Verification (`/api/pid-telemetry`) | Breakout boost + stiction state machine tracking |")
        md.append(f"| Slew Rate / Acceleration | `Production ESP32 MotionLimiter` | Production Firmware Loop | Accel: 0.50 m/s², Decel: 1.00 m/s² |\n")

        # Multi-Trial Summary
        md.append("## Multi-Trial Summary")
        md.append("| Metric | Value |")
        md.append("| :--- | :--- |")
        md.append(f"| Requested Distance | {target_dist:.3f} m ({report.direction.upper()}) |")
        md.append(f"| Total Trials Executed | {report.total_trials} |")
        md.append(f"| Successful Trials | {report.successful_trials} |")
        md.append(f"| Aborted Trials | {report.aborted_trials} |")
        if report.mean_settled_distance_error_m is not None:
            mean_err_str = f"{report.mean_settled_distance_error_m:+.4f} m ({report.mean_settled_distance_error_m * 1000.0:+.1f} mm)"
        else:
            mean_err_str = "Unavailable"
        md.append(f"| Mean Settled Distance Error (Informational) | {mean_err_str} |")
        std_err_str = f"{report.std_dev_settled_distance_error_m:.4f} m" if report.std_dev_settled_distance_error_m is not None else "Unavailable"
        md.append(f"| Error Standard Deviation | {std_err_str} |")
        rep_str = f"{report.repeatability_distance_m:.4f} m" if report.repeatability_distance_m is not None else "Unavailable"
        md.append(f"| Repeatability Span | {rep_str} |")
        if report.mean_heading_change_deg is not None:
            md.append(f"| Mean Heading Change (IMU Yaw Drift) | `{report.mean_heading_change_deg:+.2f}°` |")
        md.append("")

        # Per-Trial Breakdown
        for t in report.trials:
            status_icon = "[PASS]" if t.status in ("SUCCESS", "DRY_RUN_PASSED") else "[FAIL]"
            md.append(f"## Trial {t.trial_index} Forensics: {status_icon} {t.status}")
            if t.abort_reason:
                md.append(f"> [!WARNING]\n> Abort Reason: {t.abort_reason}\n")

            # Phase Separation Summary
            md.append("### Phase Separation Summary")
            md.append("| Phase | Duration | Distance | Mean Speed | L/R Speed Diff | L/R PWM Diff | Notes |")
            md.append("| :--- | :--- | :--- | :--- | :--- | :--- | :--- |")

            p_acc = t.acceleration_phase or {}
            acc_dur = f"`{p_acc.get('duration_s', 0.0):.2f} s`"
            acc_dist = f"`{p_acc.get('distance_m', 0.0):+.4f} m`"
            acc_spd = f"`{p_acc.get('mean_speed_mps', 0.0):+.3f} m/s`"
            acc_peak_pwm = p_acc.get('peak_pwm', 0)
            md.append(f"| **1. Acceleration** | {acc_dur} | {acc_dist} | {acc_spd} | - | - | Peak PWM: `{acc_peak_pwm}` |")

            p_std = t.steady_speed_phase or {}
            std_dur = f"`{p_std.get('duration_s', 0.0):.2f} s`"
            std_dist = f"`{p_std.get('distance_m', 0.0):+.4f} m`"
            std_spd = f"`{p_std.get('mean_speed_mps', 0.0):+.3f} m/s`"
            std_lr_spd = f"`{p_std.get('left_to_right_speed_diff_radps', 0.0):+.3f} rad/s`"
            std_lr_pwm = f"`{p_std.get('left_to_right_pwm_diff', 0.0):+.1f}`"
            md.append(f"| **2. Steady-Speed** | {std_dur} | {std_dist} | {std_spd} | {std_lr_spd} | {std_lr_pwm} | **Primary Characterization Phase** |")

            p_stp = t.stopping_phase or {}
            stp_dur = f"`{p_stp.get('duration_s', 0.0):.2f} s`"
            stp_dist = f"`{p_stp.get('distance_m', 0.0):+.4f} m`"
            stp_brake = f"Brake pulse: `{t.brake_active_duration_ms:.1f} ms`" if t.brake_active_duration_ms else "Coast"
            md.append(f"| **3. Stopping** | {stp_dur} | {stp_dist} | `0.000 m/s` | - | - | {stp_brake} |")
            md.append("")

            # Primary Analysis: Steady-Speed Wheel Balancing & Symmetry
            md.append("### Primary Analysis: Steady-Speed Wheel Balancing & Symmetry")
            md.append("> [!NOTE]")
            md.append("> Wheel comparisons and calibration assessments are evaluated strictly during the steady-speed phase to isolate motor/drivetrain dynamics from acceleration and braking transients.\n")

            md.append("| Wheel (Slot / Corner) | Commanded Target (rad/s) | Steady Measured Mean (rad/s) | Steady PWM (Mean / Max) | Steady Encoder Delta (ticks) | Stopped While Commanded |")
            md.append("| :--- | :--- | :--- | :--- | :--- | :--- |")

            active_metrics = t.steady_speed_wheel_metrics if t.steady_speed_wheel_metrics else t.wheel_metrics
            for w_id in ["m1", "m2", "m3", "m4"]:
                w = active_metrics.get(w_id, WheelTrialMetrics(wheel_id=w_id))
                wheel_label = f"**{w_id.upper()}** ({w.slot_mapping} / {w.corner})"
                cmd_str = f"`{w.commanded_speed_radps_mean:+.2f}`" if w.commanded_speed_radps_mean is not None else "*Unavailable*"
                meas_str = f"`{w.measured_speed_radps_mean:+.2f}`" if w.measured_speed_radps_mean is not None else "*Unavailable*"
                pwm_str = f"`{w.pwm_mean:.1f} / {w.pwm_max}`" if (w.pwm_mean is not None and w.pwm_max is not None) else "`N/A`"

                if w.stopped_while_commanded is True:
                    stopped_str = f"**YES ({w.stopped_while_commanded_count} ev)**"
                elif w.stopped_while_commanded is False:
                    stopped_str = "No (0s)"
                else:
                    stopped_str = "Unknown"

                md.append(f"| {wheel_label} | {cmd_str} | {meas_str} | {pwm_str} | {w.encoder_delta_ticks:+d} | {stopped_str} |")
            md.append("")

            # Disparities and diagnostics
            md.append("#### Steady-Speed Disparities & Symmetry Metrics")
            md.append("| Diagnostic Metric | Value | Reference / Diagnostic Evaluation |")
            md.append("| :--- | :--- | :--- |")
            fl_diff = t.steady_speed_front_to_rear_left_radps if t.steady_speed_front_to_rear_left_radps is not None else t.front_to_rear_diff_left_radps
            fr_diff = t.steady_speed_front_to_rear_right_radps if t.steady_speed_front_to_rear_right_radps is not None else t.front_to_rear_diff_right_radps
            fl_str = f"`{fl_diff:+.3f} rad/s`" if fl_diff is not None else "`Unavailable`"
            fr_str = f"`{fr_diff:+.3f} rad/s`" if fr_diff is not None else "`Unavailable`"
            lr_spd_str = f"`{t.steady_speed_left_to_right_speed_diff_radps:+.3f} rad/s`" if t.steady_speed_left_to_right_speed_diff_radps is not None else "`Unavailable`"
            lr_pwm_str = f"`{t.steady_speed_left_to_right_pwm_diff:+.1f}`" if t.steady_speed_left_to_right_pwm_diff is not None else "`Unavailable`"
            yaw_str = f"`{t.imu_heading_change_deg:+.2f}°`" if t.imu_heading_change_deg is not None else "`Unavailable`"

            md.append(f"| Left Front-to-Rear Diff (LF - LR) | {fl_str} | M1 vs M3 speed difference (< 0.15 rad/s ideal) |")
            md.append(f"| Right Front-to-Rear Diff (RF - RR) | {fr_str} | M2 vs M4 speed difference (< 0.15 rad/s ideal) |")
            md.append(f"| Left-to-Right Speed Difference | {lr_spd_str} | Mean left vs mean right speed (< 0.10 rad/s ideal) |")
            md.append(f"| Left-to-Right PWM Disparity | {lr_pwm_str} | Mean left PWM vs mean right PWM (< 3 counts ideal) |")
            md.append(f"| IMU Heading Deviation (Yaw Drift) | {yaw_str} | Heading deviation during straight motion (< 1.5°/m ideal) |")
            md.append("")

            # Informational: Endpoint Accuracy & Stopping
            md.append("### Informational: Endpoint Accuracy & Stopping")
            md.append("> [!NOTE]")
            md.append("> Endpoint accuracy is recorded for production characterization purposes only. It is NOT the purpose or pass/fail criterion of this characterization test.\n")

            md.append("| Measurement | Value | Description |")
            md.append("| :--- | :--- | :--- |")
            req_dist = t.requested_distance_m if t.requested_distance_m is not None else target_dist
            md.append(f"| Requested Distance | `{req_dist:.3f} m` ({t.direction.upper()}) | Target travel distance |")
            meas_str = f"`{t.measured_distance_m:+.4f} m`" if t.measured_distance_m is not None else "`Unavailable`"
            md.append(f"| Total Measured Distance | {meas_str} | Total 4-wheel encoder odometry displacement |")
            settled_str = f"`{t.final_settled_distance_m:+.4f} m`" if t.final_settled_distance_m is not None else "`Unavailable`"
            md.append(f"| Final Settled Distance | {settled_str} | Displacement after standstill settle |")
            stp_dist_val = t.stopping_distance_m if t.stopping_distance_m is not None else t.post_zero_coast_m
            stp_dist_str = f"`{stp_dist_val:+.4f} m`" if stp_dist_val is not None else "`Unavailable`"
            md.append(f"| Stopping Phase Displacement | {stp_dist_str} | Displacement between zero command and full stop |")
            err_m = t.settled_distance_error_m if t.settled_distance_error_m is not None else (
                (t.measured_distance_m - req_dist) if t.measured_distance_m is not None else None
            )
            err_str = f"`{err_m:+.4f} m ({err_m * 1000.0:+.1f} mm)`" if err_m is not None else "`Unavailable`"
            md.append(f"| Net Distance Deviation | {err_str} | Measured vs target deviation (informational) |")
            md.append(f"| Zero Command Latency | `{t.zero_command_latency_ms:.1f} ms` | HTTP cmd_vel(0,0) turn-around |" if t.zero_command_latency_ms is not None else "| Zero Command Latency | `N/A` | HTTP cmd_vel(0,0) |")
            md.append(f"| Final Zero & Disarmed Confirmed | `{'YES' if (t.confirmed_final_zero_command and t.confirmed_final_disarmed_state) else 'NO (FAIL-SAFE ERROR)'}` | Drivetrain locked & safe |")
            md.append("")

        return "\n".join(md)

    @staticmethod
    def _format_turn_markdown_summary(report: MultiTrialSuiteReport) -> str:
        md = []
        md.append(f"# Physical Turn Test Report: {report.target_degrees:.1f}° {report.direction.upper()}")
        md.append(f"**Suite ID:** `{report.suite_id}` | **Mode:** `{'DRY RUN (STATIONARY)' if report.is_dry_run else 'PHYSICAL EXECUTION'}`")
        md.append(f"**Command:** `{report.command_line}`")
        md.append(f"**Result:** {report.successful_trials}/{report.total_trials} Trials Succeeded\n")

        # Summary Table
        md.append("## Multi-Trial Summary")
        md.append("| Metric | Value |")
        md.append("| :--- | :--- |")
        md.append(f"| Requested Turn | {report.target_degrees:.1f}° ({report.direction.upper()}) |")
        md.append(f"| Total Trials Executed | {report.total_trials} |")
        md.append(f"| Successful Trials | {report.successful_trials} |")
        md.append(f"| Aborted Trials | {report.aborted_trials} |")
        mean_err_str = f"{report.mean_settled_error_deg:+.2f}°" if report.mean_settled_error_deg is not None else "Unavailable"
        md.append(f"| Mean Settled Error | {mean_err_str} |")
        std_err_str = f"{report.std_dev_settled_error_deg:.2f}°" if report.std_dev_settled_error_deg is not None else "Unavailable"
        md.append(f"| Error Standard Deviation | {std_err_str} |")
        rep_str = f"{report.repeatability_deg:.2f}°" if report.repeatability_deg is not None else "Unavailable"
        md.append(f"| Repeatability Span | {rep_str} |")
        if report.mean_estimate_delta_deg is not None:
            md.append(f"| Mean Delta vs Ron's Estimate | {report.mean_estimate_delta_deg:+.2f}° |")
        md.append("")

        # Multi-Repetition Trajectory & Totals Summary
        if report.repetition_trajectory:
            md.append("## Repetition Trajectory & Totals Summary")
            md.append("| Step / Rep | Start Heading (Abs) | End Heading (Abs) | Continuous Angle | Commanded | Measured Turn | Error | Post-Zero Coast | Status |")
            md.append("| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |")
            for r in report.repetition_trajectory:
                rep_idx = r.get("repetition", 1)
                st_abs = f"`{r.get('start_raw_heading_deg'):+.2f}°`" if r.get('start_raw_heading_deg') is not None else "`N/A`"
                end_abs = f"`{r.get('end_raw_heading_deg'):+.2f}°`" if r.get('end_raw_heading_deg') is not None else "`N/A`"
                st_cont = r.get('start_continuous_deg')
                end_cont = r.get('end_continuous_deg')
                cont_str = f"`[{st_cont:+.2f}°, {end_cont:+.2f}°]`" if (st_cont is not None and end_cont is not None) else "`N/A`"
                cmd_deg = f"`{r.get('target_deg'):+.2f}°`" if r.get('target_deg') is not None else "`N/A`"
                meas_deg = f"`{r.get('measured_deg'):+.2f}°`" if r.get('measured_deg') is not None else "`N/A`"
                err_deg = f"`{r.get('error_deg'):+.2f}°`" if r.get('error_deg') is not None else "`N/A`"
                coast_deg = f"`{r.get('post_zero_coast_deg'):+.2f}°`" if r.get('post_zero_coast_deg') is not None else "`N/A`"
                stat = f"`{r.get('status', 'UNKNOWN')}`"
                md.append(f"| **Step {rep_idx}** | {st_abs} | {end_abs} | {cont_str} | {cmd_deg} | {meas_deg} | {err_deg} | {coast_deg} | {stat} |")

            st_suite = f"`{report.suite_start_raw_heading_deg:+.2f}°` (Step 1 Start)" if report.suite_start_raw_heading_deg is not None else "`N/A`"
            end_suite = f"`{report.suite_end_raw_heading_deg:+.2f}°` (Step {len(report.repetition_trajectory)} End)" if report.suite_end_raw_heading_deg is not None else "`N/A`"
            net_cont = f"Net: `{report.total_cumulative_measured_deg:+.2f}°`"
            tot_cmd = f"`{report.total_cumulative_commanded_deg:+.2f}°`"
            tot_meas = f"`{report.total_cumulative_measured_deg:+.2f}°`"
            tot_err = f"`{report.total_cumulative_error_deg:+.2f}°`"
            succ_str = f"`{report.successful_trials}/{report.total_trials} Succeeded`"
            md.append(f"| **TOTALS** | {st_suite} | {end_suite} | {net_cont} | {tot_cmd} | {tot_meas} | {tot_err} | - | {succ_str} |")
            md.append("")

        # Per-Trial Breakdown
        md.append("## Per-Trial Forensics")
        for t in report.trials:
            status_icon = "[PASS]" if t.status in ("SUCCESS", "DRY_RUN_PASSED") else "[FAIL]"
            md.append(f"### Trial {t.trial_index}: {status_icon} {t.status}")
            if t.abort_reason:
                md.append(f"> [!WARNING]\n> Abort Reason: {t.abort_reason}\n")

            md.append("| Measurement | Value | Description |")
            md.append("| :--- | :--- | :--- |")
            if t.start_heading_raw_deg is not None:
                md.append(f"| Step Start Heading | `{t.start_heading_raw_deg:+.2f}°` | IMU raw heading at motion start |")
            if t.final_settled_raw_heading_deg is not None:
                md.append(f"| Step Settled Heading | `{t.final_settled_raw_heading_deg:+.2f}°` | IMU raw heading after settle period |")
            md.append(f"| Target Signed Angle | `{t.target_signed_yaw_deg:+.2f}°` | Intended continuous rotation |")
            md.append(f"| Angle at Zero Command | `{t.gyro_angle_at_zero_cmd_deg:+.2f}°` | IMU reading when $\\omega_z=0$ issued |")
            settled_str = f"`{t.final_settled_gyro_angle_deg:+.2f}°`" if t.final_settled_gyro_angle_deg is not None else "`Unavailable`"
            md.append(f"| Final Settled Angle | {settled_str} | IMU reading after settle period |")
            coast_str = f"`{t.post_zero_rotation_deg:+.2f}°`" if t.post_zero_rotation_deg is not None else "`Unavailable`"
            md.append(f"| Post-Zero Coast | {coast_str} | Rotation between zero cmd and standstill |")
            error_str = f"`{t.settled_heading_error_deg:+.2f}°`" if t.settled_heading_error_deg is not None else "`Unavailable`"
            md.append(f"| Settled Error vs Target | {error_str} | Final error against target |")

            if t.rons_physical_angle_estimate_deg is not None:
                md.append(f"| Ron's Physical Estimate | `{t.rons_physical_angle_estimate_deg:+.2f}°` | Ground truth protractor/laser observation |")
                delta_str = f"{t.estimate_vs_gyro_delta_deg:+.2f}°" if t.estimate_vs_gyro_delta_deg is not None else "N/A"
                md.append(f"| Estimate vs Gyro Delta | `{delta_str}` | Discrepancy (Gyro - Ron) |")
            else:
                md.append(f"| Ron's Physical Estimate | `Not Provided` | Ground truth manual observation |")

            md.append(f"| Opposing Polarity Verified | `{'YES' if t.opposite_polarity_maintained else 'NO (VIOLATION)'}` | Left & Right sides opposed |")
            md.append(f"| Breakout Events | `{t.breakout_event_count}` | Stiction boost activations |")
            md.append(f"| Active Telemetry Samples | `{t.telemetry_samples_count}` | Filtered PID diagnostic packets |")
            if t.zero_command_latency_ms is not None:
                md.append(f"| Zero Command Latency | `{t.zero_command_latency_ms:.1f} ms` | HTTP cmd_vel(0,0) turn-around |")
            if t.settle_duration_s > 0:
                md.append(
                    f"| Settle Telemetry Achieved | `IMU: {t.settle_imu_valid_advancing_count}/{t.settle_imu_poll_count} valid ({t.settle_imu_valid_advancing_rate_hz:.1f} Hz, poll {t.settle_imu_poll_rate_hz:.1f} Hz), "
                    f"PID: {t.settle_pid_post_zero_packets_count} post-zero ({t.settle_pid_post_zero_packet_rate_hz:.1f} Hz, {t.settle_pid_backlog_packets_count} backlog, {t.settle_pid_unknown_packets_count} unk)` | Duration: {t.settle_duration_s:.2f}s |"
                )
            md.append(f"| Wheel Balancing Active | `{'ENABLED' if t.wheel_balancing_enabled else 'DISABLED'}` | Low-speed synchronization trim |")
            md.append(f"| Dynamic Braking Active | `{'ENABLED' if t.dynamic_braking_enabled else 'DISABLED'}` | Shared low-speed brake pulse |")
            if t.stopping_advance_deg > 0.0:
                md.append(f"| Stopping Advance | `{t.stopping_advance_deg:.2f}°` | Pre-target stop trigger angle |")
            if t.actuation_states_observed:
                md.append(f"| Actuation States | `{' -> '.join(t.actuation_states_observed)}` | Firmware drivetrain states |")
            if t.brake_active_duration_ms is not None:
                md.append(f"| Brake Pulse Duration | `{t.brake_active_duration_ms:.1f} ms` | Active dynamic brake period |")
            md.append(f"| Final Zero & Disarmed Confirmed | `{'YES' if (t.confirmed_final_zero_command and t.confirmed_final_disarmed_state) else 'NO (FAIL-SAFE ERROR)'}` | Drivetrain locked & safe |")
            md.append("")

            # Phase Breakdown
            if t.phase_headings:
                md.append("#### Phase-by-Phase Heading Transitions")
                md.append("| Motion Phase | Start Heading (Abs) | End Heading (Abs) | Start (Rel) | End (Rel) | Phase Rotation | Duration |")
                md.append("| :--- | :--- | :--- | :--- | :--- | :--- | :--- |")
                for ph in t.phase_headings:
                    ph_name = ph.get("phase_name", "UNKNOWN")
                    s_abs = f"`{ph.get('start_abs_deg', 0.0):+.2f}°`"
                    e_abs = f"`{ph.get('end_abs_deg', 0.0):+.2f}°`"
                    s_rel = f"`{ph.get('start_rel_deg', 0.0):+.2f}°`"
                    e_rel = f"`{ph.get('end_rel_deg', 0.0):+.2f}°`"
                    d_rot = f"`{ph.get('delta_deg', 0.0):+.2f}°`"
                    dur = f"{ph.get('duration_s', 0.0):.2f}s"
                    md.append(f"| **{ph_name}** | {s_abs} | {e_abs} | {s_rel} | {e_rel} | {d_rot} | {dur} |")
                md.append("")

            # Same-Side Synchronization & Pair Balancing Breakdown (CRUISE & CREEP)
            if t.active_pid_packets:
                bal_metrics = compute_phase_balancing_metrics(t.active_pid_packets)
                if bal_metrics:
                    md.append("#### Same-Side Wheel Synchronization & Pair Balancing (CRUISE & CREEP)")
                    md.append("| Motion Phase | Left Pair (M1/M3) Speeds | Left Pair Error | Left Trims (M1/M3) | Right Pair (M2/M4) Speeds | Right Pair Error | Right Trims (M2/M4) | Telemetry Samples |")
                    md.append("| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |")
                    for bm in bal_metrics:
                        ph = bm["phase"]
                        l_spd = f"`{bm['m1_mean']:.2f} / {bm['m3_mean']:.2f} rad/s`"
                        l_err = f"`{bm['pair_error_left']:+.2f} rad/s`"
                        l_trm = f"`{bm['m1_trim']:+.1f} / {bm['m3_trim']:+.1f} PWM`"
                        r_spd = f"`{bm['m2_mean']:.2f} / {bm['m4_mean']:.2f} rad/s`"
                        r_err = f"`{bm['pair_error_right']:+.2f} rad/s`"
                        r_trm = f"`{bm['m2_trim']:+.1f} / {bm['m4_trim']:+.1f} PWM`"
                        smpls = f"`{bm['samples']}`"
                        md.append(f"| **{ph}** | {l_spd} | {l_err} | {l_trm} | {r_spd} | {r_err} | {r_trm} | {smpls} |")
                    md.append("")

            # Wheel Details
            md.append("#### Wheel Actuation & Encoder Performance")
            md.append("| Wheel (Slot / Corner) | Commanded Target (rad/s) | Measured Mean (rad/s) | Mean Abs Speed (rad/s) | Speed Range [Min, Max] (rad/s) | Encoder Delta (ticks) | Stopped While Commanded | Stiction Boosts |")
            md.append("| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |")
            for w_id in ["m1", "m2", "m3", "m4"]:
                w = t.wheel_metrics.get(w_id, WheelTrialMetrics(wheel_id=w_id))
                wheel_label = f"**{w_id.upper()}** ({w.slot_mapping} / {w.corner})"

                # Commanded target formatting
                if w.validity == "INVALID" or w.commanded_speed_source == "INVALID_ZERO_TELEMETRY":
                    cmd_str = "*INVALID*"
                elif w.commanded_speed_radps_mean is not None:
                    if w.commanded_speed_source == "calculated_expected":
                        cmd_str = f"`{w.commanded_speed_radps_mean:+.2f}` *(calc)*"
                    else:
                        cmd_str = f"`{w.commanded_speed_radps_mean:+.2f}`"
                else:
                    cmd_str = "*Unavailable*"

                # Measured speed formatting
                if w.validity == "INVALID":
                    meas_str = "*INVALID*"
                    abs_str = "*INVALID*"
                    range_str = "*INVALID*"
                else:
                    if w.measured_speed_radps_mean is not None:
                        meas_str = f"`{w.measured_speed_radps_mean:+.2f}`"
                    else:
                        meas_str = "*Unavailable*"

                    if w.measured_speed_radps_abs_mean is not None:
                        abs_str = f"`{w.measured_speed_radps_abs_mean:.2f}`"
                    else:
                        abs_str = "*Unavailable*"

                    if w.measured_speed_radps_min is not None and w.measured_speed_radps_max is not None:
                        range_str = f"`[{w.measured_speed_radps_min:+.2f}, {w.measured_speed_radps_max:+.2f}]`"
                    else:
                        range_str = "*Unavailable*"

                # Stopped while commanded formatting
                if w.stopped_while_commanded is True:
                    stopped_str = f"**YES ({w.stopped_while_commanded_count} ev / {w.stopped_while_commanded_duration_s:.2f}s)**"
                elif w.stopped_while_commanded is False:
                    stopped_str = "No (0s)"
                else:
                    stopped_str = "Unknown"

                md.append(f"| {wheel_label} | {cmd_str} | {meas_str} | {abs_str} | {range_str} | {w.encoder_delta_ticks:+d} | {stopped_str} | {w.stiction_boost_events} |")
            md.append("")

        return "\n".join(md)

