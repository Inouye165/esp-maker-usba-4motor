import unittest
import math
import sys
import os
import time
import json
from unittest.mock import MagicMock, patch

# Ensure scripts dir is on sys.path
repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
scripts_dir = os.path.join(repo_root, "scripts")
if scripts_dir not in sys.path:
    sys.path.insert(0, scripts_dir)

from characterize_trial_a_openloop import (
    TrialAHarness,
    ContinuousYawTracker,
    AccumulatedTravelTracker,
    StreamAdvancementTracker,
    TelemetryValidationError,
    TRIAL_A_STEPS,
    MAX_ABS_WHEEL_TRAVEL_M,
    MAX_MEAN_ABS_SPEED_MPS,
    MAX_ACCUMULATED_YAW_DEG,
    MAX_TELEMETRY_AGE_S,
    MAX_TOTAL_DURATION_S,
    METERS_PER_TICK,
    WHEEL_DIAMETER_M,
    main
)


class TestTrialAHarness(unittest.TestCase):
    def setUp(self):
        self.harness = TrialAHarness(host="mock-rover", port=3000, report_dir="scratch/test_reports")
        self.harness.arm_drive = MagicMock(return_value={"ok": True})
        self.harness.disarm_drive = MagicMock(return_value={"ok": True})
        self.harness.send_open_loop_pwm = MagicMock(return_value={"ok": True})

    def _make_valid_drive_status(self, armed=False, mode=0, seq=100, age_ms=10):
        return {
            "ok": True,
            "lastPacketAgeMs": age_ms,
            "status": {
                "armed": armed,
                "mode": mode,
                "seq": seq,
                "uptimeMs": 50000,
                "dataAgeMs": age_ms,
                "receivedAt": int(time.time() * 1000) - age_ms
            }
        }

    def _make_valid_pid_telemetry(self, speeds_radps=(0.0, 0.0, 0.0, 0.0), pwms=(0, 0, 0, 0), seq=200, age_ms=15):
        return {
            "ok": True,
            "valid": True,
            "stale": False,
            "sequence": seq,
            "dataAgeMs": age_ms,
            "telemetry": {
                "m1": {"measuredRadps": speeds_radps[0], "finalPwm": pwms[0]},
                "m2": {"measuredRadps": speeds_radps[1], "finalPwm": pwms[1]},
                "m3": {"measuredRadps": speeds_radps[2], "finalPwm": pwms[2]},
                "m4": {"measuredRadps": speeds_radps[3], "finalPwm": pwms[3]},
            }
        }

    def _make_valid_encoders(self, ticks=(0, 0, 0, 0), seq=300, age_ms=12):
        return {
            "ok": True,
            "serialConnected": True,
            "lastPacketAgeMs": age_ms,
            "sequence": seq,
            "encoders": {
                "m1": ticks[0],
                "m2": ticks[1],
                "m3": ticks[2],
                "m4": ticks[3]
            }
        }

    def _make_valid_imu(self, yaw_deg=0.0, seq=400, age_ms=8):
        yaw_rad = math.radians(yaw_deg)
        # Quaternion for pure yaw rotation: [cos(yaw/2), 0, 0, sin(yaw/2)]
        qw = math.cos(yaw_rad / 2.0)
        qz = math.sin(yaw_rad / 2.0)
        return {
            "ok": True,
            "stale": False,
            "serialConnected": True,
            "dataAgeMs": age_ms,
            "sequence": seq,
            "orientation": {
                "w": qw,
                "x": 0.0,
                "y": 0.0,
                "z": qz
            }
        }

    def _mock_all_valid(self, armed=False, mode=0, speeds_radps=(0.0, 0.0, 0.0, 0.0), pwms=(0, 0, 0, 0), ticks=(0, 0, 0, 0), yaw_deg=0.0):
        self.harness.fetch_drive_status = MagicMock(return_value=self.harness.parse_drive_status(self._make_valid_drive_status(armed, mode)))
        self.harness.fetch_pid_telemetry = MagicMock(return_value=self.harness.parse_pid_telemetry(self._make_valid_pid_telemetry(speeds_radps, pwms)))
        self.harness.fetch_encoders = MagicMock(return_value=self.harness.parse_encoders(self._make_valid_encoders(ticks)))
        self.harness.fetch_imu = MagicMock(return_value=self.harness.parse_imu(self._make_valid_imu(yaw_deg)))

    # ────────────────────────────────────────────────────────────
    # 1. Telemetry Validation Tests
    # ────────────────────────────────────────────────────────────
    def test_1_reject_missing_wheel_in_pid_telemetry(self):
        """Reject PID telemetry if any wheel (e.g. m3) is missing. Never substitute default zero."""
        data = self._make_valid_pid_telemetry()
        del data["telemetry"]["m3"]
        with self.assertRaises(TelemetryValidationError) as ctx:
            self.harness.parse_pid_telemetry(data)
        self.assertIn("Missing wheel 'm3'", str(ctx.exception))

    def test_2_reject_missing_wheel_in_encoders(self):
        """Reject encoders payload if any wheel is missing. Never substitute default zero."""
        data = self._make_valid_encoders()
        del data["encoders"]["m4"]
        with self.assertRaises(TelemetryValidationError) as ctx:
            self.harness.parse_encoders(data)
        self.assertIn("Missing wheel 'm4'", str(ctx.exception))

    def test_3_reject_non_finite_or_boolean_fields(self):
        """Reject non-finite or boolean values in numeric fields."""
        data_bool = self._make_valid_pid_telemetry()
        data_bool["telemetry"]["m1"]["measuredRadps"] = True
        with self.assertRaises(TelemetryValidationError) as ctx:
            self.harness.parse_pid_telemetry(data_bool)
        self.assertIn("boolean", str(ctx.exception))

        data_nan = self._make_valid_imu()
        data_nan["orientation"]["w"] = float("nan")
        with self.assertRaises(TelemetryValidationError) as ctx:
            self.harness.parse_imu(data_nan)
        self.assertIn("Non-finite", str(ctx.exception))

    def test_4_reject_stale_telemetry(self):
        """Reject telemetry if reported packet age exceeds 250 ms limit."""
        data_stale = self._make_valid_pid_telemetry(age_ms=300)
        with self.assertRaises(TelemetryValidationError) as ctx:
            self.harness.parse_pid_telemetry(data_stale)
        self.assertIn("stale", str(ctx.exception).lower())

    # ────────────────────────────────────────────────────────────
    # 2. Arming & Firmware Verification Gates
    # ────────────────────────────────────────────────────────────
    def test_5_arming_inside_cleanup_protected_try(self):
        """Arming must occur inside try block so cleanup is guaranteed if arming throws."""
        self.harness.arm_drive = MagicMock(side_effect=RuntimeError("Arming RPC Timeout"))
        self._mock_all_valid()

        with patch.object(self.harness, "_execute_cleanup", return_value=True) as mock_cleanup:
            report = self.harness.execute_trial(skip_firmware_check=True)
            mock_cleanup.assert_called_once()
            self.assertIn("Arming RPC Timeout", self.harness.abort_reason)
            self.assertEqual(report["overall_status"], "ABORTED")

    def test_6_firmware_verification_returning_false_blocks_motion(self):
        """Firmware verification returning False must strictly block arming and motion, cleaning up and returning report."""
        with patch.object(self.harness, "verify_safety_firmware", return_value=False):
            with patch.object(self.harness, "_execute_cleanup", return_value=True) as mock_cleanup:
                report = self.harness.execute_trial(skip_firmware_check=False)
                # Neither arming nor motion may be called
                self.harness.arm_drive.assert_not_called()
                self.harness.send_open_loop_pwm.assert_not_called()
                mock_cleanup.assert_called_once()
                self.assertIsNotNone(report)
                self.assertIsInstance(report, dict)
                self.assertTrue(report["aborted"])
                self.assertEqual(report["overall_status"], "ABORTED")
                self.assertIn("FIRMWARE_VERIFICATION_FAILED", report["abort_reason"])

    def test_6b_firmware_verification_ssh_query_failure_blocks_motion(self):
        """SSH or query failure during firmware verification must strictly block motion, clean up, and return report."""
        with patch.object(self.harness, "verify_safety_firmware", side_effect=RuntimeError("SSH connection refused")):
            with patch.object(self.harness, "_execute_cleanup", return_value=True) as mock_cleanup:
                report = self.harness.execute_trial(skip_firmware_check=False)
                self.harness.arm_drive.assert_not_called()
                self.harness.send_open_loop_pwm.assert_not_called()
                mock_cleanup.assert_called_once()
                self.assertTrue(report["aborted"])
                self.assertEqual(report["overall_status"], "ABORTED")
                self.assertIn("SSH connection refused", report["abort_reason"])

    def _make_mock_ssh_for_cursor(self, cursor_out: str, after_cursor_out: str):
        mock_client = MagicMock()
        def exec_side_effect(cmd):
            mock_stdout = MagicMock()
            if "--show-cursor" in cmd:
                mock_stdout.read.return_value = cursor_out.encode("utf-8")
            elif "--after-cursor" in cmd:
                mock_stdout.read.return_value = after_cursor_out.encode("utf-8")
            else:
                mock_stdout.read.return_value = b""
            return (MagicMock(), mock_stdout, MagicMock())
        mock_client.exec_command.side_effect = exec_side_effect
        return mock_client

    def test_6c_verify_safety_firmware_old_matching_entry_fails(self):
        """An old matching entry before the cursor fails when no fresh response arrives after cursor, blocking arming."""
        self.harness._http_get = MagicMock(return_value={"ok": True})
        cursor_out = "-- No entries --\n-- cursor: s=test_old_cursor_123;i=1;b=abc;m=100;t=200;x=300\n"
        after_cursor_out = "-- No entries --\n"  # No new response after cursor

        mock_ssh = self._make_mock_ssh_for_cursor(cursor_out, after_cursor_out)
        with patch("paramiko.SSHClient", return_value=mock_ssh):
            # Unit check: verify_safety_firmware raises RuntimeError on missing fresh response
            with self.assertRaises(RuntimeError) as ctx:
                self.harness.verify_safety_firmware(required_version="1.3.2-olp-safe", timeout_s=0.05)
            self.assertIn("No fresh '[Firmware Info]' entry received after cursor", str(ctx.exception))

            # Integration check: execute_trial blocks arming and returns report
            with patch.object(self.harness, "_execute_cleanup", return_value=True):
                report = self.harness.execute_trial(skip_firmware_check=False)
                self.harness.arm_drive.assert_not_called()
                self.harness.send_open_loop_pwm.assert_not_called()
                self.assertEqual(report["overall_status"], "ABORTED")
                self.assertIn("No fresh '[Firmware Info]' entry received after cursor", report["abort_reason"])

    def test_6d_verify_safety_firmware_fresh_matching_response_passes(self):
        """A fresh matching response logged after cursor passes verification."""
        self.harness._http_get = MagicMock(return_value={"ok": True})
        cursor_out = "-- cursor: s=test_fresh_cursor_456\n"
        after_cursor_out = (
            "Oct 11 12:35:00 rover-pi node[2367]: [Firmware Info] "
            "Name: Maker-ESP32-Unified-Rover, Ver: 1.3.2-olp-safe, Protocol: v1.1, "
            "Source: olp-safe-case2, Build: Oct 11 2026 12:00:00, Target: Maker-ESP32-Pro\n"
        )
        mock_ssh = self._make_mock_ssh_for_cursor(cursor_out, after_cursor_out)
        with patch("paramiko.SSHClient", return_value=mock_ssh):
            res = self.harness.verify_safety_firmware(required_version="1.3.2-olp-safe", timeout_s=0.05)
            self.assertTrue(res)

    def test_6e_verify_safety_firmware_fresh_mismatched_response_fails(self):
        """A fresh mismatched response logged after cursor returns False and blocks arming."""
        self.harness._http_get = MagicMock(return_value={"ok": True})
        cursor_out = "-- cursor: s=test_fresh_cursor_789\n"
        after_cursor_out = (
            "Oct 11 12:35:00 rover-pi node[2367]: [Firmware Info] "
            "Name: Maker-ESP32-Unified-Rover, Ver: 1.3.1-case2, Protocol: v1.1, "
            "Source: case2-lowspd, Build: Oct 11 2026 05:37:39, Target: Maker-ESP32-Pro\n"
        )
        mock_ssh = self._make_mock_ssh_for_cursor(cursor_out, after_cursor_out)
        with patch("paramiko.SSHClient", return_value=mock_ssh):
            # Unit check: returns False on version mismatch
            res = self.harness.verify_safety_firmware(required_version="1.3.2-olp-safe", timeout_s=0.05)
            self.assertFalse(res)

            # Integration check: execute_trial blocks arming and returns report
            with patch.object(self.harness, "_execute_cleanup", return_value=True):
                report = self.harness.execute_trial(skip_firmware_check=False)
                self.harness.arm_drive.assert_not_called()
                self.harness.send_open_loop_pwm.assert_not_called()
                self.assertEqual(report["overall_status"], "ABORTED")
                self.assertIn("FIRMWARE_VERIFICATION_FAILED", report["abort_reason"])

    def test_7_confirm_armed_state_before_nonzero_pwm(self):
        """Must confirm armed state in fresh telemetry before dispatching nonzero PWM."""
        self.harness.arm_drive = MagicMock(return_value={"ok": True})
        self.harness.fetch_drive_status = MagicMock(return_value={"armed": False, "mode": 0, "seq": 1, "age_ms": 10, "uptime_ms": 100})
        self.harness.fetch_pid_telemetry = MagicMock(return_value={"seq": 1, "age_ms": 10, "speeds_radps": [0,0,0,0], "pwms": [0,0,0,0]})
        self.harness.fetch_encoders = MagicMock(return_value={"seq": 1, "age_ms": 10, "ticks": [0,0,0,0]})
        self.harness.fetch_imu = MagicMock(return_value={"seq": 1, "age_ms": 10, "yaw_deg": 0.0})

        with patch.object(self.harness, "_execute_cleanup", return_value=True):
            self.harness.execute_trial(skip_firmware_check=True)
            self.assertIn("Arming confirmation failed", self.harness.abort_reason)
            pwm_calls = [c for c in self.harness.send_open_loop_pwm.call_args_list if c.args != (0, 0, 0, 0)]
            self.assertEqual(len(pwm_calls), 0)

    def test_8_establish_baselines_before_nonzero_pwm(self):
        """Travel and yaw baselines must be captured after arming confirmation and before nonzero PWM."""
        armed_state = {"armed": True, "mode": 3, "seq": 1, "age_ms": 10, "uptime_ms": 100}
        self.harness.fetch_drive_status = MagicMock(return_value=armed_state)
        self.harness.fetch_pid_telemetry = MagicMock(return_value={"seq": 1, "age_ms": 10, "speeds_radps": [0,0,0,0], "pwms": [0,0,0,0]})
        self.harness.fetch_encoders = MagicMock(return_value={"seq": 1, "age_ms": 10, "ticks": [5000, 5000, 5000, 5000]})
        self.harness.fetch_imu = MagicMock(return_value={"seq": 1, "age_ms": 10, "yaw_deg": 30.0})

        with patch("characterize_trial_a_openloop.TRIAL_A_STEPS", [TRIAL_A_STEPS[0]]):
            with patch.object(self.harness, "_execute_cleanup", return_value=True):
                self.harness.execute_trial(skip_firmware_check=True)
                baseline_sample = self.harness.samples[0]
                self.assertEqual(baseline_sample["step_name"], "Baseline")
                self.assertEqual(baseline_sample["commanded_pwm"], 0)
                self.assertEqual(baseline_sample["raw_ticks"], [5000, 5000, 5000, 5000])
                self.assertEqual(baseline_sample["raw_yaw_deg"], 30.0)

    # ────────────────────────────────────────────────────────────
    # 3. Monotonic Deadlines, Delayed Snapshot Collection, & Streams
    # ────────────────────────────────────────────────────────────
    def test_9_no_pwm_resumption_after_watchdog_expiry(self):
        """If ESP32 watchdog fires and disarms drive, harness must abort immediately without sending further PWM."""
        call_count = [0]
        def mock_drive_status():
            call_count[0] += 1
            if call_count[0] <= 2:
                return {"armed": True, "mode": 3, "seq": call_count[0], "age_ms": 10, "uptime_ms": 100}
            else:
                return {"armed": False, "mode": 0, "seq": call_count[0], "age_ms": 10, "uptime_ms": 100}

        self.harness.fetch_drive_status = MagicMock(side_effect=mock_drive_status)
        self.harness.fetch_pid_telemetry = MagicMock(return_value={"seq": 1, "age_ms": 10, "speeds_radps": [0,0,0,0], "pwms": [0,0,0,0]})
        self.harness.fetch_encoders = MagicMock(return_value={"seq": 1, "age_ms": 10, "ticks": [0,0,0,0]})
        self.harness.fetch_imu = MagicMock(return_value={"seq": 1, "age_ms": 10, "yaw_deg": 0.0})

        with patch.object(self.harness, "_execute_cleanup", return_value=True):
            self.harness.execute_trial(skip_firmware_check=True)
            self.assertIn("DRIVE_DISARMED_OR_LOCKED", self.harness.abort_reason)
            active_calls = [c for c in self.harness.send_open_loop_pwm.call_args_list if c.args != (0, 0, 0, 0)]
            self.assertLessEqual(len(active_calls), 1)

    def test_10a_delayed_snapshot_collection_aborts_before_pwm_dispatch(self):
        """If snapshot collection delay causes effective age > 250 ms, harness must abort before sending PWM."""
        armed_state = {"armed": True, "mode": 3, "seq": 1, "age_ms": 60, "uptime_ms": 100}
        self.harness.fetch_drive_status = MagicMock(return_value=armed_state)
        self.harness.fetch_pid_telemetry = MagicMock(return_value={"seq": 1, "age_ms": 60, "speeds_radps": [0,0,0,0], "pwms": [0,0,0,0]})
        self.harness.fetch_encoders = MagicMock(return_value={"seq": 1, "age_ms": 60, "ticks": [0,0,0,0]})
        self.harness.fetch_imu = MagicMock(return_value={"seq": 1, "age_ms": 60, "yaw_deg": 0.0})

        # Inject simulated 200 ms snapshot fetch delay: base age 60 ms + 200 ms delay = 260 ms > 250 ms
        call_count = [0]
        def delayed_snapshot():
            call_count[0] += 1
            now = time.monotonic()
            if call_count[0] <= 1:
                # Baseline snapshot is fast
                return {
                    "fetch_start_mono": now,
                    "receipt_mono": now,
                    "receipt_epoch_ms": int(time.time() * 1000),
                    "armed": True, "mode": 3,
                    "seq_drive": 1, "seq_pid": 1, "seq_encoders": 1, "seq_imu": 1,
                    "age_drive_ms": 10, "age_pid_ms": 10, "age_encoders_ms": 10, "age_imu_ms": 10,
                    "data_age_ms": 10, "ticks": [0,0,0,0], "speeds_radps": [0,0,0,0],
                    "speeds_mps": [0,0,0,0], "pwms": [0,0,0,0], "yaw_deg": 0.0
                }
            else:
                # Delayed step snapshot: fetch started 0.200s ago
                return {
                    "fetch_start_mono": now - 0.200,
                    "receipt_mono": now,
                    "receipt_epoch_ms": int(time.time() * 1000),
                    "armed": True, "mode": 3,
                    "seq_drive": 2, "seq_pid": 2, "seq_encoders": 2, "seq_imu": 2,
                    "age_drive_ms": 60, "age_pid_ms": 60, "age_encoders_ms": 60, "age_imu_ms": 60,
                    "data_age_ms": 60, "ticks": [0,0,0,0], "speeds_radps": [0,0,0,0],
                    "speeds_mps": [0,0,0,0], "pwms": [0,0,0,0], "yaw_deg": 0.0
                }

        self.harness.fetch_telemetry_snapshot = MagicMock(side_effect=delayed_snapshot)

        with patch.object(self.harness, "_execute_cleanup", return_value=True) as mock_cleanup:
            report = self.harness.execute_trial(skip_firmware_check=True)
            # Recheck immediately before dispatch must have aborted
            self.assertIn("STALE_TELEMETRY_POST_FETCH", self.harness.abort_reason)
            self.assertEqual(report["overall_status"], "ABORTED")
            # Crucially: active nonzero PWM was NEVER dispatched!
            active_pwm_calls = [c for c in self.harness.send_open_loop_pwm.call_args_list if c.args != (0, 0, 0, 0)]
            self.assertEqual(len(active_pwm_calls), 0)
            mock_cleanup.assert_called_once()

    def test_10b_frozen_drive_stream_triggers_abort(self):
        """If drive sequence freezes while other streams advance, harness must abort on FROZEN_STREAM_DRIVE."""
        call_count = [0]
        def mock_snapshot():
            call_count[0] += 1
            now = time.monotonic()
            # Drive sequence stays frozen at 100, while PID/encoders/IMU advance
            return {
                "fetch_start_mono": now,
                "receipt_mono": now,
                "receipt_epoch_ms": int(time.time() * 1000),
                "armed": True, "mode": 3,
                "seq_drive": 100,  # FROZEN
                "seq_pid": 200 + call_count[0],
                "seq_encoders": 300 + call_count[0],
                "seq_imu": 400 + call_count[0],
                "age_drive_ms": 10, "age_pid_ms": 10, "age_encoders_ms": 10, "age_imu_ms": 10,
                "data_age_ms": 10, "ticks": [0,0,0,0], "speeds_radps": [0,0,0,0],
                "speeds_mps": [0,0,0,0], "pwms": [0,0,0,0], "yaw_deg": 0.0
            }

        self.harness.fetch_telemetry_snapshot = MagicMock(side_effect=mock_snapshot)
        self.harness.fetch_drive_status = MagicMock(return_value={"armed": True, "mode": 3, "seq": 100, "age_ms": 10, "uptime_ms": 100})

        # Set stream tracker max_stale_s to 0.04s so test runs swiftly
        self.harness.stream_tracker = StreamAdvancementTracker(max_stale_s=0.04)

        with patch.object(self.harness, "_execute_cleanup", return_value=True):
            report = self.harness.execute_trial(skip_firmware_check=True)
            self.assertIn("FROZEN_STREAM_DRIVE", self.harness.abort_reason)
            self.assertEqual(report["overall_status"], "ABORTED")

    def test_10c_frozen_pid_stream_triggers_abort(self):
        """If PID sequence freezes while other streams advance, harness must abort on FROZEN_STREAM_PID."""
        call_count = [0]
        def mock_snapshot():
            call_count[0] += 1
            now = time.monotonic()
            return {
                "fetch_start_mono": now,
                "receipt_mono": now,
                "receipt_epoch_ms": int(time.time() * 1000),
                "armed": True, "mode": 3,
                "seq_drive": 100 + call_count[0],
                "seq_pid": 200,  # FROZEN
                "seq_encoders": 300 + call_count[0],
                "seq_imu": 400 + call_count[0],
                "age_drive_ms": 10, "age_pid_ms": 10, "age_encoders_ms": 10, "age_imu_ms": 10,
                "data_age_ms": 10, "ticks": [0,0,0,0], "speeds_radps": [0,0,0,0],
                "speeds_mps": [0,0,0,0], "pwms": [0,0,0,0], "yaw_deg": 0.0
            }

        self.harness.fetch_telemetry_snapshot = MagicMock(side_effect=mock_snapshot)
        self.harness.fetch_drive_status = MagicMock(return_value={"armed": True, "mode": 3, "seq": 100, "age_ms": 10, "uptime_ms": 100})
        self.harness.stream_tracker = StreamAdvancementTracker(max_stale_s=0.04)

        with patch.object(self.harness, "_execute_cleanup", return_value=True):
            report = self.harness.execute_trial(skip_firmware_check=True)
            self.assertIn("FROZEN_STREAM_PID", self.harness.abort_reason)
            self.assertEqual(report["overall_status"], "ABORTED")

    def test_10d_frozen_encoders_stream_triggers_abort(self):
        """If encoders sequence freezes while other streams advance, harness must abort on FROZEN_STREAM_ENCODERS."""
        call_count = [0]
        def mock_snapshot():
            call_count[0] += 1
            now = time.monotonic()
            return {
                "fetch_start_mono": now,
                "receipt_mono": now,
                "receipt_epoch_ms": int(time.time() * 1000),
                "armed": True, "mode": 3,
                "seq_drive": 100 + call_count[0],
                "seq_pid": 200 + call_count[0],
                "seq_encoders": 300,  # FROZEN
                "seq_imu": 400 + call_count[0],
                "age_drive_ms": 10, "age_pid_ms": 10, "age_encoders_ms": 10, "age_imu_ms": 10,
                "data_age_ms": 10, "ticks": [0,0,0,0], "speeds_radps": [0,0,0,0],
                "speeds_mps": [0,0,0,0], "pwms": [0,0,0,0], "yaw_deg": 0.0
            }

        self.harness.fetch_telemetry_snapshot = MagicMock(side_effect=mock_snapshot)
        self.harness.fetch_drive_status = MagicMock(return_value={"armed": True, "mode": 3, "seq": 100, "age_ms": 10, "uptime_ms": 100})
        self.harness.stream_tracker = StreamAdvancementTracker(max_stale_s=0.04)

        with patch.object(self.harness, "_execute_cleanup", return_value=True):
            report = self.harness.execute_trial(skip_firmware_check=True)
            self.assertIn("FROZEN_STREAM_ENCODERS", self.harness.abort_reason)
            self.assertEqual(report["overall_status"], "ABORTED")

    def test_10e_frozen_imu_stream_triggers_abort(self):
        """If IMU sequence freezes while other streams advance, harness must abort on FROZEN_STREAM_IMU."""
        call_count = [0]
        def mock_snapshot():
            call_count[0] += 1
            now = time.monotonic()
            return {
                "fetch_start_mono": now,
                "receipt_mono": now,
                "receipt_epoch_ms": int(time.time() * 1000),
                "armed": True, "mode": 3,
                "seq_drive": 100 + call_count[0],
                "seq_pid": 200 + call_count[0],
                "seq_encoders": 300 + call_count[0],
                "seq_imu": 400,  # FROZEN
                "age_drive_ms": 10, "age_pid_ms": 10, "age_encoders_ms": 10, "age_imu_ms": 10,
                "data_age_ms": 10, "ticks": [0,0,0,0], "speeds_radps": [0,0,0,0],
                "speeds_mps": [0,0,0,0], "pwms": [0,0,0,0], "yaw_deg": 0.0
            }

        self.harness.fetch_telemetry_snapshot = MagicMock(side_effect=mock_snapshot)
        self.harness.fetch_drive_status = MagicMock(return_value={"armed": True, "mode": 3, "seq": 100, "age_ms": 10, "uptime_ms": 100})
        self.harness.stream_tracker = StreamAdvancementTracker(max_stale_s=0.04)

        with patch.object(self.harness, "_execute_cleanup", return_value=True):
            report = self.harness.execute_trial(skip_firmware_check=True)
            self.assertIn("FROZEN_STREAM_IMU", self.harness.abort_reason)
            self.assertEqual(report["overall_status"], "ABORTED")

    def test_10f_stream_advancement_tracker_unit(self):
        """Unit test of StreamAdvancementTracker sequence and timeout mechanics."""
        tracker = StreamAdvancementTracker(max_stale_s=0.250)
        tracker.reset(100.0)

        # First update initializes last_seq
        err = tracker.update(100.0, {"drive": 1, "pid": 1, "encoders": 1, "imu": 1})
        self.assertIsNone(err)

        # Sequence doesn't change after 0.20s (under 0.250s threshold) -> no error
        err = tracker.update(100.20, {"drive": 1, "pid": 2, "encoders": 2, "imu": 2})
        self.assertIsNone(err)

        # Drive still sequence 1 after 0.26s (exceeds 0.250s threshold) -> triggers abort
        err = tracker.update(100.26, {"drive": 1, "pid": 3, "encoders": 3, "imu": 3})
        self.assertIsNotNone(err)
        self.assertIn("FROZEN_STREAM_DRIVE", err)

    # ────────────────────────────────────────────────────────────
    # 4. Common Exit Path, Cleanup-First Ordering, & Reports
    # ────────────────────────────────────────────────────────────
    def test_11_cleanup_unconfirmed_produces_cleanup_failed_status(self):
        """Unconfirmed cleanup must set overall_status to CLEANUP_FAILED."""
        self._mock_all_valid()
        with patch.object(self.harness, "_execute_cleanup", return_value=False):
            report = self.harness.execute_trial(skip_firmware_check=True)
            self.assertEqual(report["overall_status"], "CLEANUP_FAILED")
            self.assertTrue(self.harness.cleanup_failed)

    def test_12_report_saved_on_exceptions(self):
        """Reports must be saved to disk even if an unhandled exception occurs."""
        self.harness.arm_drive = MagicMock(side_effect=ValueError("Unexpected crash"))
        with patch.object(self.harness, "_execute_cleanup", return_value=True):
            report = self.harness.execute_trial(skip_firmware_check=True)
            self.assertIsNotNone(report)
            self.assertIn("Unexpected crash", report["abort_reason"])
            self.assertEqual(report["overall_status"], "ABORTED")

    def test_13_recorded_samples_contain_source_timestamps_and_delivered_pwms(self):
        """Verify samples record delivered PWMs, per-wheel speeds, and source metadata."""
        armed_state = {"armed": True, "mode": 3, "seq": 105, "age_ms": 10, "uptime_ms": 500}
        self.harness.fetch_drive_status = MagicMock(return_value=armed_state)
        self.harness.fetch_pid_telemetry = MagicMock(return_value={
            "seq": 205, "age_ms": 15, "speeds_radps": [0.5, 0.5, 0.5, 0.5], "pwms": [-48, -47, -48, -46]
        })
        self.harness.fetch_encoders = MagicMock(return_value={"seq": 305, "age_ms": 12, "ticks": [100, 100, 100, 100]})
        self.harness.fetch_imu = MagicMock(return_value={"seq": 405, "age_ms": 8, "yaw_deg": 12.5})

        with patch("characterize_trial_a_openloop.TRIAL_A_STEPS", [TRIAL_A_STEPS[0]]):
            with patch.object(self.harness, "_execute_cleanup", return_value=True):
                report = self.harness.execute_trial(skip_firmware_check=True)
                sample = [s for s in report["telemetry_samples"] if s["step_name"] != "Baseline"][0]
                self.assertEqual(sample["delivered_pwms"], [-48, -47, -48, -46])
                self.assertEqual(sample["measured_speeds_radps"], [0.5, 0.5, 0.5, 0.5])
                self.assertEqual(sample["source_metadata"]["seq_drive"], 105)
                self.assertEqual(sample["source_metadata"]["seq_pid"], 205)
                self.assertEqual(sample["source_metadata"]["seq_encoders"], 305)
                self.assertEqual(sample["source_metadata"]["seq_imu"], 405)

    def test_14_common_exit_path_returns_report_dictionary_on_all_aborts(self):
        """On any abort condition, execute_trial returns a complete report dictionary (never None)."""
        # Trigger travel limit abort
        armed_state = {"armed": True, "mode": 3, "seq": 1, "age_ms": 10, "uptime_ms": 100}
        self.harness.fetch_drive_status = MagicMock(return_value=armed_state)
        self.harness.fetch_pid_telemetry = MagicMock(return_value={"seq": 1, "age_ms": 10, "speeds_radps": [0,0,0,0], "pwms": [0,0,0,0]})
        # Exceed 150 mm travel (1500 ticks = 155 mm > 150 mm)
        self.harness.fetch_encoders = MagicMock(side_effect=[
            {"seq": 1, "age_ms": 10, "ticks": [0, 0, 0, 0]},      # baseline
            {"seq": 2, "age_ms": 10, "ticks": [1600, 0, 0, 0]}    # step 1
        ])
        self.harness.fetch_imu = MagicMock(return_value={"seq": 1, "age_ms": 10, "yaw_deg": 0.0})

        with patch.object(self.harness, "_execute_cleanup", return_value=True) as mock_cleanup:
            report = self.harness.execute_trial(skip_firmware_check=True)
            self.assertIsNotNone(report)
            self.assertIsInstance(report, dict)
            self.assertEqual(report["overall_status"], "ABORTED")
            self.assertTrue(report["aborted"])
            self.assertIn("EXCEEDED_MAX_ABS_TRAVEL", report["abort_reason"])
            self.assertIn("telemetry_samples", report)
            self.assertIn("summary_metrics", report)
            self.assertIn("step_analyses", report)
            mock_cleanup.assert_called_once()

    # ────────────────────────────────────────────────────────────
    # 5. Non-Signed Guard Cancellation Tests
    # ────────────────────────────────────────────────────────────
    def test_15_accumulated_travel_reversals_no_signed_cancellation(self):
        """Wheel reversals accumulate toward 150 mm travel bound without signed cancellation."""
        tracker = AccumulatedTravelTracker(METERS_PER_TICK)
        tracker.reset([10000, 10000, 10000, 10000])

        # Step 1: reverse 800 ticks (82.7 mm)
        _, max_accum, _ = tracker.update([9200, 10000, 10000, 10000])
        self.assertAlmostEqual(max_accum, 800 * METERS_PER_TICK, places=4)

        # Step 2: forward 800 ticks back to 10000 ticks. Net displacement = 0, but accumulated travel = 1600 ticks (165.5 mm > 150 mm)
        _, max_accum, net_disp = tracker.update([10000, 10000, 10000, 10000])
        self.assertAlmostEqual(net_disp, 0.0, places=4)
        self.assertAlmostEqual(max_accum, 1600 * METERS_PER_TICK, places=4)
        self.assertGreater(max_accum, MAX_ABS_WHEEL_TRAVEL_M)

    def test_16_speed_guard_no_signed_cancellation(self):
        """Counter-rotating wheels (+0.08 m/s, -0.08 m/s) trigger speed abort at > 0.060 m/s."""
        harness = TrialAHarness()
        harness.samples = [{"timestamp": 1000.0}]
        snap = {
            "receipt_epoch_ms": 1000000,
            "armed": True,
            "mode": 3,
            "seq_drive": 1,
            "seq_pid": 1,
            "seq_encoders": 1,
            "seq_imu": 1,
            "age_drive_ms": 10,
            "age_pid_ms": 10,
            "age_encoders_ms": 10,
            "age_imu_ms": 10,
            "data_age_ms": 10,
            "ticks": [0, 0, 0, 0],
            "speeds_radps": [2.5, -2.5, 2.5, -2.5],
            "speeds_mps": [0.08125, -0.08125, 0.08125, -0.08125],
            "pwms": [0, 0, 0, 0],
            "yaw_deg": 0.0
        }
        harness._record_telemetry_sample(1000.05, 1000.0, -48, 0, snap)
        self.assertIsNotNone(harness.abort_reason)
        self.assertIn("EXCEEDED_MAX_SPEED", harness.abort_reason)

    def test_17_yaw_guard_unwrapping_across_boundary(self):
        """Yaw unwraps cleanly across +/-180 deg boundary, preventing false abort on wrap."""
        tracker = ContinuousYawTracker()
        tracker.reset(179.0)

        # Cross from +179 to -179 deg (true delta is +2.0 deg through 180 deg)
        unwrapped, net_drift, _ = tracker.update(-179.0)
        self.assertAlmostEqual(unwrapped, 181.0, places=1)
        self.assertAlmostEqual(net_drift, 2.0, places=1)

        # Drift further to -168 deg (true drift 13 deg from initial 179 deg > 10.0 deg limit)
        unwrapped, net_drift, _ = tracker.update(-168.0)
        self.assertAlmostEqual(unwrapped, 192.0, places=1)
        self.assertAlmostEqual(net_drift, 13.0, places=1)
        self.assertGreater(net_drift, MAX_ACCUMULATED_YAW_DEG)

    # ────────────────────────────────────────────────────────────
    # 6. CLI Exit Code Tests
    # ────────────────────────────────────────────────────────────
    def test_18_cli_exit_code_0_on_dry_run(self):
        """CLI main() returns 0 on dry run."""
        ret = main(["--dry-run"])
        self.assertEqual(ret, 0)

    def test_19_cli_exit_code_1_on_abort(self):
        """CLI main() returns 1 when trial execution aborts."""
        with patch.object(TrialAHarness, "run_preflight_checks", return_value=None):
            with patch("builtins.input", return_value="CONFIRM"):
                with patch.object(TrialAHarness, "execute_trial", return_value={"overall_status": "ABORTED"}):
                    ret = main(["--skip-firmware-check"])
                    self.assertEqual(ret, 1)

    def test_20_cli_exit_code_2_on_cleanup_failed(self):
        """CLI main() returns 2 when cleanup cannot be confirmed."""
        with patch.object(TrialAHarness, "run_preflight_checks", return_value=None):
            with patch("builtins.input", return_value="CONFIRM"):
                with patch.object(TrialAHarness, "execute_trial", return_value={"overall_status": "CLEANUP_FAILED"}):
                    ret = main(["--skip-firmware-check"])
                    self.assertEqual(ret, 2)

    def test_21_cli_exit_code_1_on_preflight_failure(self):
        """CLI main() returns 1 when preflight checks fail."""
        with patch.object(TrialAHarness, "run_preflight_checks", side_effect=RuntimeError("Preflight FAIL")):
            ret = main([])
            self.assertEqual(ret, 1)


if __name__ == "__main__":
    unittest.main()
