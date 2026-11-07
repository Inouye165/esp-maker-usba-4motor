import unittest
from unittest.mock import MagicMock, patch
import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts")))
from characterize_focused_reverse_41 import (
    FocusedReverse41Harness,
    TelemetryValidationError,
    ContinuousYawTracker,
    AccumulatedTravelTracker,
    StreamAdvancementTracker,
    STARTUP_PWM,
    STARTUP_TIMEOUT_S,
    MIN_BREAKOUT_TICKS,
    MIN_BREAKOUT_SPEED_RADPS,
    CONSECUTIVE_STARTUP_FRAMES,
    SUSTAINED_PWM,
    SUSTAINED_DURATION_S,
    MAX_ABS_WHEEL_TRAVEL_M,
    MAX_MEAN_ABS_SPEED_MPS,
    MAX_ACCUMULATED_YAW_DEG,
    MAX_TOTAL_DURATION_S,
    REQUIRED_SAFETY_FIRMWARE
)


class TestFocusedReverse41Harness(unittest.TestCase):
    def setUp(self):
        self.harness = FocusedReverse41Harness(report_dir="scratch/test_reports")
        self.harness.arm_drive = MagicMock(return_value={"ok": True})
        self.harness.disarm_drive = MagicMock(return_value={"ok": True})
        self.harness.send_open_loop_pwm = MagicMock(return_value={"ok": True})

    def test_startup_parameters(self):
        """Verify revised startup parameters: -64 PWM, 350 ms timeout, 6 ticks, -0.15 rad/s, 1 frame, 2.00s cutoff."""
        self.assertEqual(STARTUP_PWM, -64)
        self.assertEqual(STARTUP_TIMEOUT_S, 0.350)
        self.assertEqual(MIN_BREAKOUT_TICKS, 6)
        self.assertEqual(MIN_BREAKOUT_SPEED_RADPS, -0.15)
        self.assertEqual(CONSECUTIVE_STARTUP_FRAMES, 1)
        self.assertEqual(SUSTAINED_PWM, -41)
        self.assertEqual(SUSTAINED_DURATION_S, 1.50)
        self.assertEqual(MAX_ABS_WHEEL_TRAVEL_M, 0.100)
        self.assertEqual(MAX_TOTAL_DURATION_S, 2.00)
        self.assertEqual(REQUIRED_SAFETY_FIRMWARE, "1.3.2-olp-safe")

    def test_dry_run_success(self):
        """Dry run must return DRY_RUN_SUCCESS without physical arming or motion."""
        report = self.harness.execute_trial(dry_run=True)
        self.assertEqual(report["overall_status"], "DRY_RUN_SUCCESS")
        self.harness.arm_drive.assert_not_called()
        self.harness.send_open_loop_pwm.assert_not_called()

    def test_firmware_verification_failure_blocks_motion(self):
        """Firmware verification returning False must strictly block arming and motion."""
        with patch.object(self.harness, "verify_safety_firmware", return_value=False):
            with patch.object(self.harness, "_execute_cleanup", return_value=True) as mock_cleanup:
                report = self.harness.execute_trial(skip_firmware_check=False)
                self.harness.arm_drive.assert_not_called()
                self.harness.send_open_loop_pwm.assert_not_called()
                mock_cleanup.assert_called_once()
                self.assertTrue(report["aborted"])
                self.assertEqual(report["overall_status"], "ABORTED")
                self.assertIn("FIRMWARE_VERIFICATION_FAILED", report["abort_reason"])

    def test_startup_breakaway_failure_aborts_and_classifies_inconclusive(self):
        """If startup is not established within timeout, abort immediately and classify hold as INCONCLUSIVE."""
        # Baseline snapshot (0 ticks, stationary)
        baseline_snap = {
            "receipt_mono": 100.0,
            "fetch_start_mono": 100.0,
            "receipt_epoch_ms": 1000,
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
            "pwms": [0, 0, 0, 0],
            "ticks": [0, 0, 0, 0],
            "yaw_deg": 0.0,
            "speeds_radps": [0.0, 0.0, 0.0, 0.0],
            "speeds_mps": [0.0, 0.0, 0.0, 0.0]
        }
        # Incomplete breakaway: only M2 and M3 move, M1 and M4 stay at 0 ticks
        stuck_snap = dict(baseline_snap)
        stuck_snap["pwms"] = [-64, -64, -64, -64]
        stuck_snap["ticks"] = [0, -10, -10, 0]  # M1 and M4 stuck
        stuck_snap["speeds_radps"] = [0.0, -0.20, -0.20, 0.0]
        stuck_snap["speeds_mps"] = [0.0, -0.0065, -0.0065, 0.0]

        call_count = [0]
        def mock_snap():
            call_count[0] += 1
            now = time.monotonic()
            if call_count[0] == 1:
                b = dict(baseline_snap)
                b["fetch_start_mono"] = now
                b["receipt_mono"] = now
                b["seq_drive"] = call_count[0]
                b["seq_pid"] = call_count[0]
                b["seq_encoders"] = call_count[0]
                b["seq_imu"] = call_count[0]
                return b
            s = dict(stuck_snap)
            s["fetch_start_mono"] = now
            s["receipt_mono"] = now
            s["seq_drive"] = call_count[0]
            s["seq_pid"] = call_count[0]
            s["seq_encoders"] = call_count[0]
            s["seq_imu"] = call_count[0]
            return s

        self.harness.fetch_drive_status = MagicMock(return_value={"armed": True, "mode": 3})
        self.harness.fetch_telemetry_snapshot = MagicMock(side_effect=mock_snap)

        with patch.object(self.harness, "_execute_cleanup", return_value=True) as mock_cleanup:
            report = self.harness.execute_trial(skip_firmware_check=True)
            self.assertIsNotNone(report)
            self.assertEqual(report["overall_status"], "ABORTED")
            self.assertTrue(report["aborted"])
            self.assertIn("STARTUP_BREAKAWAY_FAILED", report["abort_reason"])
            self.assertEqual(report["hold_analysis"]["status"], "INCONCLUSIVE")
            self.assertIn("Startup breakout not established", report["hold_analysis"]["reason"])
            mock_cleanup.assert_called_once()

    def test_startup_breakaway_success_transitions_to_hold(self):
        """When all 4 wheels achieve negative displacement and speed, transition immediately to Sustained_40 hold."""
        baseline_snap = {
            "receipt_epoch_ms": 1000,
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
            "pwms": [0, 0, 0, 0],
            "ticks": [0, 0, 0, 0],
            "yaw_deg": 0.0,
            "speeds_radps": [0.0, 0.0, 0.0, 0.0],
            "speeds_mps": [0.0, 0.0, 0.0, 0.0]
        }
        # Moving snapshot: all 4 wheels displaced by negative ticks and speed -0.30 rad/s
        moving_snap = dict(baseline_snap)
        moving_snap["pwms"] = [-64, -64, -64, -64]
        moving_snap["speeds_radps"] = [-0.30, -0.30, -0.30, -0.30]
        moving_snap["speeds_mps"] = [-0.010, -0.010, -0.010, -0.010]

        call_count = [0]
        def mock_snap():
            call_count[0] += 1
            now = time.monotonic()
            if call_count[0] == 1:
                b = dict(baseline_snap)
                b["fetch_start_mono"] = now
                b["receipt_mono"] = now
                b["seq_drive"] = call_count[0]
                b["seq_pid"] = call_count[0]
                b["seq_encoders"] = call_count[0]
                b["seq_imu"] = call_count[0]
                return b
            s = dict(moving_snap)
            s["fetch_start_mono"] = now
            s["receipt_mono"] = now
            s["seq_drive"] = call_count[0]
            s["seq_pid"] = call_count[0]
            s["seq_encoders"] = call_count[0]
            s["seq_imu"] = call_count[0]
            # Provide negative tick progression capped well below 100 mm limit
            s["ticks"] = [-8 - min(50, (call_count[0] - 2) * 2)] * 4
            return s

        self.harness.fetch_drive_status = MagicMock(return_value={"armed": True, "mode": 3})
        self.harness.fetch_telemetry_snapshot = MagicMock(side_effect=mock_snap)

        with patch.object(self.harness, "_execute_cleanup", return_value=True) as mock_cleanup:
            # Run trial with small sleep to complete quickly
            with patch("time.sleep", return_value=None):
                report = self.harness.execute_trial(skip_firmware_check=True)
                self.assertIsNotNone(report)
                self.assertEqual(report["overall_status"], "COMPLETED")
                self.assertFalse(report["aborted"])
                self.assertEqual(report["hold_analysis"]["status"], "EVALUATED")
                mock_cleanup.assert_called_once()

    def test_startup_repeated_packets_rejected_without_tick_progression(self):
        """Repeated packets or frozen sequence / duplicate ticks must not qualify startup; aborts as INCONCLUSIVE."""
        baseline_snap = {
            "receipt_mono": 100.0,
            "fetch_start_mono": 100.0,
            "receipt_epoch_ms": 1000,
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
            "pwms": [0, 0, 0, 0],
            "ticks": [0, 0, 0, 0],
            "yaw_deg": 0.0,
            "speeds_radps": [0.0, 0.0, 0.0, 0.0],
            "speeds_mps": [0.0, 0.0, 0.0, 0.0]
        }
        # First observation qualifies: deltas = -10, speed = -0.30
        snap1 = dict(baseline_snap)
        snap1["seq_drive"] = 2
        snap1["seq_pid"] = 2
        snap1["seq_encoders"] = 2
        snap1["seq_imu"] = 2
        snap1["pwms"] = [-64, -64, -64, -64]
        snap1["ticks"] = [-10, -10, -10, -10]
        snap1["speeds_radps"] = [-0.30, -0.30, -0.30, -0.30]

        # Repeated / duplicate tick reading: sequence advances but ticks are identical [-10, -10, -10, -10]
        call_idx = [0]
        def mock_snap():
            call_idx[0] += 1
            now = time.monotonic()
            if call_idx[0] == 1:
                b = dict(baseline_snap)
                b["fetch_start_mono"] = now
                b["receipt_mono"] = now
                return b
            elif call_idx[0] == 2:
                s = dict(snap1)
                s["fetch_start_mono"] = now
                s["receipt_mono"] = now
                return s
            else:
                s = dict(snap1)
                s["fetch_start_mono"] = now
                s["receipt_mono"] = now
                s["seq_drive"] = call_idx[0]
                s["seq_pid"] = call_idx[0]
                s["seq_encoders"] = call_idx[0]
                s["seq_imu"] = call_idx[0]
                # Ticks do NOT progress
                s["ticks"] = [-10, -10, -10, -10]
                return s

        self.harness.fetch_drive_status = MagicMock(return_value={"armed": True, "mode": 3})
        self.harness.fetch_telemetry_snapshot = MagicMock(side_effect=mock_snap)

        with patch.object(self.harness, "_execute_cleanup", return_value=True) as mock_cleanup:
            with patch("characterize_focused_reverse_41.CONSECUTIVE_STARTUP_FRAMES", 2):
                report = self.harness.execute_trial(skip_firmware_check=True)
                self.assertIsNotNone(report)
                self.assertEqual(report["overall_status"], "ABORTED")
                self.assertTrue(report["aborted"])
                self.assertIn("STARTUP_BREAKAWAY_FAILED", report["abort_reason"])
                self.assertEqual(report["hold_analysis"]["status"], "INCONCLUSIVE")
                mock_cleanup.assert_called_once()

    def test_startup_frozen_sequence_rejected_without_advancement(self):
        """If encoder sequence does not advance from baseline, startup does not qualify."""
        baseline_snap = {
            "receipt_mono": 100.0,
            "fetch_start_mono": 100.0,
            "receipt_epoch_ms": 1000,
            "armed": True,
            "mode": 3,
            "seq_drive": 1,
            "seq_pid": 1,
            "seq_encoders": 100,
            "seq_imu": 1,
            "age_drive_ms": 10,
            "age_pid_ms": 10,
            "age_encoders_ms": 10,
            "age_imu_ms": 10,
            "data_age_ms": 10,
            "pwms": [0, 0, 0, 0],
            "ticks": [0, 0, 0, 0],
            "yaw_deg": 0.0,
            "speeds_radps": [0.0, 0.0, 0.0, 0.0],
            "speeds_mps": [0.0, 0.0, 0.0, 0.0]
        }
        # Frozen encoder sequence snap: ticks moved but seq_encoders remains 100 == baseline
        frozen_snap = dict(baseline_snap)
        frozen_snap["seq_drive"] = 2
        frozen_snap["seq_pid"] = 2
        frozen_snap["seq_encoders"] = 100  # Frozen!
        frozen_snap["pwms"] = [-64, -64, -64, -64]
        frozen_snap["ticks"] = [-15, -15, -15, -15]
        frozen_snap["speeds_radps"] = [-0.30, -0.30, -0.30, -0.30]

        call_idx = [0]
        def mock_snap():
            call_idx[0] += 1
            now = time.monotonic()
            if call_idx[0] == 1:
                b = dict(baseline_snap)
                b["fetch_start_mono"] = now
                b["receipt_mono"] = now
                return b
            s = dict(frozen_snap)
            s["fetch_start_mono"] = now
            s["receipt_mono"] = now
            s["seq_drive"] = call_idx[0]
            s["seq_pid"] = call_idx[0]
            s["seq_imu"] = call_idx[0]
            s["seq_encoders"] = 100  # Frozen at baseline!
            return s

        self.harness.fetch_drive_status = MagicMock(return_value={"armed": True, "mode": 3})
        self.harness.fetch_telemetry_snapshot = MagicMock(side_effect=mock_snap)

        with patch.object(self.harness, "_execute_cleanup", return_value=True) as mock_cleanup:
            report = self.harness.execute_trial(skip_firmware_check=True)
            self.assertIsNotNone(report)
            self.assertEqual(report["overall_status"], "ABORTED")
            self.assertTrue(report["aborted"])
            self.assertTrue(
                "FROZEN_STREAM" in report["abort_reason"] or "STARTUP_BREAKAWAY_FAILED" in report["abort_reason"]
            )
            mock_cleanup.assert_called_once()

    def test_startup_delayed_fetch_across_deadline_aborts_without_dispatch(self):
        """If telemetry collection is delayed beyond 350 ms, post-fetch recheck must abort without dispatching -64 PWM."""
        baseline_snap = {
            "receipt_mono": 100.0,
            "fetch_start_mono": 100.0,
            "receipt_epoch_ms": 1000,
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
            "pwms": [0, 0, 0, 0],
            "ticks": [0, 0, 0, 0],
            "yaw_deg": 0.0,
            "speeds_radps": [0.0, 0.0, 0.0, 0.0],
            "speeds_mps": [0.0, 0.0, 0.0, 0.0]
        }
        self.harness.fetch_drive_status = MagicMock(return_value={"armed": True, "mode": 3})

        current_mono = [100.0]
        def mock_time():
            return current_mono[0]

        call_idx = [0]
        def mock_snap():
            call_idx[0] += 1
            if call_idx[0] == 1:
                # Baseline
                b = dict(baseline_snap)
                b["fetch_start_mono"] = current_mono[0]
                b["receipt_mono"] = current_mono[0]
                return b
            # Delayed fetch in Phase 1: advance time by 0.380s so post-fetch elapsed > 0.350s
            current_mono[0] += 0.380
            s = dict(baseline_snap)
            s["fetch_start_mono"] = current_mono[0] - 0.100
            s["receipt_mono"] = current_mono[0]
            s["seq_drive"] = 2
            s["seq_pid"] = 2
            s["seq_encoders"] = 2
            s["seq_imu"] = 2
            return s

        self.harness.fetch_telemetry_snapshot = MagicMock(side_effect=mock_snap)

        with patch("time.monotonic", side_effect=mock_time):
            with patch.object(self.harness, "_execute_cleanup", return_value=True) as mock_cleanup:
                report = self.harness.execute_trial(skip_firmware_check=True)
                self.assertIsNotNone(report)
                self.assertEqual(report["overall_status"], "ABORTED")
                self.assertTrue(report["aborted"])
                self.assertIn("STARTUP_TIMEOUT_POST_FETCH", report["abort_reason"])
                self.assertEqual(report["hold_analysis"]["status"], "INCONCLUSIVE")
                # -64 PWM must NOT be dispatched after deadline expiry
                self.harness.send_open_loop_pwm.assert_not_called()
                mock_cleanup.assert_called_once()

    def test_startup_successful_transition_prompt_dispatch_sustained_40(self):
        """When 2 consecutive qualifying frames show advancing sequences and negative tick progression, prompt -40 dispatch occurs."""
        baseline_snap = {
            "receipt_epoch_ms": 1000,
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
            "pwms": [0, 0, 0, 0],
            "ticks": [0, 0, 0, 0],
            "yaw_deg": 0.0,
            "speeds_radps": [0.0, 0.0, 0.0, 0.0],
            "speeds_mps": [0.0, 0.0, 0.0, 0.0]
        }
        # Stationary initial snapshot (ticks 0)
        snap0 = dict(baseline_snap)
        snap0["seq_drive"] = 2; snap0["seq_pid"] = 2; snap0["seq_encoders"] = 2; snap0["seq_imu"] = 2
        snap0["pwms"] = [-64, -64, -64, -64]

        # Frame 1: -8 ticks on all wheels, speed -0.25 rad/s, seq 3
        snap1 = dict(baseline_snap)
        snap1["seq_drive"] = 3
        snap1["seq_pid"] = 3
        snap1["seq_encoders"] = 3
        snap1["seq_imu"] = 3
        snap1["pwms"] = [-64, -64, -64, -64]
        snap1["ticks"] = [-8, -8, -8, -8]
        snap1["speeds_radps"] = [-0.25, -0.25, -0.25, -0.25]
        snap1["speeds_mps"] = [-0.008, -0.008, -0.008, -0.008]

        # Hold frames under -40 PWM
        hold_snap = dict(snap1)
        hold_snap["pwms"] = [-41, -41, -41, -41]
        hold_snap["speeds_radps"] = [-0.46, -0.46, -0.46, -0.46]
        hold_snap["speeds_mps"] = [-0.015, -0.015, -0.015, -0.015]

        call_idx = [0]
        def mock_snap():
            call_idx[0] += 1
            now = time.monotonic()
            if call_idx[0] == 1:
                b = dict(baseline_snap)
                b["fetch_start_mono"] = now
                b["receipt_mono"] = now
                return b
            elif call_idx[0] == 2:
                s = dict(snap0)
                s["fetch_start_mono"] = now
                s["receipt_mono"] = now
                return s
            elif call_idx[0] == 3:
                s = dict(snap1)
                s["fetch_start_mono"] = now
                s["receipt_mono"] = now
                return s
            else:
                s = dict(hold_snap)
                s["fetch_start_mono"] = now
                s["receipt_mono"] = now
                s["seq_drive"] = call_idx[0]
                s["seq_pid"] = call_idx[0]
                s["seq_encoders"] = call_idx[0]
                s["seq_imu"] = call_idx[0]
                # Cap ticks well below 100 mm limit
                s["ticks"] = [-8 - min(40, (call_idx[0]-3))] * 4
                return s

        self.harness.fetch_drive_status = MagicMock(return_value={"armed": True, "mode": 3})
        self.harness.fetch_telemetry_snapshot = MagicMock(side_effect=mock_snap)

        with patch.object(self.harness, "_execute_cleanup", return_value=True) as mock_cleanup:
            with patch("time.sleep", return_value=None):
                report = self.harness.execute_trial(skip_firmware_check=True)
                self.assertIsNotNone(report)
                self.assertEqual(report["overall_status"], "COMPLETED")
                self.assertFalse(report["aborted"])
                self.assertEqual(report["hold_analysis"]["status"], "EVALUATED")
                mock_cleanup.assert_called_once()
                # Verify prompt dispatch: first call was -64, second call immediately switched to -40
                calls = self.harness.send_open_loop_pwm.call_args_list
                self.assertGreaterEqual(len(calls), 2)
                self.assertEqual(calls[0][0], (-64, -64, -64, -64))
                self.assertEqual(calls[1][0], (-41, -41, -41, -41))

    def test_startup_replay_153359_pattern_transitions_promptly(self):
        """Replay exact sequence and tick pattern from run 153359 to prove Sample 5 transitions to -40 despite unchanged seq_pid."""
        # Baseline (Sample 0 from run 153359)
        base = {
            "receipt_mono": 100.0,
            "fetch_start_mono": 100.0,
            "receipt_epoch_ms": 1000,
            "armed": True,
            "mode": 3,
            "seq_drive": 95451,
            "seq_pid": 47726,
            "seq_encoders": 190902,
            "seq_imu": 1000,
            "age_drive_ms": 10,
            "age_pid_ms": 10,
            "age_encoders_ms": 10,
            "age_imu_ms": 10,
            "data_age_ms": 38,
            "pwms": [0, 0, 0, 0],
            "ticks": [0, 0, -1, -1],
            "yaw_deg": 0.0,
            "speeds_radps": [0.0, 0.0, 0.0, 0.0],
            "speeds_mps": [0.0, 0.0, 0.0, 0.0]
        }
        # Sample 1: t=0.005s, pre-dispatch
        s1 = dict(base)
        s1["data_age_ms"] = 46

        # Sample 2: t=0.060s, PWM reached -64, 0 ticks
        s2 = dict(base)
        s2["seq_encoders"] = 190904
        s2["seq_pid"] = 47727
        s2["seq_drive"] = 95452
        s2["pwms"] = [-64, -64, -64, -64]
        s2["speeds_radps"] = [-0.18, -0.18, -0.18, -0.09]
        s2["data_age_ms"] = 7

        # Sample 3: t=0.105s, 6-10 ticks, M4 speed filter still -0.09
        s3 = dict(base)
        s3["seq_encoders"] = 190906
        s3["seq_pid"] = 47727  # unchanged
        s3["seq_drive"] = 95453
        s3["pwms"] = [-64, -64, -64, -64]
        s3["ticks"] = [-6, -10, -8, -7]
        s3["speeds_radps"] = [-0.18, -0.18, -0.18, -0.09]
        s3["data_age_ms"] = 54

        # Sample 4: t=0.156s, 22-29 ticks, speeds -1.22 to -1.46 rad/s (Qualifying Observation 1)
        s4 = dict(base)
        s4["seq_encoders"] = 190908
        s4["seq_pid"] = 47728
        s4["seq_drive"] = 95454
        s4["pwms"] = [-64, -64, -64, -64]
        s4["ticks"] = [-23, -27, -23, -30]
        s4["speeds_radps"] = [-1.34, -1.22, -1.28, -1.46]
        s4["data_age_ms"] = 19

        # Sample 5: t=0.206s, 47-55 ticks (-24 additional ticks on every wheel!), speeds -1.22 to -1.46 rad/s
        # seq_encoders advanced to 190910, BUT seq_pid is unchanged at 47728!
        s5 = dict(base)
        s5["seq_encoders"] = 190910
        s5["seq_pid"] = 47728  # unchanged at 10 Hz!
        s5["seq_drive"] = 95455
        s5["pwms"] = [-64, -64, -64, -64]
        s5["ticks"] = [-47, -50, -49, -55]
        s5["speeds_radps"] = [-1.34, -1.22, -1.28, -1.46]
        s5["data_age_ms"] = 55

        # Hold frames under -40 PWM
        hold_s = dict(s5)
        hold_s["pwms"] = [-41, -41, -41, -41]
        hold_s["speeds_radps"] = [-0.46, -0.46, -0.46, -0.46]
        hold_s["speeds_mps"] = [-0.015, -0.015, -0.015, -0.015]

        run_samples = [base, s1, s2, s3, s4, s5]
        call_idx = [0]
        def mock_replay_snap():
            call_idx[0] += 1
            now = time.monotonic()
            if call_idx[0] <= len(run_samples):
                s = dict(run_samples[call_idx[0]-1])
                s["fetch_start_mono"] = now
                s["receipt_mono"] = now
                s["seq_imu"] = 1000 + call_idx[0]
                return s
            else:
                s = dict(hold_s)
                s["fetch_start_mono"] = now
                s["receipt_mono"] = now
                s["seq_drive"] = 95455 + (call_idx[0] - len(run_samples))
                s["seq_pid"] = 47729 + (call_idx[0] - len(run_samples)) // 2
                s["seq_encoders"] = 190910 + (call_idx[0] - len(run_samples)) * 2
                s["seq_imu"] = 1000 + call_idx[0]
                s["ticks"] = [-47 - min(30, (call_idx[0] - len(run_samples)))] * 4
                return s

        self.harness.fetch_drive_status = MagicMock(return_value={"armed": True, "mode": 3})
        self.harness.fetch_telemetry_snapshot = MagicMock(side_effect=mock_replay_snap)

        with patch.object(self.harness, "_execute_cleanup", return_value=True) as mock_cleanup:
            with patch("time.sleep", return_value=None):
                report = self.harness.execute_trial(skip_firmware_check=True)
                self.assertIsNotNone(report)
                self.assertEqual(report["overall_status"], "COMPLETED")
                self.assertFalse(report["aborted"])
                self.assertEqual(report["hold_analysis"]["status"], "EVALUATED")
                mock_cleanup.assert_called_once()
                # Under the revised logic, Sample 4 qualifies Frame 1 and prompt dispatches -40 PWM on Call 3!
                calls = self.harness.send_open_loop_pwm.call_args_list
                self.assertGreaterEqual(len(calls), 4)
                self.assertEqual(calls[2][0], (-64, -64, -64, -64))
                self.assertEqual(calls[3][0], (-41, -41, -41, -41))

    def test_startup_replay_184808_pattern_transitions_at_sample3(self):
        """Replay exact telemetry sequence from run 184808: Sample 3 qualifies Frame 1 and promptly dispatches -40 PWM."""
        base = {
            "fetch_start_mono": 100.0, "receipt_mono": 100.0, "receipt_epoch_ms": 1000,
            "armed": True, "mode": 3,
            "seq_drive": 739, "seq_pid": 369, "seq_encoders": 1478, "seq_imu": 2600,
            "age_drive_ms": 36.0, "age_pid_ms": 38.0, "age_encoders_ms": 9.0, "age_imu_ms": 1.0, "data_age_ms": 38.0,
            "pwms": [0, 0, 0, 0], "ticks": [0, 0, 0, 0],
            "yaw_deg": 0.14, "speeds_radps": [0.0, 0.0, 0.0, 0.0], "speeds_mps": [0.0, 0.0, 0.0, 0.0]
        }
        s1 = dict(base)
        s1["time_offset_s"] = 0.0079
        s2 = dict(base)
        s2["seq_drive"] = 740; s2["seq_pid"] = 370; s2["seq_encoders"] = 1480; s2["seq_imu"] = 2602
        s2["pwms"] = [-64, -64, -64, -64]; s2["ticks"] = [0, 0, 0, 0]
        s2["speeds_radps"] = [-0.17, -0.22, -0.22, -0.17]; s2["speeds_mps"] = [-0.0055, -0.0072, -0.0072, -0.0055]
        s3 = dict(base)
        s3["seq_drive"] = 741; s3["seq_pid"] = 370; s3["seq_encoders"] = 1482; s3["seq_imu"] = 2605
        s3["pwms"] = [-64, -64, -64, -64]; s3["ticks"] = [-14, -12, -12, -11]
        s3["speeds_radps"] = [-0.17, -0.22, -0.22, -0.17]; s3["speeds_mps"] = [-0.0055, -0.0072, -0.0072, -0.0055]

        # Hold frames under -40 PWM
        hold_snap = dict(s3)
        hold_snap["pwms"] = [-41, -41, -41, -41]
        hold_snap["speeds_radps"] = [-0.46, -0.46, -0.46, -0.46]
        hold_snap["speeds_mps"] = [-0.015, -0.015, -0.015, -0.015]

        run_snaps = [base, s1, s2, s3]
        call_idx = [0]
        def mock_replay_184808():
            call_idx[0] += 1
            now = time.monotonic()
            if call_idx[0] <= len(run_snaps):
                s = dict(run_snaps[call_idx[0]-1])
                s["fetch_start_mono"] = now
                s["receipt_mono"] = now
                return s
            else:
                s = dict(hold_snap)
                s["fetch_start_mono"] = now
                s["receipt_mono"] = now
                s["seq_drive"] = 741 + (call_idx[0] - len(run_snaps))
                s["seq_pid"] = 370 + (call_idx[0] - len(run_snaps)) // 2
                s["seq_encoders"] = 1482 + (call_idx[0] - len(run_snaps)) * 2
                s["seq_imu"] = 2605 + call_idx[0]
                s["ticks"] = [-14 - min(30, (call_idx[0] - len(run_snaps)))] * 4
                return s

        self.harness.fetch_drive_status = MagicMock(return_value={"armed": True, "mode": 3})
        self.harness.fetch_telemetry_snapshot = MagicMock(side_effect=mock_replay_184808)

        with patch.object(self.harness, "_execute_cleanup", return_value=True) as mock_cleanup:
            with patch("time.sleep", return_value=None):
                report = self.harness.execute_trial(skip_firmware_check=True)
                self.assertIsNotNone(report)
                self.assertEqual(report["overall_status"], "COMPLETED")
                self.assertFalse(report["aborted"])
                mock_cleanup.assert_called_once()
                calls = self.harness.send_open_loop_pwm.call_args_list
                # Calls during trial:
                # Call 0 (Sample 1): send -64
                # Call 1 (Sample 2): send -64
                # Call 2 (Sample 3): Frame 1 qualifies (deltas=[-14, -12, -12, -11]) -> PROMPT DISPATCH -40!
                self.assertGreaterEqual(len(calls), 3)
                self.assertEqual(calls[0][0], (-64, -64, -64, -64))
                self.assertEqual(calls[1][0], (-64, -64, -64, -64))
                self.assertEqual(calls[2][0], (-41, -41, -41, -41))
                # Verify that -64 is NEVER called again after Call 1
                for c in calls[2:]:
                    self.assertEqual(c[0], (-41, -41, -41, -41))

    def test_cleanup_records_full_stopped_travel_from_184808(self):
        """Cleanup captures full coasting travel through standstill matching 184808 data (16.2 mm)."""
        snap_cutoff = {
            "receipt_mono": 100.0, "fetch_start_mono": 100.0, "receipt_epoch_ms": 1000,
            "armed": False, "mode": 0,
            "seq_drive": 745, "seq_pid": 372, "seq_encoders": 1492, "seq_imu": 2616,
            "age_drive_ms": 68.0, "age_pid_ms": 70.0, "age_encoders_ms": 1.0, "age_imu_ms": 1.0, "data_age_ms": 70.0,
            "pwms": [-41, -41, -41, -41], "ticks": [-152, -144, -150, -147],
            "yaw_deg": 0.20, "speeds_radps": [-2.02, -1.96, -1.85, -1.95],
            "speeds_mps": [-0.0657, -0.0637, -0.0601, -0.0634]
        }
        snap_standstill = dict(snap_cutoff)
        snap_standstill["pwms"] = [0, 0, 0, 0]
        snap_standstill["ticks"] = [-157, -150, -156, -155]  # Exactly 16.2 mm on wheel 0
        snap_standstill["speeds_radps"] = [-0.01, -0.01, -0.01, -0.02]
        snap_standstill["speeds_mps"] = [-0.0003, -0.0003, -0.0003, -0.0007]

        call_idx = [0]
        def mock_cleanup_184808():
            call_idx[0] += 1
            if call_idx[0] == 1:
                return dict(snap_cutoff)
            return dict(snap_standstill)

        self.harness.fetch_telemetry_snapshot = MagicMock(side_effect=mock_cleanup_184808)
        self.harness.samples = [{
            "timestamp_mono": 100.0,
            "step_name": "Baseline",
            "raw_ticks": [0, 0, 0, 0],
            "max_accum_abs_travel_m": 0.0108,
            "net_yaw_drift_deg": 0.02
        }]
        self.harness.travel_tracker.reset([0, 0, 0, 0])

        with patch("time.sleep", return_value=None):
            cleanup_ok = self.harness._execute_cleanup()
            self.assertTrue(cleanup_ok)
            # Final sample captures final stopped displacement (-157 ticks = 16.2 mm)
            final_sample = self.harness.samples[-1]
            self.assertEqual(final_sample["step_name"], "Cleanup_Standstill")
            self.assertEqual(final_sample["raw_ticks"], [-157, -150, -156, -155])
            self.assertAlmostEqual(final_sample["max_accum_abs_travel_m"], 0.0162, places=4)

    def test_cleanup_records_encoder_samples_through_standstill(self):
        """Cleanup must immediately dispatch zero PWM and disarm, then record telemetry samples through standstill."""
        baseline_snap = {
            "receipt_mono": 100.0,
            "fetch_start_mono": 100.0,
            "receipt_epoch_ms": 1000,
            "armed": False,
            "mode": 0,
            "seq_drive": 100,
            "seq_pid": 100,
            "seq_encoders": 100,
            "seq_imu": 100,
            "age_drive_ms": 10,
            "age_pid_ms": 10,
            "age_encoders_ms": 10,
            "age_imu_ms": 10,
            "data_age_ms": 10,
            "pwms": [0, 0, 0, 0],
            "ticks": [-50, -50, -50, -50],
            "yaw_deg": 0.0,
            "speeds_radps": [-0.08, -0.08, -0.08, -0.08],
            "speeds_mps": [-0.0026, -0.0026, -0.0026, -0.0026]
        }
        standstill_snap = dict(baseline_snap)
        standstill_snap["ticks"] = [-75, -75, -75, -75]  # Final stopped displacement
        standstill_snap["speeds_radps"] = [0.0, 0.0, 0.0, 0.0]
        standstill_snap["speeds_mps"] = [0.0, 0.0, 0.0, 0.0]

        call_idx = [0]
        def mock_cleanup_snap():
            call_idx[0] += 1
            if call_idx[0] == 1:
                return dict(baseline_snap)
            return dict(standstill_snap)

        self.harness.fetch_telemetry_snapshot = MagicMock(side_effect=mock_cleanup_snap)
        self.harness.samples = [{
            "timestamp_mono": 100.0,
            "step_name": "Baseline",
            "raw_ticks": [0, 0, 0, 0],
            "max_accum_abs_travel_m": 0.005,
            "net_yaw_drift_deg": 0.0
        }]
        self.harness.travel_tracker.reset([0, 0, 0, 0])

        with patch("time.sleep", return_value=None):
            cleanup_ok = self.harness._execute_cleanup()
            self.assertTrue(cleanup_ok)
            # 1. Zero PWM and disarm were dispatched
            self.harness.send_open_loop_pwm.assert_called_with(0, 0, 0, 0)
            self.harness.disarm_drive.assert_called()
            # 2. Cleanup samples were recorded into self.samples
            step_names = [s["step_name"] for s in self.harness.samples]
            self.assertIn("Cleanup_Cutoff", step_names)
            self.assertIn("Cleanup_Standstill", step_names)
            # 3. Final sample captures final stopped displacement (-75 ticks = 7.8 mm)
            final_sample = self.harness.samples[-1]
            self.assertEqual(final_sample["step_name"], "Cleanup_Standstill")
            self.assertEqual(final_sample["raw_ticks"], [-75, -75, -75, -75])
            self.assertGreater(final_sample["max_accum_abs_travel_m"], 0.007)

    def test_travel_limit_abort(self):
        """Exceeding 100 mm travel triggers immediate abort."""
        self.harness.samples = [{"timestamp_mono": 100.0}]
        snap = {
            "receipt_epoch_ms": 1000,
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
            "pwms": [-41, -41, -41, -41],
            "ticks": [1000, 0, 0, 0],
            "yaw_deg": 0.0,
            "speeds_radps": [-0.46, -0.46, -0.46, -0.46],
            "speeds_mps": [-0.015, -0.015, -0.015, -0.015]
        }
        self.harness.travel_tracker.reset([0, 0, 0, 0])
        # Update with 1050 ticks = 108.6 mm > 100 mm limit
        snap["ticks"] = [1050, 0, 0, 0]
        self.harness._record_telemetry_sample(100.1, 100.0, -40, 1, snap, "Sustained_41")
        self.assertIsNotNone(self.harness.abort_reason)
        self.assertIn("EXCEEDED_MAX_ABS_TRAVEL", self.harness.abort_reason)


if __name__ == "__main__":
    unittest.main()

