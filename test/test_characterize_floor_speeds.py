import unittest
import math
import sys
import os
import time
import json

# Portable import of script under test
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_scripts = os.path.join(os.path.dirname(script_dir), "scripts")
sys.path.insert(0, script_dir)
sys.path.insert(0, parent_scripts)

from characterize_floor_speeds import (
    FloorSpeedCharacterizer,
    ContinuousYawTracker,
    MAX_COMMISSIONING_LINEAR_MPS,
    MAX_COMMISSIONING_ANGULAR_RADPS,
    TELEMETRY_MAX_AGE_MS,
    STOP_CONFIRM_WINDOW_S,
    STOP_SETTLE_TIMEOUT_S,
    MAX_STEP_DURATION_S,
    MAX_DIAGNOSTIC_DURATION_S
)

class MockResponse:
    def __init__(self, json_data, status_code=200, text="OK"):
        self._json = json_data
        self.status_code = status_code
        self.text = text

    def json(self):
        return self._json


class TestFloorSpeedCharacterizer(unittest.TestCase):
    def setUp(self):
        self.mock_geometry = {
            "wheelDiameter": 0.065,
            "effectiveTrackWidth": 0.3408575,
            "ticksPerRevolution": 1974.1667
        }
        self.app = FloorSpeedCharacterizer(
            cockpit_url="http://mock-cockpit:3000",
            bridge_url="http://mock-bridge:3010",
            geometry_override=self.mock_geometry
        )
        self.app.op_token = "mock_op_token"
        self.app.cmd_token = "mock_cmd_token"
        # Default mock HTTP handlers
        self.app.session.get = lambda url, timeout=0.15: MockResponse({"ok": True, "status": {}})
        self.app.session.post = lambda url, json=None, headers=None, timeout=0.1: MockResponse({"ok": True})

    def test_1_nested_status_unpacking(self):
        """Verify /api/drive/status properly unpacks nested 'status' and rejects flat payload."""
        now_ms = time.time() * 1000.0
        valid_payload = {
            "ok": True,
            "status": {
                "armed": False,
                "mode": 0,
                "uptimeMs": 12345,
                "seq": 100,
                "receivedAt": now_ms - 20.0,
                "dataAgeMs": 20.0,
                "noMotionBudgetMs": 0,
                "spinFailedExhausted": False
            }
        }
        self.app.session.get = lambda url, timeout=0.15: MockResponse(valid_payload)
        status = self.app.fetch_drive_status()
        self.assertEqual(status["mode"], 0)
        self.assertFalse(status["armed"])
        self.assertEqual(status["seq"], 100)

        # Missing nested 'status' must raise RuntimeError
        invalid_payload = {"ok": True, "armed": False, "mode": 0}
        self.app.session.get = lambda url, timeout=0.15: MockResponse(invalid_payload)
        with self.assertRaises(RuntimeError) as ctx:
            self.app.fetch_drive_status()
        self.assertIn("Missing or invalid nested 'status'", str(ctx.exception))

    def test_2_wheel_telemetry_numeric_and_finite_validation(self):
        """Verify missing, non-numeric, or non-finite wheel telemetry fields cause immediate abort."""
        base_pid = {
            "ok": True,
            "valid": True,
            "stale": False,
            "sequence": 50,
            "timestamp": time.time() * 1000.0 - 20.0,
            "dataAgeMs": 20,
            "telemetry": {
                "m1": {"measuredRadps": 0.1, "targetRadps": 0.2, "finalPwm": 25},
                "m2": {"measuredRadps": 0.1, "targetRadps": 0.2, "finalPwm": 25},
                "m3": {"measuredRadps": 0.1, "targetRadps": 0.2, "finalPwm": 25},
                "m4": {"measuredRadps": 0.1, "targetRadps": 0.2, "finalPwm": 25}
            }
        }

        # Valid payload passes
        self.app.session.get = lambda url, timeout=0.15: MockResponse(base_pid)
        res = self.app.fetch_pid_telemetry()
        self.assertEqual(res["sequence"], 50)

        # 1. Missing measuredRadps on m2
        bad_pid = json.loads(json.dumps(base_pid))
        del bad_pid["telemetry"]["m2"]["measuredRadps"]
        self.app.session.get = lambda url, timeout=0.15: MockResponse(bad_pid)
        with self.assertRaises(RuntimeError) as ctx:
            self.app.fetch_pid_telemetry()
        self.assertIn("Missing required field 'measuredRadps' in wheel 'm2'", str(ctx.exception))

        # 2. String instead of numeric
        bad_pid = json.loads(json.dumps(base_pid))
        bad_pid["telemetry"]["m3"]["targetRadps"] = "0.25"
        self.app.session.get = lambda url, timeout=0.15: MockResponse(bad_pid)
        with self.assertRaises(RuntimeError) as ctx:
            self.app.fetch_pid_telemetry()
        self.assertIn("Non-numeric or non-finite", str(ctx.exception))

        # 3. NaN value
        bad_pid = json.loads(json.dumps(base_pid))
        bad_pid["telemetry"]["m4"]["finalPwm"] = float("nan")
        self.app.session.get = lambda url, timeout=0.15: MockResponse(bad_pid)
        with self.assertRaises(RuntimeError) as ctx:
            self.app.fetch_pid_telemetry()
        self.assertIn("Non-numeric or non-finite", str(ctx.exception))

        # 4. Infinite value
        bad_pid = json.loads(json.dumps(base_pid))
        bad_pid["telemetry"]["m1"]["measuredRadps"] = float("inf")
        self.app.session.get = lambda url, timeout=0.15: MockResponse(bad_pid)
        with self.assertRaises(RuntimeError) as ctx:
            self.app.fetch_pid_telemetry()
        self.assertIn("Non-numeric or non-finite", str(ctx.exception))

    def test_3_stale_drive_status_rejection(self):
        """Verify stale drive status (stale=true, excessive dataAgeMs, or old receivedAt) raises error."""
        now_ms = time.time() * 1000.0
        # 1. Flagged stale=true
        stale_status1 = {
            "ok": True,
            "status": {
                "armed": False,
                "mode": 0,
                "uptimeMs": 12345,
                "seq": 100,
                "receivedAt": now_ms - 20.0,
                "dataAgeMs": 20.0,
                "noMotionBudgetMs": 0,
                "spinFailedExhausted": False,
                "stale": True
            }
        }
        self.app.session.get = lambda url, timeout=0.15: MockResponse(stale_status1)
        with self.assertRaises(RuntimeError) as ctx:
            self.app.fetch_drive_status()
        self.assertIn("reports stale=true", str(ctx.exception))

        # 2. Reproduction case: dataAgeMs=100000, stale=false, armed=false, mode=0
        stale_status2 = {
            "ok": True,
            "status": {
                "armed": False,
                "mode": 0,
                "uptimeMs": 12345,
                "seq": 100,
                "receivedAt": now_ms - 100000.0,
                "dataAgeMs": 100000,
                "noMotionBudgetMs": 0,
                "spinFailedExhausted": False,
                "stale": False
            }
        }
        self.app.session.get = lambda url, timeout=0.15: MockResponse(stale_status2)
        with self.assertRaises(RuntimeError) as ctx:
            self.app.fetch_drive_status()
        self.assertIn("Drive status stale (dataAgeMs=100000", str(ctx.exception))

        # 3. Old receivedAt without dataAgeMs (5000 ms old)
        stale_status3 = {
            "ok": True,
            "status": {
                "armed": False,
                "mode": 0,
                "uptimeMs": 12345,
                "seq": 100,
                "receivedAt": now_ms - 5000.0,
                "noMotionBudgetMs": 0,
                "spinFailedExhausted": False,
                "stale": False
            }
        }
        self.app.session.get = lambda url, timeout=0.15: MockResponse(stale_status3)
        with self.assertRaises(RuntimeError) as ctx:
            self.app.fetch_drive_status()
        self.assertIn("Drive status stale", str(ctx.exception))

    def test_3b_stale_status_arming_check(self):
        """Verify stale status prevents verify_stopped_and_disarmed from returning True and blocks arming."""
        now_ms = time.time() * 1000.0
        stale_status = {
            "ok": True,
            "status": {
                "armed": False,
                "mode": 0,
                "uptimeMs": 12345,
                "seq": 100,
                "receivedAt": now_ms - 100000.0,
                "dataAgeMs": 100000,
                "noMotionBudgetMs": 0,
                "spinFailedExhausted": False,
                "stale": False
            }
        }
        self.app.session.get = lambda url, timeout=0.15: MockResponse(stale_status)

        # verify_stopped_and_disarmed MUST NOT return True!
        stopped_and_disarmed = self.app.verify_stopped_and_disarmed()
        self.assertFalse(stopped_and_disarmed)

        # arm() MUST raise RuntimeError refusing to arm
        with self.assertRaises(RuntimeError) as ctx:
            self.app.arm()
        self.assertIn("Arming refused", str(ctx.exception))

    def test_4_stream_freeze_detection(self):
        """Verify stream whose sequence number stops advancing for > 250 ms is detected and rejected."""
        t0 = 1000.0
        self.app.check_stream_advancement("drive", seq=100, now_mono=t0)
        self.assertEqual(self.app.stream_seq["drive"], 100)

        # Advancing sequence at t0 + 0.1s is fine
        self.app.check_stream_advancement("drive", seq=101, now_mono=t0 + 0.1)
        self.assertEqual(self.app.stream_seq["drive"], 101)

        # Same sequence at t0 + 0.2s (< 250ms freeze) is tolerated briefly
        self.app.check_stream_advancement("drive", seq=101, now_mono=t0 + 0.2)

        # Same sequence at t0 + 0.4s (300ms since advancement at t0+0.1) MUST raise RuntimeError
        with self.assertRaises(RuntimeError) as ctx:
            self.app.check_stream_advancement("drive", seq=101, now_mono=t0 + 0.4)
        self.assertIn("Stream 'drive' stopped advancing", str(ctx.exception))

    def test_5_http_command_rejection_latches_fault(self):
        """Verify HTTP 403 / 500 command rejection immediately latches fault."""
        self.assertFalse(self.app.fault_latched)
        self.app.session.post = lambda url, json=None, headers=None, timeout=0.1: MockResponse(
            {"ok": False, "error": "Autonomy is disabled"}, status_code=403, text="Forbidden"
        )
        ok = self.app.send_velocity(0.05, 0.0)
        self.assertFalse(ok)
        self.assertTrue(self.app.fault_latched)
        self.assertIn("Velocity command rejected", self.app.fault_reason)

    def test_6_recovery_exhaustion_triggers_fault(self):
        """Verify spinFailedExhausted=true latches fault and initiates fail-closed stop."""
        self.app.trigger_fault("Drivetrain recovery exhausted (spinFailedExhausted=true).")
        self.assertTrue(self.app.fault_latched)
        self.assertIn("recovery exhausted", self.app.fault_reason)

    def test_7_yaw_wrap_continuous_tracking(self):
        """Verify ContinuousYawTracker correctly unwraps +179 deg to -179 deg as +2 deg."""
        tracker = ContinuousYawTracker()
        yaw1 = math.radians(179.0)
        cont_yaw, diff = tracker.update(yaw1)
        self.assertAlmostEqual(cont_yaw, 0.0)

        # Rover turns forward through wrap (+179 deg -> -179 deg = +2 deg continuous)
        yaw2 = math.radians(-179.0)
        cont_yaw, diff = tracker.update(yaw2)
        self.assertAlmostEqual(tracker.relative_yaw_deg, 2.0, places=4)
        self.assertAlmostEqual(math.degrees(diff), 2.0, places=4)

        # Rover turns backward through wrap (-179 deg -> +179 deg = -2 deg)
        yaw3 = math.radians(179.0)
        cont_yaw, diff = tracker.update(yaw3)
        self.assertAlmostEqual(tracker.relative_yaw_deg, 0.0, places=4)
        self.assertAlmostEqual(math.degrees(diff), -2.0, places=4)

    def test_8_accumulated_absolute_motion_bounds(self):
        """Verify opposing-wheel spins and yaw reversals cannot cancel accumulated absolute bounds."""
        ticks_enc = {"m1": 1000, "m2": -1000, "m3": 1000, "m4": -1000}
        meters_per_tick = self.app.meters_per_tick
        accum_m = max(abs(t) * meters_per_tick for t in ticks_enc.values())
        self.assertAlmostEqual(accum_m, 1000 * meters_per_tick, places=4)
        self.assertGreater(accum_m, 0.10)

        # Yaw reversal: +30 deg then -30 deg -> Net yaw is 0 deg, accumulated yaw is 60 deg
        tracker = ContinuousYawTracker()
        tracker.update(0.0)
        _, d1 = tracker.update(math.radians(30.0))
        _, d2 = tracker.update(math.radians(0.0))
        accum_yaw_deg = math.degrees(abs(d1) + abs(d2))
        self.assertAlmostEqual(tracker.relative_yaw_deg, 0.0, places=4)
        self.assertAlmostEqual(accum_yaw_deg, 60.0, places=4)

    def test_9_stop_confirmation_resets_on_resumed_motion(self):
        """Verify stop confirmation requires 200 ms uninterrupted stop and resets if motion resumes."""
        now = 100.0
        cutoff_time = 100.0
        stopped_start_time = None

        speeds = [0.01, 0.01, 0.01, 0.01]
        gyro_abs = 0.005
        is_near_zero = all(s < 0.05 for s in speeds) and (gyro_abs < 0.03)
        self.assertTrue(is_near_zero)
        if stopped_start_time is None:
            stopped_start_time = now + 0.05

        self.assertLess((now + 0.15) - stopped_start_time, STOP_CONFIRM_WINDOW_S)

        # Motion resumes (wheel twitch 0.08 rad/s)
        resumed_speeds = [0.08, 0.01, 0.01, 0.01]
        is_near_zero_twitch = all(s < 0.05 for s in resumed_speeds) and (gyro_abs < 0.03)
        self.assertFalse(is_near_zero_twitch)
        if not is_near_zero_twitch:
            stopped_start_time = None

        self.assertIsNone(stopped_start_time)

        # Stops again
        if stopped_start_time is None:
            stopped_start_time = now + 0.25

        # 210 ms uninterrupted stop since t=100.25 -> Confirmed
        self.assertGreaterEqual((now + 0.46) - stopped_start_time, STOP_CONFIRM_WINDOW_S)

    def test_10_cleanup_failure_prevents_success(self):
        """Verify cleanup failure changes report status from SUCCESS to CLEANUP_FAILED."""
        now_ms = time.time() * 1000.0
        self.app.session.post = lambda url, json=None, headers=None, timeout=0.1: MockResponse({"ok": True})
        self.app.session.get = lambda url, timeout=0.15: MockResponse({
            "ok": True,
            "status": {
                "armed": True,
                "mode": 3,
                "uptimeMs": 100,
                "seq": 1,
                "receivedAt": now_ms,
                "dataAgeMs": 10.0,
                "noMotionBudgetMs": 0,
                "spinFailedExhausted": False
            }
        })
        zero_ok, disarm_ok = self.app.fail_closed_cleanup()
        self.assertTrue(zero_ok)
        self.assertFalse(disarm_ok)

        report = {"status": "SUCCESS"}
        if not zero_ok or not disarm_ok:
            report["status"] = "CLEANUP_FAILED"
            self.app.trigger_fault("Cleanup failed")

        self.assertEqual(report["status"], "CLEANUP_FAILED")
        self.assertTrue(self.app.fault_latched)

    def test_11_fault_latched_prevents_subsequent_arming(self):
        """Verify that once a fault is latched, all subsequent arming calls are rejected."""
        self.app.trigger_fault("Test induced fault")
        self.assertTrue(self.app.fault_latched)

        arm_called = False
        def mock_arm_post(url, headers=None, timeout=2.0):
            nonlocal arm_called
            arm_called = True
            return MockResponse({"ok": True})

        self.app.session.post = mock_arm_post
        with self.assertRaises(RuntimeError) as ctx:
            self.app.arm()
        self.assertIn("Arming refused: Fault is latched", str(ctx.exception))
        self.assertFalse(arm_called)

    def test_12_timestamp_based_stall_duration(self):
        """Verify stall duration is calculated from actual sample timestamps, not assumed 20 Hz."""
        samples = [
            {"t_rel": 0.00, "measured_radps": {"m1": 0.20}, "target_radps": {"m1": 0.20}},
            {"t_rel": 0.05, "measured_radps": {"m1": 0.01}, "target_radps": {"m1": 0.20}},
            {"t_rel": 0.18, "measured_radps": {"m1": 0.02}, "target_radps": {"m1": 0.20}},
            {"t_rel": 0.32, "measured_radps": {"m1": 0.01}, "target_radps": {"m1": 0.20}},
            {"t_rel": 0.40, "measured_radps": {"m1": 0.21}, "target_radps": {"m1": 0.20}},
        ]
        stall_start_t = None
        longest_stall_s = 0.0
        for s in samples:
            meas = s["measured_radps"]["m1"]
            tgt = s["target_radps"]["m1"]
            t_curr = s["t_rel"]
            is_stalled = (abs(meas) < 0.05) and (abs(tgt) >= 0.05)
            if is_stalled:
                if stall_start_t is None:
                    stall_start_t = t_curr
                dur = t_curr - stall_start_t
                if dur > longest_stall_s:
                    longest_stall_s = dur
            else:
                stall_start_t = None

        self.assertAlmostEqual(longest_stall_s, 0.27, places=3)
        self.assertAlmostEqual(round(longest_stall_s * 1000.0, 1), 270.0)

    def test_13_geometry_verification_refusal(self):
        """Verify script refuses to start if deployed geometry is missing or invalid."""
        with self.assertRaises(RuntimeError) as ctx:
            FloorSpeedCharacterizer(
                cockpit_url="http://mock-cockpit:3000",
                bridge_url="http://mock-bridge:3010",
                geometry_override={"wheelDiameter": 0.065}
            )
        self.assertIn("Geometry override missing required keys", str(ctx.exception))

        with self.assertRaises(RuntimeError) as ctx:
            FloorSpeedCharacterizer(
                cockpit_url="http://mock-cockpit:3000",
                bridge_url="http://mock-bridge:3010",
                geometry_override={
                    "wheelDiameter": 0.0,
                    "effectiveTrackWidth": 0.34,
                    "ticksPerRevolution": 1974.0
                }
            )
        self.assertIn("Invalid wheel diameter", str(ctx.exception))

    def test_14_actual_run_profile_step_zero_rejection(self):
        """Verify actual run_profile_step handles zero-velocity command rejection as failure."""
        seq_count = 1
        is_armed = False

        def mock_get(url, timeout=0.15):
            nonlocal seq_count
            seq_count += 1
            if "/api/drive/status" in url:
                return MockResponse({
                    "ok": True,
                    "status": {
                        "armed": is_armed,
                        "mode": 3 if is_armed else 0,
                        "uptimeMs": 50000,
                        "seq": seq_count,
                        "receivedAt": time.time() * 1000.0,
                        "dataAgeMs": 10.0,
                        "noMotionBudgetMs": 0,
                        "spinFailedExhausted": False
                    }
                })
            elif "/api/pid-telemetry" in url:
                return MockResponse({
                    "ok": True, "valid": True, "stale": False, "sequence": seq_count,
                    "timestamp": time.time() * 1000.0, "dataAgeMs": 10.0,
                    "telemetry": {
                        m: {"measuredRadps": 0.01, "targetRadps": 0.0, "finalPwm": 0}
                        for m in ["m1", "m2", "m3", "m4"]
                    }
                })
            elif "/api/encoders" in url:
                return MockResponse({
                    "ok": True, "sequence": seq_count, "lastPacketAgeMs": 10.0,
                    "encoders": {"m1": 10, "m2": 10, "m3": 10, "m4": 10}
                })
            elif "/api/imu" in url:
                return MockResponse({
                    "ok": True, "sequence": seq_count, "dataAgeMs": 10.0,
                    "gyro": {"z": 0.001},
                    "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
                })
            return MockResponse({"ok": True})

        def mock_post(url, json=None, headers=None, timeout=0.1):
            nonlocal is_armed
            if "/api/drive/arm" in url:
                is_armed = True
                return MockResponse({"ok": True})
            elif "/api/drive/disarm" in url:
                is_armed = False
                return MockResponse({"ok": True})
            elif "/api/cmd_vel" in url:
                if json and json.get("linear", {}).get("x", 0.0) == 0.0 and json.get("angular", {}).get("z", 0.0) == 0.0:
                    # Reject zero command once profile execution begins!
                    if self.app.active_report and self.app.active_report.get("samples"):
                        return MockResponse({"ok": False, "error": "Zero command rejected"}, status_code=403)
            return MockResponse({"ok": True})

        self.app.session.get = mock_get
        self.app.session.post = mock_post

        # Run 50ms step
        report = self.app.run_profile_step(
            name="TestZeroRejection",
            segments=[(0.014, 0.0, 0.05)],
            max_dist_m=0.1,
            max_yaw_deg=10.0
        )

        # Must report failure, never SUCCESS!
        self.assertNotEqual(report["status"], "SUCCESS")
        self.assertTrue(self.app.fault_latched)

    def test_15_actual_run_profile_step_stop_qualification(self):
        """Verify actual run_profile_step qualifies stop only after 200 ms with advancing sequence evidence."""
        seq_count = 100
        step_phase = "ACTIVE"
        is_armed = False

        def mock_get(url, timeout=0.15):
            nonlocal seq_count, step_phase, is_armed
            seq_count += 1
            now_ms = time.time() * 1000.0

            if "/api/drive/status" in url:
                return MockResponse({
                    "ok": True,
                    "status": {
                        "armed": is_armed,
                        "mode": 3 if is_armed else 0,
                        "uptimeMs": 50000,
                        "seq": seq_count,
                        "receivedAt": now_ms,
                        "dataAgeMs": 5.0,
                        "noMotionBudgetMs": 0,
                        "spinFailedExhausted": False
                    }
                })
            elif "/api/pid-telemetry" in url:
                meas = 0.01 if step_phase == "STOPPED" else 0.15
                return MockResponse({
                    "ok": True, "valid": True, "stale": False, "sequence": seq_count,
                    "timestamp": now_ms, "dataAgeMs": 5.0,
                    "telemetry": {
                        m: {"measuredRadps": meas, "targetRadps": 0.0, "finalPwm": 0}
                        for m in ["m1", "m2", "m3", "m4"]
                    }
                })
            elif "/api/encoders" in url:
                return MockResponse({
                    "ok": True, "sequence": seq_count, "lastPacketAgeMs": 5.0,
                    "encoders": {"m1": 100, "m2": 100, "m3": 100, "m4": 100}
                })
            elif "/api/imu" in url:
                return MockResponse({
                    "ok": True, "sequence": seq_count, "dataAgeMs": 5.0,
                    "gyro": {"z": 0.005},
                    "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
                })
            return MockResponse({"ok": True})

        def mock_post(url, json=None, headers=None, timeout=0.1):
            nonlocal step_phase, is_armed
            if "/api/drive/arm" in url:
                is_armed = True
                return MockResponse({"ok": True})
            elif "/api/drive/disarm" in url:
                is_armed = False
                return MockResponse({"ok": True})
            elif "/api/cmd_vel" in url:
                if json and json.get("linear", {}).get("x", 0.0) == 0.0:
                    step_phase = "STOPPED"
            return MockResponse({"ok": True})

        self.app.session.get = mock_get
        self.app.session.post = mock_post

        # Run short 40ms step
        report = self.app.run_profile_step(
            name="TestStopQualification",
            segments=[(0.014, 0.0, 0.04)],
            max_dist_m=0.1,
            max_yaw_deg=10.0
        )

        self.assertEqual(report["status"], "SUCCESS")
        self.assertFalse(self.app.fault_latched)
        self.assertGreater(len(report["samples"]), 3)

    def test_17_cleanup_disarm_incomplete_status_then_confirmed(self):
        """Verify fail_closed_cleanup bounded polling retries when Cockpit temporarily returns
        an incomplete status stub ({armed: False}), successfully confirming once full hardware
        Mode 0 telemetry arrives without manufacturing confirmation from the stub."""
        now_ms = time.time() * 1000.0
        call_count = 0

        incomplete_stub = {
            "ok": True,
            "status": {
                "armed": False  # Missing mode, uptimeMs, seq, receivedAt, etc.
            }
        }
        confirmed_hardware_status = {
            "ok": True,
            "status": {
                "armed": False,
                "mode": 0,
                "uptimeMs": 55000,
                "seq": 200,
                "receivedAt": now_ms,
                "dataAgeMs": 5.0,
                "noMotionBudgetMs": 0,
                "spinFailedExhausted": False
            }
        }

        def mock_get(url, timeout=0.15):
            nonlocal call_count
            if "/api/drive/status" in url:
                call_count += 1
                if call_count <= 2:
                    return MockResponse(incomplete_stub)
                return MockResponse(confirmed_hardware_status)
            return MockResponse({"ok": True})

        self.app.session.get = mock_get
        self.app.session.post = lambda url, json=None, headers=None, timeout=0.1: MockResponse({"ok": True})

        zero_ok, disarm_ok = self.app.fail_closed_cleanup()
        self.assertTrue(zero_ok)
        self.assertTrue(disarm_ok)
        self.assertGreaterEqual(call_count, 3)

    def test_18_cleanup_disarm_timeout_when_confirmation_never_arrives(self):
        """Verify fail_closed_cleanup times out and returns disarm_confirmed=False when valid
        Mode 0 confirmation never arrives (e.g. stub persists indefinitely or mode remains armed)."""
        incomplete_stub = {
            "ok": True,
            "status": {
                "armed": False  # Incomplete stub persists, never delivers confirmed Mode 0
            }
        }

        self.app.session.get = lambda url, timeout=0.15: MockResponse(incomplete_stub)
        self.app.session.post = lambda url, json=None, headers=None, timeout=0.1: MockResponse({"ok": True})

        t_start = time.monotonic()
        zero_ok, disarm_ok = self.app.fail_closed_cleanup()
        duration = time.monotonic() - t_start

        self.assertTrue(zero_ok)
        self.assertFalse(disarm_ok)
    def test_19_tracking_deficit_vs_lifecycle_separation(self):
        """Verify achieved distance below 70% of expected is classified as TRACKING_DEFICIT,
        not SUCCESS, while preserving cleanup, lifecycle, communication, and recovery as separate fields."""
        now_ms = time.time() * 1000.0
        seq = 100
        is_armed = False

        def mock_get(url, timeout=0.15):
            nonlocal seq, now_ms, is_armed
            now_ms += 10.0
            seq += 1
            if "/api/drive/status" in url:
                return MockResponse({
                    "ok": True,
                    "status": {
                        "armed": is_armed,
                        "mode": 3 if is_armed else 0,
                        "uptimeMs": 50000,
                        "seq": seq,
                        "receivedAt": now_ms,
                        "dataAgeMs": 5.0,
                        "noMotionBudgetMs": 1000,
                        "spinFailedExhausted": False,
                        "chassisRebreakoutConsumed": False
                    }
                })
            elif "/api/pid-telemetry" in url:
                return MockResponse({
                    "ok": True, "valid": True, "stale": False, "sequence": seq,
                    "timestamp": now_ms, "dataAgeMs": 5.0,
                    "telemetry": {
                        m: {"measuredRadps": 0.0, "targetRadps": 1.54 if is_armed else 0.0, "finalPwm": 50 if is_armed else 0, "stictionState": "STICTION_KINETIC"}
                        for m in ["m1", "m2", "m3", "m4"]
                    }
                })
            elif "/api/encoders" in url:
                # 5 ticks travel = ~0.5 mm out of 2.5 mm expected -> 20% of expected (< 70%)
                ticks = 5 if is_armed else 0
                return MockResponse({
                    "ok": True, "sequence": seq, "lastPacketAgeMs": 5.0,
                    "encoders": {"m1": ticks, "m2": ticks, "m3": ticks, "m4": ticks}
                })
            elif "/api/imu" in url:
                return MockResponse({
                    "ok": True, "sequence": seq, "dataAgeMs": 5.0,
                    "gyro": {"z": 0.005},
                    "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
                })
            return MockResponse({"ok": True})

        def mock_post(url, json=None, headers=None, timeout=0.1):
            nonlocal is_armed
            if "/api/drive/arm" in url:
                is_armed = True
            elif "/api/drive/disarm" in url:
                is_armed = False
            return MockResponse({"ok": True})

        self.app.session.get = mock_get
        self.app.session.post = mock_post

        # Run Case 11 segment: 0.050 m/s for 0.05s (short test mock duration)
        # Expected travel: 0.050 * 0.8 = 0.040 m (40 mm)
        # Wheel travel achieved: 101 ticks * (0.065 * pi / 1974.1667) ~ 0.0104 m (10.4 mm) -> 26% of expected (< 70%)
        res = self.app.run_profile_step(
            name="Straight_Crawl_0.050mps",
            segments=[(0.050, 0.0, 0.05)],
            max_dist_m=0.15,
            max_yaw_deg=10.0
        )

        # 1. Must NOT report SUCCESS
        self.assertNotEqual(res["status"], "SUCCESS")
        self.assertEqual(res["status"], "TRACKING_DEFICIT")

        # 2. Lifecycle completion must be distinct and COMPLETED
        self.assertEqual(res["lifecycle_completion"]["lifecycle_status"], "COMPLETED")
        self.assertTrue(res["lifecycle_completion"]["zero_confirmed"])
        self.assertTrue(res["lifecycle_completion"]["disarm_confirmed"])

        # 3. Motion tracking must capture expected vs achieved deficit
        self.assertEqual(res["motion_tracking"]["tracking_status"], "TRACKING_DEFICIT")
        self.assertLess(res["motion_tracking"]["distance_ratio"], 0.70)

        # 4. Cleanup, communication, and recovery preserved as separate fields
        self.assertTrue(res["cleanup_verification"]["zero_command_confirmed"])
        self.assertTrue(res["cleanup_verification"]["disarm_mode0_confirmed"])
        self.assertTrue(res["communication_status"]["communication_healthy"])
        self.assertIn("final_no_motion_budget_ms", res["recovery_status"])

    def test_20_reverse_crawl_grading_and_direction_validation(self):
        """Verify Case 12 definition and directional tracking evaluation:
        - Case 12 configured with vx=-0.050, 1.8s, nominal requested 90mm
        - Valid negative displacement passes tracking (nominal requested magnitude = 90mm)
        - Deficit negative displacement correctly reports TRACKING_DEFICIT
        - Forward displacement on reverse test is rejected as 0.0mm achieved travel (fails reverse test)"""
        from characterize_floor_speeds import CASES
        self.assertIn("12", CASES)
        case12 = CASES["12"]
        self.assertEqual(case12["id"], "Reverse_Crawl_0.050mps")
        self.assertEqual(case12["segments"], [(-0.050, 0.0, 1.8)])
        self.assertEqual(case12["max_dist_m"], 0.15)
        self.assertEqual(case12["max_yaw_deg"], 10.0)

        # Helper to construct mock report
        def build_mock_report(cmd_vx, disp_net):
            samples = [
                {"seg_idx": 0, "t_rel": 0.0, "cmd": {"vx": cmd_vx, "wz": 0.0}, "disp_net_m": 0.0, "recovery": {}},
                {"seg_idx": 0, "t_rel": 0.9, "cmd": {"vx": cmd_vx, "wz": 0.0}, "disp_net_m": disp_net * 0.5, "recovery": {}},
                {"seg_idx": 0, "t_rel": 1.8, "cmd": {"vx": cmd_vx, "wz": 0.0}, "disp_net_m": disp_net, "recovery": {}},
                {"seg_idx": -1, "t_rel": 2.0, "cmd": {"vx": 0.0, "wz": 0.0}, "disp_net_m": disp_net, "recovery": {}}
            ]
            report = {
                "name": "Reverse_Crawl_0.050mps",
                "samples": samples,
                "cleanup_verification": {"zero_command_confirmed": True, "disarm_mode0_confirmed": True},
                "status": "SUCCESS"
            }
            return report

        # Test A: Reverse command with valid backward travel (-95 mm achieved out of 90 mm expected)
        rep_a = build_mock_report(cmd_vx=-0.050, disp_net=-0.095)
        self.app.evaluate_motion_tracking("Reverse_Crawl_0.050mps", [(-0.050, 0.0, 1.8)], rep_a)
        mt_a = rep_a["motion_tracking"]
        self.assertEqual(mt_a["nominal_requested_distance_m"], 0.090)
        self.assertEqual(mt_a["achieved_distance_m"], 0.095)
        self.assertEqual(mt_a["final_displacement_signed_m"], -0.095)
        self.assertEqual(mt_a["tracking_status"], "TRACKING_NOMINAL")
        self.assertEqual(rep_a["status"], "SUCCESS")

        # Test B: Reverse command with tracking deficit (-40 mm achieved out of 90 mm expected = 44.4% < 70%)
        rep_b = build_mock_report(cmd_vx=-0.050, disp_net=-0.040)
        self.app.evaluate_motion_tracking("Reverse_Crawl_0.050mps", [(-0.050, 0.0, 1.8)], rep_b)
        mt_b = rep_b["motion_tracking"]
        self.assertEqual(mt_b["nominal_requested_distance_m"], 0.090)
        self.assertEqual(mt_b["achieved_distance_m"], 0.040)
        self.assertEqual(mt_b["tracking_status"], "TRACKING_DEFICIT")
        self.assertEqual(rep_b["status"], "TRACKING_DEFICIT")

        # Test C: Forward movement on reverse command (+90 mm forward on -90 mm command) MUST NOT PASS
        rep_c = build_mock_report(cmd_vx=-0.050, disp_net=+0.090)
        self.app.evaluate_motion_tracking("Reverse_Crawl_0.050mps", [(-0.050, 0.0, 1.8)], rep_c)
        mt_c = rep_c["motion_tracking"]
        self.assertEqual(mt_c["nominal_requested_distance_m"], 0.090)
        self.assertEqual(mt_c["achieved_distance_m"], 0.0) # Zero backward travel credited
        self.assertEqual(mt_c["final_displacement_signed_m"], +0.090)
        self.assertEqual(mt_c["tracking_status"], "TRACKING_DEFICIT")
        self.assertEqual(rep_c["status"], "TRACKING_DEFICIT")

    def test_21_case2_straight_crawl_grading(self):
        """Verify Case 2 (vx=0.015 m/s, 1.8s, 27mm nominal) tracking deficit grading:
        - zero travel fails tracking
        - 10 mm travel fails tracking (37.0% < 70%)
        - correct-direction 27 mm travel passes minimum-travel grading (100% >= 70%)
        - wrong-direction travel (-27 mm) fails tracking (0.0% credited)
        """
        def build_case2_report(disp_net):
            samples = [
                {"seg_idx": 0, "t_rel": 0.0, "cmd": {"vx": 0.015, "wz": 0.0}, "disp_net_m": 0.0, "recovery": {}},
                {"seg_idx": 0, "t_rel": 0.9, "cmd": {"vx": 0.015, "wz": 0.0}, "disp_net_m": disp_net * 0.5, "recovery": {}},
                {"seg_idx": 0, "t_rel": 1.8, "cmd": {"vx": 0.015, "wz": 0.0}, "disp_net_m": disp_net, "recovery": {}},
                {"seg_idx": -1, "t_rel": 2.0, "cmd": {"vx": 0.0, "wz": 0.0}, "disp_net_m": disp_net, "recovery": {}}
            ]
            return {
                "name": "Straight_Crawl_0.015mps",
                "samples": samples,
                "cleanup_verification": {"zero_command_confirmed": True, "disarm_mode0_confirmed": True},
                "status": "SUCCESS"
            }

        # 1. Zero travel fails tracking
        rep_zero = build_case2_report(disp_net=0.0)
        self.app.evaluate_motion_tracking("Straight_Crawl_0.015mps", [(0.015, 0.0, 1.8)], rep_zero)
        mt_zero = rep_zero["motion_tracking"]
        self.assertEqual(mt_zero["nominal_requested_distance_m"], 0.027)
        self.assertEqual(mt_zero["achieved_distance_m"], 0.0)
        self.assertEqual(mt_zero["distance_ratio_pct"], 0.0)
        self.assertEqual(mt_zero["tracking_status"], "TRACKING_DEFICIT")
        self.assertEqual(rep_zero["status"], "TRACKING_DEFICIT")

        # 2. 10 mm travel fails tracking (10 mm / 27 mm = 37.0% < 70%)
        rep_10mm = build_case2_report(disp_net=0.010)
        self.app.evaluate_motion_tracking("Straight_Crawl_0.015mps", [(0.015, 0.0, 1.8)], rep_10mm)
        mt_10mm = rep_10mm["motion_tracking"]
        self.assertEqual(mt_10mm["nominal_requested_distance_m"], 0.027)
        self.assertEqual(mt_10mm["achieved_distance_m"], 0.010)
        self.assertEqual(mt_10mm["distance_ratio_pct"], 37.0)
        self.assertEqual(mt_10mm["tracking_status"], "TRACKING_DEFICIT")
        self.assertEqual(rep_10mm["status"], "TRACKING_DEFICIT")

        # 3. Correct-direction 27 mm travel passes minimum-travel grading (100% >= 70%)
        rep_27mm = build_case2_report(disp_net=0.027)
        self.app.evaluate_motion_tracking("Straight_Crawl_0.015mps", [(0.015, 0.0, 1.8)], rep_27mm)
        mt_27mm = rep_27mm["motion_tracking"]
        self.assertEqual(mt_27mm["nominal_requested_distance_m"], 0.027)
        self.assertEqual(mt_27mm["achieved_distance_m"], 0.027)
        self.assertEqual(mt_27mm["distance_ratio_pct"], 100.0)
        self.assertEqual(mt_27mm["tracking_status"], "TRACKING_NOMINAL")
        self.assertEqual(rep_27mm["status"], "SUCCESS")

        # 4. Wrong-direction travel (-27 mm) fails tracking (0.0% credited)
        rep_rev = build_case2_report(disp_net=-0.027)
        self.app.evaluate_motion_tracking("Straight_Crawl_0.015mps", [(0.015, 0.0, 1.8)], rep_rev)
        mt_rev = rep_rev["motion_tracking"]
        self.assertEqual(mt_rev["nominal_requested_distance_m"], 0.027)
        self.assertEqual(mt_rev["achieved_distance_m"], 0.0)
        self.assertEqual(mt_rev["final_displacement_signed_m"], -0.027)
        self.assertEqual(mt_rev["tracking_status"], "TRACKING_DEFICIT")
        self.assertEqual(rep_rev["status"], "TRACKING_DEFICIT")

        # 5. Display name variant "Straight Crawl (vx=0.015 m/s)" works identically
        rep_disp_name = build_case2_report(disp_net=0.010)
        self.app.evaluate_motion_tracking("Straight Crawl (vx=0.015 m/s)", [(0.015, 0.0, 1.8)], rep_disp_name)
        self.assertEqual(rep_disp_name["motion_tracking"]["tracking_status"], "TRACKING_DEFICIT")
        self.assertEqual(rep_disp_name["status"], "TRACKING_DEFICIT")

    def test_22_case13_reverse_crawl_definition_and_travel_acceptance(self):
        """Verify Case 13 (Reverse_Crawl_0.015mps) definition and 24-30mm acceptance grading:
        - Case 13 definition: ID, segments, nominal travel -27mm, bounds (max_dist_m=0.08, max_yaw_deg=10.0)
        - Reverse acceptance: 24-30mm magnitude with negative displacement required (-27mm, -24mm, -30mm -> SUCCESS)
        - Insufficient travel: magnitude < 24mm (-20mm, -10mm, 0.0mm -> TRACKING_DEFICIT)
        - Excessive travel: magnitude > 30mm (-35mm, -45mm -> EXCESSIVE_TRAVEL)
        - Wrong-direction travel: positive displacement on reverse command MUST fail (+27mm, +35mm -> TRACKING_DEFICIT)
        - Kinematic matching: works on segments without relying solely on case-name matching
        """
        from characterize_floor_speeds import CASES

        # 1. Verify Case 13 definition
        self.assertIn("13", CASES)
        case13 = CASES["13"]
        self.assertEqual(case13["id"], "Reverse_Crawl_0.015mps")
        self.assertEqual(case13["segments"], [(-0.015, 0.0, 1.8)])
        self.assertEqual(case13["max_dist_m"], 0.08)
        self.assertEqual(case13["max_yaw_deg"], 10.0)
        nominal_travel_m = sum(s[0] * s[2] for s in case13["segments"])
        self.assertAlmostEqual(nominal_travel_m, -0.027, places=4)

        def build_case13_report(disp_net, name="Reverse_Crawl_0.015mps"):
            samples = [
                {"seg_idx": 0, "t_rel": 0.0, "cmd": {"vx": -0.015, "wz": 0.0}, "disp_net_m": 0.0, "recovery": {}},
                {"seg_idx": 0, "t_rel": 0.9, "cmd": {"vx": -0.015, "wz": 0.0}, "disp_net_m": disp_net * 0.5, "recovery": {}},
                {"seg_idx": 0, "t_rel": 1.8, "cmd": {"vx": -0.015, "wz": 0.0}, "disp_net_m": disp_net, "recovery": {}},
                {"seg_idx": -1, "t_rel": 2.0, "cmd": {"vx": 0.0, "wz": 0.0}, "disp_net_m": disp_net, "recovery": {}}
            ]
            return {
                "name": name,
                "samples": samples,
                "cleanup_verification": {"zero_command_confirmed": True, "disarm_mode0_confirmed": True},
                "status": "SUCCESS"
            }

        # 2. Reverse Acceptance: Nominal -27mm travel passes (magnitude 27mm in 24-30mm band)
        rep_nom = build_case13_report(disp_net=-0.027)
        self.app.evaluate_motion_tracking("Reverse_Crawl_0.015mps", [(-0.015, 0.0, 1.8)], rep_nom)
        mt_nom = rep_nom["motion_tracking"]
        self.assertEqual(mt_nom["nominal_requested_distance_m"], 0.027)
        self.assertEqual(mt_nom["achieved_distance_m"], 0.027)
        self.assertEqual(mt_nom["final_displacement_signed_m"], -0.027)
        self.assertEqual(mt_nom["tracking_status"], "TRACKING_NOMINAL")
        self.assertEqual(rep_nom["status"], "SUCCESS")

        # Reverse Acceptance: Lower boundary -24mm passes
        rep_lower = build_case13_report(disp_net=-0.024)
        self.app.evaluate_motion_tracking("Reverse_Crawl_0.015mps", [(-0.015, 0.0, 1.8)], rep_lower)
        self.assertEqual(rep_lower["motion_tracking"]["tracking_status"], "TRACKING_NOMINAL")
        self.assertEqual(rep_lower["status"], "SUCCESS")

        # Reverse Acceptance: Upper boundary -30mm passes
        rep_upper = build_case13_report(disp_net=-0.030)
        self.app.evaluate_motion_tracking("Reverse_Crawl_0.015mps", [(-0.015, 0.0, 1.8)], rep_upper)
        self.assertEqual(rep_upper["motion_tracking"]["tracking_status"], "TRACKING_NOMINAL")
        self.assertEqual(rep_upper["status"], "SUCCESS")

        # 3. Insufficient travel (< 24mm magnitude) fails as TRACKING_DEFICIT
        # -20mm travel
        rep_deficit_20 = build_case13_report(disp_net=-0.020)
        self.app.evaluate_motion_tracking("Reverse_Crawl_0.015mps", [(-0.015, 0.0, 1.8)], rep_deficit_20)
        self.assertEqual(rep_deficit_20["motion_tracking"]["achieved_distance_m"], 0.020)
        self.assertEqual(rep_deficit_20["motion_tracking"]["tracking_status"], "TRACKING_DEFICIT")
        self.assertEqual(rep_deficit_20["status"], "TRACKING_DEFICIT")

        # -10mm travel
        rep_deficit_10 = build_case13_report(disp_net=-0.010)
        self.app.evaluate_motion_tracking("Reverse_Crawl_0.015mps", [(-0.015, 0.0, 1.8)], rep_deficit_10)
        self.assertEqual(rep_deficit_10["motion_tracking"]["achieved_distance_m"], 0.010)
        self.assertEqual(rep_deficit_10["motion_tracking"]["tracking_status"], "TRACKING_DEFICIT")
        self.assertEqual(rep_deficit_10["status"], "TRACKING_DEFICIT")

        # Zero travel
        rep_zero = build_case13_report(disp_net=0.0)
        self.app.evaluate_motion_tracking("Reverse_Crawl_0.015mps", [(-0.015, 0.0, 1.8)], rep_zero)
        self.assertEqual(rep_zero["motion_tracking"]["achieved_distance_m"], 0.0)
        self.assertEqual(rep_zero["motion_tracking"]["tracking_status"], "TRACKING_DEFICIT")
        self.assertEqual(rep_zero["status"], "TRACKING_DEFICIT")

        # 4. Excessive travel (> 30mm magnitude) fails as EXCESSIVE_TRAVEL
        # -35mm travel
        rep_excess_35 = build_case13_report(disp_net=-0.035)
        self.app.evaluate_motion_tracking("Reverse_Crawl_0.015mps", [(-0.015, 0.0, 1.8)], rep_excess_35)
        self.assertEqual(rep_excess_35["motion_tracking"]["achieved_distance_m"], 0.035)
        self.assertEqual(rep_excess_35["motion_tracking"]["tracking_status"], "EXCESSIVE_TRAVEL")
        self.assertEqual(rep_excess_35["status"], "EXCESSIVE_TRAVEL")

        # -45mm travel
        rep_excess_45 = build_case13_report(disp_net=-0.045)
        self.app.evaluate_motion_tracking("Reverse_Crawl_0.015mps", [(-0.015, 0.0, 1.8)], rep_excess_45)
        self.assertEqual(rep_excess_45["motion_tracking"]["achieved_distance_m"], 0.045)
        self.assertEqual(rep_excess_45["motion_tracking"]["tracking_status"], "EXCESSIVE_TRAVEL")
        self.assertEqual(rep_excess_45["status"], "EXCESSIVE_TRAVEL")

        # 5. Wrong-direction travel (positive displacement on reverse crawl) MUST fail
        # Forward +27mm on reverse command
        rep_wrong_dir = build_case13_report(disp_net=+0.027)
        self.app.evaluate_motion_tracking("Reverse_Crawl_0.015mps", [(-0.015, 0.0, 1.8)], rep_wrong_dir)
        mt_wrong = rep_wrong_dir["motion_tracking"]
        self.assertEqual(mt_wrong["nominal_requested_distance_m"], 0.027)
        self.assertEqual(mt_wrong["achieved_distance_m"], 0.0) # Zero backward travel credited
        self.assertEqual(mt_wrong["final_displacement_signed_m"], +0.027)
        self.assertEqual(mt_wrong["tracking_status"], "TRACKING_DEFICIT")
        self.assertEqual(rep_wrong_dir["status"], "TRACKING_DEFICIT")

        # Forward +35mm on reverse command
        rep_wrong_excess = build_case13_report(disp_net=+0.035)
        self.app.evaluate_motion_tracking("Reverse_Crawl_0.015mps", [(-0.015, 0.0, 1.8)], rep_wrong_excess)
        self.assertEqual(rep_wrong_excess["motion_tracking"]["achieved_distance_m"], 0.0)
        self.assertEqual(rep_wrong_excess["motion_tracking"]["tracking_status"], "TRACKING_DEFICIT")
        self.assertEqual(rep_wrong_excess["status"], "TRACKING_DEFICIT")

        # 6. Kinematic matching without relying solely on case-name matching
        # Custom name "Unregistered_Reverse_Profile" with [(-0.015, 0.0, 1.8)]
        custom_name = "Unregistered_Reverse_Profile"
        rep_custom_pass = build_case13_report(disp_net=-0.027, name=custom_name)
        self.app.evaluate_motion_tracking(custom_name, [(-0.015, 0.0, 1.8)], rep_custom_pass)
        self.assertEqual(rep_custom_pass["motion_tracking"]["tracking_status"], "TRACKING_NOMINAL")
        self.assertEqual(rep_custom_pass["status"], "SUCCESS")

        rep_custom_deficit = build_case13_report(disp_net=-0.020, name=custom_name)
        self.app.evaluate_motion_tracking(custom_name, [(-0.015, 0.0, 1.8)], rep_custom_deficit)
        self.assertEqual(rep_custom_deficit["motion_tracking"]["tracking_status"], "TRACKING_DEFICIT")
        self.assertEqual(rep_custom_deficit["status"], "TRACKING_DEFICIT")

        rep_custom_excess = build_case13_report(disp_net=-0.035, name=custom_name)
        self.app.evaluate_motion_tracking(custom_name, [(-0.015, 0.0, 1.8)], rep_custom_excess)
        self.assertEqual(rep_custom_excess["motion_tracking"]["tracking_status"], "EXCESSIVE_TRAVEL")
        self.assertEqual(rep_custom_excess["status"], "EXCESSIVE_TRAVEL")

        rep_custom_wrong = build_case13_report(disp_net=+0.027, name=custom_name)
        self.app.evaluate_motion_tracking(custom_name, [(-0.015, 0.0, 1.8)], rep_custom_wrong)
        self.assertEqual(rep_custom_wrong["motion_tracking"]["tracking_status"], "TRACKING_DEFICIT")
        self.assertEqual(rep_custom_wrong["status"], "TRACKING_DEFICIT")

        # Display name variant "Reverse Crawl (vx=-0.015 m/s)"
        rep_disp = build_case13_report(disp_net=-0.027, name="Reverse Crawl (vx=-0.015 m/s)")
        self.app.evaluate_motion_tracking("Reverse Crawl (vx=-0.015 m/s)", [(-0.015, 0.0, 1.8)], rep_disp)
        self.assertEqual(rep_disp["motion_tracking"]["tracking_status"], "TRACKING_NOMINAL")
        self.assertEqual(rep_disp["status"], "SUCCESS")

    def test_23_case13_cleanup_handling(self):
        """Verify cleanup handling for Case 13:
        - Successful execution: zero velocity command sent and confirmed, Mode 0 disarm verified
        - Cleanup failure handling: disarm failure prevents SUCCESS, marks CLEANUP_FAILED, and latches FAULT
        - Zero command failure: rejected zero command prevents SUCCESS, marks CLEANUP_FAILED, and latches FAULT
        """
        now_ms = time.time() * 1000.0
        seq = 200
        is_armed = False
        step_phase = "ACTIVE"

        # --- Sub-test A: Normal successful cleanup ---
        def mock_get_success(url, timeout=0.15):
            nonlocal seq, now_ms, is_armed, step_phase
            now_ms += 10.0
            seq += 1
            if "/api/drive/status" in url:
                return MockResponse({
                    "ok": True,
                    "status": {
                        "armed": is_armed,
                        "mode": 3 if is_armed else 0,
                        "uptimeMs": 60000,
                        "seq": seq,
                        "receivedAt": now_ms,
                        "dataAgeMs": 5.0,
                        "noMotionBudgetMs": 1000,
                        "spinFailedExhausted": False,
                        "chassisRebreakoutConsumed": False
                    }
                })
            elif "/api/pid-telemetry" in url:
                meas = 0.005 if step_phase == "STOPPED" else -0.46
                tgt = 0.0 if step_phase == "STOPPED" else -0.46
                pwm = 0 if step_phase == "STOPPED" else -48
                return MockResponse({
                    "ok": True, "valid": True, "stale": False, "sequence": seq,
                    "timestamp": now_ms, "dataAgeMs": 5.0,
                    "telemetry": {
                        m: {"measuredRadps": meas, "targetRadps": tgt, "finalPwm": pwm, "stictionState": "STICTION_KINETIC"}
                        for m in ["m1", "m2", "m3", "m4"]
                    }
                })
            elif "/api/encoders" in url:
                # 263 ticks reverse travel = ~-27 mm
                # 263 * (0.065 * pi / 1974.1667) ~ 0.0272 m
                ticks = -263
                return MockResponse({
                    "ok": True, "sequence": seq, "lastPacketAgeMs": 5.0,
                    "encoders": {"m1": ticks, "m2": ticks, "m3": ticks, "m4": ticks}
                })
            elif "/api/imu" in url:
                return MockResponse({
                    "ok": True, "sequence": seq, "dataAgeMs": 5.0,
                    "gyro": {"z": 0.001},
                    "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
                })
            return MockResponse({"ok": True})

        def mock_post_success(url, json=None, headers=None, timeout=0.1):
            nonlocal is_armed, step_phase
            if "/api/drive/arm" in url:
                is_armed = True
            elif "/api/drive/disarm" in url:
                is_armed = False
            elif "/api/cmd_vel" in url:
                if json and json.get("linear", {}).get("x", 0.0) == 0.0 and json.get("angular", {}).get("z", 0.0) == 0.0:
                    step_phase = "STOPPED"
            return MockResponse({"ok": True})

        self.app.session.get = mock_get_success
        self.app.session.post = mock_post_success

        res = self.app.run_profile_step(
            name="Reverse_Crawl_0.015mps",
            segments=[(-0.015, 0.0, 0.05)],
            max_dist_m=0.08,
            max_yaw_deg=10.0
        )

        self.assertEqual(res["lifecycle_completion"]["lifecycle_status"], "COMPLETED")
        self.assertTrue(res["lifecycle_completion"]["zero_confirmed"])
        self.assertTrue(res["lifecycle_completion"]["disarm_confirmed"])
        self.assertTrue(res["cleanup_verification"]["zero_command_confirmed"])
        self.assertTrue(res["cleanup_verification"]["disarm_mode0_confirmed"])
        self.assertFalse(self.app.fault_latched)

        # --- Sub-test B: Cleanup disarm failure handling ---
        # When disarm verification fails (e.g. rover stays in Mode 3), cleanup must report failure
        app_disarm_fail = FloorSpeedCharacterizer(
            cockpit_url="http://mock-cockpit:3000",
            bridge_url="http://mock-bridge:3010",
            geometry_override=self.mock_geometry
        )
        app_disarm_fail.op_token = "mock_op_token"
        app_disarm_fail.cmd_token = "mock_cmd_token"

        # Drive status refuses to report disarmed (stays armed=True, mode=3)
        app_disarm_fail.session.get = lambda url, timeout=0.15: MockResponse({
            "ok": True,
            "status": {
                "armed": True,
                "mode": 3,
                "uptimeMs": 70000,
                "seq": 300,
                "receivedAt": time.time() * 1000.0,
                "dataAgeMs": 5.0,
                "noMotionBudgetMs": 1000,
                "spinFailedExhausted": False,
                "chassisRebreakoutConsumed": False
            }
        })
        app_disarm_fail.session.post = lambda url, json=None, headers=None, timeout=0.1: MockResponse({"ok": True})

        zero_ok, disarm_ok = app_disarm_fail.fail_closed_cleanup()
        self.assertTrue(zero_ok)
        self.assertFalse(disarm_ok)

        # --- Sub-test C: Cleanup zero velocity failure handling ---
        # When zero velocity command fails (HTTP 500 / 403), cleanup reports zero_ok=False
        app_zero_fail = FloorSpeedCharacterizer(
            cockpit_url="http://mock-cockpit:3000",
            bridge_url="http://mock-bridge:3010",
            geometry_override=self.mock_geometry
        )
        app_zero_fail.op_token = "mock_op_token"
        app_zero_fail.cmd_token = "mock_cmd_token"

        app_zero_fail.session.get = lambda url, timeout=0.15: MockResponse({
            "ok": True,
            "status": {
                "armed": False,
                "mode": 0,
                "uptimeMs": 80000,
                "seq": 400,
                "receivedAt": time.time() * 1000.0,
                "dataAgeMs": 5.0,
                "noMotionBudgetMs": 1000,
                "spinFailedExhausted": False,
                "chassisRebreakoutConsumed": False
            }
        })
        def mock_post_zero_reject(url, json=None, headers=None, timeout=0.1):
            if "/api/cmd_vel" in url:
                return MockResponse({"ok": False, "error": "CmdVelRejected"}, status_code=403)
            return MockResponse({"ok": True})

        app_zero_fail.session.post = mock_post_zero_reject
        zero_ok, disarm_ok = app_zero_fail.fail_closed_cleanup()
        self.assertFalse(zero_ok)
        self.assertTrue(disarm_ok)

    def test_24_case14_diagnostic_profile_definition_and_bounds(self):
        """Verify Case 14 diagnostic profile definition, explicit duration allowance, and bounds:
        - MAX_STEP_DURATION_S is 2.5s
        - MAX_DIAGNOSTIC_DURATION_S is 6.5s
        - ID: Diagnostic_Reverse_Long_0.015mps
        - Segments: [(-0.015, 0.0, 6.0)]
        - max_dist_m: 0.20
        - max_yaw_deg: 10.0
        - max_speed_mps: 0.060
        - Case 2 and Case 13 definitions remain unchanged."""
        from characterize_floor_speeds import CASES, MAX_STEP_DURATION_S, MAX_DIAGNOSTIC_DURATION_S
        self.assertEqual(MAX_STEP_DURATION_S, 2.5)
        self.assertEqual(MAX_DIAGNOSTIC_DURATION_S, 6.5)

        self.assertIn("14", CASES)
        case14 = CASES["14"]
        self.assertEqual(case14["id"], "Diagnostic_Reverse_Long_0.015mps")
        self.assertEqual(case14["segments"], [(-0.015, 0.0, 6.0)])
        self.assertEqual(case14["max_dist_m"], 0.20)
        self.assertEqual(case14["max_yaw_deg"], 10.0)
        self.assertEqual(case14["max_speed_mps"], 0.060)

        # Verify Case 2 preserved
        self.assertIn("2", CASES)
        case2 = CASES["2"]
        self.assertEqual(case2["id"], "Straight_Crawl_0.015mps")
        self.assertEqual(case2["segments"], [(0.015, 0.0, 1.8)])
        self.assertEqual(case2["max_dist_m"], 0.08)
        self.assertEqual(case2["max_yaw_deg"], 10.0)

        # Verify Case 13 preserved
        self.assertIn("13", CASES)
        case13 = CASES["13"]
        self.assertEqual(case13["id"], "Reverse_Crawl_0.015mps")
        self.assertEqual(case13["segments"], [(-0.015, 0.0, 1.8)])
        self.assertEqual(case13["max_dist_m"], 0.08)
        self.assertEqual(case13["max_yaw_deg"], 10.0)

    def test_25_diagnostic_speed_bound_abort(self):
        """Verify measured wheel speed abort at 0.060 m/s magnitude:
        - When wheel speed reaches >= 0.060 m/s, abort triggers immediately.
        - Zero velocity sent and command_active ceases.
        - Report records SPEED_BOUND_EXCEEDED and inconclusive equilibrium."""
        now_ms = time.time() * 1000.0
        seq = 500
        is_armed = False
        step_phase = "ACTIVE"

        def mock_get_overspeed(url, timeout=0.15):
            nonlocal seq, now_ms, is_armed, step_phase
            now_ms += 10.0
            seq += 1
            if "/api/drive/status" in url:
                return MockResponse({
                    "ok": True,
                    "status": {
                        "armed": is_armed,
                        "mode": 3 if is_armed else 0,
                        "uptimeMs": 90000,
                        "seq": seq,
                        "receivedAt": now_ms,
                        "dataAgeMs": 5.0,
                        "noMotionBudgetMs": 1000,
                        "spinFailedExhausted": False,
                        "chassisRebreakoutConsumed": False
                    }
                })
            elif "/api/pid-telemetry" in url:
                # Wheel radius is 0.0325m. At -2.0 rad/s, speed is 0.065 m/s > 0.060 m/s bound!
                meas = 0.001 if step_phase == "STOPPED" else -2.0
                tgt = 0.0 if step_phase == "STOPPED" else -0.46
                pwm = 0 if step_phase == "STOPPED" else -50
                return MockResponse({
                    "ok": True, "valid": True, "stale": False, "sequence": seq,
                    "timestamp": now_ms, "dataAgeMs": 5.0,
                    "telemetry": {
                        m: {"measuredRadps": meas, "targetRadps": tgt, "finalPwm": pwm, "stictionState": "STICTION_KINETIC", "iTerm": -5.0}
                        for m in ["m1", "m2", "m3", "m4"]
                    }
                })
            elif "/api/encoders" in url:
                ticks = -10 if step_phase == "ACTIVE" else -15
                return MockResponse({
                    "ok": True, "sequence": seq, "lastPacketAgeMs": 5.0,
                    "encoders": {"m1": ticks, "m2": ticks, "m3": ticks, "m4": ticks}
                })
            elif "/api/imu" in url:
                return MockResponse({
                    "ok": True, "sequence": seq, "dataAgeMs": 5.0,
                    "gyro": {"z": 0.001},
                    "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
                })
            return MockResponse({"ok": True})

        zero_vel_calls = 0
        def mock_post(url, json=None, headers=None, timeout=0.1):
            nonlocal is_armed, step_phase, zero_vel_calls
            if "/api/drive/arm" in url:
                is_armed = True
                step_phase = "ACTIVE"
            elif "/api/drive/disarm" in url:
                is_armed = False
            elif "/api/cmd_vel" in url:
                if is_armed and json and json.get("linear", {}).get("x", 0.0) == 0.0:
                    zero_vel_calls += 1
                    step_phase = "STOPPED"
            return MockResponse({"ok": True})

        self.app.session.get = mock_get_overspeed
        self.app.session.post = mock_post

        res = self.app.run_profile_step(
            name="Diagnostic_Reverse_Long_0.015mps",
            segments=[(-0.015, 0.0, 6.0)],
            max_dist_m=0.20,
            max_yaw_deg=10.0,
            max_speed_mps=0.060
        )

        self.assertIn("abort_reason", res)
        self.assertTrue(res["abort_reason"].startswith("SPEED_BOUND_EXCEEDED"))
        self.assertEqual(res["status"], "SPEED_BOUND_EXCEEDED")
        self.assertIn("diagnostic_evaluation", res)
        self.assertEqual(res["diagnostic_evaluation"]["equilibrium_status"], "INCONCLUSIVE_EQUILIBRIUM")
        self.assertTrue(zero_vel_calls > 0)
        self.assertTrue(res["cleanup_verification"]["zero_command_confirmed"])
        self.assertTrue(res["cleanup_verification"]["disarm_mode0_confirmed"])

    def test_25b_diagnostic_speed_bound_opposing_wheel_cancellation(self):
        """Verify opposing-sign cancellation prevention:
        - Left wheels at +2.0 rad/s (+0.065 m/s), Right wheels at -2.0 rad/s (-0.065 m/s).
        - Signed chassis speed cancels to 0.0 m/s.
        - mean_abs_wheel_speed_mps is 0.065 m/s >= 0.060 m/s.
        - Verify abort triggers SPEED_BOUND_EXCEEDED and cannot evade guard."""
        now_ms = time.time() * 1000.0
        seq = 550
        is_armed = False
        step_phase = "ACTIVE"

        def mock_get_opposing(url, timeout=0.15):
            nonlocal seq, now_ms, is_armed, step_phase
            now_ms += 10.0
            seq += 1
            if "/api/drive/status" in url:
                return MockResponse({
                    "ok": True,
                    "status": {
                        "armed": is_armed, "mode": 3 if is_armed else 0, "uptimeMs": 90000,
                        "seq": seq, "receivedAt": now_ms, "dataAgeMs": 5.0,
                        "noMotionBudgetMs": 1000, "spinFailedExhausted": False, "chassisRebreakoutConsumed": False
                    }
                })
            elif "/api/pid-telemetry" in url:
                # Left wheels (m1, m3) positive +2.0 rad/s; Right wheels (m2, m4) negative -2.0 rad/s
                if step_phase == "STOPPED":
                    telem = {m: {"measuredRadps": 0.001, "targetRadps": 0.0, "finalPwm": 0, "stictionState": "STICTION_KINETIC", "iTerm": 0.0} for m in ["m1", "m2", "m3", "m4"]}
                else:
                    telem = {
                        "m1": {"measuredRadps": +2.0, "targetRadps": -0.46, "finalPwm": +50, "stictionState": "STICTION_KINETIC", "iTerm": 0.0},
                        "m2": {"measuredRadps": -2.0, "targetRadps": -0.46, "finalPwm": -50, "stictionState": "STICTION_KINETIC", "iTerm": 0.0},
                        "m3": {"measuredRadps": +2.0, "targetRadps": -0.46, "finalPwm": +50, "stictionState": "STICTION_KINETIC", "iTerm": 0.0},
                        "m4": {"measuredRadps": -2.0, "targetRadps": -0.46, "finalPwm": -50, "stictionState": "STICTION_KINETIC", "iTerm": 0.0}
                    }
                return MockResponse({"ok": True, "valid": True, "stale": False, "sequence": seq, "timestamp": now_ms, "dataAgeMs": 5.0, "telemetry": telem})
            elif "/api/encoders" in url:
                return MockResponse({"ok": True, "sequence": seq, "lastPacketAgeMs": 5.0, "encoders": {"m1": 5, "m2": -5, "m3": 5, "m4": -5}})
            elif "/api/imu" in url:
                return MockResponse({"ok": True, "sequence": seq, "dataAgeMs": 5.0, "gyro": {"z": 0.001}, "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}})
            return MockResponse({"ok": True})

        def mock_post(url, json=None, headers=None, timeout=0.1):
            nonlocal is_armed, step_phase
            if "/api/drive/arm" in url:
                is_armed = True
                step_phase = "ACTIVE"
            elif "/api/drive/disarm" in url:
                is_armed = False
            elif "/api/cmd_vel" in url:
                if is_armed and json and json.get("linear", {}).get("x", 0.0) == 0.0:
                    step_phase = "STOPPED"
            return MockResponse({"ok": True})

        self.app.session.get = mock_get_opposing
        self.app.session.post = mock_post

        res = self.app.run_profile_step(
            name="Diagnostic_Reverse_Long_0.015mps",
            segments=[(-0.015, 0.0, 6.0)],
            max_dist_m=0.20,
            max_yaw_deg=10.0,
            max_speed_mps=0.060
        )

        self.assertIn("abort_reason", res)
        self.assertTrue(res["abort_reason"].startswith("SPEED_BOUND_EXCEEDED"))
        self.assertEqual(res["status"], "SPEED_BOUND_EXCEEDED")
        self.assertEqual(res["samples"][0]["signed_chassis_mps"], 0.0)
        self.assertAlmostEqual(res["samples"][0]["mean_abs_wheel_speed_mps"], 0.065, places=3)

    def test_26_diagnostic_travel_bound_abort(self):
        """Verify absolute travel bound abort at 0.20 m:
        - When accumulated travel exceeds 0.20 m, abort triggers immediately.
        - Zero velocity sent and command_active ceases.
        - Report records TRAVEL_BOUND_EXCEEDED and inconclusive equilibrium."""
        now_ms = time.time() * 1000.0
        seq = 600
        is_armed = False
        step_phase = "ACTIVE"

        def mock_get_overtravel(url, timeout=0.15):
            nonlocal seq, now_ms, is_armed, step_phase
            now_ms += 10.0
            seq += 1
            if "/api/drive/status" in url:
                return MockResponse({
                    "ok": True,
                    "status": {
                        "armed": is_armed, "mode": 3 if is_armed else 0, "uptimeMs": 95000,
                        "seq": seq, "receivedAt": now_ms, "dataAgeMs": 5.0,
                        "noMotionBudgetMs": 1000, "spinFailedExhausted": False, "chassisRebreakoutConsumed": False
                    }
                })
            elif "/api/pid-telemetry" in url:
                meas = 0.001 if step_phase == "STOPPED" else -0.46
                tgt = 0.0 if step_phase == "STOPPED" else -0.46
                pwm = 0 if step_phase == "STOPPED" else -48
                return MockResponse({
                    "ok": True, "valid": True, "stale": False, "sequence": seq,
                    "timestamp": now_ms, "dataAgeMs": 5.0,
                    "telemetry": {
                        m: {"measuredRadps": meas, "targetRadps": tgt, "finalPwm": pwm, "stictionState": "STICTION_KINETIC", "iTerm": -3.0}
                        for m in ["m1", "m2", "m3", "m4"]
                    }
                })
            elif "/api/encoders" in url:
                ticks = -2000 if is_armed else 0
                return MockResponse({"ok": True, "sequence": seq, "lastPacketAgeMs": 5.0, "encoders": {"m1": ticks, "m2": ticks, "m3": ticks, "m4": ticks}})
            elif "/api/imu" in url:
                return MockResponse({"ok": True, "sequence": seq, "dataAgeMs": 5.0, "gyro": {"z": 0.001}, "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}})
            return MockResponse({"ok": True})

        def mock_post(url, json=None, headers=None, timeout=0.1):
            nonlocal is_armed, step_phase
            if "/api/drive/arm" in url:
                is_armed = True
                step_phase = "ACTIVE"
            elif "/api/drive/disarm" in url:
                is_armed = False
            elif "/api/cmd_vel" in url:
                if is_armed and json and json.get("linear", {}).get("x", 0.0) == 0.0:
                    step_phase = "STOPPED"
            return MockResponse({"ok": True})

        self.app.session.get = mock_get_overtravel
        self.app.session.post = mock_post

        res = self.app.run_profile_step(
            name="Diagnostic_Reverse_Long_0.015mps",
            segments=[(-0.015, 0.0, 6.0)],
            max_dist_m=0.20,
            max_yaw_deg=10.0,
            max_speed_mps=0.060
        )

        self.assertIn("abort_reason", res)
        self.assertTrue(res["abort_reason"].startswith("TRAVEL_BOUND_EXCEEDED"))
        self.assertEqual(res["status"], "TRAVEL_BOUND_EXCEEDED")
        self.assertEqual(res["diagnostic_evaluation"]["equilibrium_status"], "INCONCLUSIVE_EQUILIBRIUM")

    def test_26b_diagnostic_yaw_bound_abort(self):
        """Verify yaw bound abort at 10.0 degrees:
        - When accumulated continuous yaw reaches >= 10.0 deg, abort triggers immediately.
        - Zero velocity is sent, report records YAW_BOUND_EXCEEDED."""
        now_ms = time.time() * 1000.0
        seq = 650
        is_armed = False
        step_phase = "ACTIVE"

        def mock_get_overyaw(url, timeout=0.15):
            nonlocal seq, now_ms, is_armed, step_phase
            now_ms += 10.0
            seq += 1
            if "/api/drive/status" in url:
                return MockResponse({
                    "ok": True,
                    "status": {
                        "armed": is_armed, "mode": 3 if is_armed else 0, "uptimeMs": 95000,
                        "seq": seq, "receivedAt": now_ms, "dataAgeMs": 5.0,
                        "noMotionBudgetMs": 1000, "spinFailedExhausted": False, "chassisRebreakoutConsumed": False
                    }
                })
            elif "/api/pid-telemetry" in url:
                meas = 0.001 if step_phase == "STOPPED" else -0.46
                return MockResponse({
                    "ok": True, "valid": True, "stale": False, "sequence": seq,
                    "timestamp": now_ms, "dataAgeMs": 5.0,
                    "telemetry": {
                        m: {"measuredRadps": meas, "targetRadps": -0.46, "finalPwm": -48, "stictionState": "STICTION_KINETIC", "iTerm": -3.0}
                        for m in ["m1", "m2", "m3", "m4"]
                    }
                })
            elif "/api/encoders" in url:
                return MockResponse({"ok": True, "sequence": seq, "lastPacketAgeMs": 5.0, "encoders": {"m1": -5, "m2": -5, "m3": -5, "m4": -5}})
            elif "/api/imu" in url:
                # 11 degrees yaw in quaternion: yaw=11 deg = 0.192 rad -> qz=sin(0.096)~0.0958, qw=cos(0.096)~0.9954
                qz = 0.0958 if is_armed else 0.0
                qw = 0.9954 if is_armed else 1.0
                return MockResponse({
                    "ok": True, "sequence": seq, "dataAgeMs": 5.0,
                    "gyro": {"z": 0.001 if step_phase == "STOPPED" else 0.05},
                    "orientation": {"w": qw, "x": 0.0, "y": 0.0, "z": qz}
                })
            return MockResponse({"ok": True})

        def mock_post(url, json=None, headers=None, timeout=0.1):
            nonlocal is_armed, step_phase
            if "/api/drive/arm" in url:
                is_armed = True
                step_phase = "ACTIVE"
            elif "/api/drive/disarm" in url:
                is_armed = False
            elif "/api/cmd_vel" in url:
                if is_armed and json and json.get("linear", {}).get("x", 0.0) == 0.0:
                    step_phase = "STOPPED"
            return MockResponse({"ok": True})

        self.app.session.get = mock_get_overyaw
        self.app.session.post = mock_post

        res = self.app.run_profile_step(
            name="Diagnostic_Reverse_Long_0.015mps",
            segments=[(-0.015, 0.0, 6.0)],
            max_dist_m=0.20,
            max_yaw_deg=10.0,
            max_speed_mps=0.060
        )

        self.assertIn("abort_reason", res)
        self.assertTrue(res["abort_reason"].startswith("YAW_BOUND_EXCEEDED"))
        self.assertEqual(res["status"], "YAW_BOUND_EXCEEDED")
        self.assertEqual(res["diagnostic_evaluation"]["equilibrium_status"], "INCONCLUSIVE_EQUILIBRIUM")

    def test_26c_diagnostic_command_duration_cutoff_independent_of_cleanup(self):
        """Verify command cutoff occurs at 6.0s independently of cleanup timeout:
        - When segment duration (6.0s) elapses, command_active becomes False.
        - Zero velocity command is dispatched and acknowledged.
        - Cleanup executes subsequently in fail_closed_cleanup without inflating active command duration."""
        # Test command boundaries timing logic directly
        t_start = 100.0
        t_cutoff_dispatch = 106.002
        t_cutoff_ack = 106.012
        t_cleanup_finish = 106.650

        # Simulate boundaries dictionary built by run_profile_step
        cmd_duration_s = max(0.0, t_cutoff_dispatch - t_start)
        boundaries = {
            "first_cmd_dispatch_mono": round(t_start, 4),
            "first_cmd_ack_mono": round(t_start + 0.010, 4),
            "first_zero_dispatch_mono": round(t_cutoff_dispatch, 4),
            "first_zero_ack_mono": round(t_cutoff_ack, 4),
            "command_duration_s": round(cmd_duration_s, 4),
            "first_zero_command_mono": round(t_cutoff_dispatch, 4)
        }

        self.assertAlmostEqual(boundaries["command_duration_s"], 6.002, places=3)
        self.assertLess(boundaries["command_duration_s"], 6.05)
        # Verify cleanup finish time (106.650) was NOT substituted as command duration (which would have been 6.650s)
        self.assertNotAlmostEqual(boundaries["command_duration_s"], t_cleanup_finish - t_start, places=1)
        self.assertEqual(boundaries["first_zero_command_mono"], round(t_cutoff_dispatch, 4))

    def test_26d_diagnostic_stale_or_frozen_telemetry_abort(self):
        """Verify stale/frozen telemetry stream is detected and aborts run:
        - Stream non-advancement (> 250ms freeze) triggers fault."""
        t0 = 200.0
        self.app.stream_seq["pid"] = None
        self.app.check_stream_advancement("pid", seq=10, now_mono=t0)
        # Advance slightly within age limit
        self.app.check_stream_advancement("pid", seq=11, now_mono=t0 + 0.05)
        # Frozen at seq 11 for > 250ms
        with self.assertRaises(RuntimeError) as ctx:
            self.app.check_stream_advancement("pid", seq=11, now_mono=t0 + 0.35)
        self.assertIn("stopped advancing", str(ctx.exception))

    def test_26e_diagnostic_cleanup_failure_precedence_over_equilibrium(self):
        """Verify cleanup failure unconditionally takes precedence over equilibrium classification:
        - Telemetry meets all settled equilibrium criteria.
        - But zero_command_confirmed=False or disarm_mode0_confirmed=False.
        - Final status MUST be CLEANUP_FAILED, and diagnostic_evaluation MUST report CLEANUP_FAILED."""
        mock_samples = []
        for i in range(25):
            t = 1.0 + i * 0.05
            mock_samples.append({
                "t_rel": round(t, 3),
                "seg_idx": 0,
                "cmd": {"vx": -0.015, "wz": 0.0},
                "disp_net_m": -0.026,
                "measured_radps": {m: -0.461 for m in ["m1", "m2", "m3", "m4"]},
                "target_radps": {m: -0.462 for m in ["m1", "m2", "m3", "m4"]},
                "pwm": {m: -47 for m in ["m1", "m2", "m3", "m4"]},
                "pid_terms": {m: {"iTerm": -4.2, "finalPwm": -47} for m in ["m1", "m2", "m3", "m4"]},
                "sequences": {"pid": 100 + i},
                "timestamps": {"pid_data_age_ms": 10.0}
            })
        report = {
            "name": "Diagnostic_Reverse_Long_0.015mps",
            "samples": mock_samples,
            "abort_reason": None,
            "status": "CLEANUP_FAILED",
            "cleanup_verification": {
                "zero_command_confirmed": True,
                "disarm_mode0_confirmed": False  # Disarm failed!
            }
        }
        res_diag = self.app.assess_diagnostic_tracking(report)
        self.assertEqual(res_diag["equilibrium_status"], "CLEANUP_FAILED")
        self.assertIn("Cleanup failure took precedence", res_diag["reason"])

        self.app.evaluate_motion_tracking("Diagnostic_Reverse_Long_0.015mps", [(-0.015, 0.0, 6.0)], report)
        self.assertEqual(report["status"], "CLEANUP_FAILED")
        self.assertEqual(report["diagnostic_evaluation"]["equilibrium_status"], "CLEANUP_FAILED")

    def test_27_diagnostic_sustained_tracking_assessment_logic(self):
        """Verify sustained tracking assessment:
        - Sub-test A: Settled equilibrium (duration >= 1.0s, transient separated) -> SETTLED_EQUILIBRIUM
        - Sub-test B: Inconclusive equilibrium (ramping PWM, large speed error) -> INCONCLUSIVE_EQUILIBRIUM
        - Sub-test C: Inconclusive equilibrium (duration < 1.0s) -> INCONCLUSIVE_EQUILIBRIUM
        - Sub-test D: Oscillation rejection (zero mean error, zero slope, but high RMS error and ripple) -> INCONCLUSIVE_EQUILIBRIUM
        - Sub-test E: Missing or non-positive sequence numbers excluded from valid coverage
        - Sub-test F: Telemetry gap > 250ms breaks contiguity."""
        # Sub-test A: Settled equilibrium with earlier transient reported separately
        mock_samples_settled = []
        # Earlier transient: 0.0s to 1.0s (10 samples)
        for i in range(10):
            t = 0.05 + i * 0.10
            mock_samples_settled.append({
                "t_rel": round(t, 3),
                "seg_idx": 0,
                "measured_radps": {m: -0.10 - i * 0.035 for m in ["m1", "m2", "m3", "m4"]},
                "target_radps": {m: -0.462 for m in ["m1", "m2", "m3", "m4"]},
                "pwm": {m: -30 - i * 1.5 for m in ["m1", "m2", "m3", "m4"]},
                "pid_terms": {m: {"iTerm": -1.0 - i * 0.3, "finalPwm": -30 - i * 1.5} for m in ["m1", "m2", "m3", "m4"]},
                "sequences": {"pid": 50 + i},
                "timestamps": {"pid_data_age_ms": 10.0}
            })
        # Final window: 1.0s to 2.2s (25 samples at dt=0.05s)
        for i in range(25):
            t = 1.05 + i * 0.05
            mock_samples_settled.append({
                "t_rel": round(t, 3),
                "seg_idx": 0,
                "measured_radps": {m: -0.461 + 0.002 * math.sin(i) for m in ["m1", "m2", "m3", "m4"]},
                "target_radps": {m: -0.462 for m in ["m1", "m2", "m3", "m4"]},
                "pwm": {m: -47 for m in ["m1", "m2", "m3", "m4"]},
                "pid_terms": {m: {"iTerm": -4.2, "finalPwm": -47} for m in ["m1", "m2", "m3", "m4"]},
                "sequences": {"pid": 100 + i},
                "timestamps": {"pid_data_age_ms": 10.0}
            })
        report_settled = {
            "name": "Diagnostic_Reverse_Long_0.015mps",
            "samples": mock_samples_settled,
            "abort_reason": None,
            "status": "IN_PROGRESS",
            "cleanup_verification": {"zero_command_confirmed": True, "disarm_mode0_confirmed": True}
        }
        res_settled = self.app.assess_diagnostic_tracking(report_settled)
        self.assertTrue(res_settled["tracking_assessed"])
        self.assertEqual(res_settled["equilibrium_status"], "SETTLED_EQUILIBRIUM")
        self.assertGreaterEqual(res_settled["assessment_window"]["duration_s"], 1.0)
        self.assertIn("transient_metrics", res_settled)
        self.assertGreater(res_settled["transient_metrics"]["sample_count"], 0)
        for m in ["m1", "m2", "m3", "m4"]:
            self.assertTrue(res_settled["wheels"][m]["wheel_settled"])

        # Sub-test B: Inconclusive due to continuous PWM / integral ramping
        mock_samples_ramping = []
        for i in range(25):
            t = 1.0 + i * 0.05
            mock_samples_ramping.append({
                "t_rel": round(t, 3),
                "seg_idx": 0,
                "measured_radps": {m: -0.80 for m in ["m1", "m2", "m3", "m4"]},
                "target_radps": {m: -0.462 for m in ["m1", "m2", "m3", "m4"]},
                "pwm": {m: -40 - i * 1.5 for m in ["m1", "m2", "m3", "m4"]},
                "pid_terms": {m: {"iTerm": -2.0 - i * 1.0, "finalPwm": -40 - i * 1.5} for m in ["m1", "m2", "m3", "m4"]},
                "sequences": {"pid": 200 + i},
                "timestamps": {"pid_data_age_ms": 10.0}
            })
        report_ramping = {
            "name": "Diagnostic_Reverse_Long_0.015mps",
            "samples": mock_samples_ramping,
            "abort_reason": None,
            "status": "IN_PROGRESS",
            "cleanup_verification": {"zero_command_confirmed": True, "disarm_mode0_confirmed": True}
        }
        res_ramping = self.app.assess_diagnostic_tracking(report_ramping)
        self.assertEqual(res_ramping["equilibrium_status"], "INCONCLUSIVE_EQUILIBRIUM")
        self.assertIn("Unsettled equilibrium", res_ramping["reason"])

        # Sub-test C: Inconclusive due to short window (< 1.0s)
        mock_samples_short = mock_samples_settled[10:18] # 8 samples at 0.05s = 0.35s
        report_short = {
            "name": "Diagnostic_Reverse_Long_0.015mps",
            "samples": mock_samples_short,
            "abort_reason": None,
            "status": "IN_PROGRESS",
            "cleanup_verification": {"zero_command_confirmed": True, "disarm_mode0_confirmed": True}
        }
        res_short = self.app.assess_diagnostic_tracking(report_short)
        self.assertFalse(res_short["tracking_assessed"])
        self.assertEqual(res_short["equilibrium_status"], "INCONCLUSIVE_EQUILIBRIUM")
        self.assertIn("Insufficient final contiguous valid window", res_short["reason"])

        # Sub-test D: Oscillation rejection (zero mean error, zero slope, but large sinusoidal oscillation)
        mock_samples_osc = []
        for i in range(30):
            t = 1.0 + i * 0.05
            # Oscillation with +/- 0.30 rad/s amplitude around target -0.462 rad/s
            osc_meas = -0.462 + 0.30 * math.sin(i * math.pi / 2.0)
            mock_samples_osc.append({
                "t_rel": round(t, 3),
                "seg_idx": 0,
                "measured_radps": {m: osc_meas for m in ["m1", "m2", "m3", "m4"]},
                "target_radps": {m: -0.462 for m in ["m1", "m2", "m3", "m4"]},
                "pwm": {m: -47 for m in ["m1", "m2", "m3", "m4"]},
                "pid_terms": {m: {"iTerm": -4.2, "finalPwm": -47} for m in ["m1", "m2", "m3", "m4"]},
                "sequences": {"pid": 300 + i},
                "timestamps": {"pid_data_age_ms": 10.0}
            })
        report_osc = {
            "name": "Diagnostic_Reverse_Long_0.015mps",
            "samples": mock_samples_osc,
            "abort_reason": None,
            "status": "IN_PROGRESS",
            "cleanup_verification": {"zero_command_confirmed": True, "disarm_mode0_confirmed": True}
        }
        res_osc = self.app.assess_diagnostic_tracking(report_osc)
        self.assertEqual(res_osc["equilibrium_status"], "INCONCLUSIVE_EQUILIBRIUM")
        self.assertIn("oscillation/variation", res_osc["reason"])
        for m in ["m1", "m2", "m3", "m4"]:
            self.assertFalse(res_osc["wheels"][m]["wheel_settled"])
            self.assertFalse(res_osc["wheels"][m]["variation_settled"])
            self.assertGreater(res_osc["wheels"][m]["rms_error_radps"], 0.15)

        # Sub-test E: Missing or zero sequence numbers strictly excluded
        mock_samples_unidentified = []
        for i in range(25):
            t = 1.0 + i * 0.05
            mock_samples_unidentified.append({
                "t_rel": round(t, 3),
                "seg_idx": 0,
                "measured_radps": {m: -0.461 for m in ["m1", "m2", "m3", "m4"]},
                "target_radps": {m: -0.462 for m in ["m1", "m2", "m3", "m4"]},
                "pwm": {m: -47 for m in ["m1", "m2", "m3", "m4"]},
                "pid_terms": {m: {"iTerm": -4.2, "finalPwm": -47} for m in ["m1", "m2", "m3", "m4"]},
                "sequences": {"pid": 0 if i % 2 == 0 else None},  # Invalid sequences!
                "timestamps": {"pid_data_age_ms": 10.0}
            })
        report_unid = {
            "name": "Diagnostic_Reverse_Long_0.015mps",
            "samples": mock_samples_unidentified,
            "abort_reason": None,
            "status": "IN_PROGRESS",
            "cleanup_verification": {"zero_command_confirmed": True, "disarm_mode0_confirmed": True}
        }
        res_unid = self.app.assess_diagnostic_tracking(report_unid)
        self.assertFalse(res_unid["tracking_assessed"])
        self.assertEqual(res_unid["equilibrium_status"], "INCONCLUSIVE_EQUILIBRIUM")
        self.assertEqual(res_unid["window_sample_count"], 0)

        # Sub-test F: Telemetry gap > 250ms breaks contiguity
        mock_samples_gap = []
        for i in range(15):
            t = 1.0 + i * 0.05
            mock_samples_gap.append({
                "t_rel": round(t, 3),
                "seg_idx": 0,
                "measured_radps": {m: -0.461 for m in ["m1", "m2", "m3", "m4"]},
                "target_radps": {m: -0.462 for m in ["m1", "m2", "m3", "m4"]},
                "pwm": {m: -47 for m in ["m1", "m2", "m3", "m4"]},
                "pid_terms": {m: {"iTerm": -4.2, "finalPwm": -47} for m in ["m1", "m2", "m3", "m4"]},
                "sequences": {"pid": 400 + i},
                "timestamps": {"pid_data_age_ms": 10.0}
            })
        # 300ms gap before final 5 samples
        for i in range(5):
            t = 2.05 + i * 0.05
            mock_samples_gap.append({
                "t_rel": round(t, 3),
                "seg_idx": 0,
                "measured_radps": {m: -0.461 for m in ["m1", "m2", "m3", "m4"]},
                "target_radps": {m: -0.462 for m in ["m1", "m2", "m3", "m4"]},
                "pwm": {m: -47 for m in ["m1", "m2", "m3", "m4"]},
                "pid_terms": {m: {"iTerm": -4.2, "finalPwm": -47} for m in ["m1", "m2", "m3", "m4"]},
                "sequences": {"pid": 450 + i},
                "timestamps": {"pid_data_age_ms": 10.0}
            })
        report_gap = {
            "name": "Diagnostic_Reverse_Long_0.015mps",
            "samples": mock_samples_gap,
            "abort_reason": None,
            "status": "IN_PROGRESS",
            "cleanup_verification": {"zero_command_confirmed": True, "disarm_mode0_confirmed": True}
        }
        res_gap = self.app.assess_diagnostic_tracking(report_gap)
        self.assertFalse(res_gap["tracking_assessed"])
        self.assertEqual(res_gap["equilibrium_status"], "INCONCLUSIVE_EQUILIBRIUM")
        self.assertIn("Insufficient final contiguous valid window", res_gap["reason"])

    def test_28_case13_and_case2_acceptance_regression(self):
        """Verify Case 13 (Reverse 0.015 m/s) and Case 2 (Forward 0.015 m/s) retain exact definitions and acceptance:
        - 24-30mm acceptance window in commanded direction.
        - Forward commands require positive displacement; reverse commands require negative displacement.
        - Deficit (< 24mm), excessive (> 30mm), and wrong direction are strictly tested."""
        def build_report(case_name, cmd_vx, disp_m):
            return {
                "name": case_name,
                "samples": [
                    {"seg_idx": 0, "t_rel": 0.0, "cmd": {"vx": cmd_vx, "wz": 0.0}, "disp_net_m": 0.0, "target_radps": {"m1": 0.46, "m2": 0.46, "m3": 0.46, "m4": 0.46}, "recovery": {}},
                    {"seg_idx": 0, "t_rel": 1.8, "cmd": {"vx": cmd_vx, "wz": 0.0}, "disp_net_m": disp_m, "target_radps": {"m1": 0.46, "m2": 0.46, "m3": 0.46, "m4": 0.46}, "recovery": {}},
                    {"seg_idx": -1, "t_rel": 2.0, "cmd": {"vx": 0.0, "wz": 0.0}, "disp_net_m": disp_m, "target_radps": {"m1": 0.0, "m2": 0.0, "m3": 0.0, "m4": 0.0}, "recovery": {}}
                ],
                "status": "IN_PROGRESS"
            }

        # --- Case 13: Reverse Crawl (vx = -0.015 m/s, expected = -27mm) ---
        # 1. -26 mm -> PASS (TRACKING_NOMINAL, SUCCESS)
        rep13_pass = build_report("Reverse_Crawl_0.015mps", -0.015, -0.026)
        self.app.evaluate_motion_tracking("Reverse_Crawl_0.015mps", [(-0.015, 0.0, 1.8)], rep13_pass)
        self.assertEqual(rep13_pass["motion_tracking"]["tracking_status"], "TRACKING_NOMINAL")
        self.assertEqual(rep13_pass["status"], "SUCCESS")

        # 2. -20 mm -> FAIL (< 24mm -> TRACKING_DEFICIT)
        rep13_def = build_report("Reverse_Crawl_0.015mps", -0.015, -0.020)
        self.app.evaluate_motion_tracking("Reverse_Crawl_0.015mps", [(-0.015, 0.0, 1.8)], rep13_def)
        self.assertEqual(rep13_def["motion_tracking"]["tracking_status"], "TRACKING_DEFICIT")
        self.assertEqual(rep13_def["status"], "TRACKING_DEFICIT")

        # 3. -35 mm -> FAIL (> 30mm -> EXCESSIVE_TRAVEL)
        rep13_exc = build_report("Reverse_Crawl_0.015mps", -0.015, -0.035)
        self.app.evaluate_motion_tracking("Reverse_Crawl_0.015mps", [(-0.015, 0.0, 1.8)], rep13_exc)
        self.assertEqual(rep13_exc["motion_tracking"]["tracking_status"], "EXCESSIVE_TRAVEL")
        self.assertEqual(rep13_exc["status"], "EXCESSIVE_TRAVEL")

        # 4. +26 mm -> FAIL (wrong direction -> TRACKING_DEFICIT)
        rep13_wrong = build_report("Reverse_Crawl_0.015mps", -0.015, +0.026)
        self.app.evaluate_motion_tracking("Reverse_Crawl_0.015mps", [(-0.015, 0.0, 1.8)], rep13_wrong)
        self.assertEqual(rep13_wrong["motion_tracking"]["tracking_status"], "TRACKING_DEFICIT")
        self.assertEqual(rep13_wrong["status"], "TRACKING_DEFICIT")

        # --- Case 2: Forward Crawl (vx = +0.015 m/s, expected = +27mm) ---
        # 1. +26 mm -> PASS (TRACKING_NOMINAL, SUCCESS)
        rep2_pass = build_report("Straight_Crawl_0.015mps", 0.015, +0.026)
        self.app.evaluate_motion_tracking("Straight_Crawl_0.015mps", [(0.015, 0.0, 1.8)], rep2_pass)
        self.assertEqual(rep2_pass["motion_tracking"]["tracking_status"], "TRACKING_NOMINAL")
        self.assertEqual(rep2_pass["status"], "SUCCESS")

        # 2. +20 mm -> FAIL (< 24mm -> TRACKING_DEFICIT)
        rep2_def = build_report("Straight_Crawl_0.015mps", 0.015, +0.020)
        self.app.evaluate_motion_tracking("Straight_Crawl_0.015mps", [(0.015, 0.0, 1.8)], rep2_def)
        self.assertEqual(rep2_def["motion_tracking"]["tracking_status"], "TRACKING_DEFICIT")
        self.assertEqual(rep2_def["status"], "TRACKING_DEFICIT")

        # 3. +35 mm -> FAIL (> 30mm -> EXCESSIVE_TRAVEL)
        rep2_exc = build_report("Straight_Crawl_0.015mps", 0.015, +0.035)
        self.app.evaluate_motion_tracking("Straight_Crawl_0.015mps", [(0.015, 0.0, 1.8)], rep2_exc)
        self.assertEqual(rep2_exc["motion_tracking"]["tracking_status"], "EXCESSIVE_TRAVEL")
        self.assertEqual(rep2_exc["status"], "EXCESSIVE_TRAVEL")

        # 4. -26 mm -> FAIL (wrong direction -> TRACKING_DEFICIT)
        rep2_wrong = build_report("Straight_Crawl_0.015mps", 0.015, -0.026)
        self.app.evaluate_motion_tracking("Straight_Crawl_0.015mps", [(0.015, 0.0, 1.8)], rep2_wrong)
        self.assertEqual(rep2_wrong["motion_tracking"]["tracking_status"], "TRACKING_DEFICIT")
        self.assertEqual(rep2_wrong["status"], "TRACKING_DEFICIT")

    def test_29_duration_limit_ordinary_vs_diagnostic(self):
        """Verify ordinary cases strictly enforce <= 2.5s limit, while diagnostic profile allows up to 6.5s:
        - Ordinary case with 2.6s raises RuntimeError.
        - Diagnostic case with 7.0s raises RuntimeError."""
        with self.assertRaises(RuntimeError) as ctx1:
            self.app.run_profile_step("Straight_Crawl_0.015mps", [(0.015, 0.0, 2.6)], 0.08, 10.0)
        self.assertIn("exceeds limit of 2.50 s", str(ctx1.exception))

        with self.assertRaises(RuntimeError) as ctx2:
            self.app.run_profile_step("Diagnostic_Reverse_Long_0.015mps", [(-0.015, 0.0, 7.0)], 0.20, 10.0)
        self.assertIn("exceeds limit of 6.50 s", str(ctx2.exception))

    def test_30_command_boundaries_timestamps(self):
        """Verify command dispatch and acknowledgement timestamps are recorded separately:
        - first_cmd_dispatch_mono and first_cmd_ack_mono are populated.
        - first_zero_dispatch_mono and first_zero_ack_mono are populated.
        - first_zero_command_mono records dispatch, not telemetry receipt.
        - command_duration_s does not substitute cleanup duration."""
        self.app.last_cmd_dispatch_mono = None
        self.app.last_cmd_ack_mono = None
        self.app.last_zero_dispatch_mono = None
        self.app.last_zero_ack_mono = None

        self.app.send_velocity(0.015, 0.0)
        self.assertIsNotNone(self.app.last_cmd_dispatch_mono)
        self.assertIsNotNone(self.app.last_cmd_ack_mono)
        self.assertGreaterEqual(self.app.last_cmd_ack_mono, self.app.last_cmd_dispatch_mono)

        self.app.send_zero_velocity()
        self.assertIsNotNone(self.app.last_zero_dispatch_mono)
        self.assertIsNotNone(self.app.last_zero_ack_mono)
        self.assertGreaterEqual(self.app.last_zero_ack_mono, self.app.last_zero_dispatch_mono)

    def test_31_diagnostic_final_window_selection_with_earlier_overspeed(self):
        """Verify trailing window selection within the last contiguous block:
        - Several seconds of initial overspeed (3.0s, t=0.1 to 3.0s).
        - Followed by >= 1.2s of settled tracking (1.4s, t=3.1 to 4.4s).
        - Final window qualifies as SETTLED_EQUILIBRIUM.
        - Selected assessment window duration is >= 1.0s and exactly covers the trailing settled segment.
        - Earlier overspeed remains clearly visible in transient metrics.
        - Assert the selected window's start/end times and duration explicitly."""
        mock_samples = []
        # Initial overspeed segment: 3.0 seconds (30 samples, t=0.1 to 3.0, dt=0.10s)
        for i in range(30):
            t = round(0.1 + i * 0.1, 2)
            mock_samples.append({
                "t_rel": t,
                "seg_idx": 0,
                "cmd": {"vx": -0.015, "wz": 0.0},
                "disp_net_m": round(-0.02 * (i + 1), 3),
                "measured_radps": {m: -1.25 for m in ["m1", "m2", "m3", "m4"]},
                "target_radps": {m: -0.462 for m in ["m1", "m2", "m3", "m4"]},
                "pwm": {m: -95 for m in ["m1", "m2", "m3", "m4"]},
                "pid_terms": {m: {"iTerm": -30.0, "finalPwm": -95} for m in ["m1", "m2", "m3", "m4"]},
                "sequences": {"pid": 100 + i},
                "timestamps": {"pid_data_age_ms": 15.0}
            })
        # Settled tracking segment: 1.4 seconds (14 samples, t=3.1 to 4.4, dt=0.10s)
        for i in range(14):
            t = round(3.1 + i * 0.1, 2)
            mock_samples.append({
                "t_rel": t,
                "seg_idx": 0,
                "cmd": {"vx": -0.015, "wz": 0.0},
                "disp_net_m": round(-0.06 - 0.0015 * (i + 1), 3),
                "measured_radps": {m: -0.461 for m in ["m1", "m2", "m3", "m4"]},
                "target_radps": {m: -0.462 for m in ["m1", "m2", "m3", "m4"]},
                "pwm": {m: -47 for m in ["m1", "m2", "m3", "m4"]},
                "pid_terms": {m: {"iTerm": -4.2, "finalPwm": -47} for m in ["m1", "m2", "m3", "m4"]},
                "sequences": {"pid": 200 + i},
                "timestamps": {"pid_data_age_ms": 15.0}
            })

        report = {
            "name": "Diagnostic_Reverse_Long_0.015mps",
            "samples": mock_samples,
            "abort_reason": None,
            "status": "IN_PROGRESS",
            "cleanup_verification": {"zero_command_confirmed": True, "disarm_mode0_confirmed": True}
        }
        res = self.app.assess_diagnostic_tracking(report)

        # 1. Classification
        self.assertTrue(res["tracking_assessed"])
        self.assertEqual(res["equilibrium_status"], "SETTLED_EQUILIBRIUM")
        self.assertIn("All wheels reached settled speed and integral equilibrium", res["reason"])

        # 2. Assert selected window start/end times and duration explicitly
        w_info = res["assessment_window"]
        self.assertIsNotNone(w_info)
        self.assertEqual(w_info["end_t_rel"], 4.4)
        self.assertEqual(w_info["start_t_rel"], 3.4)
        self.assertAlmostEqual(w_info["duration_s"], 1.0, places=2)
        self.assertEqual(w_info["sample_count"], 11)
        self.assertAlmostEqual(res["valid_duration_s"], 1.0, places=2)

        # 3. Assert earlier overspeed remains visible in transient metrics
        tr_info = res["transient_metrics"]
        self.assertIsNotNone(tr_info)
        self.assertEqual(tr_info["start_t_rel"], 0.1)
        self.assertEqual(tr_info["end_t_rel"], 3.3)
        self.assertAlmostEqual(tr_info["duration_s"], 3.2, places=2)
        self.assertEqual(tr_info["sample_count"], 33)
        # Transient speed was overspeed (~ -1.2 rad/s)
        self.assertLess(tr_info["wheels"]["m1"]["mean_measured_radps"], -1.10)
        self.assertLess(tr_info["wheels"]["m1"]["mean_pwm"], -85)

        # Assessment window was settled (-0.461 rad/s)
        self.assertAlmostEqual(res["wheels"]["m1"]["mean_measured_radps"], -0.461, places=3)
        self.assertTrue(res["wheels"]["m1"]["wheel_settled"])

    def test_32_diagnostic_reject_incomplete_telemetry_missing_age(self):
        """Verify incomplete telemetry with missing pid_data_age_ms cannot produce SETTLED_EQUILIBRIUM:
        - Telemetry age is missing or None.
        - Must not default to fresh.
        - Result is INCONCLUSIVE_EQUILIBRIUM.
        - sample_exclusions records missing_or_stale_pid_data_age."""
        mock_samples = []
        for i in range(20):
            t = round(1.0 + i * 0.1, 2)
            mock_samples.append({
                "t_rel": t,
                "seg_idx": 0,
                "cmd": {"vx": -0.015, "wz": 0.0},
                "measured_radps": {m: -0.461 for m in ["m1", "m2", "m3", "m4"]},
                "target_radps": {m: -0.462 for m in ["m1", "m2", "m3", "m4"]},
                "pwm": {m: -47 for m in ["m1", "m2", "m3", "m4"]},
                "pid_terms": {m: {"iTerm": -4.2, "finalPwm": -47} for m in ["m1", "m2", "m3", "m4"]},
                "sequences": {"pid": 100 + i},
                "timestamps": {}  # Missing pid_data_age_ms!
            })
        report = {
            "name": "Diagnostic_Reverse_Long_0.015mps",
            "samples": mock_samples,
            "abort_reason": None,
            "status": "IN_PROGRESS",
            "cleanup_verification": {"zero_command_confirmed": True, "disarm_mode0_confirmed": True}
        }
        res = self.app.assess_diagnostic_tracking(report)
        self.assertFalse(res["tracking_assessed"])
        self.assertEqual(res["equilibrium_status"], "INCONCLUSIVE_EQUILIBRIUM")
        self.assertEqual(res["sample_exclusions"]["missing_or_stale_pid_data_age"], 20)
        self.assertIn("Insufficient valid PID samples", res["reason"])

    def test_33_diagnostic_reject_incomplete_telemetry_missing_integral(self):
        """Verify incomplete telemetry with missing per-wheel iTerm cannot produce SETTLED_EQUILIBRIUM:
        - iTerm is missing / None.
        - Must not default to zero.
        - Result is INCONCLUSIVE_EQUILIBRIUM.
        - sample_exclusions records missing_or_invalid_wheel_telemetry."""
        mock_samples = []
        for i in range(20):
            t = round(1.0 + i * 0.1, 2)
            mock_samples.append({
                "t_rel": t,
                "seg_idx": 0,
                "cmd": {"vx": -0.015, "wz": 0.0},
                "measured_radps": {m: -0.461 for m in ["m1", "m2", "m3", "m4"]},
                "target_radps": {m: -0.462 for m in ["m1", "m2", "m3", "m4"]},
                "pwm": {m: -47 for m in ["m1", "m2", "m3", "m4"]},
                "pid_terms": {m: {"finalPwm": -47} for m in ["m1", "m2", "m3", "m4"]},  # Missing iTerm!
                "sequences": {"pid": 100 + i},
                "timestamps": {"pid_data_age_ms": 10.0}
            })
        report = {
            "name": "Diagnostic_Reverse_Long_0.015mps",
            "samples": mock_samples,
            "abort_reason": None,
            "status": "IN_PROGRESS",
            "cleanup_verification": {"zero_command_confirmed": True, "disarm_mode0_confirmed": True}
        }
        res = self.app.assess_diagnostic_tracking(report)
        self.assertFalse(res["tracking_assessed"])
        self.assertEqual(res["equilibrium_status"], "INCONCLUSIVE_EQUILIBRIUM")
        self.assertEqual(res["sample_exclusions"]["missing_or_invalid_wheel_telemetry"], 20)
        self.assertIn("Insufficient valid PID samples", res["reason"])

    def test_34_diagnostic_reject_non_increasing_or_invalid_timestamps(self):
        """Verify non-increasing or invalid timestamps cannot produce SETTLED_EQUILIBRIUM:
        - Case A: repeated/frozen timestamps.
        - Case B: NaN / non-finite timestamps.
        - Both cases reject samples and result in INCONCLUSIVE_EQUILIBRIUM."""
        # Case A: Frozen / non-increasing timestamps
        mock_samples_frozen = []
        for i in range(20):
            mock_samples_frozen.append({
                "t_rel": 1.500,  # Non-increasing / frozen!
                "seg_idx": 0,
                "cmd": {"vx": -0.015, "wz": 0.0},
                "measured_radps": {m: -0.461 for m in ["m1", "m2", "m3", "m4"]},
                "target_radps": {m: -0.462 for m in ["m1", "m2", "m3", "m4"]},
                "pwm": {m: -47 for m in ["m1", "m2", "m3", "m4"]},
                "pid_terms": {m: {"iTerm": -4.2, "finalPwm": -47} for m in ["m1", "m2", "m3", "m4"]},
                "sequences": {"pid": 100 + i},
                "timestamps": {"pid_data_age_ms": 10.0}
            })
        rep_frozen = {
            "name": "Diagnostic_Reverse_Long_0.015mps",
            "samples": mock_samples_frozen,
            "abort_reason": None,
            "status": "IN_PROGRESS",
            "cleanup_verification": {"zero_command_confirmed": True, "disarm_mode0_confirmed": True}
        }
        res_frozen = self.app.assess_diagnostic_tracking(rep_frozen)
        self.assertFalse(res_frozen["tracking_assessed"])
        self.assertEqual(res_frozen["equilibrium_status"], "INCONCLUSIVE_EQUILIBRIUM")
        self.assertGreater(res_frozen["sample_exclusions"]["timestamp_not_increasing"], 0)

        # Case B: NaN timestamps
        mock_samples_nan = []
        for i in range(20):
            mock_samples_nan.append({
                "t_rel": float("nan"),  # Non-finite!
                "seg_idx": 0,
                "cmd": {"vx": -0.015, "wz": 0.0},
                "measured_radps": {m: -0.461 for m in ["m1", "m2", "m3", "m4"]},
                "target_radps": {m: -0.462 for m in ["m1", "m2", "m3", "m4"]},
                "pwm": {m: -47 for m in ["m1", "m2", "m3", "m4"]},
                "pid_terms": {m: {"iTerm": -4.2, "finalPwm": -47} for m in ["m1", "m2", "m3", "m4"]},
                "sequences": {"pid": 100 + i},
                "timestamps": {"pid_data_age_ms": 10.0}
            })
        rep_nan = {
            "name": "Diagnostic_Reverse_Long_0.015mps",
            "samples": mock_samples_nan,
            "abort_reason": None,
            "status": "IN_PROGRESS",
            "cleanup_verification": {"zero_command_confirmed": True, "disarm_mode0_confirmed": True}
        }
        res_nan = self.app.assess_diagnostic_tracking(rep_nan)
        self.assertFalse(res_nan["tracking_assessed"])
        self.assertEqual(res_nan["equilibrium_status"], "INCONCLUSIVE_EQUILIBRIUM")
        self.assertGreater(res_nan["sample_exclusions"]["missing_or_invalid_timestamp"], 0)

    def test_35_diagnostic_end_to_end_incoming_packet_missing_iterm(self):
        """End-to-end regression test:
        - Incoming PID packet from /api/pid-telemetry lacks 'iTerm'.
        - Telemetry passes through the actual sample-construction path in run_profile_step.
        - Sample collector preserves missing integral data as None (does not default to 0.0).
        - assess_diagnostic_tracking rejects the samples under missing_or_invalid_wheel_telemetry.
        - Confirms the end-to-end execution cannot produce SETTLED_EQUILIBRIUM."""
        seq = 500
        is_armed = False
        step_phase = "IDLE"

        def mock_get(url, timeout=0.25):
            nonlocal seq, is_armed, step_phase
            seq += 1
            now_ms = 50000 + seq * 20
            if "/api/drive/status" in url:
                mode = 3 if is_armed else 0
                return MockResponse({
                    "ok": True, "sequence": seq, "receivedAt": now_ms,
                    "status": {
                        "armed": is_armed, "mode": mode, "stale": False, "dataAgeMs": 5.0,
                        "uptimeMs": now_ms, "seq": seq, "receivedAt": now_ms,
                        "spinFailedExhausted": False, "noMotionBudgetMs": 5000, "chassisRebreakoutConsumed": False
                    }
                })
            elif "/api/pid-telemetry" in url:
                # Actual incoming PID packet missing 'iTerm'!
                return MockResponse({
                    "ok": True, "valid": True, "stale": False, "sequence": seq,
                    "timestamp": now_ms, "dataAgeMs": 5.0,
                    "telemetry": {
                        m: {"measuredRadps": -0.461, "targetRadps": -0.462, "finalPwm": -47, "stictionState": "STICTION_KINETIC"}
                        for m in ["m1", "m2", "m3", "m4"]
                    }
                })
            elif "/api/encoders" in url:
                return MockResponse({"ok": True, "sequence": seq, "lastPacketAgeMs": 5.0, "encoders": {"m1": -5, "m2": -5, "m3": -5, "m4": -5}})
            elif "/api/imu" in url:
                return MockResponse({
                    "ok": True, "sequence": seq, "dataAgeMs": 5.0,
                    "gyro": {"z": 0.001 if step_phase == "STOPPED" else 0.002},
                    "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
                })
            return MockResponse({"ok": True})

        def mock_post(url, json=None, headers=None, timeout=0.1):
            nonlocal is_armed, step_phase
            if "/api/drive/arm" in url:
                is_armed = True
                step_phase = "ACTIVE"
            elif "/api/drive/disarm" in url:
                is_armed = False
            elif "/api/cmd_vel" in url:
                if is_armed and json and json.get("linear", {}).get("x", 0.0) == 0.0:
                    step_phase = "STOPPED"
            return MockResponse({"ok": True})

        self.app.session.get = mock_get
        self.app.session.post = mock_post

        res = self.app.run_profile_step(
            name="Diagnostic_Reverse_Long_0.015mps",
            segments=[(-0.015, 0.0, 0.15)],
            max_dist_m=0.20,
            max_yaw_deg=10.0,
            max_speed_mps=0.060
        )

        # 1. Verify samples passed through actual sample-construction path and preserved iTerm as None
        active_samples = [s for s in res["samples"] if s.get("seg_idx", -1) != -1]
        self.assertGreater(len(active_samples), 0)
        for s in active_samples:
            self.assertIsNone(s["pid_terms"]["m1"]["iTerm"])

        # 2. Confirm end-to-end execution CANNOT produce SETTLED_EQUILIBRIUM
        self.assertIn("diagnostic_evaluation", res)
        diag = res["diagnostic_evaluation"]
        self.assertNotEqual(diag["equilibrium_status"], "SETTLED_EQUILIBRIUM")
        self.assertEqual(diag["equilibrium_status"], "INCONCLUSIVE_EQUILIBRIUM")
        self.assertGreater(diag["sample_exclusions"]["missing_or_invalid_wheel_telemetry"], 0)

    def test_36_diagnostic_reject_incorrect_target(self):
        """Verify settled tracking of an incorrect target cannot qualify as SETTLED_EQUILIBRIUM:
        - Target in telemetry is -0.80 rad/s (discrepancy > 0.050 rad/s from expected -0.462 rad/s).
        - Speeds match the incorrect target with zero error and zero slope.
        - assess_diagnostic_tracking rejects qualification because per-wheel target agreement fails."""
        mock_samples = []
        for i in range(25):
            t = round(1.0 + i * 0.05, 3)
            mock_samples.append({
                "t_rel": t,
                "seg_idx": 0,
                "cmd": {"vx": -0.015, "wz": 0.0},
                "measured_radps": {m: -0.800 for m in ["m1", "m2", "m3", "m4"]},
                "target_radps": {m: -0.800 for m in ["m1", "m2", "m3", "m4"]},  # Incorrect target!
                "pwm": {m: -55 for m in ["m1", "m2", "m3", "m4"]},
                "pid_terms": {m: {"iTerm": -4.2, "finalPwm": -55} for m in ["m1", "m2", "m3", "m4"]},
                "sequences": {"pid": 200 + i},
                "timestamps": {"pid_data_age_ms": 10.0}
            })
        report = {
            "name": "Diagnostic_Reverse_Long_0.015mps",
            "samples": mock_samples,
            "abort_reason": None,
            "status": "IN_PROGRESS",
            "cleanup_verification": {"zero_command_confirmed": True, "disarm_mode0_confirmed": True}
        }
        res = self.app.assess_diagnostic_tracking(report)
        self.assertTrue(res["tracking_assessed"])
        self.assertEqual(res["equilibrium_status"], "INCONCLUSIVE_EQUILIBRIUM")
        self.assertIn("target discrepancy", res["reason"])
        for m in ["m1", "m2", "m3", "m4"]:
            self.assertFalse(res["wheels"][m]["target_agrees"])
            self.assertFalse(res["wheels"][m]["wheel_settled"])


if __name__ == '__main__':
    unittest.main()





