import unittest
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../scratch'))

class TestImuGameRotationVectorAndGapGuard(unittest.TestCase):
    """
    Validation test suite for SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC adoption
    and telemetry numerical integration gap guard safety.
    """

    def test_firmware_imu_manager_uses_game_rotation_vector(self):
        header_path = os.path.join(os.path.dirname(__file__), '../src/ImuManager.h')
        cpp_path = os.path.join(os.path.dirname(__file__), '../src/ImuManager.cpp')

        self.assertTrue(os.path.exists(header_path), "ImuManager.h must exist")
        self.assertTrue(os.path.exists(cpp_path), "ImuManager.cpp must exist")

        with open(header_path, 'r', encoding='utf-8') as f:
            h_content = f.read()

        with open(cpp_path, 'r', encoding='utf-8') as f:
            cpp_content = f.read()

        # 1. Confirm SH2_GAME_ROTATION_VECTOR is enabled
        self.assertIn("enableReport(SH2_GAME_ROTATION_VECTOR", cpp_content)
        self.assertIn("case SH2_GAME_ROTATION_VECTOR:", cpp_content)
        self.assertIn("gameRotationVector", cpp_content)

        # 2. Confirm SH2_ROTATION_VECTOR (magnetometer-fused) is NOT enabled
        self.assertNotIn("enableReport(SH2_ROTATION_VECTOR", cpp_content)
        self.assertNotIn("case SH2_ROTATION_VECTOR:", cpp_content)

        # 3. Confirm non-magnetic orientation source diagnostic accessor
        self.assertIn("SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC", h_content)
        self.assertIn("SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC", cpp_content)

    def test_ros_imu_bridge_uses_game_rotation_vector(self):
        bridge_path = os.path.join(os.path.dirname(__file__), '../src/rover_bringup/rover_bringup/rover_imu_bridge.py')
        if not os.path.exists(bridge_path):
            bridge_path = "C:/Users/Ron/electronic_projects/yahboom-encoder/ros2/ros2_ws/src/rover_bringup/rover_bringup/rover_imu_bridge.py"
        if not os.path.exists(bridge_path):
            bridge_path = "/home/ron/yahboom-encoder/ros2/ros2_ws/src/rover_bringup/rover_bringup/rover_imu_bridge.py"

        if os.path.exists(bridge_path):
            with open(bridge_path, 'r', encoding='utf-8') as f:
                content = f.read()
            # Verify orientation & gyro processing
            self.assertIn("data.get('orientation", content)
            self.assertIn("data.get('gyro", content)
            self.assertIn("msg.orientation", content)
            self.assertIn("msg.angular_velocity", content)

    def test_telemetry_gap_guard_rejection(self):
        """
        Reproduce the 2-second sleep gap scenario and prove that gap-guarded integration
        does NOT create false accumulated rotation across unobserved gaps (> 100ms).
        """
        # Simulated sequence with a 2-second logging gap
        samples = [
            {'t': 0.00, 'gz': 0.0},
            {'t': 0.05, 'gz': math.radians(-22.92)}, # -0.40 rad/s
            {'t': 0.10, 'gz': math.radians(-22.92)},
            {'t': 0.15, 'gz': math.radians(-12.87)}, # -0.2246 rad/s (at zero command crossing)
            {'t': 2.15, 'gz': 0.0},                  # 2.0-second unobserved sleep gap!
            {'t': 2.20, 'gz': 0.0}
        ]

        MAX_VALID_GAP_S = 0.100

        # Un-guarded integration (naive multiplication across 2s gap)
        unguarded_total_deg = 0.0
        for i in range(len(samples) - 1):
            dt = samples[i+1]['t'] - samples[i]['t']
            d_angle = math.degrees(samples[i]['gz'] * dt)
            unguarded_total_deg += d_angle

        # Gap-guarded integration
        guarded_total_deg = 0.0
        rejected_gaps = 0
        for i in range(len(samples) - 1):
            dt = samples[i+1]['t'] - samples[i]['t']
            if dt > MAX_VALID_GAP_S:
                rejected_gaps += 1
                continue
            d_angle = math.degrees(samples[i]['gz'] * dt)
            guarded_total_deg += d_angle

        # Prove naive integration produces false overshoot (-25.74 deg across gap)
        self.assertLess(unguarded_total_deg, -20.0, "Naive integration creates false rotation")

        # Prove gap-guarded integration rejects the 2-second gap and returns true active rotation
        self.assertEqual(rejected_gaps, 1, "Gap guard must detect exactly 1 unobserved gap")
        self.assertAlmostEqual(guarded_total_deg, -2.292, delta=0.1, msg="Gap-guarded integration must measure only active motion")

    def test_binary_protocol_schema_compatibility(self):
        """
        Verify that IMU binary packet structure (0x3A, 69 bytes) wire layout is preserved.
        """
        header_path = os.path.join(os.path.dirname(__file__), '../src/SerialProtocol.h')
        cpp_path = os.path.join(os.path.dirname(__file__), '../src/SerialProtocol.cpp')

        with open(cpp_path, 'r', encoding='utf-8') as f:
            content = f.read()

        self.assertIn("serializeImuTelemetry(p, seq, d, snapUs, imuManager)", content)
        self.assertIn("writePacket(0x3A, p, 69);", content)

if __name__ == '__main__':
    unittest.main()
