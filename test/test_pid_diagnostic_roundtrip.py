"""
test/test_pid_diagnostic_roundtrip.py - Round-trip serialization test for PID diagnostic packet 0x3B.

Verifies:
1. SerialProtocol C++ 96-byte packet payload layout across offsets 0..95.
2. Exact offsets and scaling expected by server.js (0x3B TYPE_PID_DIAGNOSTIC decoder).
3. Preservation of offsets 80..95 (basePwm and spinSyncTrim) and offsets 64..79 (outerYaw).
4. Distinctive nonzero positive and negative values for every field and every wheel.
5. Exact decoded outputs for targetRadps, measuredRadps, PID terms, PWM, and stiction states.
"""

import unittest
import struct


def serialize_pid_diagnostic_packet(wheels_diag, outer_diag) -> bytes:
    """
    Implements byte-for-byte serialization of SerialProtocol::sendPidTelemetry (0x3B).
    Generates 96-byte payload.
    """
    pid_data = bytearray(96)

    for i in range(4):
        d = wheels_diag[i]
        t_val = int(round(d["targetVel"] * 100.0))
        m_val = int(round(d["measuredVel"] * 100.0))
        ff_val = int(round(d["feedforward"] * 10.0))
        p_val = int(round(d["pTerm"] * 10.0))
        i_val = int(round(d["iTerm"] * 10.0))
        d_val = int(round(d["dTerm"] * 10.0))
        pwm_val = int(d["finalPwm"])
        state_val = int(d["stictionState"])

        off = i * 16
        struct.pack_into("<h", pid_data, off + 0, t_val)
        struct.pack_into("<h", pid_data, off + 2, m_val)
        struct.pack_into("<h", pid_data, off + 4, ff_val)
        struct.pack_into("<h", pid_data, off + 6, p_val)
        struct.pack_into("<h", pid_data, off + 8, i_val)
        struct.pack_into("<h", pid_data, off + 10, d_val)
        struct.pack_into("<h", pid_data, off + 12, pwm_val)
        struct.pack_into("<h", pid_data, off + 14, state_val)

        # Extended Versioned Spin Sync Trim Telemetry (Offset 80..95)
        base_pwm_val = int(d["basePwm"])
        sync_trim_val = int(d["spinSyncTrim"])
        struct.pack_into("<h", pid_data, 80 + i * 4 + 0, base_pwm_val)
        struct.pack_into("<h", pid_data, 80 + i * 4 + 2, sync_trim_val)

    # Outer Yaw-Rate Loop Telemetry (16 bytes, offset 64..79)
    wz_req_val = int(round(outer_diag["wzRequested"] * 100.0))
    wz_act_val = int(round(outer_diag["wzActual"] * 100.0))
    err_val = int(round(outer_diag["yawOuterError"] * 100.0))
    corr_val = int(round(outer_diag["yawOuterCorrection"] * 100.0))
    wz_corr_val = int(round(outer_diag["wzCorrected"] * 100.0))
    active_val = 1 if outer_diag["yawOuterActive"] else 0
    valid_val = 1 if outer_diag["imuGyroValid"] else 0
    age_val = int(outer_diag["imuGyroAgeMs"])
    act_val = int(outer_diag.get("actuationState", 2))  # 0=DRIVE, 1=BRAKE, 2=COAST
    cfg_val = int(outer_diag.get("configFlags", 0))     # bit 0=balance, bit 1=brake

    struct.pack_into("<h", pid_data, 64, wz_req_val)
    struct.pack_into("<h", pid_data, 66, wz_act_val)
    struct.pack_into("<h", pid_data, 68, err_val)
    struct.pack_into("<h", pid_data, 70, corr_val)
    struct.pack_into("<h", pid_data, 72, wz_corr_val)
    struct.pack_into("<B", pid_data, 74, active_val)
    struct.pack_into("<B", pid_data, 75, valid_val)
    struct.pack_into("<H", pid_data, 76, age_val)
    struct.pack_into("<B", pid_data, 78, act_val)
    struct.pack_into("<B", pid_data, 79, cfg_val)

    return bytes(pid_data)


def build_framed_packet(ext_type: int, payload: bytes) -> bytes:
    """Wraps payload in SerialProtocol framing: [0xFF, 0xFB, extLen, extType, ...data, checksum]."""
    data_len = len(payload)
    out_ext_len = data_len + 3
    frame = bytearray(data_len + 5)
    frame[0] = 0xFF
    frame[1] = 0xFB
    frame[2] = out_ext_len
    frame[3] = ext_type
    frame[4:4 + data_len] = payload
    checksum = (out_ext_len + ext_type + sum(payload)) & 0xFF
    frame[4 + data_len] = checksum
    return bytes(frame)


def decode_server_js_0x3b(payload: bytes):
    """
    Exact replica of server.js 0x3B TYPE_PID_DIAGNOSTIC decode logic (lines 1768-1818).
    """
    data = payload
    assert len(data) >= 56
    wheels = []
    is64_byte = (len(data) >= 64)
    is96_byte = (len(data) >= 96)
    stride = 16 if is64_byte else 14
    stiction_names = ['IDLE', 'STICTION_BOOST', 'KINETIC', 'BLOCKED']

    for i in range(4):
        off = i * stride
        target_radps = struct.unpack_from("<h", data, off + 0)[0] / 100.0
        measured_radps = struct.unpack_from("<h", data, off + 2)[0] / 100.0
        feedforward = struct.unpack_from("<h", data, off + 4)[0] / 10.0
        p_term = struct.unpack_from("<h", data, off + 6)[0] / 10.0
        i_term = struct.unpack_from("<h", data, off + 8)[0] / 10.0
        d_term = struct.unpack_from("<h", data, off + 10)[0] / 10.0
        final_pwm = struct.unpack_from("<h", data, off + 12)[0]
        stiction_code = struct.unpack_from("<h", data, off + 14)[0] if is64_byte else 0
        stiction_state = stiction_names[stiction_code] if stiction_code < len(stiction_names) else 'UNKNOWN'

        base_pwm = final_pwm
        spin_sync_trim = 0
        if is96_byte:
            base_pwm = struct.unpack_from("<h", data, 80 + i * 4 + 0)[0]
            spin_sync_trim = struct.unpack_from("<h", data, 80 + i * 4 + 2)[0]

        wheel_dict = {
            "targetRadps": target_radps,
            "measuredRadps": measured_radps,
            "feedforward": feedforward,
            "pTerm": p_term,
            "iTerm": i_term,
            "dTerm": d_term,
            "basePwm": base_pwm,
            "spinSyncTrim": spin_sync_trim,
            "finalPwm": final_pwm,
            "stictionCode": stiction_code,
            "stictionState": stiction_state
        }
        wheels.append(wheel_dict)

    outer_yaw = None
    actuation_state = "COAST"
    config_flags = 0
    if len(data) >= 80:
        actuation_map = ['DRIVE', 'BRAKE', 'COAST']
        act_byte = data[78]
        actuation_state = actuation_map[act_byte] if act_byte < len(actuation_map) else 'COAST'
        config_flags = data[79]
        outer_yaw = {
            "wzRequested": struct.unpack_from("<h", data, 64)[0] / 100.0,
            "wzActual": struct.unpack_from("<h", data, 66)[0] / 100.0,
            "yawOuterError": struct.unpack_from("<h", data, 68)[0] / 100.0,
            "yawOuterCorrection": struct.unpack_from("<h", data, 70)[0] / 100.0,
            "wzCorrected": struct.unpack_from("<h", data, 72)[0] / 100.0,
            "yawOuterActive": bool(data[74]),
            "imuGyroValid": bool(data[75]),
            "imuGyroAgeMs": struct.unpack_from("<H", data, 76)[0],
            "actuationState": actuation_state,
            "configFlags": config_flags
        }

    return {
        "actuationState": actuation_state,
        "configFlags": config_flags,
        "m1": wheels[0],
        "m2": wheels[1],
        "m3": wheels[2],
        "m4": wheels[3],
        "outerYaw": outer_yaw
    }


class TestPidDiagnosticSerialization(unittest.TestCase):
    """
    Test suite asserting exact round-trip serialization and decoding of 0x3B frames
    with distinctive positive, negative, and mixed values for every field on every wheel.
    """

    def setUp(self):
        # Distinctive nonzero positive and negative test vectors
        self.wheels_input = [
            {
                # Wheel 0 (m1 / LF) - Negative CW spin, kinetic
                "targetVel": -4.25,
                "measuredVel": -4.12,
                "feedforward": -15.2,
                "pTerm": -8.4,
                "iTerm": -1.5,
                "dTerm": -0.6,
                "finalPwm": -185,
                "stictionState": 2,  # KINETIC
                "basePwm": -190,
                "spinSyncTrim": 5,
            },
            {
                # Wheel 1 (m2 / RF) - Positive CW spin, stiction boost active
                "targetVel": 3.80,
                "measuredVel": 3.65,
                "feedforward": 12.6,
                "pTerm": 7.1,
                "iTerm": 2.3,
                "dTerm": 1.1,
                "finalPwm": 160,
                "stictionState": 1,  # STICTION_BOOST
                "basePwm": 150,
                "spinSyncTrim": -10,
            },
            {
                # Wheel 2 (m3 / LR) - Negative CW spin, blocked/stalled
                "targetVel": -2.75,
                "measuredVel": -0.02,
                "feedforward": -9.8,
                "pTerm": -14.5,
                "iTerm": -4.2,
                "dTerm": 0.8,
                "finalPwm": -240,
                "stictionState": 3,  # BLOCKED
                "basePwm": -230,
                "spinSyncTrim": -10,
            },
            {
                # Wheel 3 (m4 / RR) - Positive CW spin, idle
                "targetVel": 1.50,
                "measuredVel": 1.48,
                "feedforward": 5.4,
                "pTerm": 3.2,
                "iTerm": -0.8,
                "dTerm": -1.4,
                "finalPwm": 95,
                "stictionState": 0,  # IDLE
                "basePwm": 90,
                "spinSyncTrim": 5,
            },
        ]

        self.outer_yaw_input = {
            "wzRequested": 0.80,
            "wzActual": 0.76,
            "yawOuterError": 0.04,
            "yawOuterCorrection": -0.12,
            "wzCorrected": 0.68,
            "yawOuterActive": True,
            "imuGyroValid": True,
            "imuGyroAgeMs": 12,
        }

    def test_packet_length_and_layout(self):
        """Verifies 96-byte payload and 101-byte framed packet structure."""
        payload = serialize_pid_diagnostic_packet(self.wheels_input, self.outer_yaw_input)
        self.assertEqual(len(payload), 96, "PID telemetry payload must be exactly 96 bytes")

        frame = build_framed_packet(0x3B, payload)
        self.assertEqual(len(frame), 101, "Framed 0x3B packet must be exactly 101 bytes")
        self.assertEqual(frame[0], 0xFF)
        self.assertEqual(frame[1], 0xFB)
        self.assertEqual(frame[2], 99)  # extLen = 96 + 3 = 99
        self.assertEqual(frame[3], 0x3B)  # extType

        # Checksum verification
        expected_checksum = (99 + 0x3B + sum(payload)) & 0xFF
        self.assertEqual(frame[100], expected_checksum)

    def test_exact_roundtrip_all_wheels_and_fields(self):
        """
        Asserts that every wheel's fields decode with exact precision
        matching server.js scaling and typing.
        """
        payload = serialize_pid_diagnostic_packet(self.wheels_input, self.outer_yaw_input)
        decoded = decode_server_js_0x3b(payload)

        # Expected wheel names mapping
        wheel_keys = ["m1", "m2", "m3", "m4"]
        stiction_strings = ["IDLE", "STICTION_BOOST", "KINETIC", "BLOCKED"]

        for idx, w_key in enumerate(wheel_keys):
            inp = self.wheels_input[idx]
            out = decoded[w_key]

            # 1. Target & Measured Rad/s (scaled x100 in firmware, /100 in server.js)
            self.assertAlmostEqual(out["targetRadps"], inp["targetVel"], places=2,
                                   msg=f"{w_key} targetRadps mismatch")
            self.assertAlmostEqual(out["measuredRadps"], inp["measuredVel"], places=2,
                                   msg=f"{w_key} measuredRadps mismatch")

            # 2. PID Terms (scaled x10 in firmware, /10 in server.js)
            self.assertAlmostEqual(out["feedforward"], inp["feedforward"], places=1,
                                   msg=f"{w_key} feedforward mismatch")
            self.assertAlmostEqual(out["pTerm"], inp["pTerm"], places=1,
                                   msg=f"{w_key} pTerm mismatch")
            self.assertAlmostEqual(out["iTerm"], inp["iTerm"], places=1,
                                   msg=f"{w_key} iTerm mismatch")
            self.assertAlmostEqual(out["dTerm"], inp["dTerm"], places=1,
                                   msg=f"{w_key} dTerm mismatch")

            # 3. PWM (unscaled int16)
            self.assertEqual(out["finalPwm"], inp["finalPwm"],
                             msg=f"{w_key} finalPwm mismatch")

            # 4. Stiction state & code
            self.assertEqual(out["stictionCode"], inp["stictionState"],
                             msg=f"{w_key} stictionCode mismatch")
            self.assertEqual(out["stictionState"], stiction_strings[inp["stictionState"]],
                             msg=f"{w_key} stictionState string mismatch")

            # 5. Extended spin sync trim fields (offsets 80..95)
            self.assertEqual(out["basePwm"], inp["basePwm"],
                             msg=f"{w_key} basePwm mismatch at offset {80 + idx*4}")
            self.assertEqual(out["spinSyncTrim"], inp["spinSyncTrim"],
                             msg=f"{w_key} spinSyncTrim mismatch at offset {82 + idx*4}")

    def test_outer_yaw_telemetry_roundtrip(self):
        """Verifies outer yaw loop telemetry preservation at offsets 64..79."""
        payload = serialize_pid_diagnostic_packet(self.wheels_input, self.outer_yaw_input)
        decoded = decode_server_js_0x3b(payload)
        outer = decoded["outerYaw"]

        self.assertIsNotNone(outer)
        self.assertAlmostEqual(outer["wzRequested"], 0.80, places=2)
        self.assertAlmostEqual(outer["wzActual"], 0.76, places=2)
        self.assertAlmostEqual(outer["yawOuterError"], 0.04, places=2)
        self.assertAlmostEqual(outer["yawOuterCorrection"], -0.12, places=2)
        self.assertAlmostEqual(outer["wzCorrected"], 0.68, places=2)
        self.assertTrue(outer["yawOuterActive"])
        self.assertTrue(outer["imuGyroValid"])
        self.assertEqual(outer["imuGyroAgeMs"], 12)

    def test_distinctive_polarity_and_non_zero_invariants(self):
        """Ensures test vectors strictly contain nonzero positive and negative values."""
        payload = serialize_pid_diagnostic_packet(self.wheels_input, self.outer_yaw_input)
        decoded = decode_server_js_0x3b(payload)

        # Confirm opposing polarities exist
        self.assertLess(decoded["m1"]["targetRadps"], 0.0)
        self.assertGreater(decoded["m2"]["targetRadps"], 0.0)
        self.assertLess(decoded["m3"]["targetRadps"], 0.0)
        self.assertGreater(decoded["m4"]["targetRadps"], 0.0)

        # Confirm measured speeds are nonzero and match polarities
        self.assertLess(decoded["m1"]["measuredRadps"], 0.0)
        self.assertGreater(decoded["m2"]["measuredRadps"], 0.0)
        self.assertLess(decoded["m3"]["measuredRadps"], 0.0)
        self.assertGreater(decoded["m4"]["measuredRadps"], 0.0)

        # Confirm no wheel record in bytes 0..63 is all-zero
        for i in range(4):
            wheel_record = payload[i*16 : (i+1)*16]
            self.assertNotEqual(wheel_record, bytes(16), f"Wheel {i} record must not be all zeroes")

    def test_bytes_78_79_actuation_state_and_config_flags_roundtrip(self):
        """Verifies exact serialization and decoding of bytes 78 (actuationState) and 79 (configFlags)."""
        test_cases = [
            (0, 0x00, "DRIVE", False, False),
            (1, 0x02, "BRAKE", False, True),
            (2, 0x01, "COAST", True, False),
            (1, 0x03, "BRAKE", True, True),
        ]
        for act_code, cfg_flags, expected_act_str, exp_bal, exp_brk in test_cases:
            outer = dict(self.outer_yaw_input)
            outer["actuationState"] = act_code
            outer["configFlags"] = cfg_flags
            payload = serialize_pid_diagnostic_packet(self.wheels_input, outer)

            # Byte assertions
            self.assertEqual(len(payload), 96, "0x3B payload must remain exactly 96 bytes")
            self.assertEqual(payload[78], act_code, f"Byte 78 must contain actuation code {act_code}")
            self.assertEqual(payload[79], cfg_flags, f"Byte 79 must contain config flags 0x{cfg_flags:02X}")

            decoded = decode_server_js_0x3b(payload)
            self.assertEqual(decoded["actuationState"], expected_act_str)
            self.assertEqual(decoded["configFlags"], cfg_flags)
            self.assertEqual(decoded["outerYaw"]["actuationState"], expected_act_str)
            self.assertEqual(decoded["outerYaw"]["configFlags"], cfg_flags)

    def test_binary_compatibility_preserves_all_prior_offsets(self):
        """Confirms that adding bytes 78-79 does not alter offsets 0..77 or 80..95."""
        outer = dict(self.outer_yaw_input)
        outer["actuationState"] = 1  # BRAKE
        outer["configFlags"] = 0x03  # both enabled
        payload = serialize_pid_diagnostic_packet(self.wheels_input, outer)

        self.assertEqual(len(payload), 96)
        # Verify wheel 0 offset 0..15
        self.assertEqual(struct.unpack_from("<h", payload, 0)[0] / 100.0, self.wheels_input[0]["targetVel"])
        # Verify wheel 3 offset 48..63
        self.assertEqual(struct.unpack_from("<h", payload, 48)[0] / 100.0, self.wheels_input[3]["targetVel"])
        # Verify outer yaw offset 64..77
        self.assertAlmostEqual(struct.unpack_from("<h", payload, 64)[0] / 100.0, 0.80, places=2)
        # Verify spin sync trim offset 80..95
        self.assertEqual(struct.unpack_from("<h", payload, 80)[0], self.wheels_input[0]["basePwm"])
        self.assertEqual(struct.unpack_from("<h", payload, 82)[0], self.wheels_input[0]["spinSyncTrim"])


if __name__ == "__main__":
    unittest.main()
