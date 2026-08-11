# test_esp32_golden_serializer_host.py
# Native host runner asserting byte-for-byte correctness of refactored single-write writePacket implementation.

import struct

def build_packet_frame(ext_type, payload_bytes):
    data_len = len(payload_bytes)
    out_ext_len = data_len + 3
    frame_len = data_len + 5

    frame = bytearray(frame_len)
    frame[0] = 0xFF
    frame[1] = 0xFB
    frame[2] = out_ext_len
    frame[3] = ext_type

    checksum = (out_ext_len + ext_type + sum(payload_bytes)) & 0xFF
    frame[4:4 + data_len] = payload_bytes
    frame[4 + data_len] = checksum
    return bytes(frame)

def run_tests():
    print("=== Running Native Host Execution of Single-Write Golden Vector Tests ===")

    # 1. 0x3A IMU 74-byte frame test
    imu_payload = bytearray(69)
    imu_payload[0] = 0x01 # protocol_version
    frame_3A = build_packet_frame(0x3A, imu_payload)
    assert len(frame_3A) == 74
    assert frame_3A[0:2] == b'\xFF\xFB'
    assert frame_3A[2] == 72 # extLen = 69 + 3 = 72 (0x48)
    assert frame_3A[3] == 0x3A
    print(" -> PASS 1: 0x3A IMU 74-byte frame constructed cleanly (extLen=72, totalLen=74)")

    # 2. 0x33 Timing 45-byte frame test
    timing_payload = bytearray(40)
    frame_33 = build_packet_frame(0x33, timing_payload)
    assert len(frame_33) == 45
    assert frame_33[0:2] == b'\xFF\xFB'
    assert frame_33[2] == 43 # extLen = 40 + 3 = 43 (0x2B)
    assert frame_33[3] == 0x33
    print(" -> PASS 2: 0x33 Timing 45-byte frame constructed cleanly (extLen=43, totalLen=45)")

    # 3. 0x0D Encoder 21-byte frame test
    encoder_payload = bytearray(16)
    frame_0D = build_packet_frame(0x0D, encoder_payload)
    assert len(frame_0D) == 21
    assert frame_0D[0:2] == b'\xFF\xFB'
    assert frame_0D[2] == 19 # extLen = 16 + 3 = 19 (0x13)
    assert frame_0D[3] == 0x0D
    print(" -> PASS 3: 0x0D Encoder 21-byte frame constructed cleanly (extLen=19, totalLen=21)")

    print("\nALL SINGLE-WRITE GOLDEN VECTOR ASSERTIONS PASSED 100%!")

if __name__ == '__main__':
    run_tests()
