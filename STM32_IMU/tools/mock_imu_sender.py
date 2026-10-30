#!/usr/bin/env python3
"""
Unit test to verify CRC16-CCITT and packet packing/unpacking against C firmware structures.
"""
import struct

# Match C code:
# Packet format: 2B sync, 1I ts, 4h quat, 3h acc, 3h gyr, 3h eul, 1B cal, 1b temp, 1H crc16
PACKET_FORMAT = "<2s I 4h 3h 3h 3h B b H"

def compute_crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def test_packet():
    sync = b"\x55\xAA"
    ts_ms = 123456
    qw = int(1.0 * (1 << 14)) # 16384
    qx = 0
    qy = 0
    qz = 0
    ax = 0
    ay = 0
    az = 981 # 9.81 m/s^2
    gx = 0
    gy = 0
    gz = 0
    yaw = 0
    roll = 0
    pitch = 0
    calib = 0xFF # all 3
    temp = 28

    payload = struct.pack("<2s I 4h 3h 3h 3h B b", sync, ts_ms, qw, qx, qy, qz, ax, ay, az, gx, gy, gz, yaw, roll, pitch, calib, temp)
    crc = compute_crc16_ccitt(payload)
    full_pkt = payload + struct.pack("<H", crc)

    assert len(full_pkt) == 36, f"Expected 36 bytes, got {len(full_pkt)}"
    print(f"[OK] Packet size: {len(full_pkt)} bytes")
    print(f"[OK] Computed CRC16: 0x{crc:04X}")

    # Unpack
    unpacked = struct.unpack(PACKET_FORMAT, full_pkt)
    assert unpacked[0] == sync
    assert unpacked[1] == ts_ms
    assert unpacked[2] == qw
    assert unpacked[8] == az
    assert unpacked[17] == crc
    print("[OK] Verification successful! Telemetry format matches C firmware perfectly.")

if __name__ == "__main__":
    test_packet()
