"""UART wire format — MicroPython half of the ESP32 ⇄ Servo2040 link.

Mirrors esp32-master/src/protocol/protocol.h byte-for-byte. See PROTOCOL.md. Keep the two
in lockstep and bump PROTOCOL_VERSION on any layout change.
"""

import struct

PROTOCOL_VERSION = 1
SOF = 0xA5

# Message types (high bit set = slave → master).
SERVO_TARGETS = 0x01
SET_MODE      = 0x02
LED           = 0x03
PING          = 0x04
STATUS        = 0x81
FAULT         = 0x82
PONG          = 0x83

# Slave modes.
RELAX  = 0
ACTIVE = 1
HOLD   = 2
SAFE   = 3

# Fault codes.
FAULT_LINK_TIMEOUT = 1
FAULT_MALFORMED    = 2
FAULT_OUT_OF_RANGE = 3
FAULT_DRIVER       = 4

# "Hold this channel" sentinel inside a SERVO_TARGETS payload.
SERVO_HOLD = -32768


def crc16(data):
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over `data` (bytes)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode_frame(msg_type, payload=b""):
    """Build a complete frame: SOF, TYPE, LEN, PAYLOAD, CRC16(LE)."""
    body = bytes((msg_type, len(payload))) + payload
    crc = crc16(body)
    return bytes((SOF,)) + body + struct.pack("<H", crc)


def decode_servo_targets(payload):
    """12 × int16 little-endian centi-degrees → list of 12 floats (degrees).

    Returns None on a bad length. SERVO_HOLD entries stay as None in the result so the
    controller knows to leave that channel where it is.
    """
    if len(payload) != 24:
        return None
    raw = struct.unpack("<12h", payload)
    return [None if v == SERVO_HOLD else v / 100.0 for v in raw]
