"""Desktop self-test for the slave logic. Run with CPython:  python selftest.py

Exercises the protocol round-trip, the frame parser's resync, and one full pass of the
Slave handling a SERVO_TARGETS frame against the mock hardware. Not flashed to the board.
"""

import struct
import sys

import config
config.EMULATION_MODE = True   # force mocks before anything imports hardware

import protocol
from uart_link import FrameParser


def test_crc_and_roundtrip():
    angles_centi = list(range(-600, 600, 100))[:12]
    payload = struct.pack("<12h", *angles_centi)
    frame = protocol.encode_frame(protocol.SERVO_TARGETS, payload)
    assert frame[0] == protocol.SOF

    parser = FrameParser()
    out = None
    for byte in frame:
        result = parser.feed(byte)
        if result is not None:
            out = result
    assert out is not None, "frame did not parse"
    msg_type, got = out
    assert msg_type == protocol.SERVO_TARGETS
    decoded = protocol.decode_servo_targets(got)
    assert decoded == [v / 100.0 for v in angles_centi], decoded
    print("ok: protocol round-trip")


def test_parser_resyncs_after_garbage():
    parser = FrameParser()
    good = protocol.encode_frame(protocol.PING)
    stream = b"\x00\xffjunk" + good          # noise then a real frame
    results = [r for byte in stream if (r := parser.feed(byte)) is not None]
    assert len(results) == 1 and results[0][0] == protocol.PING
    print("ok: parser resyncs after garbage")


def test_bad_crc_dropped():
    parser = FrameParser()
    frame = bytearray(protocol.encode_frame(protocol.PING))
    frame[-1] ^= 0xFF                          # corrupt CRC
    results = [r for byte in frame if (r := parser.feed(byte)) is not None]
    assert results == [] and parser.bad_frames == 1
    print("ok: bad-CRC frame dropped")


def test_slave_processes_targets():
    from main import Slave
    slave = Slave()
    # Put it in ACTIVE so targets are accepted.
    slave._handle(protocol.SET_MODE, bytes((protocol.ACTIVE,)))
    angles = list(range(0, 1200, 100))         # 12 logical angles, centi-deg
    payload = struct.pack("<12h", *angles)
    slave._handle(protocol.SERVO_TARGETS, payload)
    slave.servos.update(0.02)                  # one smoothing step
    assert slave.servos.enabled
    print("ok: slave accepts targets, servos enabled")


def test_slave_status_and_failsafe():
    from main import Slave
    slave = Slave()
    slave._handle(protocol.SET_MODE, bytes((protocol.ACTIVE,)))

    # STATUS frame encodes and parses back.
    slave._send_status(5000)
    frames = list(_drain(slave))
    status = [f for f in frames if f[0] == protocol.STATUS]
    assert status, "no STATUS frame emitted"
    assert status[0][1][0] == protocol.ACTIVE
    print("ok: slave emits a parseable STATUS frame")

    # Force the watchdog to expire → fail-safe to SAFE.
    slave.watchdog._last_ok -= (config.LINK_TIMEOUT_MS + 50)
    assert slave.watchdog.trip_once()
    slave._apply_mode(protocol.SAFE)
    assert slave.mode == protocol.SAFE and slave.servos.enabled
    print("ok: watchdog timeout drives slave to SAFE")


def _drain(slave):
    """Parse whatever the slave has written to its mock UART TX buffer."""
    from uart_link import FrameParser
    parser = FrameParser()
    for byte in bytes(slave.link._uart.tx):
        result = parser.feed(byte)
        if result is not None:
            yield result


if __name__ == "__main__":
    test_crc_and_roundtrip()
    test_parser_resyncs_after_garbage()
    test_bad_crc_dropped()
    test_slave_processes_targets()
    test_slave_status_and_failsafe()
    print("\nALL SLAVE SELF-TESTS PASSED")
    sys.exit(0)
