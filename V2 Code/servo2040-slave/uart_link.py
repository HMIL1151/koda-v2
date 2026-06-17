"""Framed UART transport: a resync-safe byte parser plus a thin send/poll wrapper.

The parser is a byte-at-a-time state machine, so a dropped or corrupt byte costs at most
one frame — it never desyncs permanently. It's pure (no hardware), so it can be unit
-tested by feeding it bytes directly.
"""

import protocol


class FrameParser:
    WAIT_SOF, TYPE, LEN, PAYLOAD, CRC_LO, CRC_HI = range(6)

    def __init__(self):
        self.bad_frames = 0
        self.reset()

    def reset(self):
        self._state = self.WAIT_SOF
        self._type = 0
        self._len = 0
        self._idx = 0
        self._buf = bytearray()
        self._crc = 0

    def feed(self, byte):
        """Feed one byte. Returns (msg_type, payload_bytes) on a complete valid frame."""
        s = self._state
        if s == self.WAIT_SOF:
            if byte == protocol.SOF:
                self._state = self.TYPE
        elif s == self.TYPE:
            self._type = byte
            self._state = self.LEN
        elif s == self.LEN:
            self._len = byte
            self._idx = 0
            self._buf = bytearray(byte)
            self._state = self.CRC_LO if byte == 0 else self.PAYLOAD
        elif s == self.PAYLOAD:
            self._buf[self._idx] = byte
            self._idx += 1
            if self._idx >= self._len:
                self._state = self.CRC_LO
        elif s == self.CRC_LO:
            self._crc = byte
            self._state = self.CRC_HI
        elif s == self.CRC_HI:
            self._crc |= byte << 8
            self._state = self.WAIT_SOF
            body = bytes((self._type, self._len)) + bytes(self._buf)
            if protocol.crc16(body) == self._crc:
                return (self._type, bytes(self._buf))
            self.bad_frames += 1
        return None


class UartLink:
    """Wraps a machine.UART (or a mock) with framed send/poll."""

    def __init__(self, uart):
        self._uart = uart
        self._parser = FrameParser()

    @property
    def bad_frames(self):
        return self._parser.bad_frames

    def poll(self):
        """Read whatever's buffered and return a list of complete (type, payload) frames."""
        frames = []
        data = self._uart.read()           # non-blocking: bytes or None
        if data:
            for byte in data:
                result = self._parser.feed(byte)
                if result is not None:
                    frames.append(result)
        return frames

    def send(self, msg_type, payload=b""):
        self._uart.write(protocol.encode_frame(msg_type, payload))
