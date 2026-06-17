"""Mock of machine.UART for desktop testing.

`read()` returns whatever has been pushed into the RX buffer via `feed_rx()` (so a test
can inject framed bytes), and `write()` captures TX bytes for inspection.
"""


class UART:
    def __init__(self, *args, **kwargs):
        self._rx = bytearray()
        self.tx = bytearray()

    # — test hooks —
    def feed_rx(self, data):
        self._rx.extend(data)

    # — machine.UART API —
    def read(self, n=None):
        if not self._rx:
            return None
        if n is None or n >= len(self._rx):
            data = bytes(self._rx)
            self._rx = bytearray()
        else:
            data = bytes(self._rx[:n])
            self._rx = self._rx[n:]
        return data

    def write(self, data):
        self.tx.extend(data)
        return len(data)

    def any(self):
        return len(self._rx)
