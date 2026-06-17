"""Servo2040 slave — entry point and non-blocking loop.

The slave is deliberately dumb: receive logical servo targets from the ESP32 master,
ease the servos toward them with a speed limit and end-stop clamp, and fail safe if the
master goes quiet. It knows nothing about gaits, legs or kinematics.

Loop, each iteration (~LOOP_HZ):
  drain UART frames → act on them (and feed the watchdog) →
  check the watchdog → smooth servos toward targets → emit periodic STATUS.
"""

import struct

import config
import protocol
from compat import ticks_ms, ticks_us, ticks_diff, sleep_ms
from leds import StatusLeds
from safety import Watchdog
from servo_controller import ServoController
from uart_link import UartLink


def _make_uart():
    if config.EMULATION_MODE:
        from mock_lib.uart import UART
        return UART()
    from machine import UART, Pin
    return UART(config.SLAVE_UART_ID, baudrate=config.SLAVE_BAUD,
                tx=Pin(config.SLAVE_TX_PIN), rx=Pin(config.SLAVE_RX_PIN))


class Slave:
    def __init__(self):
        self.link = UartLink(_make_uart())
        self.servos = ServoController()
        self.watchdog = Watchdog()
        self.leds = StatusLeds()
        self.mode = None
        self._last_status_ms = ticks_ms()
        self._last_loop_us = ticks_us()
        # Start in SAFE: torque on, easing to the crouch pose, until the master speaks.
        self.servos.set_pose(config.SAFE_POSE_DEG)
        self._apply_mode(protocol.SAFE)

    # ── Mode handling ────────────────────────────────────────────────────────────────
    def _apply_mode(self, mode):
        if mode == self.mode:
            return
        self.mode = mode
        if mode == protocol.RELAX:
            self.servos.disable()
        elif mode in (protocol.ACTIVE, protocol.HOLD):
            if not self.servos.enabled:
                self.servos.enable()
        elif mode == protocol.SAFE:
            if not self.servos.enabled:
                self.servos.enable()
            self.servos.set_pose(config.SAFE_POSE_DEG)
        self.leds.show_mode(mode)

    # ── Inbound frame dispatch ───────────────────────────────────────────────────────
    def _handle(self, msg_type, payload):
        if msg_type == protocol.SERVO_TARGETS:
            angles = protocol.decode_servo_targets(payload)
            if angles is None:
                self.link.send(protocol.FAULT, bytes((protocol.FAULT_MALFORMED,)))
                return
            if self.mode == protocol.ACTIVE:
                self.servos.set_targets(angles)
        elif msg_type == protocol.SET_MODE:
            if payload:
                self._apply_mode(payload[0])
        elif msg_type == protocol.LED:
            if len(payload) == 4:
                self.leds.set(payload[0], payload[1], payload[2], payload[3])
        elif msg_type == protocol.PING:
            self.link.send(protocol.PONG)

    # ── Status reporting ─────────────────────────────────────────────────────────────
    def _send_status(self, loop_dt_us):
        flags = 0
        if not self.watchdog.expired():
            flags |= 0x01           # link_ok
        if self.servos.any_clamped:
            flags |= 0x02           # clamped_any
        if self.servos.enabled:
            flags |= 0x04           # moving / active
        dt_us = min(0xFFFF, max(0, int(loop_dt_us)))
        payload = struct.pack("<BBH", self.mode, flags, dt_us) + bytes(12)
        self.link.send(protocol.STATUS, payload)

    # ── Main loop ────────────────────────────────────────────────────────────────────
    def run(self):
        loop_period_ms = max(1, int(1000 // config.LOOP_HZ))
        while True:
            for msg_type, payload in self.link.poll():
                self.watchdog.feed()
                self._handle(msg_type, payload)

            if self.watchdog.trip_once():
                self.link.send(protocol.FAULT, bytes((protocol.FAULT_LINK_TIMEOUT,)))
                self._apply_mode(protocol.SAFE)

            now = ticks_us()
            dt = ticks_diff(now, self._last_loop_us)
            self._last_loop_us = now
            self.servos.update(dt / 1_000_000.0)

            if ticks_diff(ticks_ms(), self._last_status_ms) >= config.STATUS_PERIOD_MS:
                self._last_status_ms = ticks_ms()
                self._send_status(dt)

            sleep_ms(loop_period_ms)


def main():
    print("Koda V2 slave starting")
    Slave().run()


if __name__ == "__main__":
    main()
