# Koda V2 — ESP32 ⇄ Servo2040 UART protocol

A small, framed, binary protocol. The master (ESP32) streams **servo-target** frames at
the control rate; the slave (Servo2040) optionally answers with **status** frames. Both
sides implement the same frame format — see `esp32-master/src/protocol/protocol.h` and
`servo2040-slave/protocol.py`. Keep those two files in lockstep.

## Physical layer

| Setting   | Value                                          |
| --------- | ---------------------------------------------- |
| Transport | UART, full-duplex                              |
| Baud      | `115200` (`config.h` / `config.py`)            |
| Format    | 8-N-1                                          |
| Wiring    | ESP32 TX → 2040 RX, ESP32 RX → 2040 TX, common GND |
| Levels    | both 3.3 V logic — **no level shifter needed** |

> Pick UART pins that are free on both boards. On the Servo2040, UART0 (GP0/GP1) is used
> by the USB-serial REPL, so use **UART1 on GP16/GP17** for the link and keep the REPL for
> debugging. Defaults are in `config.py`.

## Frame format

Every frame, in both directions:

```
┌──────┬──────┬──────┬───────────────┬─────────┐
│ SOF  │ TYPE │ LEN  │   PAYLOAD     │  CRC16  │
│ 0xA5 │ 1 B  │ 1 B  │   LEN bytes   │  2 B LE │
└──────┴──────┴──────┴───────────────┴─────────┘
```

- **SOF** — start-of-frame sentinel `0xA5`. Resync point for the parser.
- **TYPE** — message type (see below).
- **LEN** — payload length in bytes (`0–255`).
- **PAYLOAD** — `LEN` bytes, type-specific.
- **CRC16** — CRC-16/CCITT-FALSE over `TYPE, LEN, PAYLOAD` (poly `0x1021`, init `0xFFFF`),
  little-endian. A frame with a bad CRC is dropped silently and the parser resyncs on the
  next `0xA5`.

The parser is a byte-at-a-time state machine (`WAIT_SOF → TYPE → LEN → PAYLOAD → CRC_LO →
CRC_HI`) so a dropped byte costs at most one frame, never desyncs permanently.

## Message types

| TYPE   | Name             | Dir            | Payload                                   |
| ------ | ---------------- | -------------- | ----------------------------------------- |
| `0x01` | `SERVO_TARGETS`  | master → slave | 12 × `int16` centi-degrees (LE) = 24 B    |
| `0x02` | `SET_MODE`       | master → slave | 1 B mode enum                             |
| `0x03` | `LED`            | master → slave | 4 B: `idx, r, g, b`                        |
| `0x04` | `PING`           | master → slave | 0 B (heartbeat; resets slave watchdog)    |
| `0x81` | `STATUS`         | slave → master | see below                                 |
| `0x82` | `FAULT`          | slave → master | 1 B fault code                            |
| `0x83` | `PONG`           | slave → master | 0 B                                       |

### `0x01 SERVO_TARGETS` (master → slave)

The core message, sent every control tick. Payload is **12 signed 16-bit** values, little
-endian, each a servo angle in **centi-degrees** (degrees × 100). Centi-degrees give
0.01° resolution over a ±327° range in a compact 2 bytes — plenty for hobby servos and
avoids floats on the wire.

Channel order is fixed (`config.h:ServoChannel` mirrors `config.py`):

```
 0 FL_HIP   1 FL_KNEE_L   2 FL_KNEE_R
 3 FR_HIP   4 FR_KNEE_L   5 FR_KNEE_R
 6 RR_HIP   7 RR_KNEE_L   8 RR_KNEE_R
 9 RL_HIP  10 RL_KNEE_L  11 RL_KNEE_R
```

A value of `INT16_MIN` (`-32768`) is the **"hold / don't drive"** sentinel for a channel,
used e.g. when a leg is deliberately relaxed.

### `0x02 SET_MODE` (master → slave)

| Mode | Name      | Slave behaviour                                              |
| ---- | --------- | ----------------------------------------------------------- |
| `0`  | `RELAX`   | disable all servos (torque off)                             |
| `1`  | `ACTIVE`  | normal: track targets with smoothing + clamp                |
| `2`  | `HOLD`    | freeze at current positions, ignore new targets             |
| `3`  | `SAFE`    | ease to the configured safe/crouch pose, then hold          |

### `0x81 STATUS` (slave → master)

Sent at a lower rate (default 10 Hz) and on demand. Payload:

```
byte 0      : mode (current slave mode enum)
byte 1      : flags  bit0 link_ok, bit1 clamped_any, bit2 moving, bit3 fault
bytes 2..3  : loop_dt_us  (uint16, slave loop time, microseconds)
bytes 4..15 : per-servo clamp flags packed (12 bits → 2 bytes) + reserved
```

### `0x82 FAULT` (slave → master)

| Code | Meaning                                  |
| ---- | ---------------------------------------- |
| `1`  | link timeout (no valid frame in window)  |
| `2`  | malformed frames over threshold          |
| `3`  | target out of range on N channels        |
| `4`  | servo driver / hardware error            |

## Timing & fail-safe

- Master sends `SERVO_TARGETS` every control tick (50 Hz). `PING` is only needed if the
  gait pauses — any valid frame feeds the slave watchdog.
- The slave runs a **watchdog**: if no valid frame arrives within `LINK_TIMEOUT_MS`
  (default 200 ms) it raises `FAULT(1)`, switches itself to `SAFE`, eases to the crouch
  pose, and disables torque after settling. It re-arms automatically when frames resume.
- The slave **never** trusts a target blindly: every value is clamped to that servo's
  calibrated `[min,max]` before output, and out-of-range targets raise `FAULT(3)` so the
  master learns its maths produced something unreachable.

## Versioning

`protocol.h` / `protocol.py` both export `PROTOCOL_VERSION`. On boot the master sends a
`SET_MODE` and the slave reports its version in the first `STATUS`; a mismatch is logged
loudly. Bump the version whenever the frame layout or any payload changes.
