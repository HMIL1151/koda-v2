# Koda V2 — Architecture

V2 splits the robot's brain from its muscles. The old V1 firmware ran *everything*
on a single Pimoroni **Servo2040** (RP2040, MicroPython): gait maths, inverse
kinematics, pose sequencing and the raw servo PWM. That works, but the RP2040 is a
poor host for the new requirements — Bluetooth gamepad input, force-feedback maths
and a real balance controller all want more headroom and a proper real-time loop.

V2 therefore introduces an **ESP32 master** that does all the thinking, and demotes
the **Servo2040** to a "dumb but safe" actuator board.

```
        ┌──────────────┐   BLE/HID    ┌──────────────────────────┐   UART    ┌──────────────────────┐
        │  PS4 / DS4   │◄────────────►│      ESP32 (master)      │◄─────────►│  Servo2040 (slave)   │
        │  controller  │  Bluepad32   │  C++ / Arduino-ESP32     │  framed   │  MicroPython         │
        └──────────────┘              │                          │  packets  │                      │
                                      │  • PS4 input             │           │  • 12× servo PWM     │
        ┌──────────────┐  analog/I2C  │  • inverse kinematics    │           │  • per-servo cal     │
        │ hall-effect  │─────────────►│  • gait + bezier         │           │  • smoothing         │
        │ foot sensors │              │  • force feedback        │           │  • angle-limit clamp │
        └──────────────┘              │  • balance / COG         │           │  • link watchdog     │
                                      │  • early ground contact  │  status   │  • fail-safe pose    │
        ┌──────────────┐              │  • control state machine │◄─────────►│  • status / faults   │
        │     IMU      │─────────────►│                          │           └──────────────────────┘
        │  (optional)  │   I2C        └──────────────────────────┘                     │
        └──────────────┘                                                               ▼
                                                                              ┌──────────────────────┐
                                                                              │ 12 servos (4 legs ×  │
                                                                              │ hip + 2 knee drivers)│
                                                                              └──────────────────────┘
```

## Division of responsibility

| Concern                         | V1 (Servo2040)        | V2 master (ESP32)          | V2 slave (Servo2040)      |
| ------------------------------- | --------------------- | -------------------------- | ------------------------- |
| Operator input                  | analog pot / hardcode | **PS4 over BLE**           | —                         |
| Inverse kinematics              | ✔                     | **✔ (ported to C++)**      | —                         |
| Gait / bezier path generation   | ✔                     | **✔ (ported to C++)**      | —                         |
| Body pose → foot targets        | ✔                     | **✔**                      | —                         |
| Leg-frame → servo-angle mapping | ✔                     | **✔**                      | —                         |
| Force feedback (hall sensors)   | ✘                     | **✔ (new)**                | —                         |
| Early ground detection          | ✘                     | **✔ (new)**                | —                         |
| Balance / COG on incline        | ✘                     | **✔ (new)**                | —                         |
| Per-servo trim / calibration    | mixed into kinematics | —                          | **✔ (HAL)**               |
| Servo smoothing / interpolation | blocking `sleep`      | —                          | **✔ (non-blocking)**      |
| Angle-limit safety clamp        | ✘                     | —                          | **✔**                     |
| Link-loss fail-safe             | ✘                     | —                          | **✔ (watchdog)**          |
| Raw PWM output                  | ✔                     | —                          | **✔**                     |
| Status LEDs                     | ✔                     | command only               | **✔ (drive)**             |

### The clean boundary

The master thinks in **robot space** and emits **12 logical servo angles in degrees**
(one packet per control tick). The slave owns everything physical from that point on:
the angle→pulse calibration, per-servo trim, direction flips that depend on how a servo
is *mounted*, mechanical end-stops, and the fail-safe behaviour if packets stop arriving.

This means:

- The master never needs to know a servo was installed backwards or has a 3° offset —
  that lives in the slave's calibration table where you can tune it without touching
  any maths.
- The slave never needs to know what a "gait" or "leg" is — it just receives 12 numbers,
  clamps them, eases toward them, and writes PWM. If the master crashes or the cable is
  yanked, the slave notices the silence and relaxes to a safe pose.

> Note on V1's `kinematic_angles_to_servo_angles`: V1 folded *both* the geometric
> leg-convention sign flips **and** physical trims (e.g. `+10°` on the left-rear hip)
> into one function. In V2 these split: the geometric convention (which is a property of
> the robot model) stays on the master; the numeric trims become per-servo entries in the
> slave's calibration table.

## Why this split

- **PS4 BLE is solved on the ESP32.** Bluepad32 is the well-trodden, reliable path for
  DualShock/DualSense pairing — far less painful than BLE-HID host on the RP2040.
- **Real-time headroom.** Balance + force feedback want a steady control loop and FPU
  maths. The ESP32 has the clock and the FPU; the RP2040's job becomes trivial and
  deterministic.
- **Fault isolation.** The board physically wired to 12 servos is the one that enforces
  limits and fails safe. A bug in the gait maths can't drive a servo past its end-stop.
- **Maintainability.** The 25 KB monolithic `robot.py` becomes small, single-purpose
  modules with a state machine instead of copy-pasted `interpolate → loop → sleep`
  blocks.

## Module map (master, ESP32 / C++)

```
esp32-master/
├── platformio.ini                  PlatformIO project (Arduino-ESP32 + Bluepad32)
├── include/
│   └── config.h                    pins, robot geometry, gait + control tunables
└── src/
    ├── main.cpp                     setup() + fixed-rate control loop / scheduler
    ├── math/
    │   ├── vec.h                    Vec2 / Vec3 small-vector helpers (header-only)
    │   └── angle.h                  Angle (deg/rad), wrapping, lerp
    ├── kinematics/
    │   ├── inverse_kinematics.h
    │   └── inverse_kinematics.cpp   5-bar leg IK (ported from V1), circle intersection
    ├── gait/
    │   ├── bezier.h / bezier.cpp    swing bezier + stance line (ported from V1)
    │   └── gait.h / gait.cpp        CRAWL/TROT/GALLOP, phase offsets, foot trajectories
    ├── sensors/
    │   ├── adc.h/.cpp               ADC abstraction + ADS1115 I2C driver (8 hall channels)
    │   ├── hall_sensor.h/.cpp       inverse-cube model: calf compression → foot force (N)
    │   ├── hall_calibrator.h/.cpp   guided 2-point calibration (PS4/Serial), modal
    │   ├── hall_store.h/.cpp        save/load calibration as JSON on LittleFS
    │   └── imu.h/.cpp               optional IMU → torso pitch/roll (incline sensing)
    ├── control/
    │   ├── leg.h / leg.cpp          one leg: IK + leg-frame→servo-angle mapping
    │   ├── robot.h / robot.cpp      4 legs, pose, high-level commands, state machine
    │   ├── ground_contact.h/.cpp    early ground detection (compression spike in swing)
    │   └── balance.h / balance.cpp  static COG management on an incline
    ├── comms/
    │   ├── ps4_input.h/.cpp         Bluepad32 wrapper → normalised Command struct
    │   └── servo_link.h/.cpp        UART master: encode 12 angles, read status
    └── protocol/
        └── protocol.h               shared wire format (mirrors slave's protocol.py)
```

## Module map (slave, Servo2040 / MicroPython)

```
servo2040-slave/
├── main.py                 boot, construct objects, run the non-blocking loop
├── config.py              servo channel map, calibration table, limits, timings
├── protocol.py           frame encode/decode (mirrors master's protocol.h)
├── uart_link.py          framed UART RX + status TX, CRC, parse state machine
├── servo_controller.py   the HAL: 12 servos, calibration, smoothing, clamp
├── safety.py             watchdog + fail-safe pose on link loss
├── leds.py               status LED helper (refactor of V1 led.py)
└── mock_lib/             desktop mocks so the slave runs/tests off-hardware
```

## Control loop (master)

The master runs a single fixed-rate loop (default **50 Hz**, `config.h:CONTROL_HZ`):

```
every tick:
  1. PS4Input.poll()                       → Command {vx, vy, yaw, height, buttons, mode}
  2. HallSensors.read()                     → per-foot ground reaction force (N)
  3. GroundContact.update(forces, phase)    → per-leg "contact detected" flags
  4. State machine step (control/robot):
       SLEEP / WAKE / STAND / WALK / BALANCE / SIT …
       - WALK : Gait.foot_targets(phase) , shortened by early-contact flags
       - BALANCE : Balance.adjust(forces, imu) → body translation+rotation offsets
  5. For each leg: foot target → IK → leg-frame→servo angles → 12-angle vector
  6. ServoLink.send(angles)                 → UART frame to slave
  7. (optional) read slave status frame; surface faults on LEDs / log
```

Nothing in the loop blocks. The old V1 pattern of "build an interpolation array then
`for step: set; sleep(period)`" is gone — smoothing now happens continuously on the
slave between target updates, so the master just streams fresh targets every tick.

See `PROTOCOL.md` for the exact UART contract, and `README.md` for build/flash steps.
