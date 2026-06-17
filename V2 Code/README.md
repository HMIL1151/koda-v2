# Koda V2

Quadruped robot firmware, V2. A two-board design: an **ESP32 master** does all the
control (PS4 input, kinematics, gait, force feedback, balance) and a **Servo2040 slave**
drives the 12 servos as a calibrated, fail-safe actuator board.

Start with **[ARCHITECTURE.md](ARCHITECTURE.md)** for the design and the V1→V2 mapping,
**[PROTOCOL.md](PROTOCOL.md)** for the UART contract between the boards, and
**[CALIBRATION.md](CALIBRATION.md)** for hall-sensor force calibration.

```
V2 Code/
├── ARCHITECTURE.md        design + responsibility split + V1→V2 mapping
├── PROTOCOL.md            ESP32 ⇄ Servo2040 UART wire format
├── esp32-master/          C++ / Arduino-ESP32 + Bluepad32  (the brain)
└── servo2040-slave/       MicroPython  (the muscles)
```

## What's new vs V1

- **PS4 control over Bluetooth** (Bluepad32) instead of analog pots / hardcoded sequences.
- **Force feedback** from hall-effect sensors reading calf-spring compression → per-foot
  ground reaction force, with an **inverse-cube magnetic model** and an on-robot
  **calibration mode** (PS4 L1+R1, or `c` over Serial) whose result is saved to JSON on
  flash and reloaded every boot — see [CALIBRATION.md](CALIBRATION.md).
- **Early ground detection** — a foot that meets the ground mid-swing (an incline or
  obstacle) is detected from a force spike and the step is ended early.
- **Static balance on inclines** — the body shifts its COG over the support polygon and
  matches the torso to the slope using the per-foot forces (and optional IMU).
- A real **control loop + state machine** replacing V1's blocking pose sequences, and the
  monolithic `robot.py` broken into small single-purpose modules.

## Hardware

| Part            | Role                                                        |
| --------------- | ---------------------------------------------------------- |
| ESP32 dev board | master controller (BLE, maths, control loop)               |
| Pimoroni Servo2040 | slave servo driver (12 servos: 4 legs × hip + 2 knee)   |
| PS4 / DS4 controller | operator input over BLE                               |
| Hall-effect sensors | one per foot, read calf-spring compression (force)     |
| IMU (optional)  | torso pitch/roll for incline sensing                       |

Wiring for the UART link is in **[PROTOCOL.md](PROTOCOL.md)**; pins, geometry and tunables
live in `esp32-master/include/config.h` and `servo2040-slave/config.py`.

## Build & flash — master (ESP32)

Uses [PlatformIO](https://platformio.org/):

```bash
cd "V2 Code/esp32-master"
pio run                 # build
pio run -t upload       # flash
pio device monitor      # serial logs
```

Bluepad32 + Arduino-ESP32 are pulled in via `platformio.ini`. First run: put the DS4 into
pairing mode (Share + PS until the light bar flashes); it binds automatically.

## Deploy — slave (Servo2040)

The Servo2040 runs MicroPython (Pimoroni build). Copy the contents of
`servo2040-slave/` to the board (e.g. with [`mpremote`](https://docs.micropython.org/en/latest/reference/mpremote.html)
or Thonny):

```bash
cd "V2 Code/servo2040-slave"
mpremote connect auto fs cp -r . :
mpremote connect auto run main.py     # or reset to autostart main.py
```

### Run the slave off-hardware

`servo2040-slave/mock_lib/` mocks the Pimoroni `servo`/LED modules so the slave logic
(protocol parsing, smoothing, watchdog) can be exercised on a PC. Set
`config.EMULATION_MODE = True` and run `main.py` with CPython.
