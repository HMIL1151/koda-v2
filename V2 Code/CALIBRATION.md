# Hall-sensor force calibration

Each foot has two compliant calf springs, each watched by a hall-effect sensor. The
sensor signal follows an **inverse-cube** magnetic law (ported from your bench sketch):

```
signal = K / (distance + offset)^3
```

`K` is per-sensor and found by a quick two-point calibration; the rest of the constants
(`offset`, spring length, full-load force) live in
[esp32-master/include/config.h](esp32-master/include/config.h) under the `HALL_*` block.

You calibrate **once**; the result is saved to `/hall_cal.json` on the ESP32's flash and
reloaded automatically on every boot. No re-calibration after a power cycle.

## Hardware

8 sensors (2 calves × 4 feet) read through **two ADS1115 I2C ADCs** — not the ESP32's
own ADC, because ADC2 is unusable while Bluetooth is active and ADC1 hasn't enough free
channels.

| Sensor index | Foot | ADS1115 | Channel |
| ------------ | ---- | ------- | ------- |
| 0, 1         | FL   | 0x48    | 0, 1    |
| 2, 3         | FR   | 0x48    | 2, 3    |
| 4, 5         | RR   | 0x49    | 0, 1    |
| 6, 7         | RL   | 0x49    | 2, 3    |

I2C on `SDA=GPIO21`, `SCL=GPIO22` (configurable in `config.h`). Per-foot force = sum of
that foot's two sensors.

## How the maths works (and why it's robust)

Calibration captures two raw signals per sensor:

- **`s0`** — unloaded (spring at `ZERO_LOAD_DISTANCE_MM`, 17 mm)
- **`s1`** — fully compressed (effective magnetic distance `FULL_LOAD_DISTANCE_MM`, 1 mm)

then solves `K = (s1 − s0) / (1/d1³ − 1/d0³)`. Live, it inverts the cube to recover spring
distance, and converts distance → force with the spring constant derived from the known
full-load force (27.05 N over 16 mm ≈ 1.69 N/mm).

By construction this maps **`s0` → 0 N** and **`s1` → 27.05 N** for *any* `K`, rising- or
falling-signal sensors alike — verified in
[esp32-master/test/hall_model_crosscheck.py](esp32-master/test/hall_model_crosscheck.py).

## Running a calibration

Calibration is **modal**: while it runs the legs are relaxed (torque off) so you can move
them by hand, and normal control is suspended. Start it either way:

- **From the PS4 pad:** press **L1 + R1** together.
- **From the Serial monitor (bench, USB):** type **`c`** and press Enter.

Then follow the two prompts. Advance each step with **Cross (✕)** on the pad *or* **Enter**
on Serial. **Circle (○)** cancels and keeps the old calibration.

```
================ HALL CALIBRATION ================
Step 1/2: unload ALL springs (lift the robot), then press X (or Enter).
   → captures s0 for all 8 sensors, prints them
Step 2/2: compress ALL feet FULLY (bottom the springs), then press X (or Enter).
   → captures s1, solves K per sensor, prints them, saves /hall_cal.json
==================================================
```

That's it — power-cycle and the calibration loads itself. On boot the master logs either
`Hall calibration loaded from flash` or a prompt to calibrate if no valid file is found.

## Inspecting / editing the saved file

It's plain JSON, so you can pull it off the board (e.g. `pio run -t uploadfs` works the
other way; to read, use an esptool/LittleFS dump tool) and eyeball or tweak the `K` values:

```json
{
  "version": 1,
  "sensors": [
    { "s0": 5012.3, "s1": 19880.1, "K": 3.31e6, "cal": true },
    ...
  ]
}
```

## Tuning notes

- **`HALL_MAGNET_OFFSET_MM`** (default 5 mm) is the one experimental fudge in the model —
  it compensates for finite magnet size / Hall-IC depth. If forces read non-linear vs
  known weights, sweep this.
- **`HALL_FULL_LOAD_FORCE_N`** must match the actual spring force at full compression for
  the newton scale to be correct.
- Storage swap: if you'd rather not carry a filesystem, `hall_store.cpp` is the only file
  that touches LittleFS/JSON — it can be reimplemented over NVS `Preferences` without
  touching the model or calibrator.
