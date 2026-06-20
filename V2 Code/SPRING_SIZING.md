# Calf-spring rate & travel sizing

The calf springs are the robot's force sense. Their **rate `k`** (N/mm) and **travel** have
to satisfy two jobs at once, and those jobs pull `k` in opposite directions:

| Job | Wants | Bound |
| --- | ----- | ----- |
| Detect **slope** when standing (4 feet down) | **soft** springs (big compression per N) | `k ≤ k_max` |
| Detect **early/late contact** when walking | **stiff** springs (force rises fast on touch) | `k ≥ k_min` |

A workable design needs `k_min < k_max`. The recommended `k` is their geometric mean.

The maths lives in [sim/web/spring_sizing.js](sim/web/spring_sizing.js); the calculator is
[sim/tools/spring_sizing.mjs](sim/tools/spring_sizing.mjs); the sim's **Spring advisor**
panel runs the same maths live on the current settings.

## Derivation

Let `W` = robot weight (N), `h` = COG height above the feet (mm), `L` = foot fore/aft
separation (mm), `σ` = the smallest calf compression the hall sensor can resolve (mm),
`snr` = safety factor over that, `dt` = control tick (s).

### Slope resolution → `k_max`

Standing on a slope `θ` with the body settled parallel to the ground (symmetric stance, COG
**not** yet shifted to balance), the COG projects downhill and shifts the front/rear load by

```
ΔF_total = 2·W·h·tan θ / L
```

The compression difference between a front and a rear foot spring is

```
ΔX = ΔF_total / (2k) = W·h·tan θ / (L·k)
```

To resolve the smallest slope `θ_min`, that difference must clear the sensor floor
(`ΔX ≥ snr·σ`):

```
k_max = W·h·tan θ_min / (L · snr · σ)
```

Softer springs (smaller `k`) give a bigger `ΔX` per degree → finer slope sensing.
This is exactly the signal `SlopeEstimator` reads from the unbalanced reference stance.

### Contact detection → `k_min`

A foot descending at `v` (mm/s) compresses its spring by `v·T·dt` over `T` ticks, building
force `k·v·T·dt`. To register a contact (force ≥ `F_thresh`) within `T_detect` ticks:

```
k_min = F_thresh / (v · T_detect · dt)
```

Stiffer springs make the force rise faster → quicker, crisper early/late contact events
(which is all the sensors are used for while walking).

### Travel

Travel only has to survive **slope sensing** — standing with **all feet down** on the
steepest slope. The worst (downhill) foot carries the even share `W/feet` plus the slope
redistribution, so

```
F_worst = (W/feet)·(1 + 2·h·tan θ_max / L)      travel ≥ F_worst / k
```

Walking on two feet loads a foot more (≈ `W/2`), **but that doesn't constrain the travel**:
force isn't measured while walking, and a transient bottom-out is harmless — early/late
contact still fires from the force *rising* past the threshold, even at the stop. (The only
thing lost at bottom-out is force *magnitude*, which walking doesn't use.)

## Using it

```bash
cd "V2 Code/sim"
node tools/spring_sizing.mjs
# override any input:
node tools/spring_sizing.mjs sensorResolutionMm=0.02 slopeMinDeg=2 mass=4
```

Example (Koda V2 defaults — geometry from `config.h`, sensor/gait estimated):

```
weight: 29.4 N   worst foot load (static 20° slope sensing): 11.7 N
spring-rate window:  k_min 0.67  <  k  <  k_max 8.37  N/mm
recommended k: 2.36 N/mm
  → resolves slopes down to ~0.8°
  → detects contact in ~0.6 ticks
  → needs ~5.0 mm travel (fits 16 mm ✓)
```

The sim's current `HALL_SPRING_N_PER_MM` (≈1.69) sits inside that window, so the modelled
spring can do both jobs. If you make the **sensor coarse** (e.g. `sensorResolutionMm=0.7`)
the window collapses — the tool prints **NO WINDOW** and tells you the sensor can't both
resolve slopes and detect contact, which is itself a useful design verdict.

## The two inputs worth pinning down on the real robot

- **`sensorResolutionMm`** — the dominant unknown. Measure it: hold a foot still, log the
  hall reading, and convert the count noise to mm of compression via the calibrated
  `signal → distance` curve ([CALIBRATION.md](CALIBRATION.md)). This sets `k_max`.
- **`descentSpeedMmS`** — the foot's vertical speed as it lands (roughly step height ×
  cadence). This sets `k_min`. The sim can report it from a trace.
