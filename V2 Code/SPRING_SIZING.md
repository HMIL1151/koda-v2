# Calf-spring rate & travel sizing

The calf springs are the robot's force sense. The sizing tool picks the **physical spring rate
`kSpring`** (N/mm, of one spring inside one calf — the thing you actually order) and checks the
**travel**. Two jobs pull `kSpring` in opposite directions:

| Job | Wants | Bound |
| --- | ----- | ----- |
| Detect **slope** when standing (4 feet down) | **soft** springs (big compression per N) | `kSpring ≤ k_max` |
| Detect **early/late contact** when walking | **stiff** springs (force rises fast on touch) | `kSpring ≥ k_min` |

A workable design needs `k_min < k_max`. The recommended `kSpring` is their geometric mean —
**except when friction caps slope sensing** (see below), where stiffness buys no slope benefit and
the recommendation drops to the soft end (~`2·k_min`) to maximise visible compression instead.

The maths lives in [sim/web/spring_sizing.js](sim/web/spring_sizing.js); the calculator is
[sim/tools/spring_sizing.mjs](sim/tools/spring_sizing.mjs); the sim's **Spring advisor** panel
runs the same maths live on the current settings. The decision to key everything off `kSpring`
plus the leg geometry (rather than a lumped per-foot rate) is recorded in
[docs/adr/0001](../docs/adr/0001-spring-model-from-spring-rate-and-linkage.md).

## The leg geometry matters (`kSpring` ≠ per-foot rate)

Each foot has **two calf springs** — the two output links of the leg's sagittal 5-bar — each
compressing along its own calf axis at angle `θ` from vertical (`θ ≈ 24°` at the standing pose,
straight from the IK at the `ZERO` target). So a vertical foot load splits between the two and
each spring sees only its axial share. The per-foot **vertical** rate is

```
k_foot = kSpring · S ,   S = cos²θ₁ + cos²θ₂  (= 2cos²θ ≈ 1.67 symmetric)
```

`foot force` is the **vertical reconstruction** `Σ kSpring·cᵢ·cosθᵢ`, *not* the raw sum of the
two axial readings (which would overstate it by ~1/cosθ). Travel and the sensor operating point
are checked **per spring, axially** — the physical 16 mm limit is on one spring, not the foot.

## Derivation

Let `W` = robot weight (N), `h` = COG height above the feet (mm), `L` = foot fore/aft
separation (mm), `θ` = standing calf angle, `snr` = safety factor over the sensor noise floor,
`dt` = control tick (s).

### Slope resolution → `k_max`

Standing on a slope `θ_s` with the body settled parallel to the ground (symmetric reference
stance, COG **not** yet shifted), a front foot's vertical load exceeds a rear foot's by

```
ΔF_foot = W·h·tan θ_s / L
```

(the `tan` — not `sin` — is correct precisely because `foot force` is the *vertical*
reconstruction). That maps to a per-spring axial **compression** difference

```
Δc = ΔF_foot / (2·cosθ·kSpring)
```

To resolve the smallest slope `θ_min`, `Δc` must clear the sensor floor (`Δc ≥ snr·σ`):

```
k_max = W·h·tan θ_min / (L · 2·cosθ · snr · σ)
```

Softer springs (smaller `kSpring`) give a bigger `Δc` per degree → finer slope sensing.

### Sensor resolution `σ` is not a constant

The control system reads a **calibrated distance**; the calibration curve is
`signal = a/(d+b)³ + c`, so a fixed ADC noise floor maps to a strongly distance-dependent
resolution:

```
σ(d) = adcNoiseCounts · (d + b)⁴ / (3a)
```

The sensor is far **finer** at high compression (small `d`) than near the unloaded end. `σ` is
evaluated at the **standing slope-sensing operating distance** `d = freeLen − c_standing`, where
`c_standing = (W/feet)/(2·cosθ·kSpring)` is the resting per-spring compression. Because `d`
depends on `kSpring`, this is a self-coupled loop (a softer spring sits deeper, where the sensor
is finer, which fights the softness) — the tool solves it by damped iteration. `adcNoiseCounts`
is the dominant unknown; **measure it on the bench** (it defaults to an estimate of 3 counts).

### Contact detection → `k_min`

A foot descending at `v` (mm/s) builds vertical force at `k_foot·v`. With a preload dead-band
`F_pre = preloadMm·kSpring` (load below it doesn't move the spring and is invisible — the resting
reading is tared), the tared force after `T_detect` ticks is `k_foot·v·T·dt − F_pre`. Requiring
that to reach the contact threshold `F_thresh`:

```
k_min = F_thresh / (S·v·T_detect·dt − preloadMm)
```

Stiffer springs make the force rise faster → quicker contact. **Preload eats the margin**: too
much and the denominator vanishes (no contact possible).

### Travel

Travel only has to survive **slope sensing** — standing with **all feet down** on the steepest
slope. The worst (downhill) foot carries `F_worst = (W/feet)·(1 + 2·h·tanθ_max/L)`; its
**per-spring axial** compression must fit the travel:

```
per-spring travel needed = F_worst / (2·cosθ·kSpring)   ≤  springTravelMm (16 mm)
```

Walking on two feet loads a foot more (≈ `W/2`) and **may bottom out — that's allowed**: force
isn't measured while walking and early/late contact still fires from the force *rising* past the
threshold. (Bottom-out does shorten the leg and stress the spring, so it's a walking-regime cost,
not a free lunch — but it doesn't constrain the spec.)

### Preload dead-band

For slope sensing to see anything, the standing per-spring load `(W/feet)/(2cosθ)` must exceed
the dead-band `preloadMm·kSpring`. The tool flags **DEAD-BAND TOO HIGH** when it doesn't.

### Friction / stiction (`frictionN`)

Real linkages have a breakaway force: a calf spring won't move until the load *change* beats the
stiction in its guide/joints. It acts as a force dead-band alongside preload, and it's the usual
reason a bench robot "barely compresses" even loaded. Two effects the frictionless maths misses:

- **Visible vs gross standing compression.** The tool reports both. The *visible* compression is
  `max(0, standingLoad − preload·k − friction)/k`; raise `frictionN` until it matches what you
  actually see on the bench — that calibrates your stiction without any instrument.
- **A spring-rate-independent slope floor.** The front/rear force difference (sub-1 N per spring
  on gentle slopes) must beat stiction, and **a softer spring does not help** — the signal and
  the friction are both forces. When `frictionN` dominates the sensor floor, the tool reports the
  resolvable slope as **friction-limited** and prints **FRICTION-LIMITED**: the fix is mechanical
  (reduce binding / lubricate / align the guide), not a different spring.

`frictionN` also raises `k_min` (the foot must overcome both springs' stiction before a contact
registers). The sim's **Spring advisor** has a live Friction knob that feeds the same maths and
dead-bands the forces the control core sees, so you can watch slope sensing degrade.

## Using it

```bash
cd "V2 Code/sim"
node tools/spring_sizing.mjs
# override any input:
node tools/spring_sizing.mjs adcNoiseCounts=5 slopeMinDeg=2 mass=4 preloadMm=2
```

Example (Koda V2 defaults — geometry from `config.h` + the IK; sensor/gait estimated):

```
weight: 29.4 N   worst foot load (static 20° slope sensing): 11.7 N
calf angle 24.0° → S=1.669  (k_foot = S·kSpring)
sensor σ at standing op. point (d=15.0mm): 0.0244 mm  [3 ADC counts — *** estimate, measure on bench]
spring-rate window (kSpring):  k_min 0.44  <  k  <  k_max 9.40  N/mm
recommended kSpring: 2.04 N/mm   (k_foot 3.41 N/mm)
  → resolves slopes down to ~0.7°
  → detects contact in ~0.6 ticks
  → needs ~3.1 mm per-spring axial travel (fits 16 mm ✓)
```

The sim's current `HALL_SPRING_N_PER_MM` (≈1.69) sits inside that window, so the modelled spring
can do both jobs. If you make the **ADC noisy** (e.g. `adcNoiseCounts=300`) or the **preload
high**, the window collapses — the tool prints **NO WINDOW**, which is itself a useful verdict.

## The inputs worth pinning down on the real robot

- **`adcNoiseCounts`** — the dominant unknown, and it sets `k_max`. Measure it: hold a foot under
  the standing load, log the calibrated-distance reading, and take the count spread (RMS).
- **`calA`, `calB`** — from your calibration fit (`signal = a/(d+b)³ + c`). `b` is the magnetic
  offset; `a` the magnetic constant. (`c` cancels in the resolution derivative.)
- **`preloadMm`** — the install pre-compression. Sets the dead-band; tune to your assembly.
- **`frictionN`** — per-spring stiction breakaway. You don't have to measure it: raise it until
  the tool's *visible* standing compression matches the bench. Often the real slope-sensing limit.
- **`descentSpeedMmS`** — the foot's vertical speed as it lands (≈ step height × cadence). Sets
  `k_min`. The sim can report it from a trace.
- **`calfAngleDeg`** — already pinned at 24° from the IK; only revisit if the standing pose or
  leg geometry changes.
