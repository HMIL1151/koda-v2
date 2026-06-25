# Koda V2

The robot's domain language. Koda V2 is a quadruped whose feet sense ground-reaction
force through springs in the legs; that force sense drives slope estimation, balance, and
early/late ground-contact detection.

## Language

**Calf spring**:
An inline compression spring forming part of a calf member. Each foot has two calves (the
two output links of the leg's sagittal 5-bar), so two calf springs per foot; each compresses
along its own calf axis and is read by one hall sensor.
_Avoid_: foot spring, shock, damper.

**Foot force**:
The per-foot *vertical* ground-reaction force, reconstructed from that foot's two calf
springs as `Σ kᵢ·cᵢ·cosθᵢ` (cᵢ = axial compression, θᵢ = calf angle from vertical, from IK).
NOT the raw sum of the two axial spring readings — that overstates the vertical force by
~1/cosθ and biases the slope estimate. This vertical reconstruction is what the control core
consumes and what the sim and sizing tool model.
_Avoid_: load (ambiguous), weight, axial-sum force.

**Slope sensing** (static):
Estimating ground slope while standing on all four feet in the symmetric reference stance
(COG not yet shifted), from the front/rear and left/right imbalance in foot force. Wants
soft springs. Distinct from walking-time terrain sensing.
_Avoid_: slope detection (that's the event-based walking-time path).

**Contact detection** (early/late):
Firing a discrete event when a swinging foot loads early (ground is high) or a stance foot
stays unloaded (ground dropped). Uses only the *rising edge* of force past a threshold, not
its magnitude. Wants stiff springs.

**Preload**:
The install-time pre-compression of a calf spring, given as a length (mm) and held by a
mechanical stop. Creates a *dead-band*: foot loads below `preloadMm·k_spring` don't move the
spring and are invisible to the sensor. The resting reading is tared, so `foot force` means
ground load *above* preload, with a hard floor at zero.
_Avoid_: pretension, prestress.

**Spring-rate window**:
The band `k_min < k < k_max` in which a single spring rate can serve both slope sensing
(upper bound, soft) and contact detection (lower bound, stiff). Recommended `k` is the
geometric mean. If the window collapses, the sensor can't do both jobs.
