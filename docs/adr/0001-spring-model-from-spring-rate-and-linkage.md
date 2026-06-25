# Spring sizing keys off the physical spring rate and the leg linkage, not a lumped per-foot rate

The calf-spring sizing tool and the sim originally modelled each foot as a single vertical
penetration spring with a hand-set per-foot rate (`HALL_SPRING_N_PER_MM ≈ 1.69`, used both as
the per-spring *and* per-foot rate). That conflated two different quantities and dropped the
leg geometry entirely.

We decided the **only** spring rate worked with is `k_spring` — the rate of one physical
spring inside one calf, the thing actually ordered. Everything per-foot is **derived** from
`k_spring` plus the two calf angles (`θ₁, θ₂`) from the IK at the relevant pose:

- per-foot vertical rate `k_foot = k_spring·(cos²θ₁ + cos²θ₂)`;
- `foot force` = the vertical reconstruction `Σ kᵢ·cᵢ·cosθᵢ`, not the raw axial sum;
- travel and the sensor-resolution operating point are checked **per spring, axially**, not
  on a lumped per-foot spring.

Why it's worth recording: a future reader sees the indirection (why not just set a per-foot
rate?) and might "simplify" it back to a lumped vertical spring — which silently re-introduces
a ~40% softness error, overstates slope resolution, and understates travel. The trade-off was
fidelity vs. simplicity; we chose fidelity because the tool's whole purpose is to pick a real
spring, and the linkage projection is first-order for that.

Sensor resolution `σ` is likewise derived (from the calibration curve + ADC noise counts at
the operating distance), not a flat mm constant — see SPRING_SIZING.md.
