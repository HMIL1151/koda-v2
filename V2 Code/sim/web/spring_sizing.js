// Calf-spring rate / travel sizing. DOM-free so it's shared by the node calculator
// (tools/spring_sizing.mjs) and the in-sim advisor panel.
//
// Two competing requirements bound the usable spring rate k (N/mm):
//
//  • SLOPE RESOLUTION (upper bound).  Standing on a slope θ, the COG (height h) projects
//    downhill, shifting the front/rear load by  ΔF_total = 2·W·h·tan θ / L  (L = foot
//    fore/aft separation). The compression difference between a front and a rear foot is
//    ΔX = ΔF_total / (2k) = W·h·tan θ / (L·k). To resolve the smallest slope θ_min the
//    sensor must see it:  ΔX ≥ snr·σ  (σ = sensor resolution in mm) →
//        k_max = W·h·tan θ_min / (L · snr · σ).
//    Softer springs (smaller k) → bigger ΔX → finer slope sensing.
//
//  • CONTACT DETECTION (lower bound).  A foot descending at v must build the contact
//    threshold force within T_detect control ticks:  k·(v·T_detect·dt) ≥ F_thresh →
//        k_min = F_thresh / (v · T_detect · dt).
//    Stiffer springs (bigger k) → force rises faster → quicker contact detection.
//
//  • TRAVEL.  The spring must not bottom out while SLOPE SENSING — i.e. standing with all
//    feet down on the steepest slope. The worst (downhill) foot then carries the even share
//    W/feet plus the slope redistribution: F = (W/feet)·(1 + 2·h·tanθ_max/L). Walking 2-foot
//    support loads a foot more, but we don't sense force while walking and a transient
//    bottom-out is harmless (early/late contact still fires from the force *rising*).
//
// A usable design needs k_min < k_max; the recommended k is their geometric mean.

const DEG = Math.PI / 180;

export function springWindow(p) {
  const g = p.g ?? 9.81;
  const W = p.mass * g;                                   // total weight, N
  const h = p.cogHeight;                                  // COG height above feet, mm
  const L = p.footSeparation;                             // foot fore/aft separation, mm
  const sigma = p.sensorResolutionMm;                     // resolvable compression, mm
  const snr = p.snr ?? 3;                                 // safety factor over the noise floor
  const thetaMin = (p.slopeMinDeg ?? 3) * DEG;            // smallest slope to resolve
  const Fthresh = p.contactForceN;                        // contact-detect force, N
  const v = p.descentSpeedMmS;                            // foot descent speed near landing, mm/s
  const Tdetect = p.detectTicks ?? 2;                     // ticks allowed to detect a contact
  const dt = p.dtS ?? 0.02;                               // control tick, s
  const feet = p.feet ?? 4;                               // feet on the ground while slope sensing
  const slopeMax = (p.slopeMaxDeg ?? 20) * DEG;           // steepest slope we sense on
  const travelAvail = p.springTravelMm;                   // physical spring travel, mm

  const kMax = (W * h * Math.tan(thetaMin)) / (L * snr * sigma);
  const kMin = Fthresh / (v * Tdetect * dt);
  const feasible = kMin < kMax;
  const kRec = feasible ? Math.sqrt(kMin * kMax) : null;

  // What the recommended (or k_min) spring resolves / detects, and the travel it needs.
  const k = kRec ?? kMin;
  // Travel only has to survive STATIC slope sensing (all `feet` on the ground). The worst
  // (downhill) foot carries the even share W/feet plus the slope redistribution
  // ΔF/foot = W·h·tanθ/L over the two foot rows. Walking 2-foot support loads a foot more,
  // but we don't sense force then and a transient bottom-out is harmless.
  const maxFootLoad = (W / feet) * (1 + 2 * h * Math.tan(slopeMax) / L);
  const travelNeeded = maxFootLoad / k;
  // Slope sensing at the recommended k → the slope angle that gives a just-resolvable ΔX.
  const resolvableSlopeDeg = Math.atan((snr * sigma * L * k) / (W * h)) / DEG;
  // Contact detection time at k.
  const detectTicksAt = Fthresh / (k * v * dt);

  return {
    W, kMin, kMax, feasible, kRec,
    maxFootLoad, resolvableSlopeDeg, detectTicksAt, travelNeeded,
    travelOk: travelAvail ? travelNeeded <= travelAvail : null,
  };
}

// Plain-text report for the CLI / console.
export function report(p) {
  const r = springWindow(p);
  const L = (x, n = 2) => (x == null ? '—' : x.toFixed(n));
  const lines = [
    `weight: ${L(r.W, 1)} N   worst foot load (static ${p.slopeMaxDeg ?? 20}° slope sensing): ${L(r.maxFootLoad, 1)} N`,
    `spring-rate window:  k_min ${L(r.kMin)}  <  k  <  k_max ${L(r.kMax)}  N/mm`,
  ];
  if (!r.feasible) {
    lines.push(`*** NO WINDOW: k_min ≥ k_max — the sensor can't both resolve ${p.slopeMinDeg ?? 3}° slopes`);
    lines.push(`    and detect contact in ${p.detectTicks ?? 2} ticks. Use a finer sensor (smaller`);
    lines.push(`    resolution), a taller COG / shorter wheelbase, or relax one requirement.`);
  } else {
    lines.push(`recommended k: ${L(r.kRec)} N/mm`);
    lines.push(`  → resolves slopes down to ~${L(r.resolvableSlopeDeg, 1)}°`);
    lines.push(`  → detects contact in ~${L(r.detectTicksAt, 1)} ticks`);
    lines.push(`  → needs ~${L(r.travelNeeded, 1)} mm travel` +
               (r.travelOk == null ? '' : r.travelOk ? ` (fits ${p.springTravelMm} mm ✓)` : ` (EXCEEDS ${p.springTravelMm} mm ✗)`));
  }
  return lines.join('\n');
}
