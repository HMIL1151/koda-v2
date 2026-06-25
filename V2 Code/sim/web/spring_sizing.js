// Calf-spring rate / travel sizing. DOM-free so it's shared by the node calculator
// (tools/spring_sizing.mjs) and the in-sim advisor panel.
//
// The ONLY spring rate we work with is `kSpring` — the rate of one physical spring inside one
// calf (the thing actually ordered). Each foot has TWO calf springs (the two output links of
// the leg's sagittal 5-bar), each compressing along its own calf axis at angle θ from vertical.
// Everything per-foot is DERIVED from kSpring and the calf angles. See ADR 0001 + SPRING_SIZING.md.
//
//   per-foot vertical rate   k_foot = kSpring · S,   S = cos²θ₁ + cos²θ₂   (S = 2cos²θ symmetric)
//   foot force (vertical)     = Σ kSpring·cᵢ·cosθᵢ   (NOT the raw axial sum)
//   a foot's vertical load V  splits to per-spring axial  V/(2cosθ)
//
// Two competing requirements bound kSpring:
//
//  • SLOPE RESOLUTION (upper bound).  Standing on slope θ, the COG (height h) projects downhill;
//    a front foot's vertical load exceeds a rear foot's by  ΔF_foot = W·h·tanθ/L  (L = foot
//    fore/aft separation). That maps to a per-spring axial COMPRESSION difference
//    Δc = ΔF_foot/(2·cosθ·kSpring). To resolve the smallest slope θ_min the sensor must see it,
//    Δc ≥ snr·σ, →  kSpring_max = W·h·tanθ_min / (L·2·cosθ·snr·σ).
//    Softer springs → bigger Δc → finer slope sensing.
//
//  • CONTACT DETECTION (lower bound).  A foot descending at v builds vertical force at rate
//    k_foot·v. With a preload dead-band F_pre = preloadMm·kSpring (load below it is invisible,
//    tared), the tared force after T_detect ticks is  k_foot·v·T·dt − F_pre. Requiring that to
//    reach F_thresh →  kSpring_min = F_thresh / (S·v·T·dt − preloadMm).
//    Stiffer springs → force rises faster → quicker contact. Preload eats the margin.
//
//  • SENSOR RESOLUTION σ is NOT a constant.  The calibration curve is signal = a/(d+b)³ + c, so a
//    fixed ADC noise floor maps to  σ(d) = adcNoiseCounts·(d+b)⁴/(3a)  — coarse at large d (light
//    load), fine at small d (heavy). It is evaluated at the STANDING slope-sensing operating
//    distance d = freeLen − c_standing, which itself depends on kSpring → solved by iteration.
//
//  • TRAVEL.  The spring must not bottom out while SLOPE SENSING (standing, all `feet` down, on
//    the steepest slope). The worst (downhill) foot carries (W/feet)·(1 + 2·h·tanθ_max/L); its
//    per-spring axial compression must stay under the physical travel. Walking 2-foot support
//    loads a foot more and MAY bottom out — that's allowed (force isn't sensed while walking).
//
//  • FRICTION / STICTION (frictionN).  Real linkages have a breakaway force: the spring won't
//    move until the load CHANGE beats it. It acts as a force dead-band (alongside preload). Two
//    consequences the frictionless model misses: (1) less VISIBLE standing compression than the
//    gross load implies (this is what makes a bench robot look like "nothing is compressing");
//    (2) a k-INDEPENDENT floor on slope sensing — the tiny front/rear force difference must beat
//    friction, and softer springs DON'T help (the signal and friction are both forces). When
//    friction dominates the sensor floor it, not k_max, sets the smallest resolvable slope.
//
// A usable design needs kSpring_min < kSpring_max; the recommended kSpring is their geometric mean.

const DEG = Math.PI / 180;

export function springWindow(p) {
  const g = p.g ?? 9.81;
  const W = p.mass * g;                                   // total weight, N
  const h = p.cogHeight;                                  // COG height above feet, mm
  const L = p.footSeparation;                             // foot fore/aft separation, mm
  const snr = p.snr ?? 3;                                 // safety factor over the noise floor
  const thetaMin = (p.slopeMinDeg ?? 3) * DEG;            // smallest slope to resolve
  const Fthresh = p.contactForceN;                        // contact-detect force, N (ground load)
  const v = p.descentSpeedMmS;                            // foot descent speed near landing, mm/s
  const Tdetect = p.detectTicks ?? 2;                     // ticks allowed to detect a contact
  const dt = p.dtS ?? 0.02;                               // control tick, s
  const feet = p.feet ?? 4;                               // feet on the ground while slope sensing
  const slopeMax = (p.slopeMaxDeg ?? 20) * DEG;           // steepest slope we sense on
  const travelAvail = p.springTravelMm ?? 16;             // physical per-spring axial travel, mm
  const preloadMm = p.preloadMm ?? 0;                     // install pre-compression, mm
  const frictionN = p.frictionN ?? 0;                     // per-spring axial stiction breakaway, N

  // Leg geometry: two calves at θ from vertical (symmetric standing pose, from the IK).
  const theta = (p.calfAngleDeg ?? 24.0) * DEG;
  const cosT = Math.cos(theta);
  const S = 2 * cosT * cosT;                              // Σcos²θ over the two calves

  // Sensor calibration: signal = a/(d+b)³ + c. σ(d) = noise·(d+b)⁴/(3a).
  const a = p.calA ?? 6.6e6;                              // counts·mm³ (from the calibration fit)
  const b = p.calB ?? 5.0;                                // magnetic offset, mm (HALL_MAGNET_OFFSET_MM)
  const noise = p.adcNoiseCounts ?? 3;                    // ADC noise floor, counts (MEASURE on bench)
  const freeLen = p.zeroLoadDistMm ?? 17.0;              // unloaded spring distance (HALL_ZERO_LOAD_DIST_MM)
  const fullLen = p.fullLoadDistMm ?? 1.0;              // fully-compressed distance (HALL_FULL_LOAD_DIST_MM)

  // Standing per-spring axial load and the operating distance for σ (depends on kSpring). The
  // distance is physically bounded to [fullLen, freeLen] — a soft spring can't sit past solid.
  const standingSpringLoad = (W / feet) / (2 * cosT);    // axial N in one spring when standing
  const clampD = (d) => (d < fullLen ? fullLen : d > freeLen ? freeLen : d);
  const operatingDist = (kS) => clampD(freeLen - standingSpringLoad / kS);  // d, mm
  const sigmaAt = (d) => noise * Math.pow(d + b, 4) / (3 * a);              // σ, mm

  // Contact lower bound is constant in kSpring. Preload eats the compression margin; friction
  // (both springs) adds to the force the foot must beat before the reading registers a contact.
  const contactMargin = S * v * Tdetect * dt - preloadMm;          // mm of usable compression
  const FthreshEff = Fthresh + 2 * frictionN;                     // foot must beat both springs' stiction
  const kMin = contactMargin > 0 ? FthreshEff / contactMargin : Infinity;

  // Slope upper bound: σ (hence kMax) depends on the operating distance, which depends on the
  // chosen kSpring — a strong negative feedback (a softer spring sits deeper where the hall
  // sensor is finer). Plain iteration OSCILLATES, so solve the fixed point with log-space
  // damping at the recommended (geometric-mean) spring.
  let kGuess = 2.0, sigmaMm = 0, operatingDistMm = freeLen, kMax = 0;
  for (let i = 0; i < 60; i++) {
    operatingDistMm = operatingDist(kGuess);
    sigmaMm = sigmaAt(operatingDistMm);
    kMax = (W * h * Math.tan(thetaMin)) / (L * 2 * cosT * snr * sigmaMm);
    const target = kMin < kMax ? Math.sqrt(kMin * kMax) : kMax;
    kGuess = Math.exp(0.5 * (Math.log(kGuess) + Math.log(target)));  // damped
  }

  // A spring so soft it sits at (or past) solid just standing is no design at all — the
  // operating distance had to be clamped up off the solid stop to stay physical.
  const standingBottomsOut = freeLen - standingSpringLoad / kGuess < fullLen;

  const feasible = kMin < kMax && contactMargin > 0 && !standingBottomsOut;

  // Friction sets a spring-rate-INDEPENDENT floor on slope sensing.
  const frictionSlopeDeg = Math.atan((frictionN * 2 * cosT * L) / (W * h)) / DEG;

  // Recommendation. Normally the geometric mean balances the contact (k_min) and slope (k_max)
  // bounds. But when friction caps slope BELOW the slopeMin target, the whole window is
  // friction-limited: a stiffer spring buys NO slope improvement, only less visible compression.
  // So bias to the soft end — ~2×k_min, which still detects contact in ~1 tick — and let the
  // extra compression help. (frictionDominates ⇔ even the stiffest allowed spring is friction-
  // limited, since at k_max the sensor resolves exactly slopeMin.)
  const frictionDominates = feasible && frictionSlopeDeg > thetaMin / DEG;
  const kRec = !feasible ? null
    : frictionDominates ? Math.min(kMax, 2 * kMin)
    : Math.sqrt(kMin * kMax);
  const kS = kRec ?? kMin;                                // evaluate outcomes at the recommendation

  // Re-evaluate the operating point / σ at the recommended spring (the iteration solved them at
  // the geometric mean; a friction-biased rec sits elsewhere on the resolution curve).
  operatingDistMm = operatingDist(kS);
  sigmaMm = sigmaAt(operatingDistMm);

  // Worst static-slope foot (all feet down) → per-spring axial compression vs travel.
  const maxFootLoad = (W / feet) * (1 + 2 * h * Math.tan(slopeMax) / L);  // vertical, N
  const travelNeeded = maxFootLoad / (2 * cosT * kS);                     // per-spring axial, mm

  // Dead-band: preload force + friction. Standing load must clear it to register any signal.
  const deadbandN = preloadMm * kS + frictionN;                           // per-spring, N (at rec)
  const deadbandOk = standingSpringLoad > deadbandN;
  // VISIBLE standing compression (what you watch on the bench). Evaluated at the ACTUAL installed
  // spring `p.kSpring` when given (so you can match the model to a real build), else at the rec.
  const kEval = p.kSpring ?? kS;
  const standingDeadbandN = preloadMm * kEval + frictionN;                       // at the eval spring
  const standingCompressionMm =
    Math.max(0, standingSpringLoad - standingDeadbandN) / kEval;                 // visible
  const standingCompressionGrossMm = standingSpringLoad / kEval;                 // frictionless

  // Slope resolved at the recommended spring: the worse of the SENSOR floor (σ) and the FRICTION
  // floor (stiction, k-independent — softer won't help).
  const sensorSlopeDeg =
    Math.atan((2 * cosT * snr * sigmaMm * L * kS) / (W * h)) / DEG;
  const resolvableSlopeDeg = Math.max(sensorSlopeDeg, frictionSlopeDeg);
  const slopeLimitedBy = frictionSlopeDeg > sensorSlopeDeg ? 'friction' : 'sensor';
  const detectTicksAt = (FthreshEff + 2 * preloadMm * kS) / (S * kS * v * dt);

  return {
    W, S, kSpringMin: kMin, kSpringMax: kMax, feasible, kSpringRec: kRec,
    kFootRec: kRec == null ? null : kRec * S, frictionDominates,
    sigmaMm, operatingDistMm, deadbandN, deadbandOk,
    standingCompressionMm, standingCompressionGrossMm, standingDeadbandN,
    maxFootLoad, resolvableSlopeDeg, sensorSlopeDeg, frictionSlopeDeg, slopeLimitedBy,
    detectTicksAt, travelNeeded, travelOk: travelNeeded <= travelAvail,
  };
}

// Plain-text report — sectioned and explicit, for the CLI / console / advisor panes.
export function report(p) {
  const r = springWindow(p);
  const f = (x, n = 2) => (x == null ? '—' : x.toFixed(n));
  const feet = p.feet ?? 4;
  const standingLoad = r.W / feet / (2 * Math.cos((p.calfAngleDeg ?? 24) * Math.PI / 180));
  const preloadForceN = r.standingDeadbandN - (p.frictionN ?? 0);
  const candidate = p.kSpring;                       // the actual spring under test, if given
  const L = [];

  L.push('ROBOT');
  L.push(`  weight ............. ${f(r.W, 1)} N   (${f(r.W / 9.81, 1)} kg)`);
  L.push(`  calf angle ......... ${f(p.calfAngleDeg ?? 24, 1)}°  ->  S = ${f(r.S, 3)}   (per-foot rate = S x kSpring)`);
  L.push('');
  L.push(`STANDING  (all ${feet} feet on the ground)`);
  L.push(`  load per spring .... ${f(standingLoad, 1)} N`);
  L.push(`  dead-band .......... ${f(r.standingDeadbandN, 1)} N   (${f(preloadForceN, 1)} N preload + ${f(p.frictionN ?? 0, 1)} N friction)`);
  L.push(`  visible squash ..... ${f(r.standingCompressionMm, 1)} mm   (would be ${f(r.standingCompressionGrossMm, 1)} mm with no friction)` +
         (candidate != null ? `   [at your ${f(candidate)} N/mm spring]` : ''));
  L.push('');

  if (!r.feasible) {
    L.push('SPRING-RATE WINDOW');
    L.push(`  *** NO WORKABLE SPRING.  Contact needs k >= ${f(r.kSpringMin)} N/mm, but slope needs`);
    L.push(`      k <= ${f(r.kSpringMax)} N/mm — they don't overlap. The sensor/preload/friction can't`);
    L.push(`      do both jobs. Lower ADC noise, cut preload/friction, or relax a target.`);
    return L.join('\n');
  }

  L.push('SPRING-RATE WINDOW   (kSpring = the physical spring you order, N/mm)');
  L.push(`  contact needs   k >= ${f(r.kSpringMin)}    (stiffer = quicker, surer contact)`);
  L.push(`  slope needs     k <= ${f(r.kSpringMax)}   (softer  = finer slope + more squash)`);
  L.push(`  RECOMMENDED     k  = ${f(r.kSpringRec)}` +
         (r.frictionDominates ? '    <- soft end: friction caps the slope, so favour squash'
                              : '    (geometric middle of the window)'));
  if (candidate != null) {
    const inWin = candidate >= r.kSpringMin && candidate <= r.kSpringMax;
    L.push(`  your spring     k  = ${f(candidate)}    ${inWin ? '-> INSIDE the window, it works ✓' : '-> OUTSIDE the window ✗'}`);
  }
  L.push('');
  L.push('SLOPE SENSING  (standing, reading front/rear imbalance)');
  L.push(`  best resolvable .... ${f(r.resolvableSlopeDeg, 1)}°   (limited by ${r.slopeLimitedBy})`);
  L.push(`     sensor floor .... ${f(r.sensorSlopeDeg, 1)}°   (from ADC noise ${p.adcNoiseCounts ?? 3} counts, σ ${f(r.sigmaMm, 4)} mm)`);
  L.push(`     friction floor .. ${f(r.frictionSlopeDeg, 1)}°   (from ${f(p.frictionN ?? 0, 1)} N stiction)`);
  if (r.slopeLimitedBy === 'friction') {
    L.push('  *** FRICTION-LIMITED.  No spring rate changes this number — a softer or stiffer');
    L.push('      spring both give the same floor. Reduce mechanical friction (guide alignment,');
    L.push('      lubrication, binding) to sense shallower slopes.');
  }
  if (!r.deadbandOk) {
    L.push(`  *** DEAD-BAND TOO HIGH.  Standing load ${f(standingLoad, 1)} N < dead-band ${f(r.deadbandN, 1)} N —`);
    L.push('      the springs barely move standing, so slope sensing reads ~nothing.');
  }
  L.push('');
  L.push('CONTACT & TRAVEL  (walking)');
  L.push(`  detects contact in . ${f(r.detectTicksAt, 1)} control ticks`);
  L.push(`  travel needed ...... ${f(r.travelNeeded, 1)} mm per spring   ` +
         (r.travelOk ? `(fits ${f(p.springTravelMm ?? 16, 0)} mm ✓)` : `(EXCEEDS ${f(p.springTravelMm ?? 16, 0)} mm ✗)`));
  return L.join('\n');
}
