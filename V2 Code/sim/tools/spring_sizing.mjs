// Calf-spring sizing calculator. Prints the usable spring-rate window + a recommendation
// for the robot's geometry/gait/sensor. Run:  node tools/spring_sizing.mjs
//
// Override any input on the command line, e.g.:
//   node tools/spring_sizing.mjs adcNoiseCounts=5 slopeMinDeg=2 mass=4 preloadMm=3
//
// The maths is documented in spring_sizing.js and SPRING_SIZING.md.

import { report, springWindow } from '../web/spring_sizing.js';

// Defaults for Koda V2 (geometry from config.h + the IK; sensor/gait are best estimates to tune).
const defaults = {
  mass: 3.0,               // kg
  cogHeight: 180,          // mm above the feet (config COG_HEIGHT_MM)
  footSeparation: 221,     // mm fore/aft (config LEG_X_SEPARATION_MM)
  calfAngleDeg: 24.0,      // calf angle from vertical at the standing pose (from the IK at ZERO target)
  snr: 3,                  // safety factor over the noise floor
  slopeMinDeg: 3,          // smallest slope we want to sense
  slopeMaxDeg: 20,         // steepest slope (for the travel swing)
  contactForceN: 4,        // config CONTACT_FORCE_N
  descentSpeedMmS: 150,    // foot vertical speed near landing (TUNE: ~ step height × cadence)
  detectTicks: 2,          // control ticks allowed to register a contact
  dtS: 0.02,               // 50 Hz control tick
  feet: 4,                 // feet on the ground while slope sensing
  springTravelMm: 16,      // physical per-spring axial travel (HALL_ZERO−HALL_FULL load dist)
  preloadMm: 1.0,          // install pre-compression of each calf spring (TUNE to assembly)
  frictionN: 0.0,          // per-spring stiction breakaway, N — raise until VISIBLE compression
                           // matches the bench (the frictionless default hides real friction)
  // Sensor: signal = a/(d+b)³ + c.  σ derives from the curve + ADC noise at the operating point.
  calA: 6.6e6,             // counts·mm³ from the calibration fit (TUNE to your bench fit)
  calB: 5.0,               // magnetic offset mm (config HALL_MAGNET_OFFSET_MM)
  adcNoiseCounts: 3,       // *** ADC noise floor, counts — MEASURE on the bench (dominant unknown)
  zeroLoadDistMm: 17.0,    // unloaded spring distance (config HALL_ZERO_LOAD_DIST_MM)
  fullLoadDistMm: 1.0,     // fully-compressed distance (config HALL_FULL_LOAD_DIST_MM)
};

const args = Object.fromEntries(process.argv.slice(2).map((a) => {
  const [k, v] = a.split('=');
  return [k, isNaN(parseFloat(v)) ? v : parseFloat(v)];
}));
const params = { ...defaults, ...args };

console.log('Koda V2 — calf-spring sizing');
console.log('inputs:', Object.entries(params).map(([k, v]) => `${k}=${v}`).join(' '));
console.log('');
console.log(report(params));

process.exit(springWindow(params).feasible ? 0 : 1);
