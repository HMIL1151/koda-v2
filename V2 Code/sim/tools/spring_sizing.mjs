// Calf-spring sizing calculator. Prints the usable spring-rate window + a recommendation
// for the robot's geometry/gait/sensor. Run:  node tools/spring_sizing.mjs
//
// Override any input on the command line, e.g.:
//   node tools/spring_sizing.mjs sensorResolutionMm=0.02 slopeMinDeg=2 mass=4
//
// The maths is documented in spring_sizing.js and SPRING_SIZING.md.

import { report, springWindow } from '../web/spring_sizing.js';

// Defaults for Koda V2 (geometry from config.h; sensor/gait are best estimates to tune).
const defaults = {
  mass: 3.0,               // kg
  cogHeight: 180,          // mm above the feet (config COG_HEIGHT_MM)
  footSeparation: 221,     // mm fore/aft (config LEG_X_SEPARATION_MM)
  sensorResolutionMm: 0.05, // mm of calf compression the hall sensor can resolve (TUNE to your ADC/noise)
  snr: 3,                  // safety factor over the noise floor
  slopeMinDeg: 3,          // smallest slope we want to sense
  slopeMaxDeg: 20,         // steepest slope (for the travel swing)
  contactForceN: 4,        // config CONTACT_FORCE_N
  descentSpeedMmS: 150,    // foot vertical speed near landing (TUNE: ~ step height × cadence)
  detectTicks: 2,          // control ticks allowed to register a contact
  dtS: 0.02,               // 50 Hz control tick
  stanceFeet: 2,           // trot supports on 2 feet
  springTravelMm: 16,      // physical spring travel available
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
