// Headless trace dump — run a scenario through the real control core and print a CSV of
// every tick to stdout. Lets you debug the control logic offline and plot the result.
//
//   node tools/trace.mjs <scenario> <ticks> <slopeDeg>
//   node tools/trace.mjs slope 600 12 > trace.csv
//   node tools/trace.mjs step  800     > step.csv
//
// scenarios: flat | slope | step | bumps   (default: flat, 400 ticks)
// Each run: stand for 40 ticks, then walk forward. Columns include command, body pose,
// per-foot force, contact/early flags and all 12 servo angles — see web/logger.js.

import KodaCore from '../web/koda-core.mjs';
import { World, BTN } from '../web/world.js';
import { Recorder } from '../web/logger.js';

const scenario = process.argv[2] || 'flat';
const ticks = parseInt(process.argv[3] || '400', 10);
const slope = (parseFloat(process.argv[4] || '12') * Math.PI) / 180;

const terrains = {
  flat: () => 0,
  slope: (x) => x * Math.tan(slope),
  step: (x) => (x > 120 ? 35 : 0),
  bumps: (x) => 14 * Math.sin(x / 120),
};

const M = await KodaCore();
const world = new World(M, { terrain: terrains[scenario] || terrains.flat });
const rec = new Recorder();

world.step({ buttons: BTN.STAND }, 0.02);          // stand up
for (let i = 0; i < ticks; i++) {
  const cmd = i > 40 ? { vx: 0.9 } : {};           // settle, then walk
  world.step(cmd, 0.02);
  rec.push(world.state(), 0.02);
}

process.stdout.write(rec.toCSV() + '\n');
process.stderr.write(`# ${scenario}: ${rec.length} ticks\n`);
