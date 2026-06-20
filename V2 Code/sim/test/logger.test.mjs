// Tests for the trace recorder + event detection. Run: node sim/test/logger.test.mjs

import KodaCore from '../web/koda-core.mjs';
import { World, BTN } from '../web/world.js';
import { Recorder, diffEvents } from '../web/logger.js';

const M = await KodaCore();
let failures = 0;
const ok = (c, m) => (c ? console.log('  ok:', m) : (console.error('  FAIL:', m), failures++));

console.log('Recorder:');
{
  const w = new World(M);
  const rec = new Recorder(100);
  w.step({ buttons: BTN.STAND }, 0.02);
  for (let i = 0; i < 50; i++) { w.step({ vx: 0.5 }, 0.02); rec.push(w.state(), 0.02); }

  ok(rec.length === 50, 'records one row per push');

  const csv = rec.toCSV();
  const lines = csv.trim().split('\n');
  const header = lines[0].split(',');
  ok(lines.length === 51, 'CSV has header + one line per tick');
  ok(header.includes('f_FL') && header.includes('servo_0') && header.includes('cmd_vx'),
     'CSV header has forces, servos and command columns');
  ok(lines[1].split(',').length === header.length, 'each CSV row matches the header width');

  const json = JSON.parse(rec.toJSON());
  ok(Array.isArray(json) && json.length === 50 && 'pitch_deg' in json[0],
     'JSON export round-trips to flat rows');

  rec.cap = 10;
  for (let i = 0; i < 20; i++) rec.push(w.state(), 0.02);
  ok(rec.length === 10, 'ring buffer caps at capacity');
}

console.log('Events:');
{
  const w = new World(M);
  let prev = null, sawStand = false;
  for (let i = 0; i < 5; i++) {
    const buttons = i === 1 ? BTN.STAND : 0;
    w.step({ buttons }, 0.02);
    const st = w.state();
    const ev = diffEvents(prev, st);
    if (ev.some((e) => e.includes('STAND'))) sawStand = true;
    prev = st;
  }
  ok(sawStand, 'a state change emits a readable event');
}

console.log(failures === 0 ? '\nALL LOGGER TESTS PASSED' : `\n${failures} TEST(S) FAILED`);
process.exit(failures === 0 ? 0 : 1);
