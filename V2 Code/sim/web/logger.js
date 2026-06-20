// Trace recorder + event detection. DOM-free so it works in the browser and under node
// (headless trace dumps). Records a flat row per tick — easy to export to CSV and plot,
// or to JSON for replay.

const LEG = ['FL', 'FR', 'RR', 'RL'];
const STATE = ['SLEEP', 'STAND', 'WALK', 'SIT', 'STOPPING', 'MEASURING'];

// Flatten a World.state() snapshot (+ elapsed time) into a flat row of scalar columns.
export function flatten(st, t) {
  const row = {
    t: +t.toFixed(4),
    state: st.robotState,
    phase: +st.cyclePhase.toFixed(4),
    body_x: +st.body.x.toFixed(2), body_y: +st.body.y.toFixed(2), body_z: +st.body.z.toFixed(2),
    pitch_deg: +(st.body.pitch * 57.2958).toFixed(2),
    roll_deg: +(st.body.roll * 57.2958).toFixed(2),
    yaw_deg: +(st.body.yaw * 57.2958).toFixed(2),
    total_N: +st.totalForce.toFixed(2),
  };
  st.forces.forEach((f, i) => (row['f_' + LEG[i]] = +f.toFixed(2)));
  st.contact.forEach((c, i) => (row['contact_' + LEG[i]] = c ? 1 : 0));
  st.early.forEach((e, i) => (row['early_' + LEG[i]] = e ? 1 : 0));
  if (st.command) {
    row.cmd_vx = +(st.command.vx ?? 0).toFixed(3);
    row.cmd_vy = +(st.command.vy ?? 0).toFixed(3);
    row.cmd_yaw = +(st.command.yaw ?? 0).toFixed(3);
    row.cmd_height = +(st.command.height ?? 0).toFixed(3);
  }
  if (st.servos) st.servos.forEach((s, i) => (row['servo_' + i] = +s.toFixed(2)));
  return row;
}

export class Recorder {
  constructor(cap = 50000) { this.cap = cap; this.rows = []; this.t = 0; }

  push(state, dt = 0) {
    this.t += dt;
    this.rows.push(flatten(state, this.t));
    if (this.rows.length > this.cap) this.rows.splice(0, this.rows.length - this.cap);
  }

  clear() { this.rows = []; this.t = 0; }
  get length() { return this.rows.length; }

  toCSV() {
    if (!this.rows.length) return '';
    const cols = Object.keys(this.rows[0]);
    const head = cols.join(',');
    const body = this.rows.map((r) => cols.map((c) => r[c] ?? '').join(',')).join('\n');
    return head + '\n' + body;
  }

  toJSON() { return JSON.stringify(this.rows); }
}

// Detect notable changes between two snapshots → human-readable event strings.
export function diffEvents(prev, cur) {
  const ev = [];
  if (!prev || prev.robotState !== cur.robotState) {
    ev.push(`state → ${STATE[cur.robotState] ?? '?'}`);
  }
  for (let i = 0; i < 4; i++) {
    if (cur.early[i] && (!prev || !prev.early[i])) ev.push(`early contact: ${LEG[i]}`);
    if (cur.late && cur.late[i] && (!prev || !prev.late || !prev.late[i])) {
      ev.push(`late contact: ${LEG[i]} (reaching for ground)`);
    }
    if (prev && prev.contact[i] !== cur.contact[i]) {
      ev.push(`${LEG[i]} ${cur.contact[i] ? 'planted' : 'lifted'}`);
    }
    if (cur.ikOk && !cur.ikOk[i] && (!prev || !prev.ikOk || prev.ikOk[i])) {
      ev.push(`⚠ ${LEG[i]} OUT OF RANGE (no IK solution)`);
    }
  }
  return ev;
}
