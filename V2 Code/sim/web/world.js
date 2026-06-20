// Reduced-physics quadruped world. DOM-free: it owns the simulation state and drives the
// real control core (the WASM SimCore), so it imports cleanly under node for headless
// tests as well as in the browser.
//
// The model (deliberately reduced — kinematic body + penetration springs, quasi-static):
//   1. The control core decides each foot's target (leg frame) from command + foot forces.
//   2. We place those feet in the world through the current body pose.
//   3. Feet below the terrain compress a spring → an upward force (this IS the hall-sensor
//      reading the calf compression). force = compression · springRate.
//   4. The body relaxes its height / pitch / roll toward force equilibrium (first-order,
//      stable), and advances forward/lateral/turn in step with the gait phase.
//   5. Those per-foot forces feed back into the core next tick — closing the loop.
//
// Leg order everywhere: FL, FR, RR, RL.

import { clamp, rotate3, add3, phaseDelta } from './helpers.js';

export const RobotState = { SLEEP: 0, STAND: 1, WALK: 2, SIT: 3 };
export const BTN = { STAND: 1, SIT: 2, GAIT: 4 };

export class World {
  // `M` is the loaded WASM module (has M.SimCore + constants).
  constructor(M, opts = {}) {
    this.M = M;
    this.core = new M.SimCore();

    this.weightN = opts.weightN ?? 30;             // robot weight
    this.comHeight = opts.comHeight ?? 60;         // COG height above the hip line (mm)
    this.springRate = M.HALL_SPRING_N_PER_MM;      // per-foot spring (N/mm)
    this.maxCompMm = opts.maxCompMm ?? 16;         // spring travel
    this.halfX = M.LEG_X_SEPARATION_MM / 2;        // hip fore/aft half-span (fixed)
    this.mountX = [this.halfX, this.halfX, -this.halfX, -this.halfX];

    // Step geometry (runtime-tunable, kept in lockstep with the core). Read the core's
    // defaults, then derive the world geometry that depends on stance width.
    const [sx, sy, sz, sh, sl] = this.core.gaitParams();
    this.gaitParams = { stanceX: sx, stanceY: sy, stanceZ: sz, stepHeight: sh, stepLen: sl };
    this._applyStanceWidth(sz);

    // Settle gains. First-order relaxation; sized so gain·stiffness stays well under the
    // stability limit (verified by sim/test/world.test.mjs) — fast but non-oscillatory.
    // The lateral base is narrower than the fore/aft base, so the roll restoring torque is
    // weaker (∝ lever²); scale kRoll up by (halfX/halfZ)² so the body settles parallel to a
    // side slope as readily as pitch does to an incline (still within the stability limit).
    this.kY = opts.kY ?? 2.0;
    this.kPitch = opts.kPitch ?? 5e-4;
    this.kRoll = opts.kRoll ?? this.kPitch * (this.halfX / this.halfZ) ** 2;

    // Terrain: world ground height at (x,z). Default flat.
    this.terrain = opts.terrain ?? (() => 0);
    this.useImu = opts.useImu ?? false;            // feed body tilt as IMU (default: faithful = off)

    this.reset();
  }

  // Keep the world geometry that depends on stance width in step with the params.
  _applyStanceWidth(stanceZ) {
    this.halfZ = stanceZ;
    this.mountZ = [-stanceZ, stanceZ, stanceZ, -stanceZ];
  }

  // Live step-geometry tuning: update the core AND the world's matching geometry together.
  setGaitParams(p) {
    this.gaitParams = { ...this.gaitParams, ...p };
    const g = this.gaitParams;
    this.core.setGaitParams(g.stanceX, g.stanceY, g.stanceZ, g.stepHeight, g.stepLen);
    this._applyStanceWidth(g.stanceZ);
  }

  reset() {
    this.body = { x: 0, y: this.gaitParams.stanceY, z: 0, pitch: 0, roll: 0, yaw: 0 };
    this.forces = [0, 0, 0, 0];
    this.feet = [null, null, null, null];
    this.contact = [false, false, false, false];
    this.early = [false, false, false, false];
    this.prevPhase = 0;
    this.prevLocal = null;
  }

  // Hip mount world position for a leg (for drawing the leg segment).
  hipWorld(leg) {
    const local = { x: this.mountX[leg], y: 0, z: this.mountZ[leg] };
    return add3(this.body, rotate3(local, this.body.pitch, this.body.roll, this.body.yaw));
  }

  // Foot target (leg frame, from the core) → position in the BODY frame (before the body
  // pose/position is applied). This is what the gait commands relative to the body.
  footLocal(leg) {
    const ft = this.core.footTarget(leg);          // [x fwd, y down, z right] (leg frame)
    const sideZ = Math.sign(this.mountZ[leg]) || 1;
    return {
      x: this.mountX[leg] + ft[0],
      y: -ft[1],
      z: this.mountZ[leg] + sideZ * (ft[2] - this.gaitParams.stanceZ),
    };
  }

  // Body-frame foot position → world, through the body pose.
  footWorld(leg) {
    return add3(this.body, rotate3(this.footLocal(leg),
                                   this.body.pitch, this.body.roll, this.body.yaw));
  }

  step(command, dt) {
    const tiltP = this.useImu ? this.body.pitch : 0;
    const tiltR = this.useImu ? this.body.roll : 0;
    const f = this.forces;

    // 1. Real control core: command + last forces → servo targets (+ foot targets, state).
    this.lastCommand = command;
    this.lastServos = this.core.step(
      command.vx ?? 0, command.vy ?? 0, command.yaw ?? 0, command.height ?? 0,
      command.buttons ?? 0, f[0], f[1], f[2], f[3], tiltP, tiltR, dt);

    // 1b. Body advance is DRIVEN BY THE STANCE FEET, not a command heuristic. A foot in
    // contact is planted in the world, so as the gait slides it backward in the body frame
    // the body must move forward to keep it put. Averaging over the contact feet gives the
    // body's translation — exactly how the stance push propels a real robot, and why a
    // proper gait start never drags a planted foot.
    const local = [0, 1, 2, 3].map((l) => this.footLocal(l));
    if (this.prevLocal) {
      let dx = 0, dz = 0, n = 0;
      for (let l = 0; l < 4; l++) {
        if (this.contact[l]) {                       // previous tick's contact state
          dx += local[l].x - this.prevLocal[l].x;
          dz += local[l].z - this.prevLocal[l].z;
          n++;
        }
      }
      if (n > 0) {
        dx /= n; dz /= n;
        this.body.yaw += (command.yaw ?? 0) * 0.4 * Math.abs(phaseDelta(this.core.cyclePhase(), this.prevPhase));
        const c = Math.cos(this.body.yaw), s = Math.sin(this.body.yaw);
        this.body.x -= dx * c + dz * s;              // move body to keep the stance feet planted
        this.body.z -= -dx * s + dz * c;
      }
    }
    this.prevLocal = local;
    this.prevPhase = this.core.cyclePhase();

    // Centre of gravity: above the hip line, so it swings horizontally as the body tilts —
    // this is what makes a slope create a real front/rear load imbalance to correct.
    this.cog = add3(this.body, rotate3({ x: 0, y: this.comHeight, z: 0 },
                                       this.body.pitch, this.body.roll, this.body.yaw));

    // 2 & 3. Place feet, compute contact spring forces. Torques are taken about the COG, so
    // equilibrium = the gravity line through the COG passing through the support centroid.
    let totalF = 0, torquePitch = 0, torqueRoll = 0;
    for (let leg = 0; leg < 4; leg++) {
      const fw = this.footWorld(leg);
      this.feet[leg] = fw;
      const ground = this.terrain(fw.x, fw.z);
      const penetration = ground - fw.y;           // >0 when the foot is below ground
      const comp = clamp(penetration, 0, this.maxCompMm);
      const force = comp * this.springRate;
      this.forces[leg] = force;
      this.contact[leg] = this.core.inContact(leg);
      this.early[leg] = this.core.earlyContact(leg);
      totalF += force;
      torquePitch += force * (fw.x - this.cog.x);
      torqueRoll += force * (fw.z - this.cog.z);
    }

    // 4. Quasi-static settle: relax body height + tilt toward force equilibrium.
    this.body.y += this.kY * (totalF - this.weightN) * dt;
    this.body.pitch = clamp(this.body.pitch + this.kPitch * torquePitch * dt, -0.5, 0.5);
    this.body.roll = clamp(this.body.roll - this.kRoll * torqueRoll * dt, -0.5, 0.5);
  }

  // Centroid of the feet currently in contact (the support-polygon centre).
  supportCentroid() {
    const inC = this.feet.filter((p, i) => p && this.contact[i]);
    if (!inC.length) return null;
    return {
      x: inC.reduce((s, p) => s + p.x, 0) / inC.length,
      z: inC.reduce((s, p) => s + p.z, 0) / inC.length,
    };
  }

  // Snapshot for rendering / assertions.
  state() {
    return {
      body: { ...this.body },
      cog: this.cog ? { ...this.cog } : { ...this.body },
      centroid: this.supportCentroid(),
      feet: this.feet.map((p) => (p ? { ...p } : null)),
      footTargets: [0, 1, 2, 3].map((l) => this.core.footTarget(l)),  // [x,y,z] leg frame
      ikOk: [0, 1, 2, 3].map((l) => this.core.ikOk(l)),               // false = unreachable
      late: [0, 1, 2, 3].map((l) => this.core.lateContact(l)),        // reaching down for ground
      slopePitch: this.core.slopePitch(),      // live estimate from the springs (rad)
      slopeRoll: this.core.slopeRoll(),
      measuredPitch: this.core.measuredPitch(), // latched from the last MEASURING window
      measuredRoll: this.core.measuredRoll(),
      terrain: this.core.terrain(),            // 0 flat, 1 uneven, 2 slope
      autoSlope: this.core.autoSlope(),
      forces: [...this.forces],
      contact: [...this.contact],
      early: [...this.early],
      robotState: this.core.state(),
      cyclePhase: this.core.cyclePhase(),
      totalForce: this.forces.reduce((a, b) => a + b, 0),
      command: this.lastCommand ? { ...this.lastCommand } : null,
      servos: this.lastServos ? [...this.lastServos] : null,
    };
  }
}
