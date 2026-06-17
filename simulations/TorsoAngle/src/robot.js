import { Leg } from './leg.js';
import * as h from './helpers.js';

// A two-legged robot in 2D side view. Each leg is the existing 5-bar sprung
// linkage (two thighs + two calf springs -> one foot). The two legs share one
// rigid torso; all four hips lie on the torso line.
//
// Convention (inherited from leg.js): +x right, +y DOWN. Gravity points +y.
//
//   solveEquilibrium(groundAngleDeg) -- pin both feet on a plane, then drop the
//       torso under gravity and let the springs settle. Torso angle + the four
//       compressions are OUTPUTS.
//   inferFromCompressions(...) -- given the four compressions (a proprioceptive
//       reading) estimate the ground angle, assuming the net reaction is vertical.
//
// Each calf is a PRELOADED compression spring between two surfaces with end stops
// at both max compression and max extension (still in compression at the extension
// stop). Within the travel band [minComp, maxComp] the force is a clean function of
// compression (F = k*c) -- the accurate measurement region. At either stop the
// strut goes rigid (modelled as a very stiff wall); the extension stop carries
// TENSION through the stop structure, which is what keeps the body up. Beyond a
// stop the compression reading saturates, so the force is no longer measurable.
//
// Because the spring energy is bounded on both ends, the equilibrium is well posed
// (no runaway). The robot stands until its weight vector falls outside the two foot
// contacts (CoM-vs-support), at which point it tips -- this is detected separately,
// NOT by spring slack.

const L0 = 120;                 // calf free length (mm) -- matches leg.js calfLength
const WALL_STIFFNESS_RATIO = 30; // rigid end-stop stiffness multiplier

export class Robot
{
    constructor({ thighFrontDeg = 120, thighRearDeg = 120,
                  flFrontDeg, flRearDeg, rlFrontDeg, rlRearDeg,
                  legBaseMm = 150,
                  weightN = 40, comOffsetMm = 0, comHeightMm = 0, springRateNmm = 3,
                  minCompressionMm = 1, maxCompressionMm = 25 } = {})
    {
        // Four independent thigh angles. They default to the per-leg pair values
        // (thighFrontDeg/thighRearDeg) so older two-angle callers still work.
        this.thigh = {
            flF: flFrontDeg ?? thighFrontDeg, flR: flRearDeg ?? thighFrontDeg,
            rlF: rlFrontDeg ?? thighRearDeg, rlR: rlRearDeg ?? thighRearDeg,
        };
        this.legBase = legBaseMm;
        this.weight = weightN;
        this.comOffset = comOffsetMm;
        this.comHeight = comHeightMm;
        this.springRate = springRateNmm;
        this.minComp = minCompressionMm;
        this.maxComp = maxCompressionMm;

        // Per-leg geometry: each leg's two thighs can differ.
        const geomF = new Leg(); geomF.setThighAngles(this.thigh.flF, this.thigh.flR);
        const geomR = new Leg(); geomR.setThighAngles(this.thigh.rlF, this.thigh.rlR);
        const knF = geomF.computeKneePositions();
        const knR = geomR.computeKneePositions();
        this.servoDistance = geomF.servoDistance;

        // Leg-centre x positions in the torso frame (origin = torso centre).
        const half = this.servoDistance / 2 + this.legBase / 2;
        this.flOff = -half;   // front leg
        this.rlOff =  half;   // rear leg

        this.kneesLocal = {
            fl: { front: this._legPt(this.flOff, knF.front), rear: this._legPt(this.flOff, knF.rear) },
            rl: { front: this._legPt(this.rlOff, knR.front), rear: this._legPt(this.rlOff, knR.rear) },
        };

        // Natural foot depth below the hips at zero compression (warm-start ref),
        // averaged over the two legs.
        this.restFootDepth = (this._restFootDepth(this.thigh.flF, this.thigh.flR) +
                              this._restFootDepth(this.thigh.rlF, this.thigh.rlR)) / 2;
        this.footSpacing = this.rlOff - this.flOff;

        // Results (filled by the solvers).
        this.torso = { x: 0, y: 0, phi: 0 };
        this.feet = { fl: null, rl: null };
        this.comps = { flF: 0, flR: 0, rlF: 0, rlR: 0 };
        this.reactions = { fl: { x: 0, y: 0 }, rl: { x: 0, y: 0 } };
        this.groundAngleDeg = 0;
        this.legAngles = null;
        this.confidence = null;
        this.confidenceNote = '';
    }

    _legPt(offX, localKnee) { return { x: offX + localKnee.x, y: localKnee.y }; }

    _restFootDepth(frontThighDeg, rearThighDeg)
    {
        const probe = new Leg();
        probe.setThighAngles(frontThighDeg, rearThighDeg);
        probe.setFrontCalfCompression(0);
        probe.setRearCalfCompression(0);
        probe.solveForwardKinematics();
        return probe.footPosition.y;
    }

    _toWorld(localPt, pose)
    {
        return h.add(h.rotate(localPt, pose[2]), { x: pose[0], y: pose[1] });
    }

    // World-space knee/foot of one leg for given thigh angles and FREE calves, at
    // the current torso pose. Used to draw a swing leg whose foot follows the
    // linkage as its thighs sweep (the foot is unloaded during swing).
    legWorldFK(side, frontThighDeg, rearThighDeg)
    {
        const leg = new Leg();
        leg.setThighAngles(frontThighDeg, rearThighDeg);
        leg.setFrontCalfCompression(0);
        leg.setRearCalfCompression(0);
        leg.solveForwardKinematics();
        const off = side === 'fl' ? this.flOff : this.rlOff;
        const pose = [this.torso.x, this.torso.y, this.torso.phi];
        const w = (p) => this._toWorld({ x: off + p.x, y: p.y }, pose);
        return { kneeF: w(leg.frontKneePosition), kneeR: w(leg.rearKneePosition), foot: w(leg.footPosition) };
    }

    // Local-frame centre of mass: fore/aft offset + height above the torso line
    // (up = -y).
    _comLocal() { return { x: this.comOffset, y: -this.comHeight }; }

    // --- trilinear strut: soft spring within travel, rigid stops beyond ---
    // comp = signed deflection from free length (positive = compressed). Below the
    // extension stop and above the compression stop the strut is rigid (stiff
    // wall); the extension-stop wall lets the force go negative (tension carried by
    // the stop). Continuous in value and slope at both knees.
    _springForce(comp)
    {
        const k = this.springRate;
        const kw = k * WALL_STIFFNESS_RATIO;
        if (comp < this.minComp) return k * this.minComp + kw * (comp - this.minComp);
        if (comp > this.maxComp) return k * this.maxComp + kw * (comp - this.maxComp);
        return k * comp;
    }

    _springEnergy(comp)
    {
        const k = this.springRate;
        const kw = k * WALL_STIFFNESS_RATIO;
        if (comp < this.minComp) {
            const d = comp - this.minComp;
            return 0.5 * k * this.minComp * this.minComp + k * this.minComp * d + 0.5 * kw * d * d;
        }
        if (comp > this.maxComp) {
            const d = comp - this.maxComp;
            return 0.5 * k * this.maxComp * this.maxComp + k * this.maxComp * d + 0.5 * kw * d * d;
        }
        return 0.5 * k * comp * comp;
    }

    // Signed geometric compressions for a candidate pose + pinned feet.
    _compsForPose(pose, footFL, footRL)
    {
        const k = this.kneesLocal;
        const comp = (kneeLocal, foot) => {
            const knee = this._toWorld(kneeLocal, pose);
            return L0 - h.len(h.sub(foot, knee));
        };
        return {
            flF: comp(k.fl.front, footFL), flR: comp(k.fl.rear, footFL),
            rlF: comp(k.rl.front, footRL), rlR: comp(k.rl.rear, footRL),
        };
    }

    // Ground reaction on a foot = -(sum of strut forces on the foot). A strut
    // pushes the foot away from the knee when compressed (F>0) and pulls it back
    // when at the extension stop (F<0, tension carried by the stop).
    _legReaction(pose, kneeF, kneeR, foot, compF, compR)
    {
        let strut = { x: 0, y: 0 };
        for (const [kneeLocal, comp] of [[kneeF, compF], [kneeR, compR]]) {
            const f = this._springForce(comp);
            const knee = this._toWorld(kneeLocal, pose);
            const dir = h.sub(foot, knee);
            const dlen = h.len(dir) || 1;
            strut = h.add(strut, h.scale(dir, f / dlen));
        }
        return h.scale(strut, -1);
    }

    // ---- ground angle -> push-only equilibrium -> compressions ----
    // Thin wrapper: pin the feet at the natural stance on a plane at the given
    // angle (each leg centre projected onto the plane through the origin), then
    // solve. groundAngleDeg is reported as the plane angle.
    solveEquilibrium(groundAngleDeg)
    {
        const theta = h.degToRad(groundAngleDeg);
        const d = { x: Math.cos(theta), y: Math.sin(theta) };
        const footFL = h.scale(d, -this.footSpacing / 2);
        const footRL = h.scale(d,  this.footSpacing / 2);
        return this.solveWithFeet(footFL, footRL, { groundAngleDeg, warmPhi: theta });
    }

    // ---- arbitrary pinned feet -> push-only equilibrium ----
    // Both feet are pinned at the given world positions (e.g. on uneven terrain);
    // the torso settles under gravity. Used by both the plane wrapper above and
    // the walking gait. `groundAngleDeg`, if omitted, is taken as the slope of the
    // line through the two feet.
    solveWithFeet(footFL, footRL, { groundAngleDeg = null, warmPhi = null } = {})
    {
        this.feet = { fl: footFL, rl: footRL };
        const footLineDeg = h.radToDeg(Math.atan2(footRL.y - footFL.y, footRL.x - footFL.x));
        const theta = warmPhi !== null ? warmPhi : h.degToRad(footLineDeg);

        const R = 100;                                   // phi scaling (mm/rad)
        const unpack = (v) => [v[0], v[1], v[2] / R];

        const U = (v) => {
            const pose = unpack(v);
            const c = this._compsForPose(pose, footFL, footRL);
            const com = this._toWorld(this._comLocal(), pose);
            const gravity = -this.weight * com.y;
            const spring = this._springEnergy(c.flF) + this._springEnergy(c.flR)
                         + this._springEnergy(c.rlF) + this._springEnergy(c.rlR);
            return gravity + spring;
        };

        // Warm start: torso parallel to the foot line, above the feet midpoint.
        const mid = h.scale(h.add(footFL, footRL), 0.5);
        const upNormal = { x: Math.sin(theta), y: -Math.cos(theta) };
        const standoff = this.restFootDepth - 25;
        const start = [mid.x + standoff * upNormal.x, mid.y + standoff * upNormal.y, theta * R];

        const pose = unpack(h.minimize(U, start, { iterations: 1000, trustRadius: 4 }));

        this.torso = { x: pose[0], y: pose[1], phi: pose[2] };
        this.comps = this._compsForPose(pose, footFL, footRL);

        const k = this.kneesLocal;
        this.reactions = {
            fl: this._legReaction(pose, k.fl.front, k.fl.rear, footFL, this.comps.flF, this.comps.flR),
            rl: this._legReaction(pose, k.rl.front, k.rl.rear, footRL, this.comps.rlF, this.comps.rlR),
        };
        this.groundAngleDeg = groundAngleDeg !== null ? groundAngleDeg : footLineDeg;
        this.legAngles = null;
        this.confidence = null;
        return this.results();
    }

    // Force the robot's sensor can actually report: F = k*compression within the
    // travel band, saturating at the stops (the stop's structural force is NOT
    // measurable). Never negative -- the sensor cannot read the extension-stop
    // tension. This is what inference must work from.
    _measuredForce(comp)
    {
        const c = Math.max(this.minComp, Math.min(this.maxComp, comp));
        return this.springRate * c;
    }

    // Leg-local ground reaction from measured forces (torso frame; before pose).
    _measuredLegReaction(kneeF, kneeR, foot, compF, compR)
    {
        let strut = { x: 0, y: 0 };
        for (const [kneeLocal, comp] of [[kneeF, compF], [kneeR, compR]]) {
            const f = this._measuredForce(comp);
            const dir = h.sub(foot, kneeLocal);
            const dlen = h.len(dir) || 1;
            strut = h.add(strut, h.scale(dir, f / dlen));
        }
        return h.scale(strut, -1);
    }

    // ---- compressions -> estimated ground angle (sensor inference) ----
    inferFromCompressions({ flF, flR, rlF, rlR })
    {
        flF = Math.max(0, flF); flR = Math.max(0, flR);
        rlF = Math.max(0, rlF); rlR = Math.max(0, rlR);

        // Foot positions come from the leg geometry (FK on the actual readings).
        const footLocal = (frontThigh, rearThigh, cF, cR, off) => {
            const leg = new Leg();
            leg.setThighAngles(frontThigh, rearThigh);
            leg.setFrontCalfCompression(cF);
            leg.setRearCalfCompression(cR);
            leg.solveForwardKinematics();
            return { x: off + leg.footPosition.x, y: leg.footPosition.y };
        };
        const footFLlocal = footLocal(this.thigh.flF, this.thigh.flR, flF, flR, this.flOff);
        const footRLlocal = footLocal(this.thigh.rlF, this.thigh.rlR, rlF, rlR, this.rlOff);

        // Per-leg ground reactions (torso frame) from MEASURED forces.
        const k = this.kneesLocal;
        const reFLlocal = this._measuredLegReaction(k.fl.front, k.fl.rear, footFLlocal, flF, flR);
        const reRLlocal = this._measuredLegReaction(k.rl.front, k.rl.rear, footRLlocal, rlF, rlR);
        const netReaction = h.add(reFLlocal, reRLlocal);

        // Orient the torso so the net reaction points straight up (= gravity).
        let phi = 0;
        if (h.len(netReaction) > 1e-6) {
            phi = -Math.PI / 2 - Math.atan2(netReaction.y, netReaction.x);
        }

        const pose = [0, -this.restFootDepth, phi];
        this.torso = { x: pose[0], y: pose[1], phi };
        this.feet = {
            fl: this._toWorld(footFLlocal, pose),
            rl: this._toWorld(footRLlocal, pose),
        };
        this.comps = { flF, flR, rlF, rlR };

        // World reactions = local reactions rotated into the world frame.
        this.reactions = { fl: h.rotate(reFLlocal, phi), rl: h.rotate(reRLlocal, phi) };

        this.groundAngleDeg = h.radToDeg(Math.atan2(this.feet.rl.y - this.feet.fl.y,
                                                     this.feet.rl.x - this.feet.fl.x));
        // Per-leg reaction tilt from vertical (informational).
        const tilt = (r) => h.radToDeg(Math.atan2(r.x, -r.y));
        this.legAngles = { fl: tilt(reFLlocal), rl: tilt(reRLlocal) };

        // --- Confidence: is this a physically valid standing configuration? ---
        // For the inferred ground angle to be trustworthy, both feet must push UP
        // (positive vertical reaction) and the centre of pressure must lie between
        // them. A foot with zero / negative load means the compressions cannot
        // correspond to a stable stance on a single flat plane.
        const loadFL = -this.reactions.fl.y;
        const loadRL = -this.reactions.rl.y;
        const total = loadFL + loadRL;

        // Any reading at/beyond a stop means the force is saturated -> the value of
        // F = k*c is wrong, so the inferred angle cannot be trusted.
        const outOfBand = [flF, flR, rlF, rlR].some(
            (c) => c < this.minComp - 1e-6 || c > this.maxComp + 1e-6);

        if (h.len(netReaction) < 1e-3 || total <= 1e-6) {
            this.confidence = 0;
            this.confidenceNote = 'No net support — compressions do not define a stance.';
        } else if (loadFL <= 0 || loadRL <= 0) {
            this.confidence = 0;
            this.confidenceNote = 'A foot carries no load — robot would tip, angle unreliable.';
        } else {
            // Centre-of-pressure margin: 1 = perfectly centred, 0 = at a foot.
            const margin = (2 * Math.min(loadFL, loadRL)) / total;
            let conf = Math.round(margin * 100);
            if (outOfBand) {
                conf = Math.min(conf, 20);   // saturated force reading -> untrustworthy
                this.confidenceNote = 'A spring is at an end stop — force saturated, angle unreliable.';
            } else {
                this.confidenceNote = conf > 60 ? 'Well-supported stance.'
                    : conf > 25 ? 'Off-centre load — usable but check.'
                    : 'Near tipping — low confidence.';
            }
            this.confidence = conf;
        }

        return this.results();
    }

    results()
    {
        const pose = [this.torso.x, this.torso.y, this.torso.phi];
        const hip = (off, sign) => this._toWorld({ x: off + sign * this.servoDistance / 2, y: 0 }, pose);
        const k = this.kneesLocal;
        const w = this.weight;

        const loadFL = -this.reactions.fl.y;
        const loadRL = -this.reactions.rl.y;

        const residual = {
            x: this.reactions.fl.x + this.reactions.rl.x,
            y: this.reactions.fl.y + this.reactions.rl.y + w,
        };
        const residualMag = Math.hypot(residual.x, residual.y);

        const deviationDeg = h.radToDeg(this.torso.phi) - this.groundAngleDeg;

        // --- Tipping: weight vector (vertical through CoM) vs the support, i.e.
        // the segment between the two feet. Gravity is vertical, so compare CoM x
        // to the feet x. High foot friction holds the feet, so the robot stands as
        // long as the weight line lands between them. ---
        const com = this._toWorld(this._comLocal(), pose);
        const footLo = Math.min(this.feet.fl.x, this.feet.rl.x);
        const footHi = Math.max(this.feet.fl.x, this.feet.rl.x);
        const span = (footHi - footLo) || 1;
        const stable = com.x >= footLo && com.x <= footHi;
        // Margin: +ve = inside (fraction of half-span to nearest edge), -ve = past edge.
        const tipMargin = (Math.min(com.x - footLo, footHi - com.x)) / (span / 2);
        const tipSide = !stable ? (com.x > footHi
            ? (this.feet.fl.x > this.feet.rl.x ? 'front' : 'rear')
            : (this.feet.fl.x < this.feet.rl.x ? 'front' : 'rear')) : null;

        // Per-calf state: at the extension stop (c<minComp) or compression stop
        // (c>maxComp) the strut is rigid and the force reading is inaccurate.
        const calfState = (c) => c < this.minComp - 0.05 ? 'ext-stop'
                               : c > this.maxComp + 0.05 ? 'comp-stop' : 'ok';
        const states = {
            flF: calfState(this.comps.flF), flR: calfState(this.comps.flR),
            rlF: calfState(this.comps.rlF), rlR: calfState(this.comps.rlR),
        };
        const atStop = Object.values(states).some((s) => s !== 'ok');

        return {
            torsoAngleDeg: h.radToDeg(this.torso.phi),
            groundAngleDeg: this.groundAngleDeg,
            deviationDeg,
            stable,
            tipMargin,
            tipSide,
            atStop,
            states,
            minComp: this.minComp,
            maxComp: this.maxComp,
            comps: this.comps,
            reactions: this.reactions,
            loadFL, loadRL, loadTotal: loadFL + loadRL,
            residual, residualMag,
            legAngles: this.legAngles,
            confidence: this.confidence,
            confidenceNote: this.confidenceNote,
            points: {
                com,
                hips: {
                    flFront: hip(this.flOff, -1), flRear: hip(this.flOff, 1),
                    rlFront: hip(this.rlOff, -1), rlRear: hip(this.rlOff, 1),
                },
                knees: {
                    flFront: this._toWorld(k.fl.front, pose), flRear: this._toWorld(k.fl.rear, pose),
                    rlFront: this._toWorld(k.rl.front, pose), rlRear: this._toWorld(k.rl.rear, pose),
                },
                feet: this.feet,
                torsoEnds: {
                    left:  this._toWorld({ x: this.flOff - this.servoDistance / 2 - 35, y: 0 }, pose),
                    right: this._toWorld({ x: this.rlOff + this.servoDistance / 2 + 35, y: 0 }, pose),
                },
            },
        };
    }
}
