// kinematics.js — single-leg 5-bar forward kinematics, generalised so the front
// and rear thighs and calves can each have their own length (the TorsoAngle Leg
// class hardcodes one thigh + one calf length per leg).
//
// Frame: +x right, +y DOWN. Two hip pivots sit on the torso at x = ±hipSpacing/2,
// y = 0. Each thigh swings down to a knee; the two calves close on the foot, which
// is the LOWER of the two circle-circle intersections (largest y).
//
// A "lengths" candidate is { thighFront, thighRear, calfFront, calfRear, hipSpacing }.

import { Circle, degToRad, radToDeg } from './helpers.js';

// Knee + foot for a single thigh-angle pair, or null when the calves can't close
// (that (aFront,aRear) is outside the leg's reachable set).
export function footFK(L, aFrontDeg, aRearDeg) {
    const aF = degToRad(aFrontDeg - 90);
    const aR = degToRad(aRearDeg - 90);

    const kneeF = { x: -L.hipSpacing / 2 - L.thighFront * Math.sin(aF),
                    y: L.thighFront * Math.cos(aF) };
    const kneeR = { x:  L.hipSpacing / 2 + L.thighRear * Math.sin(aR),
                    y: L.thighRear * Math.cos(aR) };

    const inter = Circle.getCircleIntersection(
        new Circle(kneeF.x, kneeF.y, L.calfFront),
        new Circle(kneeR.x, kneeR.y, L.calfRear));
    if (!inter) return null;

    const foot = inter[0].y > inter[1].y ? inter[0] : inter[1];  // lower (largest y)
    return { kneeF, kneeR, foot,
             hipF: { x: -L.hipSpacing / 2, y: 0 }, hipR: { x: L.hipSpacing / 2, y: 0 } };
}

// Inverse kinematics: given a desired foot point, solve the linkage that puts the
// foot there (used by the click-drag interaction). Each knee is found independently
// as a circle–circle intersection: the front knee lies on circle(hipF, thighFront)
// AND circle(foot, calfFront); likewise the rear knee. Of the two intersections we
// take the assembly mode the forward kinematics uses — knees splayed OUTWARD (front
// knee to the left, rear knee to the right), which keeps the calves converging down
// onto the foot. Returns the same shape as footFK plus the thigh angles in degrees,
// or null if the foot is out of reach (the circles don't meet).
export function footIK(L, foot) {
    const hipF = { x: -L.hipSpacing / 2, y: 0 }, hipR = { x: L.hipSpacing / 2, y: 0 };
    const kf = Circle.getCircleIntersection(
        new Circle(hipF.x, hipF.y, L.thighFront), new Circle(foot.x, foot.y, L.calfFront));
    const kr = Circle.getCircleIntersection(
        new Circle(hipR.x, hipR.y, L.thighRear), new Circle(foot.x, foot.y, L.calfRear));
    if (!kf || !kr) return null;

    const kneeF = kf[0].x <= kf[1].x ? kf[0] : kf[1];   // outermost (left)
    const kneeR = kr[0].x >= kr[1].x ? kr[0] : kr[1];   // outermost (right)

    // Invert the FK angle relations (see footFK): for the front thigh
    // kneeF - hipF = thighFront * (-sin aF, cos aF); for the rear, (+sin aR, cos aR).
    const aF = Math.atan2(-(kneeF.x - hipF.x), kneeF.y - hipF.y);
    const aR = Math.atan2(kneeR.x - hipR.x, kneeR.y - hipR.y);
    return { kneeF, kneeR, foot, hipF, hipR,
             aFront: radToDeg(aF) + 90, aRear: radToDeg(aR) + 90 };
}

// Sweep both thigh angles over [minAngle, maxAngle] on an N×N grid and collect the
// reachable foot points. Returns the points plus their world bounding box (and the
// hip span, so the view can keep the torso in frame even for a tiny cloud).
export function sampleWorkspace(L, { minAngle = 91, maxAngle = 179, N = 60 } = {}) {
    const points = [];
    let minX = -L.hipSpacing / 2, maxX = L.hipSpacing / 2, minY = 0, maxY = 0;
    const step = (maxAngle - minAngle) / (N - 1);

    for (let i = 0; i < N; i++) {
        const aF = minAngle + i * step;
        for (let j = 0; j < N; j++) {
            const aR = minAngle + j * step;
            const r = footFK(L, aF, aR);
            if (!r) continue;
            const f = r.foot;
            points.push(f);
            if (f.x < minX) minX = f.x; if (f.x > maxX) maxX = f.x;
            if (f.y < minY) minY = f.y; if (f.y > maxY) maxY = f.y;
        }
    }
    return { points, bounds: { minX, maxX, minY, maxY } };
}
