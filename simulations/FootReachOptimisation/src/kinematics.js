// kinematics.js — single-leg 5-bar forward kinematics, generalised so the front
// and rear thighs and calves can each have their own length (the TorsoAngle Leg
// class hardcodes one thigh + one calf length per leg).
//
// Frame: +x right, +y DOWN. Two hip pivots sit on the torso at x = ±hipSpacing/2,
// y = 0. Each thigh swings down to a knee; the two calves close on the foot, which
// is the LOWER of the two circle-circle intersections (largest y).
//
// A "lengths" candidate is { thighFront, thighRear, calfFront, calfRear, hipSpacing }.

import { Circle, degToRad } from './helpers.js';

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
