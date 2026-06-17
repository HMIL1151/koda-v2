// objective.js — the "stride x clearance" reach metric.
//
// Sweep both thigh angles to get the foot's reachable workspace, rasterise it into
// an occupancy mask, then find the largest axis-aligned rectangle that fits wholly
// inside. Its width = stride, height = clearance, and score = stride * clearance
// (mm^2). The mask is filled by rasterising the angle-grid QUADS (not just marking
// sample points), so the region is solid and genuine concavities/holes — where the
// linkage can't reach — are preserved.

import { footFK } from './kinematics.js';

// Mark every mask cell whose centre lies inside triangle ABC (world coords).
function fillTriangle(mask, cols, rows, originX, originY, cell, A, B, C) {
    const minX = Math.min(A.x, B.x, C.x), maxX = Math.max(A.x, B.x, C.x);
    const minY = Math.min(A.y, B.y, C.y), maxY = Math.max(A.y, B.y, C.y);
    let c0 = Math.floor((minX - originX) / cell), c1 = Math.floor((maxX - originX) / cell);
    let r0 = Math.floor((minY - originY) / cell), r1 = Math.floor((maxY - originY) / cell);
    c0 = Math.max(0, c0); c1 = Math.min(cols - 1, c1);
    r0 = Math.max(0, r0); r1 = Math.min(rows - 1, r1);

    // Edge functions (sign test). Allow either winding.
    const d1 = (px, py) => (px - B.x) * (A.y - B.y) - (A.x - B.x) * (py - B.y);
    const d2 = (px, py) => (px - C.x) * (B.y - C.y) - (B.x - C.x) * (py - C.y);
    const d3 = (px, py) => (px - A.x) * (C.y - A.y) - (C.x - A.x) * (py - A.y);

    for (let r = r0; r <= r1; r++) {
        const py = originY + (r + 0.5) * cell;
        for (let c = c0; c <= c1; c++) {
            const px = originX + (c + 0.5) * cell;
            const s1 = d1(px, py), s2 = d2(px, py), s3 = d3(px, py);
            const hasNeg = s1 < 0 || s2 < 0 || s3 < 0;
            const hasPos = s1 > 0 || s2 > 0 || s3 > 0;
            if (!(hasNeg && hasPos)) mask[r * cols + c] = 1;
        }
    }
}

// Largest all-ones axis-aligned rectangle in a boolean mask (histogram method).
// Returns cell indices {top,bottom,left,right,area} (inclusive), area in cells.
function maxRectangle(mask, cols, rows) {
    const heights = new Array(cols).fill(0);
    let best = { area: 0, top: 0, bottom: 0, left: 0, right: 0 };
    const stack = [];
    for (let r = 0; r < rows; r++) {
        for (let c = 0; c < cols; c++) heights[c] = mask[r * cols + c] ? heights[c] + 1 : 0;
        stack.length = 0;
        for (let c = 0; c <= cols; c++) {
            const h = c === cols ? 0 : heights[c];
            while (stack.length && h < heights[stack[stack.length - 1]]) {
                const top = stack.pop();
                const height = heights[top];
                const leftCol = stack.length ? stack[stack.length - 1] + 1 : 0;
                const rightCol = c - 1;
                const area = height * (rightCol - leftCol + 1);
                if (area > best.area) {
                    best = { area, top: r - height + 1, bottom: r, left: leftCol, right: rightCol };
                }
            }
            stack.push(c);
        }
    }
    return best;
}

// Evaluate a lengths candidate. opts: { minAngle, maxAngle, N, cellMm }.
// Returns { score, stride, clearance, rect, points, bounds, centrePose }.
export function evaluate(L, opts = {}) {
    const { minAngle = 91, maxAngle = 179, N = 60, cellMm = 2 } = opts;
    const step = (maxAngle - minAngle) / (N - 1);

    // Foot point per (i=front, j=rear) angle sample, or null if infeasible.
    const fk = new Array(N);
    const points = [];
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    for (let i = 0; i < N; i++) {
        fk[i] = new Array(N);
        const aF = minAngle + i * step;
        for (let j = 0; j < N; j++) {
            const aR = minAngle + j * step;
            const r = footFK(L, aF, aR);
            fk[i][j] = r ? r.foot : null;
            if (r) {
                const f = r.foot;
                points.push(f);
                if (f.x < minX) minX = f.x; if (f.x > maxX) maxX = f.x;
                if (f.y < minY) minY = f.y; if (f.y > maxY) maxY = f.y;
            }
        }
    }

    if (points.length === 0) {
        const hb = { minX: -L.hipSpacing, maxX: L.hipSpacing, minY: -10, maxY: 10 };
        return { score: 0, stride: 0, clearance: 0, rect: null, points: [], bounds: hb, centrePose: null };
    }

    const bounds = { minX, maxX, minY, maxY };
    const cols = Math.max(1, Math.ceil((maxX - minX) / cellMm));
    const rows = Math.max(1, Math.ceil((maxY - minY) / cellMm));
    const mask = new Uint8Array(cols * rows);

    // Rasterise every quad whose four corners are all feasible (two triangles).
    for (let i = 0; i < N - 1; i++) {
        for (let j = 0; j < N - 1; j++) {
            const a = fk[i][j], b = fk[i + 1][j], c = fk[i + 1][j + 1], d = fk[i][j + 1];
            if (a && b && c && d) {
                fillTriangle(mask, cols, rows, minX, minY, cellMm, a, b, c);
                fillTriangle(mask, cols, rows, minX, minY, cellMm, a, c, d);
            }
        }
    }

    const mr = maxRectangle(mask, cols, rows);
    if (mr.area === 0) {
        return { score: 0, stride: 0, clearance: 0, rect: null, points, bounds, centrePose: null };
    }

    const rect = {
        minX: minX + mr.left * cellMm,
        maxX: minX + (mr.right + 1) * cellMm,
        minY: minY + mr.top * cellMm,
        maxY: minY + (mr.bottom + 1) * cellMm,
    };
    const stride = rect.maxX - rect.minX;
    const clearance = rect.maxY - rect.minY;
    const score = stride * clearance;

    // Representative pose: the feasible sample whose foot is nearest the rect centre,
    // so the renderer can draw the linkage actually reaching into the box.
    const cx = (rect.minX + rect.maxX) / 2, cy = (rect.minY + rect.maxY) / 2;
    let centrePose = null, bestD = Infinity;
    for (let i = 0; i < N; i++) {
        for (let j = 0; j < N; j++) {
            const f = fk[i][j];
            if (!f) continue;
            const dd = (f.x - cx) ** 2 + (f.y - cy) ** 2;
            if (dd < bestD) { bestD = dd; centrePose = { aFront: minAngle + i * step, aRear: minAngle + j * step }; }
        }
    }

    return { score, stride, clearance, rect, points, bounds, centrePose };
}
