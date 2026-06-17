// helpers.js — math / geometry / vector / colour / minimiser utilities.
// Copied (and trimmed) from ../../TorsoAngle/src/helpers.js so this sim stays
// self-contained, matching the one-folder-per-sim convention.

export function radToDeg(rad) { return rad * (180 / Math.PI); }
export function degToRad(deg) { return deg * (Math.PI / 180); }

export function clamp(v, lo, hi) { return v < lo ? lo : (v > hi ? hi : v); }

export class Circle {
    constructor(x, y, r) { this.x = x; this.y = y; this.r = r; }

    // Two intersection points of two circles, or null if they don't meet
    // (separate, or one wholly inside the other).
    static getCircleIntersection(circle1, circle2) {
        const d = Math.sqrt((circle2.x - circle1.x) ** 2 + (circle2.y - circle1.y) ** 2);
        if (d > circle1.r + circle2.r || d < Math.abs(circle1.r - circle2.r) || d === 0) {
            return null;
        }
        const a = (circle1.r ** 2 - circle2.r ** 2 + d ** 2) / (2 * d);
        const hSq = circle1.r ** 2 - a ** 2;
        if (hSq < 0) return null;
        const h = Math.sqrt(hSq);
        const x2 = circle1.x + a * (circle2.x - circle1.x) / d;
        const y2 = circle1.y + a * (circle2.y - circle1.y) / d;
        return [
            { x: x2 + h * (circle2.y - circle1.y) / d, y: y2 - h * (circle2.x - circle1.x) / d },
            { x: x2 - h * (circle2.y - circle1.y) / d, y: y2 + h * (circle2.x - circle1.x) / d },
        ];
    }
}

// --- colour helpers ---
export function lerpColor(a, b, t) {
    const u = clamp(t, 0, 1);
    return [Math.round(a[0] + (b[0] - a[0]) * u),
            Math.round(a[1] + (b[1] - a[1]) * u),
            Math.round(a[2] + (b[2] - a[2]) * u)];
}

// --- small 2D vector helpers ({x, y}) ---
export function add(a, b)  { return { x: a.x + b.x, y: a.y + b.y }; }
export function sub(a, b)  { return { x: a.x - b.x, y: a.y - b.y }; }
export function scale(a, s){ return { x: a.x * s,   y: a.y * s   }; }
export function len(a)     { return Math.sqrt(a.x * a.x + a.y * a.y); }

// Generic box-constrained minimiser: gradient descent with a numerical (central
// difference) gradient, a trust-region step cap, and a backtracking line search.
// `x0` is an array; `fn` maps an array -> scalar. Used for gradient ascent on the
// reach objective (the caller negates the score). Running it with a small
// `iterations` and warm-restarting each tick lets the search animate live.
export function minimize(fn, x0, { iterations = 500, tol = 1e-7, eps = 1e-3,
                                   trustRadius = 6, lower = null, upper = null } = {}) {
    const clip = (v) => {
        if (!lower && !upper) return v;
        return v.map((q, i) => {
            if (lower && q < lower[i]) return lower[i];
            if (upper && q > upper[i]) return upper[i];
            return q;
        });
    };

    let x = clip(x0.slice());
    let fx = fn(x);

    for (let iter = 0; iter < iterations; iter++) {
        const grad = new Array(x.length);
        let gradNorm = 0;
        for (let i = 0; i < x.length; i++) {
            const xi = x[i];
            x[i] = xi + eps; const fPlus = fn(x);
            x[i] = xi - eps; const fMinus = fn(x);
            x[i] = xi;
            grad[i] = (fPlus - fMinus) / (2 * eps);
            gradNorm += grad[i] * grad[i];
        }
        gradNorm = Math.sqrt(gradNorm);
        if (gradNorm < tol) break;

        let alpha = trustRadius / gradNorm;
        let xNext = x.slice();
        let improved = false;
        for (let ls = 0; ls < 40; ls++) {
            for (let i = 0; i < x.length; i++) xNext[i] = x[i] - alpha * grad[i];
            xNext = clip(xNext);
            const fNext = fn(xNext);
            if (fNext < fx - 1e-6 * alpha * gradNorm * gradNorm) {
                x = xNext.slice(); fx = fNext; improved = true; break;
            }
            alpha *= 0.5;
        }
        if (!improved) break;
    }
    return x;
}
