export function radToDeg(rad) {
  return rad * (180 / Math.PI);
}

export function degToRad(deg) {
    return deg * (Math.PI / 180);
}

export class Circle {
    constructor(x, y, r) {
        this.x = x;
        this.y = y;
        this.r = r;
    }

    static getCircleIntersection(circle1, circle2) {
        let d = Math.sqrt(Math.pow(circle2.x - circle1.x, 2) + Math.pow(circle2.y - circle1.y, 2));
        if (d > circle1.r + circle2.r || d < Math.abs(circle1.r - circle2.r)) {
            return null; // No intersection
        }

        let a = (Math.pow(circle1.r, 2) - Math.pow(circle2.r, 2) + Math.pow(d, 2)) / (2 * d);
        let h = Math.sqrt(Math.pow(circle1.r, 2) - Math.pow(a, 2));

        let x2 = circle1.x + a * (circle2.x - circle1.x) / d;
        let y2 = circle1.y + a * (circle2.y - circle1.y) / d;

        let intersection1 = { x: x2 + h * (circle2.y - circle1.y) / d, y: y2 - h * (circle2.x - circle1.x) / d };
        let intersection2 = { x: x2 - h * (circle2.y - circle1.y) / d, y: y2 + h * (circle2.x - circle1.x) / d };

        return [intersection1, intersection2];
    }
}

export function calculateAngleBetweenPoints(pointA, pointB) {
    let deltaY = pointB.y - pointA.y;
    let deltaX = pointB.x - pointA.x;
    let angleRad = Math.atan2(deltaY, deltaX);
    return angleRad;
}

// --- colour helpers ---
// Linear interpolate between two [r,g,b] colours, t in [0,1].
export function lerpColor(a, b, t) {
    const u = Math.max(0, Math.min(1, t));
    return [Math.round(a[0] + (b[0] - a[0]) * u),
            Math.round(a[1] + (b[1] - a[1]) * u),
            Math.round(a[2] + (b[2] - a[2]) * u)];
}

// Map a spring compression to a green -> amber -> red gradient across the travel
// band [minComp, maxComp]. Returns a CSS rgb() string.
export function compressionColor(comp, minComp, maxComp) {
    const span = Math.max(1e-6, maxComp - minComp);
    const t = (comp - minComp) / span;          // 0 at min travel, 1 at max travel
    const green = [46, 204, 113], amber = [230, 162, 60], red = [231, 76, 60];
    const rgb = t < 0.5 ? lerpColor(green, amber, t * 2) : lerpColor(amber, red, (t - 0.5) * 2);
    return `rgb(${rgb[0]}, ${rgb[1]}, ${rgb[2]})`;
}

// --- small 2D vector helpers ({x, y}) ---
export function add(a, b)  { return { x: a.x + b.x, y: a.y + b.y }; }
export function sub(a, b)  { return { x: a.x - b.x, y: a.y - b.y }; }
export function scale(a, s){ return { x: a.x * s,   y: a.y * s   }; }
export function len(a)     { return Math.sqrt(a.x * a.x + a.y * a.y); }

// Rotate a vector by angleRad (screen convention: +x right, +y down).
export function rotate(a, angleRad) {
    const c = Math.cos(angleRad), s = Math.sin(angleRad);
    return { x: a.x * c - a.y * s, y: a.x * s + a.y * c };
}

// Generic unconstrained minimiser: gradient descent with a numerical gradient,
// a trust-region step cap, and backtracking line search. `x0` is an array; `fn`
// maps an array -> scalar. The trust region keeps each step bounded, which is
// essential here because the spring potential is push-only and therefore
// unbounded below if the optimiser leaps out of the physical basin -- we want
// the *local* equilibrium near the warm start, not the runaway global minimum.
export function minimize(fn, x0, { iterations = 500, tol = 1e-7, eps = 1e-4,
                                   trustRadius = 8, lower = null, upper = null } = {}) {
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
        // Numerical gradient (central difference)
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

        // First trial moves exactly `trustRadius` along the steepest-descent
        // direction; backtracking then shrinks it until Armijo is satisfied.
        // Bounds are enforced by clipping each trial point into the box.
        let alpha = trustRadius / gradNorm;
        let xNext = x.slice();
        let improved = false;
        for (let ls = 0; ls < 40; ls++) {
            for (let i = 0; i < x.length; i++) xNext[i] = x[i] - alpha * grad[i];
            xNext = clip(xNext);
            const fNext = fn(xNext);
            if (fNext < fx - 1e-6 * alpha * gradNorm * gradNorm) {
                x = xNext.slice();
                fx = fNext;
                improved = true;
                break;
            }
            alpha *= 0.5;
        }
        if (!improved) break; // converged / stuck
    }

    return x;
}