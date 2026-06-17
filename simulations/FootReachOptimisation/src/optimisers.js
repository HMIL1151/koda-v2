// optimisers.js — four interchangeable search strategies for the reach problem.
//
// A `problem` (built by controls.readProblem) exposes:
//   names[]            unlocked dimension names (the search vector's components)
//   vec0[]             starting values         lower[]/upper[]  box bounds
//   evaluate(vec)      -> full objective result { score, stride, clearance, rect,
//                         points, bounds, centrePose }  (locked dims are baked in)
//
// Every optimiser exposes the SAME interface so main.js can swap them freely:
//   step()      advance the search one tick (updates best + history)
//   best()      -> { vec, result, score }   (drawn each frame)
//   ghosts()    -> [{ vec, result }]         (evolutionary population; else [])
//   status()    -> { label, value }          (algorithm-specific readout)
// plus a `history` array of best-score-per-tick for the convergence sparkline.

import { clamp, minimize } from './helpers.js';

const HIST_CAP = 4000;
function randn() {                          // standard normal (Box-Muller)
    let u = 0, v = 0;
    while (u === 0) u = Math.random();
    while (v === 0) v = Math.random();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}
const clampVec = (vec, lo, hi) => vec.map((x, i) => clamp(x, lo[i], hi[i]));
const randInBox = (lo, hi) => lo.map((l, i) => l + Math.random() * (hi[i] - l));

class Base {
    constructor(problem) {
        this.p = problem;
        this.history = [];
        this.bestRec = null;                // { vec, result, score }
    }
    _recordBest(vec, result) {
        if (!this.bestRec || result.score > this.bestRec.score) {
            this.bestRec = { vec: vec.slice(), result, score: result.score };
        }
        this.history.push(this.bestRec.score);
        if (this.history.length > HIST_CAP) this.history.shift();
    }
    best()    { return this.bestRec; }
    ghosts()  { return []; }
}

// --- Simulated annealing -------------------------------------------------------
class SimAnneal extends Base {
    constructor(problem, { T0 = 900, cooling = 0.997, stepFrac = 0.18 } = {}) {
        super(problem);
        this.T0 = T0; this.T = T0; this.cooling = cooling; this.stepFrac = stepFrac;
        const r = problem.evaluate(problem.vec0);
        this.cur = { vec: problem.vec0.slice(), result: r };
        this.span = problem.upper.map((u, i) => u - problem.lower[i]);
        this._recordBest(this.cur.vec, r);
    }
    step() {
        const p = this.p, frac = Math.max(0.04, this.T / this.T0);
        const cand = this.cur.vec.map((x, i) => x + randn() * this.stepFrac * this.span[i] * frac);
        const vec = clampVec(cand, p.lower, p.upper);
        const res = p.evaluate(vec);
        const dE = res.score - this.cur.result.score;        // maximising
        if (dE >= 0 || Math.random() < Math.exp(dE / Math.max(1e-6, this.T))) {
            this.cur = { vec, result: res };
        }
        this.T *= this.cooling;
        this._recordBest(this.cur.vec, this.cur.result);
    }
    current() { return this.cur.result; }
    status()  { return { label: 'Temp', value: this.T.toFixed(1) }; }
}

// --- Evolutionary / swarm ------------------------------------------------------
class Evolutionary extends Base {
    constructor(problem, { popSize = 24, eliteFrac = 0.34, mutFrac = 0.16, mutDecay = 0.985 } = {}) {
        super(problem);
        this.popSize = popSize;
        this.eliteCount = Math.max(2, Math.round(popSize * eliteFrac));
        this.mutFrac = mutFrac; this.mutDecay = mutDecay; this.gen = 0;
        this.span = problem.upper.map((u, i) => u - problem.lower[i]);
        // Seed with the start vector plus random individuals across the box.
        this.pop = [{ vec: problem.vec0.slice() }];
        for (let k = 1; k < popSize; k++) this.pop.push({ vec: randInBox(problem.lower, problem.upper) });
        for (const ind of this.pop) ind.result = problem.evaluate(ind.vec);
        this._sortAndRecord();
    }
    _sortAndRecord() {
        this.pop.sort((a, b) => b.result.score - a.result.score);
        this._recordBest(this.pop[0].vec, this.pop[0].result);
    }
    step() {
        const p = this.p;
        const scale = this.mutFrac * Math.pow(this.mutDecay, this.gen);
        const elite = this.pop.slice(0, this.eliteCount);
        const next = elite.map((e) => ({ vec: e.vec.slice(), result: e.result }));  // keep elites
        while (next.length < this.popSize) {
            const parent = elite[Math.floor(Math.random() * elite.length)];
            const vec = clampVec(
                parent.vec.map((x, i) => x + randn() * scale * this.span[i]), p.lower, p.upper);
            next.push({ vec, result: p.evaluate(vec) });
        }
        this.pop = next;
        this.gen++;
        this._sortAndRecord();
    }
    ghosts() { return this.pop; }
    status() { return { label: 'Gen', value: String(this.gen) }; }
}

// --- Hill-climbing (coordinate ascent) ----------------------------------------
class HillClimb extends Base {
    constructor(problem, { stepFrac = 0.12, shrink = 0.5, minFrac = 0.0015 } = {}) {
        super(problem);
        this.span = problem.upper.map((u, i) => u - problem.lower[i]);
        this.stepSize = this.span.map((s) => s * stepFrac);
        this.minStep = this.span.map((s) => s * minFrac);
        this.shrink = shrink;
        const r = problem.evaluate(problem.vec0);
        this.cur = { vec: problem.vec0.slice(), result: r };
        this._recordBest(this.cur.vec, r);
    }
    step() {                                // one full coordinate sweep per tick
        const p = this.p;
        let improved = false;
        for (let i = 0; i < this.cur.vec.length; i++) {
            for (const sign of [+1, -1]) {
                const vec = this.cur.vec.slice();
                vec[i] = clamp(vec[i] + sign * this.stepSize[i], p.lower[i], p.upper[i]);
                if (vec[i] === this.cur.vec[i]) continue;
                const res = p.evaluate(vec);
                if (res.score > this.cur.result.score) { this.cur = { vec, result: res }; improved = true; }
            }
        }
        if (!improved) {                    // local optimum at this scale — refine
            for (let i = 0; i < this.stepSize.length; i++) {
                this.stepSize[i] = Math.max(this.minStep[i], this.stepSize[i] * this.shrink);
            }
        }
        this._recordBest(this.cur.vec, this.cur.result);
    }
    current() { return this.cur.result; }
    status()  {
        const avg = this.stepSize.reduce((a, b) => a + b, 0) / this.stepSize.length;
        return { label: 'Step', value: avg.toFixed(2) + ' mm' };
    }
}

// --- Gradient ascent (reuses helpers.minimize on the negated score) -----------
class GradientAscent extends Base {
    constructor(problem, { itersPerTick = 3 } = {}) {
        super(problem);
        this.iters = itersPerTick; this.tick = 0;
        this.vec = problem.vec0.slice();
        this.cur = problem.evaluate(this.vec);
        this._recordBest(this.vec, this.cur);
    }
    step() {
        const p = this.p;
        const fn = (v) => -p.evaluate(v).score;          // minimise the negated score
        this.vec = minimize(fn, this.vec,
            { iterations: this.iters, lower: p.lower, upper: p.upper, trustRadius: 5, eps: 0.4 });
        this.cur = p.evaluate(this.vec);
        this.tick++;
        this._recordBest(this.vec, this.cur);
    }
    current() { return this.cur; }
    status()  { return { label: 'Iter', value: String(this.tick * this.iters) }; }
}

const REGISTRY = {
    anneal:      SimAnneal,
    evolution:   Evolutionary,
    hillclimb:   HillClimb,
    gradient:    GradientAscent,
};

export const OPTIMISER_NAMES = {
    anneal:    'Simulated annealing',
    evolution: 'Evolutionary / swarm',
    hillclimb: 'Hill-climbing',
    gradient:  'Gradient ascent',
};

export function makeOptimiser(name, problem, params = {}) {
    const Cls = REGISTRY[name];
    if (!Cls) throw new Error(`Unknown optimiser "${name}"`);
    return new Cls(problem, params);
}
