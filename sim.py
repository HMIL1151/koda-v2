"""
Robot Dog 2D Simulator — 5-Bar Linkage with Series Springs
==========================================================
Each leg is a 5-bar linkage:
  - Two hip joints fixed in torso frame (separated by hip_spread)
  - Rigid thighs from each hip → knee joints
  - Compliant calves (series springs) from each knee → shared foot

Angle convention: thigh angles measured from torso-local downward (-y body axis).
Solver finds torso (x, y, pitch) at static equilibrium with feet fixed.
"""

import tkinter as tk
from tkinter import ttk
import numpy as np
from scipy.optimize import fsolve
import traceback
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.patches as mpatches

# ─────────────────────────────────────────────
#  DEFAULT PARAMETERS
# ─────────────────────────────────────────────
DEFAULTS = dict(
    body_length      = 0.40,    # m — distance between front/rear hip centres
    body_mass        = 12.0,    # kg
    g                = 9.81,    # m/s²
    thigh_length     = 0.15,    # m
    calf_free_len    = 0.20,    # m — unloaded calf spring length
    calf_spring_k    = 2500.0,  # N/m
    front_thigh1_deg = -20.0,   # deg from torso-down, front leg hip-1
    front_thigh2_deg =  20.0,   # deg from torso-down, front leg hip-2
    rear_thigh1_deg  = -20.0,   # deg from torso-down, rear leg hip-1
    rear_thigh2_deg  =  20.0,   # deg from torso-down, rear leg hip-2
    hip_spread       = 0.04,    # m — half-separation of two hip joints per leg
    torso_x          = 0.20,    # m — initial guess
    torso_y          = 0.30,    # m — initial guess
    torso_theta_deg  = 0.0,     # deg pitch — initial guess
    solver_tol       = 1e-9,
    solver_maxiter   = 2000,
)


# ─────────────────────────────────────────────
#  KINEMATICS
# ─────────────────────────────────────────────

def rot2d(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def hip_world_positions(torso_pose, body_length, hip_spread):
    """
    Four hip joints in world frame.
    Torso-local layout (x=forward, y=up):
      front hips: (+L/2 ∓ hs, 0)
      rear  hips: (-L/2 ∓ hs, 0)
    """
    tx, ty, tth = torso_pose
    R = rot2d(tth)
    c = np.array([tx, ty])

    def w(lx, ly):
        return c + R @ np.array([lx, ly])

    hl = body_length / 2
    fh1 = w( hl - hip_spread, 0.0)
    fh2 = w( hl + hip_spread, 0.0)
    rh1 = w(-hl - hip_spread, 0.0)
    rh2 = w(-hl + hip_spread, 0.0)
    return fh1, fh2, rh1, rh2


def knee_world(hip, thigh_len, world_angle):
    return hip + thigh_len * np.array([np.cos(world_angle), np.sin(world_angle)])


def world_thigh_angle(torso_pitch, local_deg):
    """Convert local thigh angle (from torso-down) to world frame."""
    return torso_pitch - np.pi / 2 + np.radians(local_deg)


def calf_unit(knee, foot):
    d = foot - knee
    n = np.linalg.norm(d)
    return d / n if n > 1e-12 else np.array([0.0, -1.0])


def spring_force_scalar(knee, foot, free_len, k):
    """Signed spring force: positive = tension (longer than free), negative = compression."""
    return k * (np.linalg.norm(foot - knee) - free_len)


# ─────────────────────────────────────────────
#  EQUILIBRIUM RESIDUAL
# ─────────────────────────────────────────────

def compute_residual(torso_pose, params, foot_front, foot_rear):
    """
    Static equilibrium: ΣF = 0, Στ_CoM = 0.
    Returns 3-vector [Fx, Fy, Tau].

    Forces on torso:
      - Gravity at CoM
      - Spring reaction forces via thighs at each hip joint
        (thigh is rigid → knee spring force transmitted to hip)
    """
    tx, ty, tth = torso_pose
    p = params
    com = np.array([tx, ty])

    fh1, fh2, rh1, rh2 = hip_world_positions(torso_pose, p['body_length'], p['hip_spread'])

    fk1 = knee_world(fh1, p['thigh_length'], world_thigh_angle(tth, p['front_thigh1_deg']))
    fk2 = knee_world(fh2, p['thigh_length'], world_thigh_angle(tth, p['front_thigh2_deg']))
    rk1 = knee_world(rh1, p['thigh_length'], world_thigh_angle(tth, p['rear_thigh1_deg']))
    rk2 = knee_world(rh2, p['thigh_length'], world_thigh_angle(tth, p['rear_thigh2_deg']))

    Fx = Fy = Tau = 0.0

    def add_calf(knee, foot, hip):
        nonlocal Fx, Fy, Tau
        fs = spring_force_scalar(knee, foot, p['calf_free_len'], p['calf_spring_k'])
        u  = calf_unit(knee, foot)
        # Force on torso at hip = spring force vector (thigh transmits it)
        Fvec = fs * u
        Fx  += Fvec[0]
        Fy  += Fvec[1]
        r    = hip - com          # moment arm from CoM to point of application
        Tau += r[0] * Fvec[1] - r[1] * Fvec[0]

    add_calf(fk1, foot_front, fh1)
    add_calf(fk2, foot_front, fh2)
    add_calf(rk1, foot_rear,  rh1)
    add_calf(rk2, foot_rear,  rh2)

    # Gravity
    Fy -= p['body_mass'] * p['g']
    # gravity acts at CoM so no torque contribution

    return np.array([Fx, Fy, Tau])


# ─────────────────────────────────────────────
#  FULL STATE SNAPSHOT
# ─────────────────────────────────────────────

def compute_state(torso_pose, params, foot_front, foot_rear):
    tx, ty, tth = torso_pose
    p = params

    fh1, fh2, rh1, rh2 = hip_world_positions(torso_pose, p['body_length'], p['hip_spread'])
    fk1 = knee_world(fh1, p['thigh_length'], world_thigh_angle(tth, p['front_thigh1_deg']))
    fk2 = knee_world(fh2, p['thigh_length'], world_thigh_angle(tth, p['front_thigh2_deg']))
    rk1 = knee_world(rh1, p['thigh_length'], world_thigh_angle(tth, p['rear_thigh1_deg']))
    rk2 = knee_world(rh2, p['thigh_length'], world_thigh_angle(tth, p['rear_thigh2_deg']))

    def info(knee, foot):
        l = np.linalg.norm(foot - knee)
        f = spring_force_scalar(knee, foot, p['calf_free_len'], p['calf_spring_k'])
        return l, f

    fl1, ff1 = info(fk1, foot_front)
    fl2, ff2 = info(fk2, foot_front)
    rl1, rf1 = info(rk1, foot_rear)
    rl2, rf2 = info(rk2, foot_rear)

    return dict(
        torso_pose=np.array(torso_pose),
        fh1=fh1, fh2=fh2, rh1=rh1, rh2=rh2,
        fk1=fk1, fk2=fk2, rk1=rk1, rk2=rk2,
        foot_front=np.array(foot_front),
        foot_rear=np.array(foot_rear),
        front_calf1_len=fl1, front_calf1_force=ff1,
        front_calf2_len=fl2, front_calf2_force=ff2,
        rear_calf1_len=rl1,  rear_calf1_force=rf1,
        rear_calf2_len=rl2,  rear_calf2_force=rf2,
    )


# ─────────────────────────────────────────────
#  SOLVER
# ─────────────────────────────────────────────

def solve_equilibrium(params, foot_front, foot_rear, x0):
    """
    Returns: (torso_pose, converged, history, final_residual, error_msg)
    history: list of (pose, residual_norm) per iteration
    """
    history = []

    def residual_logged(x):
        r = compute_residual(x, params, foot_front, foot_rear)
        history.append((x.copy(), float(np.linalg.norm(r))))
        return r

    x_sol   = x0.copy()
    converged      = False
    final_residual = float('inf')
    error_msg      = ""

    try:
        # Sanity check inputs
        for name, arr in [("foot_front", foot_front), ("foot_rear", foot_rear), ("x0", x0)]:
            if not np.all(np.isfinite(arr)):
                raise ValueError(f"Non-finite values in {name}: {arr}")

        result = fsolve(
            residual_logged, x0,
            full_output=True,
            xtol=params['solver_tol'],
            maxfev=int(params['solver_maxiter']),
        )
        x_sol, _infodict, ier, mesg = result
        converged      = (ier == 1)
        final_residual = float(np.linalg.norm(
            compute_residual(x_sol, params, foot_front, foot_rear)))
        if not converged:
            error_msg = mesg

    except Exception:
        error_msg = traceback.format_exc()
        print("[Solver exception]\n", error_msg)

    return x_sol, converged, history, final_residual, error_msg


# ─────────────────────────────────────────────
#  FOOT INITIALISATION HELPER
# ─────────────────────────────────────────────

def compute_initial_feet(params):
    """
    Place feet below the midpoint of each leg's knee pair,
    calf_free_len below the knee midpoint, clamped to y >= 0.
    """
    p = params
    pose0 = [p['torso_x'], p['torso_y'], np.radians(p['torso_theta_deg'])]
    tth = pose0[2]

    fh1, fh2, rh1, rh2 = hip_world_positions(pose0, p['body_length'], p['hip_spread'])
    fk1 = knee_world(fh1, p['thigh_length'], world_thigh_angle(tth, p['front_thigh1_deg']))
    fk2 = knee_world(fh2, p['thigh_length'], world_thigh_angle(tth, p['front_thigh2_deg']))
    rk1 = knee_world(rh1, p['thigh_length'], world_thigh_angle(tth, p['rear_thigh1_deg']))
    rk2 = knee_world(rh2, p['thigh_length'], world_thigh_angle(tth, p['rear_thigh2_deg']))

    fmid = (fk1 + fk2) / 2
    rmid = (rk1 + rk2) / 2

    foot_front = np.array([fmid[0], max(0.0, fmid[1] - p['calf_free_len'])])
    foot_rear  = np.array([rmid[0], max(0.0, rmid[1]  - p['calf_free_len'])])
    return foot_front, foot_rear


# ─────────────────────────────────────────────────────────────────────────────
#  G U I
# ─────────────────────────────────────────────────────────────────────────────

DARK   = "#0d1117"
PANEL  = "#161b22"
BORDER = "#2d3748"
GREEN  = "#58d68d"
BLUE   = "#5dade2"
AMBER  = "#f0b27a"
RED    = "#e74c3c"
MUTED  = "#aab7b8"
MONO   = "Courier New"


class RobotDogApp(tk.Tk):

    def __init__(self):
        super().__init__()
        self.title("Robot Dog 5-Bar Linkage Simulator")
        self.configure(bg=DARK)
        self.resizable(True, True)
        self.geometry("1200x760")

        self.params     = dict(DEFAULTS)
        self._sliders   = {}          # key -> tk.DoubleVar
        self._foot_front = None
        self._foot_rear  = None
        self.solution   = None
        self.history    = []
        self._converged = False
        self._final_res = float('inf')
        self._error_msg = ""

        self._build_ui()

        # Initialise feet then solve
        self._foot_front, self._foot_rear = compute_initial_feet(self.params)
        self._run_simulation()

    # ──────────────────────────────────────────────────────────────────────
    #  UI CONSTRUCTION
    # ──────────────────────────────────────────────────────────────────────

    def _build_ui(self):
        # Top bar
        top = tk.Frame(self, bg=DARK, pady=5)
        top.pack(fill=tk.X, padx=12)

        tk.Label(top, text="🐾  ROBOT DOG 5-BAR LINKAGE SIMULATOR",
                 font=(MONO, 13, "bold"), fg=GREEN, bg=DARK).pack(side=tk.LEFT)

        tk.Button(top, text="▶  Solve",
                  command=self._run_simulation,
                  bg="#1a2a3a", fg=BLUE,
                  activebackground="#2e86c1", activeforeground="white",
                  relief=tk.FLAT, padx=12, font=(MONO, 9, "bold")
                  ).pack(side=tk.RIGHT, padx=4)

        tk.Button(top, text="⟳  Reset Feet + Solve",
                  command=self._reset_feet_and_solve,
                  bg="#1e3a2f", fg=GREEN,
                  activebackground="#27ae60", activeforeground="white",
                  relief=tk.FLAT, padx=12, font=(MONO, 9, "bold")
                  ).pack(side=tk.RIGHT, padx=4)

        # Main pane
        pane = tk.PanedWindow(self, orient=tk.HORIZONTAL,
                              bg=DARK, sashwidth=5, sashrelief=tk.FLAT)
        pane.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)

        ctrl = tk.Frame(pane, bg=DARK, width=320)
        pane.add(ctrl, minsize=300)

        plot_frame = tk.Frame(pane, bg=DARK)
        pane.add(plot_frame, minsize=420)

        self._build_controls(ctrl)
        self._build_plots(plot_frame)

    # ── Controls panel ──────────────────────────────────────────────────


    def _build_controls(self, parent):
        inner = tk.Frame(parent, bg=DARK)
        inner.pack(fill=tk.BOTH, expand=True)

        def section(title):
            f = tk.LabelFrame(inner, text=f"  {title}  ",
                              fg=MUTED, bg=PANEL,
                              font=(MONO, 8, "bold"),
                              bd=1, relief=tk.GROOVE, padx=6, pady=3)
            f.pack(fill=tk.X, padx=6, pady=3)
            return f

        # ── Thigh angles
        tf = section("Thigh Angles  (° from body-down)")
        for key, lbl in [
            ("front_thigh1_deg", "Front Hip-1"),
            ("front_thigh2_deg", "Front Hip-2"),
            ("rear_thigh1_deg",  "Rear  Hip-1"),
            ("rear_thigh2_deg",  "Rear  Hip-2"),
        ]:
            self._slider(tf, key, lbl, -180, 180, 0.5, is_length=False)

        # ── Geometry (all lengths in mm for UI)
        gf = section("Geometry")
        for key, lbl, lo, hi, res in [
            ("body_length",   "Body Length (mm)",   150, 400, 1),
            ("thigh_length",  "Thigh Len   (mm)",    20, 200, 1),
            ("calf_free_len", "Calf Free   (mm)",    40, 200, 1),
            ("hip_spread",    "Hip Spread  (mm)",    10, 80, 1),
        ]:
            self._slider(gf, key, lbl, lo, hi, res, is_length=True)

        # ── Physics
        pf = section("Physics")
        for key, lbl, lo, hi, res in [
            ("body_mass",     "Mass  (kg)",    1.0,  50.0, 0.2),
            ("calf_spring_k", "Spring k (N/m)", 200, 8000, 50),
        ]:
            self._slider(pf, key, lbl, lo, hi, res, is_length=False)

        # ── Initial guess
        ig = section("Solver Initial Guess")
        for key, lbl, lo, hi, res in [
            ("torso_x",        "Torso X (m)",   -0.5, 1.5, 0.01),
            ("torso_y",        "Torso Y (m)",    0.05, 0.9, 0.01),
            ("torso_theta_deg","Pitch (°)",      -30,  30,  0.5),
        ]:
            self._slider(ig, key, lbl, lo, hi, res, is_length=False)

        # ── Status line
        self._status_var = tk.StringVar(value="Initialising…")
        sf = tk.Frame(inner, bg=DARK)
        sf.pack(fill=tk.X, padx=6, pady=4)
        tk.Label(sf, textvariable=self._status_var,
                 fg=AMBER, bg=DARK, font=(MONO, 8),
                 wraplength=285, justify=tk.LEFT).pack(anchor=tk.W)

        # ── Numeric results
        self._results_text = tk.Text(
            inner, height=14, width=36,
            bg=PANEL, fg=GREEN, font=(MONO, 8),
            relief=tk.FLAT, state=tk.DISABLED,
            insertbackground="white")
        self._results_text.pack(padx=6, pady=2, fill=tk.X)

        # ── Error box (hidden until needed)
        self._error_text = tk.Text(
            inner, height=5, width=36,
            bg="#2d0a0a", fg="#f1948a", font=(MONO, 7),
            relief=tk.FLAT, state=tk.DISABLED)
        self._error_text.pack(padx=6, pady=2, fill=tk.X)


    def _slider(self, parent, key, label, lo, hi, resolution, is_length=False):
        row = tk.Frame(parent, bg=PANEL)
        row.pack(fill=tk.X, pady=2)

        # Label
        tk.Label(row, text=label, fg=MUTED, bg=PANEL,
                 font=(MONO, 8), width=16, anchor=tk.W).pack(side=tk.LEFT, padx=(0, 2))

        # For length parameters, store in meters internally, show mm in UI
        if is_length:
            val = self.params[key] * 1000
        else:
            val = self.params[key]

        var = tk.DoubleVar(value=val)
        self._sliders[key] = var

        # Frame for slider and entry
        input_frame = tk.Frame(row, bg=PANEL)
        input_frame.pack(side=tk.LEFT, fill=tk.X, expand=True)

        # Slider
        scale = tk.Scale(input_frame, variable=var,
                 from_=lo, to=hi, resolution=resolution,
                 orient=tk.HORIZONTAL, showvalue=False,
                 bg=PANEL, fg=GREEN,
                 troughcolor=DARK, activebackground="#27ae60",
                 highlightthickness=0, bd=0,
                 length=120,
                 command=lambda _: self._on_slider())
        scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4))

        # Numeric entry with border and clear label
        entry_frame = tk.Frame(input_frame, bg=PANEL)
        entry_frame.pack(side=tk.LEFT, padx=(0, 0))
        tk.Label(entry_frame, text="Value:", fg=MUTED, bg=PANEL, font=(MONO, 8)).pack(side=tk.LEFT)
        entry = tk.Entry(entry_frame, width=7, font=(MONO, 8),
                         relief=tk.RIDGE, bd=2, justify='right', bg="#222933", fg=BLUE, insertbackground=BLUE)
        entry.pack(side=tk.LEFT)
        entry.config(highlightbackground=BORDER, highlightcolor=BLUE)
        entry.insert(0, f"{val:.1f}")
        entry.bind('<FocusIn>', lambda e: entry.select_range(0, tk.END))
        entry.bind('<Return>', lambda e: self._on_slider())

        # Validate input
        def validate_entry(P):
            if P == "" or P == "-":
                return True
            try:
                float(P)
                return True
            except Exception:
                return False
        vcmd = (row.register(validate_entry), '%P')
        entry.config(validate='key', validatecommand=vcmd)

        # Keep entry and slider in sync, but only set var if valid
        def on_entry_change(event=None):
            val_str = entry.get()
            try:
                v = float(val_str)
                var.set(v)
            except Exception:
                pass  # Don't update var if not a valid float
        entry.bind('<KeyRelease>', on_entry_change)

        # Update entry when slider moves, but only if entry is not focused
        def on_var_change(*_):
            if entry.focus_get() != entry:
                v = var.get()
                entry.delete(0, tk.END)
                entry.insert(0, f"{v:.1f}")
        var.trace_add('write', lambda *_: on_var_change())

    # ── Plots panel ──────────────────────────────────────────────────────

    def _build_plots(self, parent):
        self.fig = Figure(figsize=(8.5, 7), facecolor=DARK)
        self.fig.subplots_adjust(hspace=0.38, wspace=0.32,
                                 left=0.09, right=0.97,
                                 top=0.95, bottom=0.07)
        gs = self.fig.add_gridspec(2, 2)
        self.ax_robot = self.fig.add_subplot(gs[:, 0])
        self.ax_conv  = self.fig.add_subplot(gs[0, 1])
        self.ax_force = self.fig.add_subplot(gs[1, 1])

        for ax in (self.ax_robot, self.ax_conv, self.ax_force):
            ax.set_facecolor(PANEL)
            for sp in ax.spines.values():
                sp.set_edgecolor(BORDER)
            ax.tick_params(colors=MUTED, labelsize=7)
            ax.xaxis.label.set_color(MUTED)
            ax.yaxis.label.set_color(MUTED)
            ax.title.set_color("#ecf0f1")

        self.ax_robot.set_title("Robot Pose", fontsize=10, fontweight="bold")
        self.ax_conv.set_title("Solver Convergence", fontsize=9, fontweight="bold")
        self.ax_force.set_title("Spring Forces (N)", fontsize=9, fontweight="bold")

        self._canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self._canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # ──────────────────────────────────────────────────────────────────────
    #  CALLBACKS
    # ──────────────────────────────────────────────────────────────────────


    def _on_slider(self):
        """Sync all slider vars → self.params, then re-solve."""
        for key, var in self._sliders.items():
            val = var.get()
            # Convert mm to m for length params
            if key in ("body_length", "thigh_length", "calf_free_len", "hip_spread"):
                self.params[key] = val / 1000.0
            else:
                self.params[key] = val
        self._run_simulation()

    def _reset_feet_and_solve(self):
        for key, var in self._sliders.items():
            self.params[key] = var.get()
        self._foot_front, self._foot_rear = compute_initial_feet(self.params)
        self._run_simulation()

    # ──────────────────────────────────────────────────────────────────────
    #  SIMULATION
    # ──────────────────────────────────────────────────────────────────────

    def _run_simulation(self):
        p = self.params

        if self._foot_front is None or self._foot_rear is None:
            self._foot_front, self._foot_rear = compute_initial_feet(p)

        x0 = np.array([p['torso_x'], p['torso_y'],
                        np.radians(p['torso_theta_deg'])])

        sol, converged, history, final_res, error_msg = solve_equilibrium(
            p, self._foot_front, self._foot_rear, x0)

        self.solution   = sol
        self.history    = history
        self._converged = converged
        self._final_res = final_res
        self._error_msg = error_msg

        state = compute_state(sol, p, self._foot_front, self._foot_rear)
        self._update_plots(state)
        self._update_results(sol, state)
        self._update_status()

    # ──────────────────────────────────────────────────────────────────────
    #  PLOT DRAWING
    # ──────────────────────────────────────────────────────────────────────

    def _update_plots(self, state):
        self._draw_robot(state)
        self._draw_convergence()
        self._draw_forces(state)
        self._canvas.draw()

    def _draw_robot(self, s):
        ax = self.ax_robot
        ax.cla()
        ax.set_facecolor(PANEL)
        ax.set_title("Robot Pose", fontsize=10, fontweight="bold", color="#ecf0f1")
        ax.set_aspect("equal")
        ax.grid(True, color="#1e2a3a", lw=0.5, alpha=0.7)
        for sp in ax.spines.values():
            sp.set_edgecolor(BORDER)
        ax.tick_params(colors=MUTED, labelsize=7)

        p = self.params
        tx, ty, tth = s['torso_pose']
        com = np.array([tx, ty])

        # Ground
        ax.axhline(0, color="#2d4a2d", lw=1.5, zorder=0)
        ax.fill_between([-1, 2], -0.05, 0, color="#1a2e1a", alpha=0.8, zorder=0)

        # Torso rectangle
        R = rot2d(tth)
        hl = p['body_length'] / 2
        corners_local = np.array([[ hl, 0.025],[-hl, 0.025],
                                   [-hl,-0.025],[ hl,-0.025]])
        corners_world = np.array([com + R @ c for c in corners_local])
        from matplotlib.patches import Polygon as MplPoly
        ax.add_patch(MplPoly(corners_world, closed=True,
                             facecolor="#1a3a5c", edgecolor=BLUE, lw=1.8, zorder=5))
        ax.plot(tx, ty, 'D', color="#f39c12", ms=7, zorder=10, label="CoM")

        # Spring colour helper
        def sc(force):
            if force > 5:  return RED
            if force < -5: return BLUE
            return GREEN

        # Draw one leg
        def draw_leg(h1, h2, k1, k2, foot, c_thigh, f1, f2):
            for h, k in [(h1, k1), (h2, k2)]:
                ax.plot([h[0], k[0]], [h[1], k[1]],
                        '-', color=c_thigh, lw=3,
                        solid_capstyle='round', zorder=6)
                ax.plot(*h, 'o', color="#ecf0f1", ms=5, zorder=8)
                ax.plot(*k, 'o', color="#ecf0f1", ms=4, zorder=8)

            for k, col in [(k1, sc(f1)), (k2, sc(f2))]:
                ax.plot([k[0], foot[0]], [k[1], foot[1]],
                        '--', color=col, lw=2.2, zorder=7, dashes=(4, 3))

            ax.plot(*foot, 's', color="#e74c3c", ms=10, zorder=10)
            ax.plot(*foot, 's', color="#f1948a", ms=5,  zorder=11)

        draw_leg(s['fh1'], s['fh2'], s['fk1'], s['fk2'], s['foot_front'],
                 BLUE, s['front_calf1_force'], s['front_calf2_force'])
        draw_leg(s['rh1'], s['rh2'], s['rk1'], s['rk2'], s['foot_rear'],
                 "#a569bd", s['rear_calf1_force'], s['rear_calf2_force'])

        # Gravity arrow
        gscale = 0.018
        gy = ty - p['body_mass'] * p['g'] * gscale
        ax.annotate("", xy=(tx, gy), xytext=(tx, ty),
                    arrowprops=dict(arrowstyle="-|>", color="#f39c12",
                                   lw=1.5, mutation_scale=12))

        # Auto-scale axes
        pts = np.array([s['fh1'], s['fh2'], s['rh1'], s['rh2'],
                        s['fk1'], s['fk2'], s['rk1'], s['rk2'],
                        s['foot_front'], s['foot_rear'], com])
        m = 0.12
        ax.set_xlim(pts[:, 0].min() - m, pts[:, 0].max() + m)
        ax.set_ylim(-0.06, pts[:, 1].max() + m)
        ax.set_xlabel("x (m)", fontsize=8)
        ax.set_ylabel("y (m)", fontsize=8)

        patches = [
            mpatches.Patch(color=BLUE,     label="Front thighs"),
            mpatches.Patch(color="#a569bd",label="Rear thighs"),
            mpatches.Patch(color=RED,      label="Compressed spring"),
            mpatches.Patch(color=BLUE,     label="Extended spring"),
            mpatches.Patch(color=GREEN,    label="Neutral spring"),
        ]
        ax.legend(handles=patches, fontsize=6, loc="upper right",
                  facecolor=DARK, edgecolor=BORDER, labelcolor=MUTED)

    def _draw_convergence(self):
        ax = self.ax_conv
        ax.cla()
        ax.set_facecolor(PANEL)
        ax.set_title("Solver Convergence", fontsize=9, fontweight="bold", color="#ecf0f1")
        ax.tick_params(colors=MUTED, labelsize=7)
        for sp in ax.spines.values():
            sp.set_edgecolor(BORDER)

        history = self.history
        if not history:
            ax.text(0.5, 0.5, "No iterations", transform=ax.transAxes,
                    ha='center', va='center', color=MUTED, fontsize=9)
            return

        norms = [h[1] for h in history]
        iters = list(range(len(norms)))

        ax.semilogy(iters, norms, color=GREEN, lw=1.5)
        ax.fill_between(iters, norms,
                        [min(n for n in norms if n > 0) * 0.5] * len(norms),
                        alpha=0.12, color=GREEN)

        tol = self.params['solver_tol']
        ax.axhline(tol, color=AMBER, lw=1, ls="--",
                   label=f"tol = {tol:.0e}")
        ax.set_xlabel("Iteration", fontsize=8)
        ax.set_ylabel("|Residual|", fontsize=8)
        ax.legend(fontsize=6, facecolor=DARK, edgecolor=BORDER, labelcolor=MUTED)

        label = "✓ CONVERGED" if self._converged else "✗ NOT CONVERGED"
        color = GREEN if self._converged else RED
        ax.text(0.98, 0.97, label, transform=ax.transAxes,
                ha="right", va="top", color=color,
                fontsize=7, fontfamily=MONO, fontweight="bold")

        ax.grid(True, color="#1e2a3a", lw=0.5, alpha=0.7)

    def _draw_forces(self, s):
        ax = self.ax_force
        ax.cla()
        ax.set_facecolor(PANEL)
        ax.set_title("Spring Forces (N)", fontsize=9, fontweight="bold", color="#ecf0f1")
        ax.tick_params(colors=MUTED, labelsize=7)
        for sp in ax.spines.values():
            sp.set_edgecolor(BORDER)

        labels = ["F-Calf1", "F-Calf2", "R-Calf1", "R-Calf2"]
        forces = [s['front_calf1_force'], s['front_calf2_force'],
                  s['rear_calf1_force'],  s['rear_calf2_force']]
        lengths = [s['front_calf1_len'], s['front_calf2_len'],
                   s['rear_calf1_len'],  s['rear_calf2_len']]
        free = self.params['calf_free_len']
        colors = [RED if f > 0 else BLUE for f in forces]

        bars = ax.bar(labels, forces, color=colors, alpha=0.85, width=0.55)
        ax.axhline(0, color=MUTED, lw=0.8)

        ymin = min(forces + [0])
        for bar, force, ln in zip(bars, forces, lengths):
            xc = bar.get_x() + bar.get_width() / 2
            offset = 3 if force >= 0 else -14
            ax.text(xc, force + offset,
                    f"{force:+.1f} N", ha='center', color="#ecf0f1", fontsize=7)
            ax.text(xc, ymin - abs(ymin) * 0.08 - 8,
                    f"L={ln:.3f}\nΔ{(ln-free)*1000:+.0f}mm",
                    ha='center', color=AMBER, fontsize=6)

        ax.set_ylabel("Force (N)  + = compressed", fontsize=7)
        ax.tick_params(axis='x', labelsize=7, colors=MUTED)
        ax.grid(True, color="#1e2a3a", lw=0.5, alpha=0.5, axis='y')

    # ──────────────────────────────────────────────────────────────────────
    #  STATUS & RESULTS TEXT
    # ──────────────────────────────────────────────────────────────────────

    def _update_status(self):
        if self._converged:
            msg = (f"✓ Converged  |res|={self._final_res:.2e}"
                   f"  iters={len(self.history)}")
            self._status_var.set(msg)
        else:
            msg = (f"✗ Did not converge  |res|={self._final_res:.2e}"
                   f"  iters={len(self.history)}")
            self._status_var.set(msg)

        # Show/hide error details
        self._error_text.config(state=tk.NORMAL)
        self._error_text.delete("1.0", tk.END)
        if self._error_msg:
            self._error_text.insert(tk.END, self._error_msg)
        self._error_text.config(state=tk.DISABLED)

    def _update_results(self, sol, state):
        tx, ty, tth = sol
        free = self.params['calf_free_len']

        def dl(l): return f"({(l-free)*1000:+.1f}mm)"

        lines = [
            "═══ EQUILIBRIUM RESULT ═══",
            f"Torso X     : {tx:+.4f} m",
            f"Torso Y     : {ty:+.4f} m",
            f"Torso Pitch : {np.degrees(tth):+.3f}°",
            "",
            f"Converged   : {'YES ✓' if self._converged else 'NO  ✗'}",
            f"Residual    : {self._final_res:.3e}",
            f"Iterations  : {len(self.history)}",
            "",
            "─── Calf Lengths ──────────",
            f"Front-1 : {state['front_calf1_len']:.4f} m  {dl(state['front_calf1_len'])}",
            f"Front-2 : {state['front_calf2_len']:.4f} m  {dl(state['front_calf2_len'])}",
            f"Rear-1  : {state['rear_calf1_len']:.4f}  m  {dl(state['rear_calf1_len'])}",
            f"Rear-2  : {state['rear_calf2_len']:.4f}  m  {dl(state['rear_calf2_len'])}",
            "",
            "─── Spring Forces ─────────",
            f"Front-1 : {state['front_calf1_force']:+7.2f} N",
            f"Front-2 : {state['front_calf2_force']:+7.2f} N",
            f"Rear-1  : {state['rear_calf1_force']:+7.2f}  N",
            f"Rear-2  : {state['rear_calf2_force']:+7.2f}  N",
            "",
            "─── Foot Positions ────────",
            f"Front   : ({self._foot_front[0]:.3f}, {self._foot_front[1]:.3f}) m",
            f"Rear    : ({self._foot_rear[0]:.3f},  {self._foot_rear[1]:.3f}) m",
        ]

        self._results_text.config(state=tk.NORMAL)
        self._results_text.delete("1.0", tk.END)
        self._results_text.insert(tk.END, "\n".join(lines))
        self._results_text.config(state=tk.DISABLED)


# ─────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────

if __name__ == "__main__":
    app = RobotDogApp()
    app.mainloop()