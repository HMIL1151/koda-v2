"""Interactive matplotlib dashboard.

The dashboard *is* the entry point: front/rear thigh angles, calf spring rate
and convergence threshold are tuned via sliders, and pressing "Solve" rebuilds
the robot and re-runs the iterative force-feedback solver. The resulting
iteration history is then visualised (pose + reactions + link lengths + link
loads) and scrubbable with the bottom "Iteration" slider.
"""

from __future__ import annotations

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

from units import Angle, Coordinate, JointType, Mass
from joint import Joint
from link import Link
from linkage import Linkage
from body import Body
from constants import (
    servo_seperation_mm,
    thigh_length_mm,
    calf_length_mm,
    wheelbase_mm,
    spring_rate_N_per_mm,
    robot_mass_kg,
)


# --------------------------------------------------------------------------- #
# Robot factory
# --------------------------------------------------------------------------- #

def build_and_solve(front_thigh_deg: float,
                    rear_thigh_deg: float,
                    spring_rate: float,
                    threshold: float,
                    mass_kg: float = robot_mass_kg) -> Body:
    """Construct a fresh robot from the supplied parameters and solve it."""
    front_servo = Joint(JointType.DRIVEN_STATIC,
                        Coordinate(-servo_seperation_mm / 2, 0),
                        id="front servo")
    rear_servo = Joint(JointType.DRIVEN_STATIC,
                       Coordinate(servo_seperation_mm / 2, 0),
                       id="rear servo")
    front_knee = Joint(id="front knee")
    rear_knee = Joint(id="rear knee")
    foot = Joint(id="foot")

    front_thigh = Link(front_servo, front_knee, thigh_length_mm,
                       angle=Angle.from_degrees(front_thigh_deg),
                       rate=None, id="front thigh")
    rear_thigh = Link(rear_servo, rear_knee, thigh_length_mm,
                      angle=Angle.from_degrees(rear_thigh_deg),
                      rate=None, id="rear thigh")
    front_calf = Link(front_knee, foot, calf_length_mm,
                      rate=spring_rate, id="front calf")
    rear_calf = Link(rear_knee, foot, calf_length_mm,
                     rate=spring_rate, id="rear calf")

    leg = Linkage([front_thigh, rear_thigh, front_calf, rear_calf])
    robot = Body(Mass.from_kg(mass_kg), wheelbase_mm, leg,
                 convergence_threshold=threshold)
    robot.solve_robot()
    return robot


# --------------------------------------------------------------------------- #
# Dashboard
# --------------------------------------------------------------------------- #

LEG_COLORS = ["#1f77b4", "#d62728"]


class Dashboard:
    def __init__(self,
                 front_thigh_deg: float = 135.0,
                 rear_thigh_deg: float = -45.0,
                 spring_rate: float = spring_rate_N_per_mm,
                 threshold: float = 0.1):
        self.fig = plt.figure(figsize=(15, 9))
        # Top plot area: pose (left), 3 stacked convergence plots (right).
        gs = self.fig.add_gridspec(
            3, 2, width_ratios=[2, 1], hspace=0.45, wspace=0.25,
            left=0.06, right=0.97, top=0.96, bottom=0.40,
        )
        self.ax_pose = self.fig.add_subplot(gs[:, 0])
        self.ax_react = self.fig.add_subplot(gs[0, 1])
        self.ax_len = self.fig.add_subplot(gs[1, 1])
        self.ax_load = self.fig.add_subplot(gs[2, 1])

        # Vertical iteration marker on each convergence plot.
        self._react_vline = self.ax_react.axvline(0, color="k", alpha=0.4)
        self._len_vline = self.ax_len.axvline(0, color="k", alpha=0.4)
        self._load_vline = self.ax_load.axvline(0, color="k", alpha=0.4)

        # --- setup sliders ------------------------------------------------- #
        self.s_front = Slider(self.fig.add_axes([0.08, 0.30, 0.35, 0.025]),
                              "front thigh (°)", 0.0, 360.0,
                              valinit=front_thigh_deg, valstep=1.0)
        self.s_rear = Slider(self.fig.add_axes([0.08, 0.25, 0.35, 0.025]),
                             "rear thigh (°)", -180.0, 180.0,
                             valinit=rear_thigh_deg, valstep=1.0)
        self.s_rate = Slider(self.fig.add_axes([0.55, 0.30, 0.35, 0.025]),
                             "spring rate (N/mm)", 0.1, 20.0,
                             valinit=spring_rate, valstep=0.1)
        self.s_thresh = Slider(self.fig.add_axes([0.55, 0.25, 0.35, 0.025]),
                               "convergence thresh", 0.001, 2.0,
                               valinit=threshold, valstep=0.001)

        # --- solve/play buttons + iteration slider ------------------------ #
        self.b_solve = Button(self.fig.add_axes([0.40, 0.16, 0.10, 0.05]),
                              "Solve", color="#cce5ff", hovercolor="#99ccff")
        self.b_solve.on_clicked(self._on_solve)

        self.b_play = Button(self.fig.add_axes([0.52, 0.16, 0.10, 0.05]),
                             "Play", color="#eaffcc", hovercolor="#cfff99")
        self.b_play.on_clicked(self._on_play)

        self.iter_slider = Slider(self.fig.add_axes([0.15, 0.08, 0.70, 0.025]),
                                  "Iteration", 0, 1, valinit=0, valstep=1)
        self.iter_slider.on_changed(self._on_iter_change)

        # Live preview of the current robot (unsolved, just pose).
        self.body: Body | None = None
        self._xlim = None
        self._ylim = None
        self._preview_body()

        # Live update pose on slider change
        for s in [self.s_front, self.s_rear, self.s_rate, self.s_thresh]:
            s.on_changed(self._on_param_change)

    # ------------------------------------------------------------------ #
    # actions
    # ------------------------------------------------------------------ #


    def _on_param_change(self, _val) -> None:
        self._preview_body()

    def _on_solve(self, _event) -> None:
        self._solve(jump_to_final=True)


    def _solve(self, jump_to_final=False) -> None:
        try:
            self.body = build_and_solve(
                front_thigh_deg=self.s_front.val,
                rear_thigh_deg=self.s_rear.val,
                spring_rate=self.s_rate.val,
                threshold=self.s_thresh.val,
            )
        except Exception as exc:
            self.ax_pose.clear()
            self.ax_pose.text(
                0.5, 0.5, f"Solver failed:\n{exc}",
                ha="center", va="center", color="red",
                transform=self.ax_pose.transAxes, fontsize=11,
            )
            self.ax_pose.set_axis_off()
            self.fig.canvas.draw_idle()
            return

        self._refresh_convergence_plots()
        self._refresh_iter_slider()
        final_idx = len(self.body.history) - 1 if jump_to_final else 0
        self._refresh_pose(final_idx)
        self.iter_slider.set_val(final_idx)
        self.fig.canvas.draw_idle()

    def _preview_body(self) -> None:
        # Show the robot pose for the current parameters, unsolved (iteration 0 only)
        try:
            self.body = build_and_solve(
                front_thigh_deg=self.s_front.val,
                rear_thigh_deg=self.s_rear.val,
                spring_rate=self.s_rate.val,
                threshold=self.s_thresh.val,
            )
        except Exception as exc:
            self.ax_pose.clear()
            self.ax_pose.text(
                0.5, 0.5, f"Preview failed:\n{exc}",
                ha="center", va="center", color="red",
                transform=self.ax_pose.transAxes, fontsize=11,
            )
            self.ax_pose.set_axis_off()
            self.fig.canvas.draw_idle()
            return
        self._refresh_pose(0)
        self.fig.canvas.draw_idle()

    def _on_play(self, _event) -> None:
        import time
        if not self.body:
            return
        n = len(self.body.history)
        for idx in range(n):
            self.iter_slider.set_val(idx)
            plt.pause(0.15)

    def _on_iter_change(self, _val) -> None:
        if not self.body:
            return
        idx = int(self.iter_slider.val)
        idx = max(0, min(idx, len(self.body.history) - 1))
        self._refresh_pose(idx)
        for vline in (self._react_vline, self._len_vline, self._load_vline):
            vline.set_xdata([idx, idx])
        self.fig.canvas.draw_idle()

    # ------------------------------------------------------------------ #
    # rendering
    # ------------------------------------------------------------------ #

    def _refresh_iter_slider(self) -> None:
        assert self.body is not None
        n = len(self.body.history)
        upper = max(n - 1, 1)
        self.iter_slider.valmax = upper
        self.iter_slider.ax.set_xlim(0, upper)
        self.iter_slider.set_val(0)

    def _refresh_convergence_plots(self) -> None:
        assert self.body is not None
        history = self.body.history
        iters = list(range(len(history)))

        for ax in (self.ax_react, self.ax_len, self.ax_load):
            ax.clear()

        front_react = [h["reactions_N"][0] for h in history]
        rear_react = [h["reactions_N"][1] for h in history]
        self.ax_react.plot(iters, front_react, "-o", ms=3, label="front foot")
        self.ax_react.plot(iters, rear_react, "-o", ms=3, label="rear foot")
        self.ax_react.set_title("Vertical foot reactions (N)")
        self.ax_react.set_xlabel("iteration")
        self.ax_react.grid(alpha=0.3)
        self.ax_react.legend(fontsize=8)

        # Only plot calves (not thighs) for lengths/loads
        for leg_idx, leg0 in enumerate(history[0]["legs"]):
            leg_label = leg0["id"].replace(" leg", "")
            for link_idx, link0 in enumerate(leg0["links"]):
                if "calf" not in link0["id"]:
                    continue
                lengths = [h["legs"][leg_idx]["links"][link_idx]["length"]
                           for h in history]
                loads = [h["legs"][leg_idx]["links"][link_idx]["load_N"]
                         for h in history]
                lab = f"{leg_label}/{link0['id']}"
                self.ax_len.plot(iters, lengths, "-o", ms=2, label=lab)
                self.ax_load.plot(iters, loads, "-o", ms=2, label=lab)

        self.ax_len.set_title("Calf lengths (mm)")
        self.ax_len.set_xlabel("iteration")
        self.ax_len.grid(alpha=0.3)
        self.ax_len.legend(fontsize=6, ncol=2, loc="best")

        self.ax_load.set_title("Calf loads (N)")
        self.ax_load.set_xlabel("iteration")
        self.ax_load.grid(alpha=0.3)
        self.ax_load.legend(fontsize=6, ncol=2, loc="best")

        # Re-attach the vertical iteration markers (cleared above).
        self._react_vline = self.ax_react.axvline(0, color="k", alpha=0.4)
        self._len_vline = self.ax_len.axvline(0, color="k", alpha=0.4)
        self._load_vline = self.ax_load.axvline(0, color="k", alpha=0.4)

        # Cache stable pose bounds for the run.
        all_x, all_y = [], []
        for h in history:
            for leg in h["legs"]:
                all_x.extend(p[0] for p in leg["joints"])
                all_y.extend(p[1] for p in leg["joints"])
            all_x.append(h["cog"][0])
            all_y.append(h["cog"][1])
        pad = 20
        self._xlim = (min(all_x) - pad, max(all_x) + pad)
        self._ylim = (min(all_y) - pad, max(all_y) + pad)

    def _refresh_pose(self, idx: int) -> None:
        assert self.body is not None
        history = self.body.history
        idx = max(0, min(idx, len(history) - 1))
        h = history[idx]

        ax = self.ax_pose
        ax.clear()
        ax.set_axis_on()

        # Torso line between the midpoints of each leg's two servos.
        mids = []
        for leg in h["legs"]:
            s0 = leg["joints"][0]
            s1 = leg["joints"][-1]
            mids.append(((s0[0] + s1[0]) / 2, (s0[1] + s1[1]) / 2))
        ax.plot([mids[0][0], mids[1][0]], [mids[0][1], mids[1][1]],
                "k--", lw=1.5, alpha=0.6, label="torso")

        for leg_idx, leg in enumerate(h["legs"]):
            xs = [p[0] for p in leg["joints"]]
            ys = [p[1] for p in leg["joints"]]
            ax.plot(xs, ys, "-", lw=2, color=LEG_COLORS[leg_idx], label=leg["id"])
            ax.scatter([xs[0], xs[-1]], [ys[0], ys[-1]],
                       c="blue", s=40, zorder=3, edgecolors="k", linewidths=0.5)
            ax.scatter([xs[1], xs[-2]], [ys[1], ys[-2]],
                       c="green", s=40, zorder=3, edgecolors="k", linewidths=0.5)
            fx, fy = leg["foot"]
            ax.scatter([fx], [fy], c="red", s=70, zorder=4,
                       edgecolors="k", linewidths=0.5)

        cx, cy = h["cog"]
        ax.scatter([cx], [cy], c="black", s=90, marker="x", zorder=5, label="COG")

        ground_y = min(leg["foot"][1] for leg in h["legs"])
        ax.axhline(ground_y, color="brown", alpha=0.3, lw=1)

        if self._xlim and self._ylim:
            ax.set_xlim(*self._xlim)
            ax.set_ylim(*self._ylim)
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)
        ax.set_title(
            f"{h['label']}   "
            f"COG=({cx:.1f}, {cy:.1f}) mm   "
            f"torso={h['torso_angle_deg']:.2f}°   "
            f"Rf={h['reactions_N'][0]:.2f} N  Rr={h['reactions_N'][1]:.2f} N   "
            f"iters={len(history)}"
        )


def launch_dashboard(**kwargs) -> Dashboard:
    """Open the dashboard window. Blocks until the window is closed."""
    dash = Dashboard(**kwargs)
    plt.show()
    return dash
