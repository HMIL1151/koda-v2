import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox, Button, RadioButtons, CheckButtons
from matplotlib.patches import Arc, Circle


# ============================================================
# 5-BAR LEG MECHANISM SIMULATOR
# ============================================================
#
# Mechanism:
#
# A and B are fixed hip pivots.
# A -> C is the left actuated thigh.
# B -> D is the right actuated thigh.
# C -> P is the left calf / lower link.
# D -> P is the right calf / lower link.
# P is the foot / output point.
#
# Simplified assumptions in this version:
#
# - Left and right thighs are the same length.
# - Left and right calves are the same length.
# - Thigh angle limits are mirrored left/right.
#
# Angle convention:
#
# - Angles are measured from +X.
# - Anticlockwise is positive.
# - Left thigh angle is entered directly.
# - Right thigh angle is also entered directly, measured from +X.
#
# Mirrored limits:
#
# If the left side is allowed from:
#
#     left_min to left_max
#
# then the right side limits are:
#
#     right_min = 180 - left_max
#     right_max = 180 - left_min
#
# Example:
#
#     left limits = -120 deg to -40 deg
#
# gives:
#
#     right limits = 220 deg to 300 deg
#
# which is equivalent to:
#
#     -140 deg to -60 deg
#
# after wrapping.
#
# Units:
# - mm for lengths
# - degrees for angles
#
# ============================================================


def wrap_angle_deg(angle):
    """
    Wrap angle to the range -180 to 180 degrees.
    Useful for display only.
    """
    return ((angle + 180) % 360) - 180


def circle_intersections(c1, r1, c2, r2):
    """
    Return the two intersections between two circles.

    c1, c2:
        Circle centre points as np.array([x, y])

    r1, r2:
        Circle radii

    Returns:
        None if there is no valid intersection.
        Otherwise returns (p_upper, p_lower).
    """

    c1 = np.asarray(c1, dtype=float)
    c2 = np.asarray(c2, dtype=float)

    d_vec = c2 - c1
    d = np.linalg.norm(d_vec)

    if d < 1e-9:
        return None

    if d > r1 + r2:
        return None

    if d < abs(r1 - r2):
        return None

    a = (r1**2 - r2**2 + d**2) / (2 * d)

    h_sq = r1**2 - a**2

    if h_sq < -1e-9:
        return None

    h = np.sqrt(max(h_sq, 0.0))

    e = d_vec / d
    p_mid = c1 + a * e

    perp = np.array([-e[1], e[0]])

    p1 = p_mid + h * perp
    p2 = p_mid - h * perp

    if p1[1] >= p2[1]:
        return p1, p2
    else:
        return p2, p1


def five_bar_forward_kinematics(
    base_width,
    thigh_length,
    calf_length,
    theta_left_deg,
    theta_right_deg,
    assembly_mode="lower",
):
    """
    Forward kinematics for the simplified symmetrical 5-bar leg.

    Returns a dictionary containing:
        valid
        A, B, C, D, P
    """

    A = np.array([-base_width / 2.0, 0.0])
    B = np.array([base_width / 2.0, 0.0])

    theta_l = np.deg2rad(theta_left_deg)
    theta_r = np.deg2rad(theta_right_deg)

    C = A + thigh_length * np.array([np.cos(theta_l), np.sin(theta_l)])
    D = B + thigh_length * np.array([np.cos(theta_r), np.sin(theta_r)])

    intersections = circle_intersections(C, calf_length, D, calf_length)

    if intersections is None:
        return {
            "valid": False,
            "A": A,
            "B": B,
            "C": C,
            "D": D,
            "P": None,
        }

    p_upper, p_lower = intersections

    if assembly_mode == "upper":
        P = p_upper
    else:
        P = p_lower

    return {
        "valid": True,
        "A": A,
        "B": B,
        "C": C,
        "D": D,
        "P": P,
    }


def mirrored_right_limits(left_min_deg, left_max_deg):
    """
    Calculates mirrored right thigh limits from left thigh limits.

    Mirror relationship:
        theta_right = 180 - theta_left

    If left range is [left_min, left_max], the mirrored right range is:
        [180 - left_max, 180 - left_min]
    """

    right_min = 180.0 - left_max_deg
    right_max = 180.0 - left_min_deg

    return right_min, right_max


def sample_workspace(
    base_width,
    thigh_length,
    calf_length,
    left_min_deg,
    left_max_deg,
    assembly_mode="lower",
    samples_per_axis=70,
):
    """
    Samples the foot workspace using mirrored left/right thigh limits.
    """

    right_min_deg, right_max_deg = mirrored_right_limits(left_min_deg, left_max_deg)

    left_values = np.linspace(left_min_deg, left_max_deg, samples_per_axis)
    right_values = np.linspace(right_min_deg, right_max_deg, samples_per_axis)

    points = []

    for theta_l in left_values:
        for theta_r in right_values:
            fk = five_bar_forward_kinematics(
                base_width,
                thigh_length,
                calf_length,
                theta_l,
                theta_r,
                assembly_mode,
            )

            if fk["valid"]:
                points.append(fk["P"])

    if len(points) == 0:
        return np.empty((0, 2))

    return np.asarray(points)


def draw_angle_limit_arc(ax, centre, radius, angle_min, angle_max, colour, label):
    """
    Draws an arc and radial lines showing an angle limit range.
    """

    arc = Arc(
        centre,
        2 * radius,
        2 * radius,
        angle=0,
        theta1=angle_min,
        theta2=angle_max,
        color=colour,
        linewidth=2,
        alpha=0.7,
    )
    ax.add_patch(arc)

    for angle in [angle_min, angle_max]:
        a = np.deg2rad(angle)
        end = centre + radius * np.array([np.cos(a), np.sin(a)])

        ax.plot(
            [centre[0], end[0]],
            [centre[1], end[1]],
            linestyle="--",
            color=colour,
            alpha=0.45,
            linewidth=1.2,
        )

    mid_angle = np.deg2rad((angle_min + angle_max) / 2)
    text_pos = centre + radius * 1.18 * np.array(
        [np.cos(mid_angle), np.sin(mid_angle)]
    )

    ax.text(
        text_pos[0],
        text_pos[1],
        label,
        color=colour,
        fontsize=9,
        ha="center",
        va="center",
    )


# ============================================================
# Initial parameters
# ============================================================

params = {
    "base_width": 120.0,
    "thigh_length": 80.0,
    "calf_length": 120.0,
    "theta_left": -70.0,
    "theta_right": -110.0,
    "left_min": -130.0,
    "left_max": -30.0,
}

assembly_mode = "lower"
show_workspace = True
show_limits = True


# ============================================================
# Figure layout
# ============================================================

plt.close("all")

fig = plt.figure(figsize=(14, 8))

ax = fig.add_axes([0.06, 0.12, 0.63, 0.82])
ax.set_aspect("equal", adjustable="box")
ax.grid(True)
ax.set_title("5-Bar Leg Mechanism Simulator")
ax.set_xlabel("X position [mm]")
ax.set_ylabel("Y position [mm]")

status_ax = fig.add_axes([0.72, 0.58, 0.26, 0.36])
status_ax.axis("off")


# ============================================================
# Text input boxes
# ============================================================

input_colour = "lavender"

textboxes = {}

box_specs = [
    ("base_width", "Base width AB [mm]", 0.80),
    ("thigh_length", "Thigh length [mm]", 0.74),
    ("calf_length", "Calf length [mm]", 0.68),
    ("theta_left", "Left thigh angle [deg]", 0.58),
    ("theta_right", "Right thigh angle [deg]", 0.52),
    ("left_min", "Left angle min [deg]", 0.42),
    ("left_max", "Left angle max [deg]", 0.36),
]

for key, label, y in box_specs:
    label_ax = fig.add_axes([0.72, y, 0.12, 0.035])
    label_ax.axis("off")
    label_ax.text(0.0, 0.5, label, va="center", fontsize=9)

    box_ax = fig.add_axes([0.86, y, 0.10, 0.035], facecolor=input_colour)
    textboxes[key] = TextBox(box_ax, "", initial=str(params[key]))


# Buttons and toggles
update_ax = fig.add_axes([0.72, 0.28, 0.11, 0.045])
update_button = Button(update_ax, "Update")

reset_ax = fig.add_axes([0.85, 0.28, 0.11, 0.045])
reset_button = Button(reset_ax, "Reset")

print_ax = fig.add_axes([0.72, 0.22, 0.24, 0.045])
print_button = Button(print_ax, "Print config")

radio_ax = fig.add_axes([0.72, 0.08, 0.10, 0.10])
radio = RadioButtons(radio_ax, ("lower", "upper"), active=0)

check_ax = fig.add_axes([0.85, 0.08, 0.12, 0.10])
checks = CheckButtons(
    check_ax,
    ("workspace", "limits"),
    (show_workspace, show_limits),
)


# ============================================================
# Input handling
# ============================================================

def read_inputs():
    """
    Reads all numerical inputs from the text boxes.

    If an input is invalid, the previous value is retained.
    """

    global params

    for key, textbox in textboxes.items():
        raw = textbox.text.strip()

        try:
            params[key] = float(raw)
        except ValueError:
            print(f"Invalid input for {key}: {raw!r}. Keeping previous value.")

            # Reset displayed text to previous valid value
            textbox.set_val(str(params[key]))

    # Ensure min <= max
    if params["left_min"] > params["left_max"]:
        params["left_min"], params["left_max"] = (
            params["left_max"],
            params["left_min"],
        )
        textboxes["left_min"].set_val(str(params["left_min"]))
        textboxes["left_max"].set_val(str(params["left_max"]))


def update_plot(event=None):
    """
    Updates the plot using current numerical input values.
    """

    read_inputs()

    base_width = params["base_width"]
    thigh_length = params["thigh_length"]
    calf_length = params["calf_length"]
    theta_left = params["theta_left"]
    theta_right = params["theta_right"]
    left_min = params["left_min"]
    left_max = params["left_max"]

    right_min, right_max = mirrored_right_limits(left_min, left_max)

    fk = five_bar_forward_kinematics(
        base_width,
        thigh_length,
        calf_length,
        theta_left,
        theta_right,
        assembly_mode,
    )

    ax.clear()
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)
    ax.set_title("5-Bar Leg Mechanism Simulator")
    ax.set_xlabel("X position [mm]")
    ax.set_ylabel("Y position [mm]")

    A = fk["A"]
    B = fk["B"]
    C = fk["C"]
    D = fk["D"]
    P = fk["P"]

    workspace_points = np.empty((0, 2))

    if show_workspace:
        workspace_points = sample_workspace(
            base_width,
            thigh_length,
            calf_length,
            left_min,
            left_max,
            assembly_mode,
            samples_per_axis=70,
        )

        if len(workspace_points) > 0:
            ax.scatter(
                workspace_points[:, 0],
                workspace_points[:, 1],
                s=4,
                alpha=0.16,
                color="tab:blue",
                label="Reachable workspace",
            )

    if show_limits:
        limit_radius = max(20.0, thigh_length * 0.65)

        draw_angle_limit_arc(
            ax,
            A,
            limit_radius,
            left_min,
            left_max,
            "tab:green",
            "Left thigh limits",
        )

        draw_angle_limit_arc(
            ax,
            B,
            limit_radius,
            right_min,
            right_max,
            "tab:orange",
            "Right thigh limits",
        )

    # Draw base
    ax.plot(
        [A[0], B[0]],
        [A[1], B[1]],
        color="black",
        linewidth=4,
        solid_capstyle="round",
        label="Base AB",
    )

    # Draw thighs
    ax.plot(
        [A[0], C[0]],
        [A[1], C[1]],
        color="tab:green",
        linewidth=4,
        marker="o",
        label="Left thigh",
    )

    ax.plot(
        [B[0], D[0]],
        [B[1], D[1]],
        color="tab:orange",
        linewidth=4,
        marker="o",
        label="Right thigh",
    )

    if fk["valid"]:
        ax.plot(
            [C[0], P[0]],
            [C[1], P[1]],
            color="tab:purple",
            linewidth=3,
            marker="o",
            label="Left calf",
        )

        ax.plot(
            [D[0], P[0]],
            [D[1], P[1]],
            color="tab:red",
            linewidth=3,
            marker="o",
            label="Right calf",
        )

        ax.scatter([P[0]], [P[1]], s=130, color="red", zorder=5)
        ax.text(P[0], P[1], "  P foot", fontsize=10, va="center")

        valid_text = "VALID"
        foot_text = f"Foot P: x = {P[0]:.2f} mm, y = {P[1]:.2f} mm"

    else:
        valid_text = "INVALID"
        foot_text = "Foot P: no valid circle intersection"

        ax.add_patch(
            Circle(
                C,
                calf_length,
                fill=False,
                linestyle="--",
                alpha=0.25,
                color="tab:purple",
            )
        )

        ax.add_patch(
            Circle(
                D,
                calf_length,
                fill=False,
                linestyle="--",
                alpha=0.25,
                color="tab:red",
            )
        )

    # Draw point labels
    for label, point in [("A", A), ("B", B), ("C", C), ("D", D)]:
        ax.scatter([point[0]], [point[1]], s=55, color="black", zorder=4)
        ax.text(point[0], point[1], f"  {label}", fontsize=10, va="center")

    # Auto-scale
    all_points = [A, B, C, D]

    if fk["valid"]:
        all_points.append(P)

    if len(workspace_points) > 0:
        step = max(1, len(workspace_points) // 400)
        all_points.extend(workspace_points[::step])

    all_points = np.asarray(all_points)

    x_min, y_min = np.min(all_points, axis=0)
    x_max, y_max = np.max(all_points, axis=0)

    span = max(x_max - x_min, y_max - y_min, 100.0)
    margin = 0.18 * span

    ax.set_xlim(x_min - margin, x_max + margin)
    ax.set_ylim(y_min - margin, y_max + margin)

    ax.legend(loc="upper left", fontsize=8)

    # Status panel
    status_ax.clear()
    status_ax.axis("off")

    status_text = (
        f"Status: {valid_text}\n\n"
        f"{foot_text}\n\n"
        f"Assembly mode: {assembly_mode}\n\n"
        f"Geometry:\n"
        f"  Base width AB = {base_width:.2f} mm\n"
        f"  Thigh length = {thigh_length:.2f} mm\n"
        f"  Calf length = {calf_length:.2f} mm\n\n"
        f"Current angles:\n"
        f"  Left thigh  = {theta_left:.2f}°\n"
        f"  Right thigh = {theta_right:.2f}°\n\n"
        f"Mirrored limits:\n"
        f"  Left:  {left_min:.2f}° to {left_max:.2f}°\n"
        f"  Right: {right_min:.2f}° to {right_max:.2f}°\n"
        f"         ({wrap_angle_deg(right_min):.2f}° to "
        f"{wrap_angle_deg(right_max):.2f}° wrapped)"
    )

    status_ax.text(
        0.0,
        1.0,
        status_text,
        va="top",
        ha="left",
        fontsize=9,
        family="monospace",
    )

    fig.canvas.draw_idle()


def reset(event=None):
    """
    Resets all numerical inputs to the default values.
    """

    defaults = {
        "base_width": 120.0,
        "thigh_length": 80.0,
        "calf_length": 120.0,
        "theta_left": -70.0,
        "theta_right": -110.0,
        "left_min": -130.0,
        "left_max": -30.0,
    }

    for key, value in defaults.items():
        params[key] = value
        textboxes[key].set_val(str(value))

    update_plot()


def print_config(event=None):
    """
    Prints the current configuration to the terminal.
    """

    read_inputs()

    right_min, right_max = mirrored_right_limits(
        params["left_min"],
        params["left_max"],
    )

    fk = five_bar_forward_kinematics(
        params["base_width"],
        params["thigh_length"],
        params["calf_length"],
        params["theta_left"],
        params["theta_right"],
        assembly_mode,
    )

    print("\nCurrent 5-bar leg configuration")
    print("--------------------------------")
    print(f"base_width = {params['base_width']:.3f}")
    print(f"thigh_length = {params['thigh_length']:.3f}")
    print(f"calf_length = {params['calf_length']:.3f}")
    print(f"theta_left = {params['theta_left']:.3f}")
    print(f"theta_right = {params['theta_right']:.3f}")
    print(f"left_min = {params['left_min']:.3f}")
    print(f"left_max = {params['left_max']:.3f}")
    print(f"right_min = {right_min:.3f}")
    print(f"right_max = {right_max:.3f}")
    print(f"assembly_mode = '{assembly_mode}'")

    if fk["valid"]:
        P = fk["P"]
        print(f"foot_x = {P[0]:.3f}")
        print(f"foot_y = {P[1]:.3f}")
    else:
        print("foot = invalid")


def on_radio_change(label):
    global assembly_mode
    assembly_mode = label
    update_plot()


def on_check_change(label):
    global show_workspace, show_limits

    if label == "workspace":
        show_workspace = not show_workspace

    if label == "limits":
        show_limits = not show_limits

    update_plot()


# ============================================================
# Connect callbacks
# ============================================================

update_button.on_clicked(update_plot)
reset_button.on_clicked(reset)
print_button.on_clicked(print_config)

radio.on_clicked(on_radio_change)
checks.on_clicked(on_check_change)

# Optional: pressing Enter inside any text box also updates the plot.
for textbox in textboxes.values():
    textbox.on_submit(lambda text: update_plot())

update_plot()
plt.show()