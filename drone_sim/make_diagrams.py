"""
Explanatory diagrams for the Introduction / Methodology document.

    python drone_sim/make_diagrams.py

These are drawn rather than measured, so they live apart from evaluate.py:
nothing here is a result, everything here is an explanation of the method.
"""
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Rectangle

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import config as C

FIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "figures")
os.makedirs(FIG, exist_ok=True)

PAPER = "#1f6fb4"      # blocks taken from the base paper
OURS = "#e08214"       # blocks this project adds
NEUTRAL = "#6b7482"    # the physical world
INK = "#22303c"

plt.rcParams.update({"figure.facecolor": "white", "font.size": 9.5,
                     "savefig.bbox": "tight"})


def _box(ax, x, y, w, h, text, colour, fontsize=8.6, text_colour="white"):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.012,rounding_size=0.02",
                                linewidth=0, facecolor=colour, zorder=2))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", zorder=3,
            fontsize=fontsize, color=text_colour, linespacing=1.35)


def _arrow(ax, p0, p1, colour=INK, style="-|>", lw=1.3, rad=0.0):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle=style, mutation_scale=11,
                                 linewidth=lw, color=colour, zorder=4,
                                 connectionstyle="arc3,rad=%.2f" % rad,
                                 shrinkA=2, shrinkB=2))


# ------------------------------------------------------------------ pipeline
def fig_pipeline():
    fig, ax = plt.subplots(figsize=(9.8, 5.9))
    ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis("off")

    # --- sensing row
    _box(ax, 0.02, 0.72, 0.17, 0.13,
         "downward camera\n640 x 480, 25 Hz", NEUTRAL)
    _box(ax, 0.22, 0.72, 0.17, 0.13,
         "AprilTag detector\ntag36h11", PAPER)
    _box(ax, 0.42, 0.72, 0.17, 0.13,
         "pose from corners\nsolvePnP + known size", PAPER)
    _box(ax, 0.62, 0.72, 0.19, 0.13,
         "camera -> body -> world\ncoordinate transforms", PAPER)

    _box(ax, 0.02, 0.50, 0.17, 0.12,
         "AirTag radio\n2 Hz, +-2.5 m", OURS)
    _box(ax, 0.22, 0.50, 0.17, 0.12,
         "marker id filter\naccept only this address", OURS)
    _box(ax, 0.42, 0.50, 0.17, 0.12,
         "pad tracker\nalpha-beta-gamma", OURS)
    _box(ax, 0.62, 0.50, 0.19, 0.12,
         "IMU attitude\nroll, pitch, yaw", NEUTRAL)

    # --- guidance
    _box(ax, 0.28, 0.28, 0.20, 0.13,
         "PID, eq. (9)\nkp 0.20  ki 0.03  kd 0.35", PAPER)
    _box(ax, 0.52, 0.28, 0.22, 0.13,
         "feed-forward + gain schedule\n+ approach-cone interlocks", OURS)

    # --- inner loops and plant
    _box(ax, 0.28, 0.07, 0.20, 0.12,
         "attitude and rate loop\naccel -> tilt -> torque", OURS)
    _box(ax, 0.52, 0.07, 0.22, 0.12,
         "quadrotor\nbody-axis thrust + 3 torques", NEUTRAL)
    _box(ax, 0.83, 0.07, 0.15, 0.12, "the world\n(MuJoCo)", NEUTRAL)

    a = _arrow
    a(ax, (0.19, 0.785), (0.22, 0.785))
    a(ax, (0.39, 0.785), (0.42, 0.785))
    a(ax, (0.59, 0.785), (0.62, 0.785))
    a(ax, (0.305, 0.72), (0.305, 0.62))          # detector -> id filter
    a(ax, (0.39, 0.56), (0.42, 0.56))            # id filter -> tracker
    a(ax, (0.19, 0.56), (0.22, 0.56))            # radio -> id filter
    a(ax, (0.715, 0.72), (0.715, 0.62), style="<|-")   # imu -> transforms
    a(ax, (0.715, 0.50), (0.63, 0.41), rad=-0.15)      # transforms -> guidance
    a(ax, (0.50, 0.50), (0.42, 0.41), rad=0.15)        # tracker -> guidance
    a(ax, (0.48, 0.345), (0.52, 0.345))
    a(ax, (0.63, 0.28), (0.63, 0.19), style="-|>")
    a(ax, (0.52, 0.13), (0.48, 0.13), style="<|-")
    a(ax, (0.74, 0.13), (0.83, 0.13))
    # the world closes the loop back into the camera; route it around the
    # outside so it does not cut through the blocks it is meant to connect
    ax.plot([0.905, 0.905, 0.105], [0.19, 0.945, 0.945], color="#b6bec8",
            lw=1.1, zorder=1)
    a(ax, (0.105, 0.945), (0.105, 0.855), colour="#b6bec8", lw=1.1)
    ax.text(0.50, 0.962, "the image the camera really produces closes this loop",
            ha="center", fontsize=8.4, color="#98a2ad", style="italic")

    ax.text(0.02, 0.90, "PERCEPTION", fontsize=8.6, color="#6b7482", weight="bold")
    ax.text(0.28, 0.435, "GUIDANCE", fontsize=8.6, color="#6b7482", weight="bold")
    ax.text(0.28, 0.205, "CONTROL", fontsize=8.6, color="#6b7482", weight="bold")

    for x, colour, label in ((0.02, PAPER, "from the base paper"),
                             (0.26, OURS, "added by this project"),
                             (0.52, NEUTRAL, "hardware / physics")):
        ax.add_patch(Rectangle((x, 0.005), 0.020, 0.026, facecolor=colour, lw=0))
        ax.text(x + 0.028, 0.018, label, fontsize=8.4, color=INK, va="center")

    ax.set_title("Signal flow: what the base paper provides, and what is added around it",
                 fontsize=10.5, weight="bold", loc="left")
    path = os.path.join(FIG, "mj_diag_pipeline.png")
    fig.savefig(path, dpi=190)
    plt.close(fig)
    print("   wrote figures/mj_diag_pipeline.png")
    return path


# ----------------------------------------------------------------- pad layout
def fig_pad_geometry():
    # stacked rather than side by side: the document page is portrait, and a
    # wide two-panel figure has to be shrunk until the labels stop being readable
    fig, axes = plt.subplots(2, 1, figsize=(7.8, 6.6),
                             gridspec_kw={"height_ratios": [1.15, 0.85]})

    # ---- left: the pad seen from above
    ax = axes[0]
    half = C.PAD_SIZE / 2
    ax.add_patch(Rectangle((-half, -half), C.PAD_SIZE, C.PAD_SIZE,
                           facecolor="#f4f6f8", edgecolor="#1c96dc", lw=2.5))
    b = C.TAG_LARGE
    ax.add_patch(Rectangle((C.TAG_LARGE_OFFSET - b / 2, -b / 2), b, b,
                           facecolor="#20242a", edgecolor="none"))
    ax.text(C.TAG_LARGE_OFFSET, 0, "id k", color="white", ha="center", va="center",
            fontsize=9, weight="bold")
    s = C.TAG_SMALL
    ax.add_patch(Rectangle((-s / 2, -s / 2), s, s, facecolor="#20242a"))
    ax.text(-0.30, -0.30, "id k+10", ha="center", fontsize=8.2, color=INK)
    ax.annotate("", xy=(-0.03, -0.05), xytext=(-0.24, -0.26),
                arrowprops=dict(arrowstyle="->", color=INK, lw=0.9))

    ax.annotate("", xy=(C.TAG_LARGE_OFFSET, 0.42), xytext=(0, 0.42),
                arrowprops=dict(arrowstyle="<->", color=OURS, lw=1.3))
    ax.text(C.TAG_LARGE_OFFSET / 2, 0.47, "%.2f m" % C.TAG_LARGE_OFFSET,
            ha="center", fontsize=8.4, color=OURS)
    ax.plot(0, 0, marker="+", ms=13, color="#c8352b", mew=2)
    ax.text(-0.60, 0.34, "pad centre\n= aim point", fontsize=8.2, color="#c8352b",
            ha="center", linespacing=1.3)
    ax.annotate("", xy=(-0.04, 0.04), xytext=(-0.50, 0.27),
                arrowprops=dict(arrowstyle="->", color="#c8352b", lw=1.0))
    ax.annotate("", xy=(-half, -half - 0.10), xytext=(half, -half - 0.10),
                arrowprops=dict(arrowstyle="<->", color=INK, lw=1.1))
    ax.text(0, -half - 0.20, "pad %.2f m" % C.PAD_SIZE, ha="center", fontsize=8.4)
    ax.set_xlim(-1.0, 1.0); ax.set_ylim(-1.05, 0.95)
    ax.set_aspect("equal"); ax.axis("off")
    ax.set_title("One pad: two markers, two scales, two ids", fontsize=10, loc="left")

    # ---- right: which marker is usable at which altitude
    ax = axes[1]
    alts = np.linspace(0.15, 9.5, 600)
    px_big = C.FOCAL_PX * C.TAG_LARGE / alts
    px_small = C.FOCAL_PX * C.TAG_SMALL / alts
    fov = alts * (C.IMG_H / 2.0) / C.FOCAL_PX

    big_ok = (px_big >= C.MIN_TAG_PX) & (fov >= C.TAG_LARGE_OFFSET + C.TAG_LARGE / 2)
    small_ok = px_small >= C.MIN_TAG_PX

    ax.fill_between(alts, 0, 1, where=big_ok, color=PAPER, alpha=0.30, lw=0)
    ax.fill_between(alts, 1.1, 2.1, where=small_ok, color=OURS, alpha=0.30, lw=0)
    ax.text(4.6, 0.5, "0.42 m marker usable", color=PAPER, fontsize=9, ha="center")
    ax.text(1.2, 1.6, "0.12 m marker usable", color=OURS, fontsize=9, ha="center")

    g = C.check_geometry()
    lo, hi = g["handover_window_m"]
    ax.axvspan(lo, hi, color="#2a9d5c", alpha=0.16, lw=0)
    ax.text((lo + hi) / 2, 2.45, "hand-over\n%.2f - %.2f m" % (lo, hi), ha="center",
            fontsize=8.6, color="#2a9d5c")
    ax.axvline(C.CRUISE_ALT, color=INK, ls="--", lw=1.1)
    ax.text(C.CRUISE_ALT - 0.15, 2.45, "search altitude", rotation=90, va="top",
            ha="right", fontsize=8.2, color=INK)

    ax.set_xlim(0, 9.5); ax.set_ylim(0, 2.9)
    ax.set_yticks([]); ax.set_xlabel("altitude above the pad (m)")
    ax.grid(True, axis="x", color="#e3e7ec")
    ax.set_title("Why two scales: neither marker covers the whole descent",
                 fontsize=10, loc="left")

    path = os.path.join(FIG, "mj_diag_pad.png")
    fig.savefig(path, dpi=190)
    plt.close(fig)
    print("   wrote figures/mj_diag_pad.png")
    return path


# --------------------------------------------------------------- state machine
def fig_state_machine():
    fig, ax = plt.subplots(figsize=(10.2, 3.3))
    ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis("off")

    states = [("TAKEOFF", 0.015, NEUTRAL), ("TRANSIT", 0.155, OURS),
              ("SEARCH", 0.295, OURS), ("ALIGN", 0.435, PAPER),
              ("DESCEND", 0.575, PAPER), ("FINAL", 0.715, PAPER),
              ("LANDED", 0.855, "#2a9d5c")]
    w, h, y = 0.125, 0.20, 0.52
    for name, x, colour in states:
        _box(ax, x, y, w, h, name, colour, fontsize=9)
    for i in range(len(states) - 1):
        _arrow(ax, (states[i][1] + w, y + h / 2), (states[i + 1][1], y + h / 2))

    notes = [
        (0.155, "radio fix\nsteers the cruise"),
        (0.295, "spiral until the\naddressed tag decodes"),
        (0.435, "hold height,\nnull the offset"),
        (0.575, "descend only inside\nthe approach cone"),
        (0.715, "committed,\nmotors cut on contact"),
    ]
    for x, txt in notes:
        ax.text(x + w / 2, y - 0.06, txt, ha="center", va="top", fontsize=7.8,
                color="#4a5560", linespacing=1.3)

    _arrow(ax, (0.575 + w / 2, y + h), (0.435 + w / 2, y + h), colour="#c8352b", rad=0.45)
    ax.text(0.53, y + h + 0.20, "tag lost, or offset outside the cone -> climb and re-acquire",
            ha="center", fontsize=8, color="#c8352b")

    ax.text(0.0, 0.06, "Guidance authority passes from radio to vision exactly once: "
                       "at the first decode of the addressed marker.",
            fontsize=8.4, color="#6b7482", style="italic")
    ax.set_title("Mission phases", fontsize=10.5, weight="bold", loc="left")

    path = os.path.join(FIG, "mj_diag_states.png")
    fig.savefig(path, dpi=190)
    plt.close(fig)
    print("   wrote figures/mj_diag_states.png")
    return path


if __name__ == "__main__":
    fig_pipeline()
    fig_pad_geometry()
    fig_state_machine()
