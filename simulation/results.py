"""
Batch experiments + every figure used in the review document.

    python results.py            -> runs everything, writes ../figures/*.png
                                    and ../results/summary.json

Experiments
  E1  static landing on address 5           (paper section 4.1 analogue)
  E2  dynamic tracking of address 4         (paper section 4.2 analogue)
  E3  velocity step response with / without PID   (paper Fig.10 vs Fig.12)
  E4  controller ablation on a moving pad   (PID | +feed-forward | +gain sched.)
  E5  Monte Carlo over all ten addresses    (accuracy + success rate)
  E6  sensor model: tag detectability and positioning noise vs altitude
"""
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import config as C
from airtags import build_fleet
from drone import Drone
from mission import LandingMission
from vision import AprilTagCamera

FIG = os.path.join(os.path.dirname(__file__), "..", "figures")
RES = os.path.join(os.path.dirname(__file__), "..", "results")
os.makedirs(FIG, exist_ok=True)
os.makedirs(RES, exist_ok=True)

BLUE, ORANGE, GREEN, RED, GREY = "#1f6fb4", "#e08214", "#2a9d5c", "#c8352b", "#7a8590"

plt.rcParams.update({
    "figure.facecolor": "white", "axes.facecolor": "white",
    "axes.grid": True, "grid.color": "#e3e7ec", "grid.linewidth": 0.7,
    "axes.edgecolor": "#b9c0c9", "font.size": 9.5,
    "axes.titlesize": 10.5, "axes.titleweight": "bold", "figure.dpi": 150,
    "savefig.bbox": "tight",
})


def save(fig, name):
    path = os.path.join(FIG, name)
    fig.savefig(path, dpi=180)
    plt.close(fig)
    print("   wrote figures/%s" % name)
    return path


def fly(tag_id, seed=0, **kw):
    m = LandingMission(build_fleet(), tag_id, seed=seed, **kw)
    m.run()
    return m


# ---------------------------------------------------------------- fig: arena
def fig_arena():
    fleet = build_fleet()
    fig, ax = plt.subplots(figsize=(6.6, 6.2))
    for t in fleet:
        moving = t.motion != "static"
        ax.scatter(*t.home, s=170, marker="s",
                   color=(ORANGE if moving else BLUE), zorder=3,
                   edgecolor="white", linewidth=1.2)
        ax.annotate("%d" % t.id, t.home, textcoords="offset points",
                    xytext=(0, -3.5), ha="center", color="white",
                    fontsize=8, fontweight="bold", zorder=4)
        ax.annotate(t.address.split("  ")[0], t.home, textcoords="offset points",
                    xytext=(11, 6), fontsize=7.5, color="#404a55")
        if moving:                          # sketch the path the pad travels
            xs, ys = [], []
            for _ in range(1200):
                t.step(0.05); xs.append(t.pos[0]); ys.append(t.pos[1])
            ax.plot(xs, ys, color=ORANGE, lw=0.9, alpha=0.45, zorder=2)
            t.reset()
    ax.scatter([0], [0], marker="^", s=110, color=GREEN, zorder=3,
               edgecolor="white", linewidth=1.0)
    ax.annotate("home / launch", (0, 0), textcoords="offset points",
                xytext=(10, -12), fontsize=7.5, color=GREEN)
    ax.set_xlim(-32, 32); ax.set_ylim(-32, 32); ax.set_aspect("equal")
    ax.set_xlabel("x, east (m)"); ax.set_ylabel("y, north (m)")
    ax.set_title("Fig. 1  The ten AirTag addresses (orange = pad on a moving vehicle)")
    return save(fig, "fig1_arena.png")


# ------------------------------------------------------- E1: static landing
def fig_static_landing():
    m = fly(5, seed=3)
    L = m.log
    t, x, y, z = L.col("t"), L.col("x"), L.col("y"), L.col("z")
    fig = plt.figure(figsize=(11.0, 4.4))
    ax1 = fig.add_subplot(1, 2, 1, projection="3d")
    ax1.plot(x, y, z, color=BLUE, lw=1.6)
    ax1.scatter([m.tag.pos[0]], [m.tag.pos[1]], [0], color=RED, s=45, marker="s")

    # Key points in the style of Table 1 of the base paper. Each is the FIRST
    # sample after the previous one that satisfies its condition - searching the
    # whole log would match t=0, where the drone is still sitting on the ground.
    states = np.array([r["state"] for r in L.rows])
    conds = (("A", states == "TRANSIT"),          # top of climb
             ("B", states == "DESCEND"),          # above the pad, descent begins
             ("C", z <= C.TAG_SWITCH_ALT),        # hand-over to the small tag
             ("D", states == "FINAL"))            # committed to touchdown
    keys, since = {}, 0
    for label, cond in conds:
        later = np.where(cond[since:])[0]
        idx = since + (later[0] if len(later) else 0)
        since = idx
        keys[label] = (float(x[idx] - m.tag.pos[0]), float(y[idx] - m.tag.pos[1]),
                       float(z[idx]))
        ax1.scatter([x[idx]], [y[idx]], [z[idx]], color=ORANGE, s=28)
        ax1.text(x[idx], y[idx], z[idx] + 0.35, label, fontsize=9, weight="bold")
    ax1.set_xlabel("x (m)"); ax1.set_ylabel("y (m)"); ax1.set_zlabel("h (m)")
    ax1.set_title("landing trajectory, static pad (address 5)")
    ax1.view_init(elev=22, azim=-58)

    ax2 = fig.add_subplot(1, 2, 2)
    err = np.hypot(L.col("ex"), L.col("ey"))
    ax2.plot(t, z, color=BLUE, lw=1.5, label="altitude h")
    ax2.plot(t, err, color=ORANGE, lw=1.3, label="horizontal offset")
    ax2.axhline(C.TAG_SWITCH_ALT, color=GREY, ls=":", lw=1.0)
    ax2.text(t[-1] * 0.02, C.TAG_SWITCH_ALT + 0.12, "tag switch 0.30 m -> 0.06 m",
             fontsize=7.5, color="#404a55")
    seen = L.col("seen")
    ax2.fill_between(t, 0, 7, where=seen > 0.5, color=GREEN, alpha=0.07, lw=0)
    ax2.set_xlabel("t (s)"); ax2.set_ylabel("m")
    ax2.set_ylim(0, 7)
    ax2.set_title("altitude and offset (green = AprilTag decoded)")
    ax2.legend(loc="upper right", fontsize=8)
    fig.suptitle("Fig. 2  Static landing: approach, tag hand-over, touchdown",
                 fontsize=11, fontweight="bold", y=1.02)
    path = save(fig, "fig2_static_landing.png")
    return path, keys, m.result


# ------------------------------------------------------ E2: dynamic tracking
def fig_dynamic_tracking():
    m = fly(4, seed=3)
    L = m.log
    t = L.col("t"); ex, ey = L.col("ex"), L.col("ey")
    track = np.array([r["state"] in ("ALIGN", "DESCEND", "FINAL") for r in L.rows])

    fig, axes = plt.subplots(1, 2, figsize=(11.0, 4.0))
    ax = axes[0]
    x, y = L.col("x"), L.col("y")
    lock = int(np.argmax(track))                 # first sample with a vision lock
    ax.plot(x[:lock + 1], y[:lock + 1], color="#9ec5e8", lw=1.5,
            label="drone, radio guidance")
    ax.plot(x[lock:], y[lock:], color=BLUE, lw=1.6, label="drone, vision tracking")
    ax.plot(L.col("tag_x")[lock:], L.col("tag_y")[lock:], color=ORANGE, lw=1.4,
            ls="--", label="pad, moving at 1.6 m/s")
    ax.scatter([x[0]], [y[0]], color=GREY, s=40, zorder=4, marker="^", label="launch")
    ax.scatter([x[lock]], [y[lock]], color=BLUE, s=45, zorder=4, marker="*",
               label="AprilTag lock")
    ax.scatter([x[-1]], [y[-1]], color=GREEN, s=55, zorder=5, label="touchdown")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_aspect("equal")
    ax.legend(fontsize=7.5, loc="lower left")
    ax.set_title("ground tracks, address 4 (the pad reverses at 26 m)")

    ax = axes[1]
    ax.plot(t[track], ex[track], color=BLUE, lw=1.2, label="x offset")
    ax.plot(t[track], ey[track], color=ORANGE, lw=1.2, label="y offset")
    ax.axhspan(-0.2, 0.5, color=GREY, alpha=0.16, lw=0)
    ax.text(t[track][0], 0.55, "band reported in the base paper (-0.2 m, +0.5 m)",
            fontsize=7.5, color="#404a55")
    ax.axhline(0, color="#8b949e", lw=0.8)
    ax.set_xlabel("t (s)"); ax.set_ylabel("offset (m)")
    ax.set_ylim(-1.2, 1.2)
    ax.legend(fontsize=8, loc="lower right")
    ax.set_title("offset while tracking, from vision lock to touchdown")
    fig.suptitle("Fig. 3  Dynamic tracking of a pad on a moving vehicle",
                 fontsize=11, fontweight="bold", y=1.03)
    path = save(fig, "fig3_dynamic_tracking.png")

    # The band the paper quotes is the STEADY tracking offset, so it is measured
    # after the acquisition transient - from the start of the descent onwards.
    steady = np.array([r["state"] in ("DESCEND", "FINAL") for r in L.rows])
    band = (float(np.min(np.r_[ex[steady], ey[steady]])),
            float(np.max(np.r_[ex[steady], ey[steady]])))
    return path, band, m.result


# ------------------------------------------------------- E3: step response
def fig_step_response():
    """
    Reproduction of Fig.10 vs Fig.12 of the base paper.

    Section 3.3: "the velocity input of UAV is the displacement deviation
    between the actual camera and the landing target". So the experiment is: put
    the drone 3 m off a static pad, feed the offset to the velocity API two ways,
    and watch what the airframe actually does.

      (a) no PID - the offset is sent straight to the velocity API. The servo
          lag turns that into overshoot: the drone flies past the pad and
          oscillates back, exactly the behaviour the paper shows in Fig.10.
      (b) PID eq.(9) - the derivative term anticipates the lag and the offset
          settles without crossing zero.
    """
    from controller import PID

    dt, n = C.DT, int(25.0 / C.DT)
    t = np.arange(n) * dt
    runs = {}

    for label in ("no_pid", "pid"):
        d = Drone((3.0, 0.0, 3.0), seed=0)
        px = PID()
        err_hist, cmd_hist, ach_hist = [], [], []
        for _ in range(n):
            err = -d.p[0]                      # pad sits at x = 0
            if label == "no_pid":
                u = 1.4 * err                  # deviation straight into the API
            else:
                u = px.step(err, dt)           # eq. (9) at the paper's own gains
            u = float(np.clip(u, -C.V_MAX_TRACK, C.V_MAX_TRACK))
            cmd_hist.append(u)
            d.step([u, 0.0, 0.0], dt, wind=False)
            ach_hist.append(d.v[0])
            err_hist.append(err)
        runs[label] = (np.array(err_hist), np.array(cmd_hist), np.array(ach_hist))

    def overshoot(err):
        """
        Percent of the initial offset that the drone flies PAST the pad.

        err starts at -3 m (drone 3 m beyond the pad in +x) and should approach
        zero from below; anything positive is overshoot.
        """
        return 100.0 * max(0.0, float(np.max(err))) / 3.0

    fig, axes = plt.subplots(1, 3, figsize=(13.0, 3.5))
    for ax, key, ttl, col in ((axes[0], "no_pid", "(a) without PID", RED),
                              (axes[1], "pid", "(b) with PID, eq. (9)", GREEN)):
        err, cmd, ach = runs[key]
        ax.plot(t, cmd, color=GREY, ls="--", lw=1.1, label="velocity command")
        ax.plot(t, ach, color=col, lw=1.5, label="velocity achieved")
        ax.axhline(0, color="#b9c0c9", lw=0.8)
        ax.set_xlabel("t (s)"); ax.set_ylabel("velocity (m/s)")
        ax.set_title("%s   overshoot %.1f %%" % (ttl, overshoot(err)))
        ax.legend(fontsize=8, loc="lower right")

    ax = axes[2]
    for key, ttl, col in (("no_pid", "without PID", RED), ("pid", "with PID", GREEN)):
        ax.plot(t, runs[key][0], color=col, lw=1.5, label=ttl)
    ax.axhline(0, color="#b9c0c9", lw=0.8)
    ax.axhspan(-0.15, 0.15, color=GREEN, alpha=0.10, lw=0)
    ax.set_xlabel("t (s)"); ax.set_ylabel("offset to pad (m)")
    ax.set_title("(c) the offset the two loops produce")
    ax.legend(fontsize=8)
    fig.suptitle("Fig. 4  Velocity-loop response: the overshoot the PID removes",
                 fontsize=11, fontweight="bold", y=1.04)
    return (save(fig, "fig4_step_response.png"),
            overshoot(runs["no_pid"][0]), overshoot(runs["pid"][0]))


# -------------------------------------------------- E4: controller ablation
def fig_ablation():
    cfgs = [("paper PID only", dict(feedforward=False, gain_schedule=False),
             RED, "-", 1.4),
            ("+ pad-velocity feed-forward", dict(feedforward=True, gain_schedule=False),
             ORANGE, "--", 2.4),
            ("+ altitude gain scheduling", dict(feedforward=True, gain_schedule=True),
             GREEN, "-", 1.4)]
    fig, axes = plt.subplots(1, 2, figsize=(11.0, 3.9))
    table = []
    for name, kw, col, ls, lw in cfgs:
        m = fly(6, seed=2, **kw)
        L = m.log
        t = L.col("t"); e = np.hypot(L.col("ex"), L.col("ey"))
        keep = np.array([r["state"] in ("ALIGN", "DESCEND", "FINAL") for r in L.rows])
        rel = t[keep] - t[keep][0]
        axes[0].plot(rel, e[keep], color=col, ls=ls, lw=lw, label=name)
        axes[1].plot(rel, L.col("z")[keep], color=col, ls=ls, lw=lw, label=name)
        tail = e[keep][-int(5 / C.DT):]
        table.append({"config": name, "outcome": m.result["outcome"],
                      "final_error_m": round(m.result["error_m"], 3),
                      "time_s": round(m.result["time_s"], 1),
                      "steady_offset_m": round(float(np.mean(tail)), 3)})

    # Only the first 40 s are interesting: the two working configurations have
    # landed by then, and the failing one just keeps circling until the timeout.
    axes[0].axhline(C.FINAL_TOL, color=GREY, ls=":", lw=1.0)
    axes[0].text(24, C.FINAL_TOL + 0.04, "landing tolerance 0.12 m",
                 fontsize=7.5, color="#404a55")
    axes[0].set_xlabel("time since vision lock (s)"); axes[0].set_ylabel("offset (m)")
    axes[0].set_title("tracking offset, pad circling at 1.1 m/s")
    axes[0].set_xlim(0, 40); axes[0].set_ylim(0, 2.0)
    axes[0].legend(fontsize=8, loc="upper right")
    axes[1].set_xlabel("time since vision lock (s)"); axes[1].set_ylabel("altitude (m)")
    axes[1].set_title("descent: a standing offset stalls it")
    axes[1].set_xlim(0, 40)
    axes[1].annotate("PID alone never enters the cone;\nmission times out at 240 s",
                     xy=(30, 5.9), xytext=(14, 4.2), fontsize=8, color=RED,
                     arrowprops=dict(arrowstyle="->", color=RED, lw=1.0))
    axes[1].legend(fontsize=8, loc="center left")
    fig.suptitle("Fig. 5  Why fixed-gain PID alone does not land on a moving pad",
                 fontsize=11, fontweight="bold", y=1.04)
    path = save(fig, "fig5_ablation.png")

    # One trace proves nothing on its own, so the same three configurations are
    # run over every moving address and several noise seeds.
    sweep = []
    for name, kw, _, _, _ in cfgs:
        landed, errs, times = 0, [], []
        for seed in range(4):
            for tid in (2, 4, 6, 8, 10):
                r = fly(tid, seed=seed, **kw).result
                if r["outcome"] == "LANDED":
                    landed += 1
                    errs.append(r["error_m"]); times.append(r["time_s"])
        sweep.append({
            "config": name, "runs": 20, "landed": landed,
            "mean_error_m": round(float(np.mean(errs)), 3) if errs else None,
            "mean_time_s": round(float(np.mean(times)), 1) if times else None})
    return path, {"trace_address_6": table, "all_moving_addresses": sweep}


# ------------------------------------------------------------ E5: monte carlo
def fig_monte_carlo(seeds=12):
    fleet_ref = build_fleet()
    rows, per_tag = [], {t.id: [] for t in fleet_ref}
    for s in range(seeds):
        for t in fleet_ref:
            m = fly(t.id, seed=s)
            r = m.result
            rows.append(r)
            per_tag[t.id].append(r)
    ok = [r for r in rows if r["outcome"] == "LANDED"]
    err = np.array([r["error_m"] for r in ok])
    stat = np.array([r["error_m"] for r in ok if not r["moving"]])
    mov = np.array([r["error_m"] for r in ok if r["moving"]])

    fig, axes = plt.subplots(1, 2, figsize=(11.0, 3.9))
    ax = axes[0]
    for t in fleet_ref:
        e = [r["error_m"] for r in per_tag[t.id] if r["outcome"] == "LANDED"]
        col = ORANGE if t.motion != "static" else BLUE
        ax.scatter([t.id] * len(e), e, color=col, alpha=0.65, s=22)
        if e:
            ax.scatter([t.id], [np.mean(e)], color="black", marker="_", s=220)
    ax.set_xticks(range(1, 11)); ax.set_xlabel("AirTag address")
    ax.set_ylabel("touchdown error (m)")
    ax.set_title("touchdown error per address, %d runs each" % seeds)
    ax.axhline(0.15, color=GREY, ls=":", lw=1.0)
    ax.text(0.6, 0.155, "0.15 m", fontsize=7.5, color="#404a55")

    ax = axes[1]
    bins = np.linspace(0, max(0.25, err.max() * 1.05), 22)
    ax.hist(stat, bins=bins, color=BLUE, alpha=0.75, label="static pads")
    ax.hist(mov, bins=bins, color=ORANGE, alpha=0.75, label="moving pads")
    ax.set_xlabel("touchdown error (m)"); ax.set_ylabel("runs")
    ax.set_title("error distribution, %d missions" % len(rows))
    ax.legend(fontsize=8)
    fig.suptitle("Fig. 6  Repeatability across all ten addresses",
                 fontsize=11, fontweight="bold", y=1.04)
    path = save(fig, "fig6_monte_carlo.png")

    summary = {
        "missions": len(rows),
        "landed": len(ok),
        "success_rate": round(100.0 * len(ok) / len(rows), 1),
        "mean_error_m": round(float(err.mean()), 3),
        "median_error_m": round(float(np.median(err)), 3),
        "p95_error_m": round(float(np.percentile(err, 95)), 3),
        "max_error_m": round(float(err.max()), 3),
        "static_mean_m": round(float(stat.mean()), 3),
        "moving_mean_m": round(float(mov.mean()), 3),
        "mean_time_s": round(float(np.mean([r["time_s"] for r in ok])), 1),
        "per_address": {
            str(t.id): {
                "address": t.address,
                "moving": t.motion != "static",
                "landed": sum(1 for r in per_tag[t.id] if r["outcome"] == "LANDED"),
                "runs": seeds,
                "mean_error_m": round(float(np.mean(
                    [r["error_m"] for r in per_tag[t.id]
                     if r["outcome"] == "LANDED"] or [np.nan])), 3),
                "mean_time_s": round(float(np.mean(
                    [r["time_s"] for r in per_tag[t.id]
                     if r["outcome"] == "LANDED"] or [np.nan])), 1),
            } for t in fleet_ref},
    }
    return path, summary


# ------------------------------------------------------------ E6: sensor model
def fig_sensor_model():
    cam = AprilTagCamera(seed=0)
    h = np.linspace(0.05, 9.0, 500)
    fig, axes = plt.subplots(1, 2, figsize=(11.0, 3.7))

    ax = axes[0]
    for size, col, lab in ((C.TAG_BIG, BLUE, "0.30 m tag"),
                           (C.TAG_SMALL, ORANGE, "0.06 m tag")):
        px = C.FOCAL_PX * size / h
        usable = (px >= C.MIN_TAG_PX) & (px <= C.MAX_TAG_FRAC * C.IMG_H)
        ax.plot(h, px, color=col, lw=1.5, label=lab)
        ax.fill_between(h, 0, 1e3, where=usable, color=col, alpha=0.10, lw=0)
    ax.axhline(C.MIN_TAG_PX, color=GREY, ls="--", lw=1.0)
    ax.text(6.2, C.MIN_TAG_PX * 1.15, "decode limit %d px" % C.MIN_TAG_PX,
            fontsize=7.5, color="#404a55")
    ax.axhline(C.MAX_TAG_FRAC * C.IMG_H, color=GREY, ls="--", lw=1.0)
    ax.text(4.0, C.MAX_TAG_FRAC * C.IMG_H * 1.06, "tag overflows the frame",
            fontsize=7.5, color="#404a55")
    ax.set_yscale("log"); ax.set_xlabel("altitude (m)")
    ax.set_ylabel("apparent tag edge (px)")
    ax.set_title("why two tag sizes: shaded = usable altitude band")
    ax.legend(fontsize=8)

    ax = axes[1]
    for size, col, lab in ((C.TAG_BIG, BLUE, "0.30 m tag"),
                           (C.TAG_SMALL, ORANGE, "0.06 m tag")):
        px = C.FOCAL_PX * size / h
        sigma = C.DETECT_NOISE_K * h / px
        usable = (px >= C.MIN_TAG_PX) & (px <= C.MAX_TAG_FRAC * C.IMG_H)
        s = np.where(usable, sigma, np.nan)
        ax.plot(h, s * 100, color=col, lw=1.6, label=lab)
    ax.axhline(C.BEACON_SIGMA * 100, color=GREEN, ls="--", lw=1.2)
    ax.text(4.6, C.BEACON_SIGMA * 100 * 1.05, "AirTag radio fix (250 cm)",
            fontsize=7.5, color=GREEN)
    ax.set_yscale("log"); ax.set_xlabel("altitude (m)")
    ax.set_ylabel("1-sigma position error (cm)")
    ax.set_title("positioning accuracy: radio hands over to vision")
    ax.legend(fontsize=8)
    fig.suptitle("Fig. 7  Sensor models used by the simulator",
                 fontsize=11, fontweight="bold", y=1.04)
    return save(fig, "fig7_sensor_model.png")


# --------------------------------------------------------------- fig: phases
def fig_phase_timeline():
    m = fly(8, seed=1)
    L = m.log
    t, z = L.col("t"), L.col("z")
    states = [r["state"] for r in L.rows]
    colors = {"TAKEOFF": "#c9d3de", "TRANSIT": "#9ec5e8", "SEARCH": "#c7b3e8",
              "ALIGN": "#f6d08a", "DESCEND": "#f2b179", "FINAL": "#9ad9b3",
              "LANDED": "#79c99a", "ABORTED": "#e79a94"}
    fig, ax = plt.subplots(figsize=(10.6, 3.5))
    start = 0
    for i in range(1, len(states) + 1):
        if i == len(states) or states[i] != states[start]:
            ax.axvspan(t[start], t[i - 1], color=colors.get(states[start], "#eee"),
                       alpha=0.75, lw=0)
            if t[i - 1] - t[start] > 1.5:
                ax.text((t[start] + t[i - 1]) / 2, 6.3, states[start], ha="center",
                        fontsize=7.5, color="#333c46", rotation=0)
            start = i
    ax.plot(t, z, color="#12324f", lw=1.8, label="altitude")
    ax.plot(t, np.hypot(L.col("ex"), L.col("ey")), color=RED, lw=1.2,
            label="offset to pad")
    for et, txt in L.events:
        if "tag switch" in txt:
            ax.axvline(et, color=GREEN, lw=1.2, ls="--")
            ax.text(et + 0.3, 1.9, "0.30 m -> 0.06 m tag", fontsize=7.5, color=GREEN)
    ax.set_xlabel("t (s)"); ax.set_ylabel("m"); ax.set_ylim(0, 6.8)
    ax.legend(fontsize=8, loc="upper right", framealpha=0.95)
    ax.set_title("Fig. 8  Mission phases, address 8 (pad on a service cart)")
    return save(fig, "fig8_phases.png")


def main():
    print("\n  running experiments ...")
    out = {}
    out["arena_fig"] = fig_arena()
    _, keys, static_res = fig_static_landing()
    out["static"] = {"key_points": {k: [round(v, 4) for v in val]
                                    for k, val in keys.items()},
                     "result": static_res}
    _, band, dyn_res = fig_dynamic_tracking()
    out["dynamic"] = {"offset_band_m": [round(b, 3) for b in band],
                      "result": dyn_res}
    _, ov_raw, ov_pid = fig_step_response()
    out["step_response"] = {"overshoot_no_pid_pct": round(float(ov_raw), 1),
                            "overshoot_pid_pct": round(float(ov_pid), 1)}
    _, ablation = fig_ablation()
    out["ablation"] = ablation
    _, mc = fig_monte_carlo(seeds=12)
    out["monte_carlo"] = mc
    fig_sensor_model()
    fig_phase_timeline()

    with open(os.path.join(RES, "summary.json"), "w") as f:
        json.dump(out, f, indent=2)
    print("\n  wrote results/summary.json")
    print("  success %.1f %%   mean error %.3f m   p95 %.3f m"
          % (mc["success_rate"], mc["mean_error_m"], mc["p95_error_m"]))


if __name__ == "__main__":
    main()
