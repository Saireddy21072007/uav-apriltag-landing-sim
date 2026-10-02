"""
Batch experiments and every figure used in the review document.

    python evaluate.py                 everything (takes a while - real rendering
                                       and real tag decoding on every frame)
    python evaluate.py --quick         fewer seeds, for a smoke test
    python evaluate.py --only mc       run one experiment by name

Experiments
  E1  perception    measured accuracy of the vision pipeline against MuJoCo truth
  E2  static        landing on a fixed pad, with the paper's key-point table
  E3  dynamic       landing on a pad carried by a moving vehicle
  E4  ablation      paper PID | + feed-forward | + gain scheduling
  E5  mc            Monte Carlo over all ten addresses
"""
import argparse
import json
import contextlib
import os
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import config as C
from control import quaternion as Q
from mission import LandingMission
from perception.vision import PadVision
from physics.quad import QuadEnv

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIG = os.path.join(ROOT, "figures")
RES = os.path.join(ROOT, "results")
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

_ENV = None
_VIS = None


def shared():
    """One environment and one detector for the whole run - both are expensive."""
    global _ENV, _VIS
    if _ENV is None:
        _ENV = QuadEnv(render_camera=True, seed=0)
        _VIS = PadVision()
    return _ENV, _VIS


@contextlib.contextmanager
def extra_renderer(env, height, width):
    """
    A second renderer, for the wide shots the figures need.

    It has to be closed explicitly and the environment's own renderer rebuilt
    afterwards: see QuadEnv.renew_renderer. Getting this wrong is silent - the
    onboard camera goes black, the detector finds nothing in a valid image, and
    the perception experiment reports a 0 % detection rate rather than failing.
    """
    r = mujoco.Renderer(env.model, height, width)
    r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
    try:
        yield r
    finally:
        r.close()
        env.renew_renderer()


def fly(pad_id, seed=0, **kw):
    env, vis = shared()
    env.reset(seed=seed)
    m = LandingMission(env, pad_id, seed=seed, vision=vis, **kw)
    m.run()
    return m


def save(fig, name):
    path = os.path.join(FIG, name)
    fig.savefig(path, dpi=180)
    plt.close(fig)
    print("   wrote figures/%s" % name, flush=True)
    return path


# ==================================================================== E0 scene
def exp_scene():
    """
    What the world looks like, and what the drone actually sees in it.

    The onboard panels are real renders through the drone's camera with the real
    detector run on them; the green outlines are the tags it decoded.
    """
    import cv2
    env, vis = shared()
    env.reset(seed=0)

    with extra_renderer(env, 900, 1200) as big:
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = [0.0, -4.5, 1.0]
        cam.distance, cam.elevation, cam.azimuth = 38.0, -52.0, 96.0
        mujoco.mj_forward(env.model, env.data)
        big.update_scene(env.data, camera=cam)
        overview = big.render()

    shots = []
    pad = env.pad_by_id(4)
    for alt in (8.0, 2.0, 0.45):
        env.data.qpos[env.qadr:env.qadr + 3] = [pad.pos[0] + 0.05 * alt,
                                                pad.pos[1] - 0.04 * alt, alt]
        env.data.qpos[env.qadr + 3:env.qadr + 7] = [1, 0, 0, 0]
        mujoco.mj_forward(env.model, env.data)
        frame = env.camera_frame().copy()
        for s_ in vis.look(frame, Q.identity()):
            pts = s_.corners.astype(np.int32)
            cv2.polylines(frame, [pts], True, (40, 230, 90), 2)
            c = pts.mean(axis=0).astype(int)
            cv2.putText(frame, "id %d" % s_.tag_id, (c[0] - 24, c[1] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (40, 230, 90), 2, cv2.LINE_AA)
        shots.append((alt, frame))

    fig = plt.figure(figsize=(12.6, 5.4))
    gs = fig.add_gridspec(3, 2, width_ratios=[1.35, 1.0], hspace=0.18, wspace=0.06)
    ax = fig.add_subplot(gs[:, 0]); ax.imshow(overview); ax.axis("off")
    ax.set_title("the street: ten addressed pads - ground, vehicles and roofs",
                 fontsize=10)
    for i, (alt, frame) in enumerate(shots):
        a = fig.add_subplot(gs[i, 1]); a.imshow(frame); a.axis("off")
        a.set_title("onboard camera at %.2f m" % alt, fontsize=9)
    fig.suptitle("Fig. 0  The simulated world and the images the detector works on",
                 fontsize=11, fontweight="bold", y=0.98)
    return save(fig, "mj_fig0_scene.png")


# ================================================================ E0b addressing
def exp_addressing(samples=60, seed=11):
    """
    Does the addressing actually do anything?

    It only means something if other pads are genuinely in frame and genuinely
    ignored. The drone is placed at random points along the street at search
    altitude; for each frame we count how many DIFFERENT pads the detector
    decoded, and how many of those an address-filtered query returns.
    """
    env, vis = shared()
    rng = np.random.default_rng(seed)
    env.reset(seed=seed)
    counts, filtered, ids_seen = [], [], set()
    for _ in range(samples):
        y = float(rng.uniform(-20.0, 11.0))
        x = float(rng.uniform(-2.2, 2.2))
        alt = float(rng.uniform(6.0, 8.5))
        env.data.qpos[env.qadr:env.qadr + 3] = [x, y, alt]
        env.data.qpos[env.qadr + 3:env.qadr + 7] = [1, 0, 0, 0]
        mujoco.mj_forward(env.model, env.data)
        frame = env.camera_frame()
        all_s = vis.look(frame, env.imu())
        pads = {s.pad_id for s in all_s}
        counts.append(len(pads))
        ids_seen |= {s.tag_id for s in all_s}
        if pads:
            want = sorted(pads)[0]
            got = vis.look(frame, env.imu(), only_pad=want)
            filtered.append(len({s.pad_id for s in got}))
    counts = np.array(counts)

    fig, ax = plt.subplots(figsize=(5.6, 3.4))
    vals, edges = np.histogram(counts, bins=np.arange(-0.5, counts.max() + 1.5))
    ax.bar(edges[:-1] + 0.5, 100.0 * vals / len(counts), width=0.7, color=BLUE, alpha=0.85)
    ax.set_xlabel("different pads decoded in one frame")
    ax.set_ylabel("% of frames")
    ax.set_xticks(range(0, int(counts.max()) + 1))
    ax.set_title("Fig. A2  At search altitude the camera usually sees several addresses")
    path = save(fig, "mj_fig1b_addressing.png")

    return path, {
        "samples": int(samples),
        "mean_pads_in_frame": round(float(counts.mean()), 2),
        "max_pads_in_frame": int(counts.max()),
        "frames_with_multiple_pads_pct": round(100.0 * float((counts > 1).mean()), 1),
        "filtered_always_single_pad": bool(all(f <= 1 for f in filtered)),
        "distinct_tag_ids_seen": sorted(int(i) for i in ids_seen),
    }


# =============================================================== E1 perception
def exp_perception(samples=90, seed=3):
    """
    How accurate is the vision pipeline, measured against MuJoCo ground truth?

    The drone is teleported to random poses around a pad, a real frame is
    rendered, the real detector runs, and the recovered pad-centre offset is
    compared with the true one. No control loop is involved.
    """
    env, vis = shared()
    rng = np.random.default_rng(seed)
    pad = env.pad_by_id(4)
    truth = pad.pos.copy()
    rows = []
    for _ in range(samples):
        alt = float(rng.uniform(0.25, 8.5))
        rad = min(0.55 * alt, 2.5)
        dx, dy = rng.uniform(-rad, rad, 2)
        roll, pitch = rng.uniform(-0.20, 0.20, 2)
        yaw = rng.uniform(-np.pi, np.pi)
        env.data.qpos[env.qadr:env.qadr + 3] = [truth[0] + dx, truth[1] + dy, alt]
        cr, sr = np.cos(roll / 2), np.sin(roll / 2)
        cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
        cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
        env.data.qpos[env.qadr + 3:env.qadr + 7] = [
            cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]
        mujoco.mj_forward(env.model, env.data)
        s = PadVision.pick(vis.look(env.camera_frame(), env.imu(), only_pad=4))
        if s is None:
            rows.append((alt, np.nan, None))
            continue
        err = float(np.linalg.norm(s.offset - np.array([-dx, -dy])))
        rows.append((alt, err, s.is_large))

    alts = np.array([r[0] for r in rows])
    errs = np.array([r[1] for r in rows])
    big = np.array([r[2] is True for r in rows])
    small = np.array([r[2] is False for r in rows])
    miss = np.isnan(errs)

    fig, axes = plt.subplots(1, 2, figsize=(11.0, 3.8))
    ax = axes[0]
    ax.scatter(alts[big], 100 * errs[big], s=22, color=BLUE, alpha=0.75,
               label="0.42 m tag")
    ax.scatter(alts[small], 100 * errs[small], s=22, color=ORANGE, alpha=0.75,
               label="0.12 m tag")
    if miss.any():
        ax.scatter(alts[miss], np.full(miss.sum(), 0.4), marker="x", s=26,
                   color=RED, label="no detection")
    ax.axhline(C.BEACON_SIGMA * 100, color=GREEN, ls="--", lw=1.2)
    ax.text(4.4, C.BEACON_SIGMA * 105, "AirTag radio fix (250 cm)", fontsize=7.5,
            color=GREEN)
    ax.set_yscale("log")
    ax.set_xlabel("altitude (m)"); ax.set_ylabel("pad-centre error (cm)")
    ax.set_title("measured positioning error vs altitude")
    ax.legend(fontsize=8)

    ax = axes[1]
    bins = np.linspace(0, 9, 19)
    idx = np.digitize(alts, bins)
    rate = [100.0 * np.mean(~np.isnan(errs[idx == i])) if (idx == i).any() else np.nan
            for i in range(1, len(bins))]
    ax.bar(bins[:-1] + 0.25, rate, width=0.42, color=BLUE, alpha=0.85)
    ax.set_xlabel("altitude (m)"); ax.set_ylabel("frames with a decode (%)")
    ax.set_ylim(0, 105)
    ax.set_title("detection rate vs altitude")
    fig.suptitle("Fig. A  The vision pipeline, measured against MuJoCo ground truth",
                 fontsize=11, fontweight="bold", y=1.04)
    path = save(fig, "mj_fig1_perception.png")

    ok = ~np.isnan(errs)
    # A zero detection rate is not a result, it is a broken camera: a black
    # frame is a valid image that simply contains no marker, so nothing
    # downstream raises. Fail loudly here instead of writing NaNs into the
    # summary the report is generated from.
    assert ok.any(), ("the detector found nothing in %d rendered frames - the "
                      "onboard camera is almost certainly rendering black; see "
                      "QuadEnv.renew_renderer" % samples)
    return path, {
        "samples": int(samples),
        "detected_pct": round(100.0 * ok.mean(), 1),
        "mean_error_cm": round(100 * float(np.nanmean(errs)), 2),
        "median_error_cm": round(100 * float(np.nanmedian(errs)), 2),
        "p95_error_cm": round(100 * float(np.nanpercentile(errs[ok], 95)), 2),
        "large_tag_mean_cm": round(100 * float(np.nanmean(errs[big])), 2) if big.any() else None,
        "small_tag_mean_cm": round(100 * float(np.nanmean(errs[small])), 2) if small.any() else None,
        "geometry": C.check_geometry(),
    }


# =================================================================== E2 static
def exp_static(pad_id=4, seed=3):
    m = fly(pad_id, seed=seed)
    L = m.log
    t, x, y, z = L.col("t"), L.col("x"), L.col("y"), L.col("z")
    deck = m.pad.deck_height
    states = np.array([r["state"] for r in L.rows])

    keys, since = {}, 0
    for label, cond in (("A", states == "TRANSIT"),
                        ("B", states == "DESCEND"),
                        ("C", L.col("tag") == 2),
                        ("D", states == "FINAL")):
        later = np.where(cond[since:])[0]
        idx = since + (later[0] if len(later) else 0)
        since = idx
        keys[label] = (round(float(x[idx] - m.pad.pos[0]), 4),
                       round(float(y[idx] - m.pad.pos[1]), 4),
                       round(float(z[idx] - deck), 3))

    fig = plt.figure(figsize=(11.0, 4.3))
    ax1 = fig.add_subplot(1, 2, 1, projection="3d")
    ax1.plot(x, y, z, color=BLUE, lw=1.5)
    ax1.scatter([m.pad.pos[0]], [m.pad.pos[1]], [deck], color=RED, s=45, marker="s")
    for label, (kx, ky, kz) in keys.items():
        ax1.scatter([kx + m.pad.pos[0]], [ky + m.pad.pos[1]], [kz + deck],
                    color=ORANGE, s=26)
        ax1.text(kx + m.pad.pos[0], ky + m.pad.pos[1], kz + deck + 0.3, label,
                 fontsize=9, weight="bold")
    ax1.set_xlabel("x (m)"); ax1.set_ylabel("y (m)"); ax1.set_zlabel("h (m)")
    ax1.set_title("trajectory, static pad (address %d)" % pad_id)
    ax1.view_init(elev=20, azim=-62)

    ax2 = fig.add_subplot(1, 2, 2)
    ax2.plot(t, z - deck, color=BLUE, lw=1.5, label="height above deck")
    ax2.plot(t, L.col("err"), color=ORANGE, lw=1.3, label="offset to pad centre")
    tag = L.col("tag")
    ax2.fill_between(t, 0, 9, where=tag == 1, color=BLUE, alpha=0.07, lw=0)
    ax2.fill_between(t, 0, 9, where=tag == 2, color=ORANGE, alpha=0.12, lw=0)
    ax2.set_ylim(0, 9)
    ax2.set_xlabel("t (s)"); ax2.set_ylabel("m")
    ax2.set_title("blue band = 0.42 m tag in use, orange = 0.12 m tag")
    ax2.legend(fontsize=8)
    fig.suptitle("Fig. B  Static landing with real AprilTag detection in the loop",
                 fontsize=11, fontweight="bold", y=1.02)
    path = save(fig, "mj_fig2_static.png")
    return path, keys, m.result


# ================================================================== E3 dynamic
def exp_dynamic(pad_id=3, seed=3):
    m = fly(pad_id, seed=seed)
    L = m.log
    t, ex, ey = L.col("t"), L.col("ex"), L.col("ey")
    track = np.array([r["state"] in ("ALIGN", "DESCEND", "FINAL") for r in L.rows])
    lock = int(np.argmax(track))

    fig, axes = plt.subplots(1, 3, figsize=(13.4, 4.2),
                             gridspec_kw={"width_ratios": [0.8, 1.25, 1.1]})
    ax = axes[0]
    ax.plot(L.col("x")[:lock + 1], L.col("y")[:lock + 1], color="#9ec5e8", lw=1.4,
            label="radio guidance")
    ax.plot(L.col("x")[lock:], L.col("y")[lock:], color=BLUE, lw=1.6,
            label="vision tracking")
    ax.plot(L.col("pad_x")[lock:], L.col("pad_y")[lock:], color=ORANGE, ls="--",
            lw=1.4, label="pad")
    ax.scatter([L.col("x")[-1]], [L.col("y")[-1]], color=GREEN, s=52, zorder=5,
               label="touchdown")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(fontsize=7, loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=2)
    ax.set_title("ground tracks, address %d" % pad_id)

    ax = axes[1]
    ax.plot(t[track], ex[track], color=BLUE, lw=1.2, label="x offset")
    ax.plot(t[track], ey[track], color=ORANGE, lw=1.2, label="y offset")
    ax.axhspan(-0.2, 0.5, color=GREY, alpha=0.16, lw=0)
    ax.text(t[track][0], 0.55, "band reported in the base paper", fontsize=7.5,
            color="#404a55")
    ax.axhline(0, color="#8b949e", lw=0.8)
    ax.set_xlabel("t (s)"); ax.set_ylabel("offset (m)"); ax.set_ylim(-1.0, 1.0)
    ax.legend(fontsize=8, loc="lower right")
    ax.set_title("tracking offset while descending")

    ax = axes[2]
    ax.plot(t, np.degrees(np.hypot(L.col("roll"), L.col("pitch"))), color=RED, lw=1.1,
            label="tilt")
    ax.plot(t, L.col("pad_speed"), color=GREEN, lw=1.2, label="pad speed (m/s)")
    ax.set_xlabel("t (s)"); ax.set_ylabel("deg  /  m/s")
    ax.legend(fontsize=8)
    ax.set_title("the drone has to tilt to chase the pad")
    fig.suptitle("Fig. C  Landing on a pad carried by a moving vehicle",
                 fontsize=11, fontweight="bold", y=1.03)
    path = save(fig, "mj_fig3_dynamic.png")

    steady = np.array([r["state"] in ("DESCEND", "FINAL") for r in L.rows])
    band = (round(float(np.min(np.r_[ex[steady], ey[steady]])), 3),
            round(float(np.max(np.r_[ex[steady], ey[steady]])), 3))
    return path, band, m.result


# ================================================================= E4 ablation
def exp_ablation(seeds=2, trace_pad=3):
    base = dict(feedforward=False, gain_schedule=False, anti_windup=False,
                model_derivative=False, tracker="abg")
    cfgs = [("paper PID only", dict(base),
             RED, "-", 1.4),
            ("+ integral anti-windup", dict(base, anti_windup=True),
             "#b07aa1", ":", 1.6),
            ("+ pad-velocity feed-forward",
             dict(base, anti_windup=True, feedforward=True),
             ORANGE, "--", 2.4),
            ("+ altitude gain scheduling",
             dict(base, anti_windup=True, feedforward=True, gain_schedule=True),
             GREEN, "-", 1.4),
            ("+ coordinated-turn tracker",
             dict(base, anti_windup=True, feedforward=True, gain_schedule=True,
                  tracker="ct"),
             "#7b5cd6", "-", 2.0)]

    fig, axes = plt.subplots(1, 2, figsize=(11.0, 3.9))
    table = []
    for name, kw, col, ls, lw in cfgs:
        m = fly(trace_pad, seed=1, **kw)
        L = m.log
        keep = np.array([r["state"] in ("ALIGN", "DESCEND", "FINAL") for r in L.rows])
        if keep.any():
            rel = L.col("t")[keep] - L.col("t")[keep][0]
            axes[0].plot(rel, L.col("err")[keep], color=col, ls=ls, lw=lw, label=name)
            axes[1].plot(rel, L.col("z")[keep] - m.pad.deck_height, color=col,
                         ls=ls, lw=lw, label=name)
        table.append({"config": name, "outcome": m.result["outcome"],
                      "error_m": round(m.result["error_m"], 3),
                      "time_s": m.result["time_s"]})

    axes[0].axhline(C.FINAL_TOL, color=GREY, ls=":", lw=1.0)
    axes[0].text(2, C.FINAL_TOL + 0.03, "landing tolerance", fontsize=7.5, color="#404a55")
    axes[0].set_xlabel("time since vision lock (s)"); axes[0].set_ylabel("offset (m)")
    axes[0].set_ylim(0, 1.6); axes[0].set_xlim(0, 60)
    axes[0].set_title("tracking offset, moving pad")
    axes[0].legend(fontsize=8, loc="upper right")
    axes[1].set_xlabel("time since vision lock (s)"); axes[1].set_ylabel("height above deck (m)")
    axes[1].set_xlim(0, 60)
    axes[1].set_title("descent")
    axes[1].legend(fontsize=8)
    fig.suptitle("Fig. D  What the fixed-gain PID cannot do on its own",
                 fontsize=11, fontweight="bold", y=1.04)
    path = save(fig, "mj_fig4_ablation.png")

    sweep = []
    for name, kw, _, _, _ in cfgs:
        landed, errs, times = 0, [], []
        for seed in range(seeds):
            for pid in C.MOVING_IDS:
                r = fly(pid, seed=seed, **kw).result
                if r["outcome"] == "LANDED":
                    landed += 1
                    errs.append(r["error_m"]); times.append(r["time_s"])
        sweep.append({"config": name, "runs": seeds * len(C.MOVING_IDS),
                      "landed": landed,
                      "mean_error_m": round(float(np.mean(errs)), 3) if errs else None,
                      "mean_time_s": round(float(np.mean(times)), 1) if times else None})
        print("     %-30s %2d/%2d landed" % (name, landed, seeds * len(C.MOVING_IDS)),
              flush=True)
    return path, {"trace": table, "sweep": sweep}


# ================================================================ E4b divert
def exp_divert(seed=1):
    """
    Retasking in flight - the panel's path, which the Monte Carlo does not take.

    Every batch mission starts from a reset, so a defect that appears only when
    the drone is given a new address in mid-air, or straight after a touchdown,
    is invisible to it. Both are the normal way the panel is used, and both were
    broken: the motors stayed off after a landing, the mission timeout was
    compared against the global clock instead of the mission's own, and the
    descent gates were measured against absolute altitude - which only matters
    when the pad is on a roof. Each address is flown twice here: once diverted
    from another address in mid-air, once immediately after landing elsewhere.
    """
    env, vis = shared()
    rows = []
    for target in range(1, len(C.PADS) + 1):
        for scen, (first, t_div) in enumerate([(1, 15.0), (4, None)]):
            env.reset(seed=seed)
            m = LandingMission(env, first, seed=seed, vision=vis)
            if t_div is None:
                m.run()                       # land first, then divert
            else:
                while env.t < t_div and m.state not in ("LANDED", "ABORTED"):
                    m.step()
            m = LandingMission(env, target, seed=seed, vision=vis)
            m.run()
            r = m.result
            churn = sum(1 for _, e in m.log.events
                        if "DESCEND -> ALIGN" in e or "DESCEND -> TRANSIT" in e
                        or "ALIGN -> TRANSIT" in e)
            rows.append(dict(pad_id=target,
                             scenario=("divert in flight" if scen == 0
                                       else "divert after landing"),
                             outcome=r["outcome"], error_m=round(r["error_m"], 4),
                             time_s=r["time_s"], go_arounds=churn))
            print("     address %2d  %-22s %-8s %5.1f cm  %5.1f s"
                  % (target, rows[-1]["scenario"], r["outcome"],
                     100 * r["error_m"], r["time_s"]), flush=True)
    ok = [r for r in rows if r["outcome"] == "LANDED"]
    e = np.array([r["error_m"] for r in ok]) if ok else np.array([np.nan])
    return {
        "missions": len(rows), "landed": len(ok),
        "mean_error_cm": round(100 * float(np.nanmean(e)), 2),
        "p95_error_cm": round(100 * float(np.nanpercentile(e, 95)), 2),
        "max_error_cm": round(100 * float(np.nanmax(e)), 2),
        "go_arounds": int(sum(r["go_arounds"] for r in rows)),
        "runs": rows,
    }


# ============================================================== E5 monte carlo
def exp_monte_carlo(seeds=4):
    rows, per_pad = [], {p["id"]: [] for p in C.PADS}
    for s in range(seeds):
        for p in C.PADS:
            r = fly(p["id"], seed=s).result
            rows.append(r); per_pad[p["id"]].append(r)
            print("     seed %d pad %2d %-8s %.3f m" % (s, p["id"], r["outcome"],
                                                        r["error_m"]), flush=True)
    ok = [r for r in rows if r["outcome"] == "LANDED"]
    err = np.array([r["error_m"] for r in ok])
    stat = np.array([r["error_m"] for r in ok if not r["moving"]])
    mov = np.array([r["error_m"] for r in ok if r["moving"]])
    vz = np.array([abs(r["touchdown_vz"]) for r in ok])

    fig, axes = plt.subplots(1, 2, figsize=(11.0, 3.9))
    ax = axes[0]
    for p in C.PADS:
        e = [r["error_m"] for r in per_pad[p["id"]] if r["outcome"] == "LANDED"]
        col = ORANGE if p["motion"] != "static" else BLUE
        ax.scatter([p["id"]] * len(e), [100 * v for v in e], color=col, alpha=0.7, s=24)
        if e:
            ax.scatter([p["id"]], [100 * np.mean(e)], color="black", marker="_", s=200)
    ax.set_xticks(range(1, 11)); ax.set_xlabel("AirTag address")
    ax.set_ylabel("touchdown error (cm)")
    ax.set_title("error per address (blue static, orange moving)")

    ax = axes[1]
    bins = np.linspace(0, max(30, 100 * err.max() * 1.05), 16)
    ax.hist(100 * stat, bins=bins, color=BLUE, alpha=0.75, label="static pads")
    ax.hist(100 * mov, bins=bins, color=ORANGE, alpha=0.75, label="moving pads")
    ax.set_xlabel("touchdown error (cm)"); ax.set_ylabel("missions")
    ax.legend(fontsize=8)
    ax.set_title("error distribution, %d missions" % len(rows))
    fig.suptitle("Fig. E  Repeatability across all ten addresses",
                 fontsize=11, fontweight="bold", y=1.04)
    path = save(fig, "mj_fig5_monte_carlo.png")

    return path, {
        "missions": len(rows), "landed": len(ok),
        "success_rate": round(100.0 * len(ok) / len(rows), 1),
        "mean_error_cm": round(100 * float(err.mean()), 2),
        "median_error_cm": round(100 * float(np.median(err)), 2),
        "p95_error_cm": round(100 * float(np.percentile(err, 95)), 2),
        "max_error_cm": round(100 * float(err.max()), 2),
        "static_mean_cm": round(100 * float(stat.mean()), 2) if len(stat) else None,
        "moving_mean_cm": round(100 * float(mov.mean()), 2) if len(mov) else None,
        "mean_time_s": round(float(np.mean([r["time_s"] for r in ok])), 1),
        "mean_touchdown_vz": round(float(vz.mean()), 3),
        "per_address": {str(p["id"]): {
            "address": p["address"],
            "moving": p["motion"] != "static",
            "landed": sum(1 for r in per_pad[p["id"]] if r["outcome"] == "LANDED"),
            "runs": seeds,
            "mean_error_cm": round(100 * float(np.mean(
                [r["error_m"] for r in per_pad[p["id"]]
                 if r["outcome"] == "LANDED"] or [np.nan])), 2),
            "mean_time_s": round(float(np.mean(
                [r["time_s"] for r in per_pad[p["id"]]
                 if r["outcome"] == "LANDED"] or [np.nan])), 1),
        } for p in C.PADS},
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--seeds", type=int, default=None,
                    help="override the number of Monte Carlo seeds")
    ap.add_argument("--abl-seeds", type=int, default=None,
                    help="override the number of ablation seeds")
    ap.add_argument("--only", default=None,
                    help="scene | addressing | perception | static | dynamic | ablation | mc")
    args = ap.parse_args()
    seeds_mc = args.seeds if args.seeds else (2 if args.quick else 4)
    seeds_abl = args.abl_seeds if args.abl_seeds else (1 if args.quick else 2)
    samples = 40 if args.quick else 90

    out_path = os.path.join(RES, "mujoco_summary.json")
    out = {}
    if os.path.exists(out_path):
        with open(out_path) as f:
            out = json.load(f)

    t0 = time.time()
    want = (lambda n: args.only is None or args.only == n)

    if want("scene"):
        print("\n  [E0] scene and onboard views ...", flush=True)
        exp_scene()
    if want("addressing"):
        print("\n  [E0b] addressing and rejection ...", flush=True)
        _, out["addressing"] = exp_addressing()
    if want("perception"):
        print("\n  [E1] perception accuracy ...", flush=True)
        _, out["perception"] = exp_perception(samples=samples)
    if want("static"):
        print("\n  [E2] static landing ...", flush=True)
        _, keys, res = exp_static()
        out["static"] = {"key_points": keys, "result": res}
    if want("dynamic"):
        print("\n  [E3] dynamic tracking ...", flush=True)
        _, band, res = exp_dynamic()
        out["dynamic"] = {"offset_band_m": band, "result": res}
    if want("ablation"):
        print("\n  [E4] controller ablation ...", flush=True)
        _, out["ablation"] = exp_ablation(seeds=seeds_abl)
    if want("divert"):
        print("\n  [E4b] retasking in flight ...", flush=True)
        out["divert"] = exp_divert()
    if want("mc"):
        print("\n  [E5] Monte Carlo ...", flush=True)
        _, out["monte_carlo"] = exp_monte_carlo(seeds=seeds_mc)

    out["config"] = {
        "cruise_alt_m": C.CRUISE_ALT, "tag_large_m": C.TAG_LARGE,
        "tag_small_m": C.TAG_SMALL, "pad_size_m": C.PAD_SIZE,
        "control_hz": round(1 / C.DT_CTRL), "camera_hz": C.CAM_HZ,
        "physics_hz": round(1 / C.DT_SIM), "max_tilt_deg": round(np.degrees(C.MAX_TILT)),
        "gains": {"kp": C.KP, "ki": C.KI, "kd": C.KD},
    }
    with open(out_path, "w") as f:
        json.dump(out, f, indent=2)
    print("\n  wrote results/mujoco_summary.json   (%.1f min)" % ((time.time() - t0) / 60))
    if "monte_carlo" in out:
        mc = out["monte_carlo"]
        print("  success %.1f %%   mean %.2f cm   p95 %.2f cm"
              % (mc["success_rate"], mc["mean_error_cm"], mc["p95_error_cm"]))


if __name__ == "__main__":
    main()
