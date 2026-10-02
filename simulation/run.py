"""
The live demo.

    python run.py            -> asks which AirTag address to fly to
    python run.py 4          -> flies straight to address 4
    python run.py 3 7 10     -> flies all three, one after another
    python run.py 6 --no-ff  -> address 6 with the paper's plain PID (no
                                velocity feed-forward, no gain scheduling),
                                which is how you see the moving-pad lag

Left panel is the arena from above: ten AirTag pads, the addressed one
highlighted, the drone with its live camera footprint. Right panels are the
altitude profile, the tracking offset, and a heads-up readout.
"""
import argparse
import sys

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle, Rectangle

import config as C
from airtags import build_fleet, resolve
from mission import LandingMission

# --------------------------------------------------------------------- colours
BG = "#0e1117"
FG = "#e6edf3"
ACCENT = "#4cc9f0"
TARGET = "#f7b32b"
PAD = "#5a6572"
TRAIL = "#4cc9f0"
OK = "#57d38c"
WARN = "#ff6b6b"

PHASE_COLOR = {"TAKEOFF": "#7f8c9b", "TRANSIT": "#4cc9f0", "SEARCH": "#b07cff",
               "ALIGN": "#f7b32b", "DESCEND": "#ff9f45", "FINAL": "#57d38c",
               "LANDED": "#57d38c", "ABORTED": "#ff6b6b"}


def print_directory(fleet):
    print("\n  AirTag address book")
    print("  " + "-" * 54)
    for t in fleet:
        kind = "moving" if t.motion != "static" else "static"
        print("   %2d  %-30s  %-6s  (%5.1f, %5.1f)"
              % (t.id, t.address, kind, t.home[0], t.home[1]))
    print("  " + "-" * 54)


def ask_address(fleet):
    print_directory(fleet)
    while True:
        raw = input("\n  Fly to which address? (1-10, or 'q') > ").strip()
        if raw.lower() in ("q", "quit", "exit"):
            sys.exit(0)
        tag = resolve(fleet, raw)
        if tag is not None:
            return tag
        print("  no such address - type a number 1-10")


class Demo:
    """Runs a queue of addresses and draws the whole thing."""

    def __init__(self, fleet, ids, feedforward=True, gain_schedule=True,
                 seed=0, speed=4):
        self.fleet = fleet
        self.queue = list(ids)
        self.seed = seed
        self.feedforward = feedforward
        self.gain_schedule = gain_schedule
        self.speed = speed              # simulation steps drawn per frame
        self.first = True
        self.mission = None
        self.completed = []
        self.pad_trail = []
        self.trail = []
        self._next_mission(start=(0.0, 0.0, 0.0))
        self._build_figure()

    # ------------------------------------------------------------- missions
    def _next_mission(self, start):
        if not self.queue:
            self.mission = None
            return False
        tid = self.queue.pop(0)
        self.mission = LandingMission(self.fleet, tid, seed=self.seed,
                                      feedforward=self.feedforward,
                                      gain_schedule=self.gain_schedule,
                                      start=start, reset_fleet=self.first)
        self.first = False
        self.trail = []
        self.pad_trail = []
        self.seed += 1
        print("\n  >> address %d : %s" % (self.mission.tag.id,
                                          self.mission.tag.address))
        return True

    # -------------------------------------------------------------- drawing
    def _build_figure(self):
        plt.rcParams.update({
            "figure.facecolor": BG, "axes.facecolor": BG,
            "axes.edgecolor": "#2a3441", "axes.labelcolor": FG,
            "text.color": FG, "xtick.color": "#8b949e", "ytick.color": "#8b949e",
            "grid.color": "#1c2430", "font.size": 9,
        })
        self.fig = plt.figure(figsize=(15.0, 8.2))
        self.fig.canvas.manager.set_window_title(
            "AirTag-addressed autonomous landing - AprilTag vision + PID")
        gs = self.fig.add_gridspec(3, 2, width_ratios=[1.35, 1.0],
                                   height_ratios=[1, 1, 1],
                                   left=0.05, right=0.975, top=0.93, bottom=0.07,
                                   wspace=0.18, hspace=0.42)
        self.ax_map = self.fig.add_subplot(gs[:, 0])
        self.ax_alt = self.fig.add_subplot(gs[0, 1])
        self.ax_off = self.fig.add_subplot(gs[1, 1])
        self.ax_hud = self.fig.add_subplot(gs[2, 1])

        # ---- map
        m = C.WORLD_SIZE / 2
        ax = self.ax_map
        ax.set_xlim(-m, m); ax.set_ylim(-m, m); ax.set_aspect("equal")
        ax.set_title("arena, top view   (x east / y north, metres)", loc="left")
        ax.grid(True, lw=0.4)

        self.pad_marks, self.pad_labels = [], []
        for t in self.fleet:
            r = Rectangle((t.pos[0] - 0.9, t.pos[1] - 0.9), 1.8, 1.8,
                          fc=PAD, ec="#8b949e", lw=0.8, alpha=0.85, zorder=2)
            ax.add_patch(r)
            lab = ax.text(t.pos[0] + 1.2, t.pos[1] + 1.0, str(t.id),
                          fontsize=8, color="#8b949e", zorder=3, clip_on=True)
            self.pad_marks.append(r)
            self.pad_labels.append(lab)

        self.footprint = Circle((0, 0), 0.1, fc=ACCENT, alpha=0.10,
                                ec=ACCENT, lw=0.8, zorder=1)
        ax.add_patch(self.footprint)
        (self.trail_line,) = ax.plot([], [], color=TRAIL, lw=1.2, alpha=0.75, zorder=4)
        (self.pad_line,) = ax.plot([], [], color=TARGET, lw=1.0, ls="--",
                                   alpha=0.8, zorder=4)
        (self.drone_dot,) = ax.plot([], [], marker="o", ms=9, color=ACCENT,
                                    mec="white", mew=0.8, zorder=6)
        (self.los,) = ax.plot([], [], color="#ffffff", lw=0.7, alpha=0.35, zorder=5)
        self.phase_txt = ax.text(-m + 1, m - 2.5, "", fontsize=13, weight="bold")

        # ---- altitude
        ax = self.ax_alt
        ax.set_title("altitude", loc="left")
        ax.set_xlabel("t (s)"); ax.set_ylabel("h (m)")
        ax.grid(True, lw=0.4)
        (self.alt_line,) = ax.plot([], [], color=ACCENT, lw=1.4)
        self.switch_mark = ax.axhline(C.TAG_SWITCH_ALT, color=TARGET, lw=0.8,
                                      ls=":", alpha=0.8)
        ax.text(0.02, 0.06, "0.06 m tag takes over below this line",
                transform=ax.transAxes, fontsize=7, color=TARGET, alpha=0.9)

        # ---- offset
        ax = self.ax_off
        ax.set_title("tracking offset drone -> pad", loc="left")
        ax.set_xlabel("t (s)"); ax.set_ylabel("offset (m)")
        ax.grid(True, lw=0.4)
        (self.ex_line,) = ax.plot([], [], color="#4cc9f0", lw=1.2, label="x")
        (self.ey_line,) = ax.plot([], [], color="#f7b32b", lw=1.2, label="y")
        ax.axhline(0, color="#3a4553", lw=0.8)
        # symmetric log: one plot has to show a 25 m approach and a 5 cm touchdown
        ax.set_yscale("symlog", linthresh=0.2, linscale=0.6)
        ax.set_ylim(-40, 40)
        ax.axhspan(-C.FINAL_TOL, C.FINAL_TOL, color=OK, alpha=0.16, lw=0)
        ax.legend(loc="upper right", fontsize=7, facecolor=BG, edgecolor="#2a3441")

        # ---- hud
        self.ax_hud.axis("off")
        self.hud = self.ax_hud.text(0.0, 0.98, "", va="top", ha="left",
                                    family="monospace", fontsize=9.5)

    # --------------------------------------------------------------- update
    def _refresh_pads(self):
        tgt = self.mission.tag if self.mission else None
        for r, lab, t in zip(self.pad_marks, self.pad_labels, self.fleet):
            hot = tgt is not None and t.id == tgt.id
            size = 2.6 if hot else 1.8
            r.set_bounds(t.pos[0] - size / 2, t.pos[1] - size / 2, size, size)
            r.set_facecolor(TARGET if hot else PAD)
            r.set_alpha(0.95 if hot else 0.6)
            r.set_zorder(3 if hot else 2)
            lab.set_position((t.pos[0] + size / 2 + 0.3, t.pos[1] + size / 2))
            lab.set_color(TARGET if hot else "#6b7482")

    def frame(self, _):
        m = self.mission
        if m is None:
            return []

        for _ in range(self.speed):
            if m.state in ("LANDED", "ABORTED"):
                break
            m.step()

        d = m.drone.p
        self.trail.append((d[0], d[1]))
        self.pad_trail.append(tuple(m.tag.pos))
        self._refresh_pads()

        self.trail_line.set_data(*zip(*self.trail))
        self.pad_line.set_data(*zip(*self.pad_trail))
        self.drone_dot.set_data([d[0]], [d[1]])
        self.los.set_data([d[0], m.tag.pos[0]], [d[1], m.tag.pos[1]])
        fp = d[2] * (C.IMG_H / 2.0) / C.FOCAL_PX
        self.footprint.set_center((d[0], d[1]))
        self.footprint.set_radius(max(fp, 0.05))
        col = PHASE_COLOR.get(m.state, FG)
        self.footprint.set_edgecolor(col); self.footprint.set_facecolor(col)
        self.phase_txt.set_text("%s   address %d" % (m.state, m.tag.id))
        self.phase_txt.set_color(col)

        t = m.log.col("t"); z = m.log.col("z")
        ex = m.log.col("ex"); ey = m.log.col("ey")
        self.alt_line.set_data(t, z)
        self.ax_alt.set_xlim(0, max(t[-1], 5)); self.ax_alt.set_ylim(0, max(z.max() * 1.15, 1))
        self.ex_line.set_data(t, ex); self.ey_line.set_data(t, ey)
        self.ax_off.set_xlim(0, max(t[-1], 5))

        row = m.log.rows[-1]
        tag_txt = ("0.30 m tag" if row["tag_size"] == C.TAG_BIG else
                   "0.06 m tag" if row["tag_size"] == C.TAG_SMALL else "-- no lock --")
        src = "VISION" if row["seen"] else ("RADIO" if m.last_good_fix is not None else "NONE")
        self.hud.set_text(
            "ADDRESS   {id:>2}  {addr}\n"
            "PAD       {kind}, {spd:.2f} m/s\n"
            "\n"
            "PHASE     {st}\n"
            "GUIDANCE  {src}    {tag}\n"
            "\n"
            "ALTITUDE  {h:6.2f} m\n"
            "OFFSET    {e:6.2f} m   (x {ex:+.2f}, y {ey:+.2f})\n"
            "SPEED     {v:6.2f} m/s\n"
            "MISSION   {t:6.1f} s     battery {b:3.0f}%\n"
            "DETECTION {dr:3.0f}% of frames decoded\n"
            "{done}".format(
                id=m.tag.id, addr=m.tag.address,
                kind=("moving" if m.tag.motion != "static" else "static"),
                spd=m.tag.speed, st=m.state, src=src, tag=tag_txt,
                h=d[2], e=row["err"], ex=row["ex"], ey=row["ey"],
                v=float(np.linalg.norm(m.drone.v[:2])), t=m.t,
                b=100 * m.drone.battery_left,
                dr=100 * m.cam.hits / max(m.cam.frames, 1),
                done=self._done_text()))

        if m.state in ("LANDED", "ABORTED") and m.result is not None:
            if not self.completed or self.completed[-1] is not m.result:
                self.completed.append(m.result)
                print("     %s  error %.3f m in %.1f s"
                      % (m.result["outcome"], m.result["error_m"], m.result["time_s"]))
            self._hold = getattr(self, "_hold", 0) + 1
            if self._hold > 45:
                self._hold = 0
                if not self._next_mission(start=tuple(m.drone.p)):
                    self.mission = m       # keep the last frame on screen
        return []

    def _done_text(self):
        if not self.completed:
            return ""
        lines = ["\nCOMPLETED"]
        for r in self.completed:
            lines.append("  addr %-2d  %-7s  %5.3f m  %5.1f s"
                         % (r["tag_id"], r["outcome"], r["error_m"], r["time_s"]))
        return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description="AirTag-addressed autonomous landing demo")
    ap.add_argument("addresses", nargs="*", help="AirTag addresses to visit, 1-10")
    ap.add_argument("--no-ff", action="store_true",
                    help="disable pad-velocity feed-forward (paper's plain PID)")
    ap.add_argument("--no-gs", action="store_true",
                    help="disable altitude gain scheduling")
    ap.add_argument("--speed", type=int, default=4,
                    help="simulation steps per drawn frame (higher = faster)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--save", metavar="FILE.gif", help="write the run to a GIF")
    ap.add_argument("--frames", type=int, default=420,
                    help="frames to record when --save is used")
    args = ap.parse_args()

    if args.save:                      # switch backend BEFORE the figure exists
        matplotlib.use("Agg")

    fleet = build_fleet()
    if args.addresses:
        ids = []
        for a in args.addresses:
            t = resolve(fleet, a)
            if t is None:
                print("  no such address: %s" % a)
                sys.exit(1)
            ids.append(t.id)
    else:
        ids = [ask_address(fleet).id]

    demo = Demo(fleet, ids, feedforward=not args.no_ff,
                gain_schedule=not args.no_gs, seed=args.seed, speed=args.speed)

    if args.save:
        anim = FuncAnimation(demo.fig, demo.frame, frames=args.frames,
                             interval=40, blit=False, cache_frame_data=False)
        anim.save(args.save, writer="pillow", fps=20, dpi=70)
        print("  saved %s (%d frames)" % (args.save, args.frames))
    else:
        anim = FuncAnimation(demo.fig, demo.frame, interval=40,
                             blit=False, cache_frame_data=False)
        plt.show()


if __name__ == "__main__":
    main()
