"""
The control panel: one window, click a number, the drone flies there and lands.

    python panel.py            start flying immediately
    python panel.py 6          start on address 6
    python panel.py --paper    fly with the base paper's fixed-gain PID only

Layout: the world on the left, the drone's own camera top right, and the
addressed landing sites as buttons underneath it. Press a number key or click a
button at any time - including mid-flight - and the drone diverts to that
address and lands on it.

    1 - 9, 0        fly to that address (0 = address 10)
    click a button  the same thing
    [space]         pause          [r]  restart the current address
    [a] [d]         orbit the view [w] [s]  zoom
    [q] or [esc]    quit
"""
import argparse
import os
import sys
import time

import cv2
import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import config as C
from control import quaternion as Q
from mission import LandingMission
from perception.vision import PadVision
from physics.quad import QuadEnv

# ---------------------------------------------------------------- layout (px)
WORLD_W, WORLD_H = 940, 700
CAM_W, CAM_H = 460, 345
PANEL_W = 480
WIN_W, WIN_H = WORLD_W + PANEL_W, WORLD_H + 96
BTN_W, BTN_H = 214, 62
BTN_X0, BTN_Y0 = WORLD_W + 14, CAM_H + 60
BTN_GAP_X, BTN_GAP_Y = 12, 10

BG = (24, 27, 32)
INK = (232, 238, 244)
DIM = (150, 160, 172)
KIND_COLOUR = {"ground": (196, 132, 62), "vehicle": (60, 140, 235),
               "roof": (96, 176, 96)}
PHASE_COLOUR = {
    "TAKEOFF": (170, 170, 170), "TRANSIT": (235, 190, 80), "SEARCH": (220, 130, 200),
    "ALIGN": (70, 200, 245), "DESCEND": (80, 170, 255), "FINAL": (110, 235, 130),
    "LANDED": (110, 235, 130), "ABORTED": (80, 80, 240),
}


def button_rect(i):
    col, row = i % 2, i // 2
    x = BTN_X0 + col * (BTN_W + BTN_GAP_X)
    y = BTN_Y0 + row * (BTN_H + BTN_GAP_Y)
    return x, y, BTN_W, BTN_H


class Panel:
    def __init__(self, start_id=1, paper=False, wind=True, seed=1, speed=1.0):
        self.env = QuadEnv(render_camera=True, seed=seed)
        self.vision = PadVision()
        self.env.reset(seed=seed)
        self.paper = paper
        self.wind = wind
        self.seed = seed
        self.speed = speed
        self.paused = False
        self.results = {}                      # pad id -> last outcome
        self.hold = 0

        self.chase = mujoco.Renderer(self.env.model, WORLD_H, WORLD_W)
        self.chase.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.cam.azimuth, self.cam.elevation, self.cam.distance = 118.0, -24.0, 12.0
        self.look = self.env.state()["pos"].copy()

        self.mission = None
        self.fly_to(start_id)

    # ------------------------------------------------------------- missions
    def fly_to(self, pad_id):
        """
        Retask the drone. Deliberately does NOT reset the world: the vehicles
        keep driving and the drone keeps its position and velocity, so pressing
        a number mid-flight is a divert, not a restart.
        """
        self.mission = LandingMission(
            self.env, pad_id, seed=self.seed, vision=self.vision, wind=self.wind,
            feedforward=not self.paper, gain_schedule=not self.paper)
        self.hold = 0
        p = self.mission.pad
        print("  -> address %d : %s   (%s, deck %.2f m)"
              % (p.id, p.address, p.kind, p.deck_height), flush=True)

    # -------------------------------------------------------------- drawing
    def _world_view(self):
        st = self.env.state()
        self.look = 0.84 * self.look + 0.16 * st["pos"]
        self.cam.lookat[:] = self.look
        self.chase.update_scene(self.env.data, camera=self.cam)
        return cv2.cvtColor(self.chase.render(), cv2.COLOR_RGB2BGR)

    def _camera_view(self):
        m = self.mission
        img = (cv2.cvtColor(m.frame, cv2.COLOR_RGB2BGR) if m.frame is not None
               else np.zeros((C.IMG_H, C.IMG_W, 3), np.uint8))
        img = img.copy()
        s = m.sighting
        live = m.since_vision < 1.5 / C.CAM_HZ
        if s is not None and live:
            pts = s.corners.astype(np.int32)
            cv2.polylines(img, [pts], True, (90, 235, 110), 2)
            c = pts.mean(axis=0).astype(int)
            cv2.line(img, (int(C.CX), int(C.CY)), tuple(c), (90, 235, 110), 1)
            cv2.putText(img, "id %d  %s" % (s.tag_id, "LARGE" if s.is_large else "SMALL"),
                        (c[0] + 10, c[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (90, 235, 110), 1, cv2.LINE_AA)
        cv2.drawMarker(img, (int(C.CX), int(C.CY)), (255, 255, 255),
                       cv2.MARKER_CROSS, 18, 1)
        img = cv2.resize(img, (CAM_W, CAM_H))
        cv2.rectangle(img, (0, 0), (CAM_W - 1, CAM_H - 1), (70, 78, 88), 1)
        # "NO LOCK" below the commit height is the geometry working, not a
        # fault: the marker is wider than the frame from there down. Saying so
        # keeps the panel honest - the drone is not lost, it is committed.
        if live:
            tag = ("0.12 m marker" if s is not None and not s.is_large
                   else "0.42 m marker")
        elif m.state == "FINAL":
            tag = "committed - marker wider than frame"
        else:
            tag = "NO LOCK"
        cv2.rectangle(img, (0, CAM_H - 24), (CAM_W, CAM_H), (18, 20, 24), -1)
        cv2.putText(img, "onboard camera   %s" % tag, (8, CAM_H - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44,
                    (110, 235, 130) if live else
                    ((90, 200, 250) if m.state == "FINAL" else (110, 110, 235)),
                    1, cv2.LINE_AA)
        return img

    def _draw_buttons(self, canvas):
        m = self.mission
        for i, spec in enumerate(C.PADS):
            x, y, w, h = button_rect(i)
            pid = spec["id"]
            kind = spec.get("kind", "ground")
            active = (pid == m.pad.id)
            base = KIND_COLOUR[kind]
            fill = tuple(int(c * (1.0 if active else 0.42)) for c in base)
            cv2.rectangle(canvas, (x, y), (x + w, y + h), fill, -1)
            if active:
                cv2.rectangle(canvas, (x, y), (x + w, y + h), (255, 255, 255), 2)

            cv2.putText(canvas, str(pid), (x + 12, y + 43),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.15, INK, 2, cv2.LINE_AA)
            # two-digit numbers are wider, so the label starts further right
            tx = x + (52 if pid < 10 else 74)
            name = spec["address"].split("  ")[-1]
            cv2.putText(canvas, name[:18], (tx, y + 27),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, INK, 1, cv2.LINE_AA)
            extra = kind if kind != "roof" else "roof %.1f m" % C.pad_height(spec)
            cv2.putText(canvas, extra, (tx, y + 47),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.36, (225, 232, 240), 1, cv2.LINE_AA)

            r = self.results.get(pid)
            if r is not None:
                ok = r["outcome"] == "LANDED"
                cv2.putText(canvas, "%.0f cm" % (100 * r["error_m"]) if ok else "fail",
                            (x + w - 58, y + 47), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                            (110, 235, 130) if ok else (90, 90, 240), 1, cv2.LINE_AA)

    def _draw_status(self, canvas):
        m = self.mission
        st = self.env.state()
        colour = PHASE_COLOUR.get(m.state, INK)
        off = m.held_offset if m.held_offset is not None else np.zeros(2)
        tilt = np.degrees(np.arccos(np.clip(Q.to_rot(st["quat"])[2, 2], -1.0, 1.0)))
        agl = st["pos"][2] - m.pad.deck_height

        y0 = WORLD_H + 8
        cv2.putText(canvas, "%-8s" % m.state, (16, y0 + 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.85, colour, 2, cv2.LINE_AA)
        cv2.putText(canvas, "address %d  %s" % (m.pad.id, m.pad.address),
                    (190, y0 + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.58, INK, 1, cv2.LINE_AA)
        cv2.putText(canvas,
                    "alt %5.2f m   above pad %5.2f m   offset %5.2f m   tilt %4.1f deg"
                    "   pad %4.2f m/s   t %5.1f s"
                    % (st["pos"][2], agl, float(np.linalg.norm(off)), tilt,
                       m.pad.speed, self.env.t),
                    (16, y0 + 62), cv2.FONT_HERSHEY_SIMPLEX, 0.52, DIM, 1, cv2.LINE_AA)
        if m.result is not None:
            txt = "%s - %.1f cm from centre" % (m.result["outcome"],
                                                100 * m.result["error_m"])
            cv2.putText(canvas, txt, (WORLD_W - 330, y0 + 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.66, colour, 2, cv2.LINE_AA)
        if self.paused:
            cv2.putText(canvas, "PAUSED", (WORLD_W - 150, y0 + 62),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80, 200, 255), 2, cv2.LINE_AA)

    def render(self):
        canvas = np.full((WIN_H, WIN_W, 3), BG, np.uint8)
        canvas[0:WORLD_H, 0:WORLD_W] = self._world_view()
        canvas[14:14 + CAM_H, WORLD_W + 10:WORLD_W + 10 + CAM_W] = self._camera_view()
        cv2.putText(canvas, "press a number to land there",
                    (WORLD_W + 14, CAM_H + 44), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                    DIM, 1, cv2.LINE_AA)
        self._draw_buttons(canvas)
        self._draw_status(canvas)
        return canvas

    # ---------------------------------------------------------------- input
    def on_mouse(self, event, x, y, flags, _):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        for i, spec in enumerate(C.PADS):
            bx, by, bw, bh = button_rect(i)
            if bx <= x <= bx + bw and by <= y <= by + bh:
                self.fly_to(spec["id"])
                return

    def on_key(self, k):
        if k in (ord("q"), 27):
            return False
        if k == 32:
            self.paused = not self.paused
        elif k in (ord("r"), ord("R")):
            self.fly_to(self.mission.pad.id)
        elif k in (ord("a"), ord("A")):
            self.cam.azimuth += 6
        elif k in (ord("d"), ord("D")):
            self.cam.azimuth -= 6
        elif k in (ord("w"), ord("W")):
            self.cam.distance = max(4.0, self.cam.distance - 1.0)
        elif k in (ord("s"), ord("S")):
            self.cam.distance = min(40.0, self.cam.distance + 1.0)
        elif ord("1") <= k <= ord("9"):
            self.fly_to(k - ord("0"))
        elif k == ord("0"):
            self.fly_to(10)
        return True

    # ----------------------------------------------------------------- loop
    def run(self):
        win = "AirTag-addressed autonomous landing"
        cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(win, self.on_mouse)
        steps = max(1, int(round(self.speed / (30.0 * C.DT_CTRL))))

        while True:
            tic = time.time()
            if not self.paused:
                for _ in range(steps):
                    if self.mission.state in ("LANDED", "ABORTED"):
                        break
                    self.mission.step()

            if self.mission.result is not None and self.hold == 0:
                r = self.mission.result
                self.results[self.mission.pad.id] = r
                print("     %s  |  %.1f cm from centre  |  %.1f s  |  touchdown %.2f m/s"
                      % (r["outcome"], 100 * r["error_m"], r["time_s"],
                         abs(r["touchdown_vz"])), flush=True)
                self.hold = 1

            cv2.imshow(win, self.render())
            if not self.on_key(cv2.waitKey(1) & 0xFF):
                break
            if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
            lag = (steps * C.DT_CTRL) / max(self.speed, 1e-3) - (time.time() - tic)
            if lag > 0:
                time.sleep(lag)

        cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser(description="AirTag-addressed autonomous landing")
    ap.add_argument("address", nargs="?", type=int, default=1,
                    help="address to start on (1-10)")
    ap.add_argument("--paper", action="store_true",
                    help="fly with the base paper's fixed-gain PID only")
    ap.add_argument("--no-wind", action="store_true")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()

    if not 1 <= args.address <= len(C.PADS):
        print("  address must be between 1 and %d" % len(C.PADS))
        sys.exit(1)

    print("\n  AirTag-addressed autonomous landing")
    print("  " + "-" * 58)
    for p in C.PADS:
        extra = ("roof at %.1f m" % C.pad_height(p)) if p.get("kind") == "roof" \
            else p.get("kind", "ground")
        print("   %2d  %-26s %s" % (p["id"], p["address"], extra))
    print("  " + "-" * 58)
    print("  press a number in the window to fly there;  [space] pause  [q] quit\n")

    Panel(start_id=args.address, paper=args.paper, wind=not args.no_wind,
          seed=args.seed, speed=args.speed).run()


if __name__ == "__main__":
    main()
