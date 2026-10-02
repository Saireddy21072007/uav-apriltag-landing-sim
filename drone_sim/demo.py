"""
The live demo: type an AirTag address, watch the drone go there and land on it.

    python demo.py              ask which address to fly to
    python demo.py 4            fly straight to address 4
    python demo.py 3 9 1        fly all three, one after another
    python demo.py 5 --paper    address 5 flown with the base paper's fixed-gain
                                PID alone, so the moving-pad lag is visible

Two windows open:

  * the MuJoCo 3D viewer - orbit with the left mouse button, zoom with the right
  * "onboard camera" - what the drone's downward camera actually sees, with the
    detected AprilTag outlined and the flight state overlaid

Keys in the camera window:  [space] pause   [r] restart   [q] quit
"""
import argparse
import os
import sys
import time

import cv2
import mujoco
import mujoco.viewer
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import config as C
from mission import LandingMission
from perception.vision import PadVision
from physics.quad import QuadEnv

PHASE_BGR = {
    "TAKEOFF": (170, 170, 170), "TRANSIT": (240, 200, 90), "SEARCH": (255, 140, 200),
    "ALIGN": (60, 200, 250), "DESCEND": (80, 170, 255), "FINAL": (110, 240, 130),
    "LANDED": (110, 240, 130), "ABORTED": (80, 80, 245),
}


def print_directory():
    print("\n  AirTag address book")
    print("  " + "-" * 58)
    for p in C.PADS:
        kind = "moving" if p["motion"] != "static" else "static"
        print("   %2d  %-28s %-7s  at (%5.1f,%6.1f)"
              % (p["id"], p["address"], kind, p["home"][0], p["home"][1]))
    print("  " + "-" * 58)


def resolve(text):
    q = str(text).strip().lower()
    if q.isdigit() and 1 <= int(q) <= len(C.PADS):
        return int(q)
    for p in C.PADS:
        if p["address"].lower().startswith(q) or (q and q in p["address"].lower()):
            return p["id"]
    return None


def ask_address():
    print_directory()
    while True:
        raw = input("\n  Fly to which address? (1-10, q to quit) > ").strip()
        if raw.lower() in ("q", "quit", "exit"):
            sys.exit(0)
        pid = resolve(raw)
        if pid is not None:
            return pid
        print("  no such address - type a number from 1 to 10")


def draw_hud(frame, mission, paused):
    """The onboard view with the detection and the flight state drawn on it."""
    img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if frame is not None else \
        np.zeros((C.IMG_H, C.IMG_W, 3), np.uint8)
    img = cv2.resize(img, (C.IMG_W, C.IMG_H))
    st = mission.env.state()
    s = mission.sighting
    live = mission.since_vision < 1.5 / C.CAM_HZ
    colour = PHASE_BGR.get(mission.state, (220, 220, 220))

    if s is not None and live:
        pts = s.corners.astype(np.int32)
        cv2.polylines(img, [pts], True, (110, 240, 130), 2)
        ctr = pts.mean(axis=0).astype(int)
        cv2.line(img, (int(C.CX), int(C.CY)), tuple(ctr), (110, 240, 130), 1)
        cv2.circle(img, tuple(ctr), 4, (110, 240, 130), -1)
        cv2.putText(img, "id %d  %s" % (s.tag_id, "LARGE" if s.is_large else "SMALL"),
                    (ctr[0] + 10, ctr[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (110, 240, 130), 1, cv2.LINE_AA)
    # camera boresight
    cv2.drawMarker(img, (int(C.CX), int(C.CY)), (255, 255, 255),
                   cv2.MARKER_CROSS, 18, 1)

    cv2.rectangle(img, (0, 0), (C.IMG_W, 74), (18, 20, 24), -1)
    cv2.putText(img, "%-8s  addr %d  %s" % (mission.state, mission.pad.id,
                                            mission.pad.address),
                (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.58, colour, 2, cv2.LINE_AA)
    off = mission.held_offset if mission.held_offset is not None else np.zeros(2)
    src = "VISION" if live else ("RADIO" if mission.last_fix is not None else "-")
    cv2.putText(img, "alt %5.2f m   offset %5.2f m (%+.2f,%+.2f)   %s"
                % (st["pos"][2], float(np.linalg.norm(off)), off[0], off[1], src),
                (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (215, 225, 235), 1, cv2.LINE_AA)
    cv2.putText(img, "tilt %4.1f deg   pad %4.2f m/s   t %5.1f s"
                % (np.degrees(np.hypot(st["euler"][0], st["euler"][1])),
                   mission.pad.speed, mission.env.t),
                (10, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (215, 225, 235), 1, cv2.LINE_AA)

    if mission.result is not None:
        txt = "%s  -  %.1f cm from pad centre" % (mission.result["outcome"],
                                                  100 * mission.result["error_m"])
        cv2.rectangle(img, (0, C.IMG_H - 42), (C.IMG_W, C.IMG_H), (18, 20, 24), -1)
        cv2.putText(img, txt, (10, C.IMG_H - 14), cv2.FONT_HERSHEY_SIMPLEX,
                    0.62, colour, 2, cv2.LINE_AA)
    if paused:
        cv2.putText(img, "PAUSED", (C.IMG_W - 110, 26), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (80, 200, 255), 2, cv2.LINE_AA)
    return img


def run(ids, feedforward=True, gain_schedule=True, wind=True, speed=1.0, seed=1):
    env = QuadEnv(render_camera=True, seed=seed)
    vision = PadVision()
    queue = list(ids)
    env.reset(seed=seed)

    def new_mission(pid):
        print("\n  >> address %d : %s" % (pid, C.pad(pid)["address"]))
        return LandingMission(env, pid, seed=seed, feedforward=feedforward,
                              gain_schedule=gain_schedule, wind=wind, vision=vision)

    mission = new_mission(queue.pop(0))
    cv2.namedWindow("onboard camera", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("onboard camera", 760, 570)

    paused = False
    hold = 0
    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        viewer.cam.distance = 14.0
        viewer.cam.elevation = -28.0
        viewer.cam.azimuth = 150.0
        px, py = C.pad(mission.pad.id)["home"]
        viewer.cam.lookat[:] = [px, py, 2.0]

        while viewer.is_running():
            tic = time.time()
            if not paused and mission.state not in ("LANDED", "ABORTED"):
                mission.step()
                # keep the 3D camera gently following the drone
                d = env.state()["pos"]
                viewer.cam.lookat[:] = 0.97 * np.array(viewer.cam.lookat) + 0.03 * d
            elif mission.state in ("LANDED", "ABORTED"):
                hold += 1
                if hold > 90 and queue:
                    hold = 0
                    mission = new_mission(queue.pop(0))

            viewer.sync()
            cv2.imshow("onboard camera", draw_hud(mission.frame, mission, paused))
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == 32:
                paused = not paused
            if k in (ord("r"), ord("R")):
                env.reset(seed=seed)
                mission = new_mission(mission.pad.id)
                hold = 0

            if mission.result is not None and hold == 1:
                r = mission.result
                print("     %s  |  %.1f cm from pad centre  |  %.1f s  |  "
                      "touchdown %.2f m/s  |  tags decoded on %.0f%% of frames"
                      % (r["outcome"], 100 * r["error_m"], r["time_s"],
                         abs(r["touchdown_vz"]), 100 * r["detect_rate"]))

            lag = C.DT_CTRL / max(speed, 1e-3) - (time.time() - tic)
            if lag > 0:
                time.sleep(lag)

    cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser(description="AirTag-addressed autonomous landing")
    ap.add_argument("addresses", nargs="*", help="addresses to visit, 1-10")
    ap.add_argument("--paper", action="store_true",
                    help="fly with the base paper's fixed-gain PID only")
    ap.add_argument("--no-ff", action="store_true", help="disable pad-velocity feed-forward")
    ap.add_argument("--no-gs", action="store_true", help="disable altitude gain scheduling")
    ap.add_argument("--no-wind", action="store_true", help="switch the wind off")
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed multiplier")
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()

    if args.addresses:
        ids = []
        for a in args.addresses:
            pid = resolve(a)
            if pid is None:
                print("  no such address: %s" % a)
                sys.exit(1)
            ids.append(pid)
    else:
        ids = [ask_address()]

    run(ids,
        feedforward=not (args.no_ff or args.paper),
        gain_schedule=not (args.no_gs or args.paper),
        wind=not args.no_wind, speed=args.speed, seed=args.seed)


if __name__ == "__main__":
    main()
