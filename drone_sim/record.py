"""
Records the control panel to an animated GIF.

    python record.py 4 3 9        visit three addresses in turn
    python record.py 9            just the tower roof
    python record.py 4 3 9 --out clip.gif

The clip shows what the panel shows: the world, the drone's own camera with the
markers it decodes, and the numbered landing sites with the current one lit. It
is the artefact to put in a slide deck - one sequence covers a ground pad, a
moving vehicle and a rooftop.
"""
import argparse
import os
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import config as C
from panel import Panel


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("addresses", nargs="*", type=int, default=[4, 3, 9],
                    help="addresses to visit in order")
    ap.add_argument("--out", default=None)
    ap.add_argument("--every", type=int, default=10, help="control steps per frame")
    ap.add_argument("--max-frames", type=int, default=420,
                    help="frame budget; longer flights are sampled down to fit")
    ap.add_argument("--fps", type=int, default=12)
    ap.add_argument("--scale", type=float, default=0.46,
                    help="output scale; a GIF of a long flight gets large fast")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--max-seconds", type=float, default=90.0,
                    help="give up on an address after this long")
    args = ap.parse_args()

    ids = [a for a in args.addresses if 1 <= a <= len(C.PADS)]
    if not ids:
        print("  no valid addresses given")
        sys.exit(1)

    pan = Panel(start_id=ids[0], seed=args.seed)
    queue = list(ids[1:])
    frames, k, hold, t_started = [], 0, 0, pan.env.t

    print("  recording addresses %s ..." % ids)
    while True:
        if pan.mission.state not in ("LANDED", "ABORTED"):
            pan.mission.step()
            if pan.env.t - t_started > args.max_seconds:
                print("     address %d timed out" % pan.mission.pad.id)
                pan.mission._set("ABORTED")
                pan.mission._finish("ABORTED")
        else:
            if hold == 0:
                r = pan.mission.result
                pan.results[pan.mission.pad.id] = r
                print("     address %-2d %-8s %.1f cm in %.1f s"
                      % (pan.mission.pad.id, r["outcome"], 100 * r["error_m"],
                         r["time_s"]), flush=True)
            hold += 1
            if hold > 30:                      # linger on the result, then move on
                if not queue:
                    break
                pan.fly_to(queue.pop(0))
                hold, t_started = 0, pan.env.t

        k += 1
        if k % args.every == 0:
            frames.append(Image.fromarray(pan.render()[:, :, ::-1]))

    # A three-address sequence can run for minutes; keeping every sampled frame
    # would produce a file too large to share, so thin them to the budget.
    if len(frames) > args.max_frames:
        idx = np.linspace(0, len(frames) - 1, args.max_frames).round().astype(int)
        frames = [frames[i] for i in idx]
    for _ in range(args.fps):                  # hold the last frame
        frames.append(frames[-1])

    out = args.out or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "figures",
        "mj_demo_" + "_".join(str(i) for i in ids) + ".gif")
    w = int(frames[0].width * args.scale)
    h = int(frames[0].height * args.scale)
    # One shared adaptive palette: without it every frame carries its own
    # 256-colour table and the file is several times larger.
    small = [f.resize((w, h), Image.LANCZOS).quantize(
        colors=128, method=Image.Quantize.FASTOCTREE, dither=Image.Dither.NONE)
        for f in frames]
    small[0].save(out, save_all=True, append_images=small[1:],
                  duration=int(1000 / args.fps), loop=0, optimize=True)
    print("  wrote %s  (%d frames, %.1f MB)"
          % (out, len(small), os.path.getsize(out) / 1e6))


if __name__ == "__main__":
    main()
