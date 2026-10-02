"""
A quick health check: everything imports, the world loads, the camera renders
and the detector decodes a marker.

    python drone_sim/selftest.py          (or: run.bat check)

Exits non-zero if anything is broken, so run.bat can stop before opening a
window that would only show a black screen.
"""
import os
import sys
import traceback

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def main():
    import numpy as np

    import config as C
    from perception.vision import PadVision
    from physics.quad import QuadEnv
    import mission          # noqa: F401  - imported to prove it loads
    import demo             # noqa: F401

    print("   modules OK")

    g = C.check_geometry()
    lo, hi = g["handover_window_m"]
    print("   marker hand-over window: {:.2f} m to {:.2f} m".format(lo, hi))
    print("   addresses: {}   of which moving: {}".format(
        len(C.PADS), len(C.MOVING_IDS)))

    C.check_layout()
    print("   every pad stays on the roadway")

    env = QuadEnv(render_camera=True, seed=0)
    env.reset(seed=0)
    print("   MuJoCo world loaded: {} bodies, {} geoms".format(
        env.model.nbody, env.model.ngeom))

    # put the drone over one pad and check the real detector reads it
    pad = env.pad_by_id(4)
    env.data.qpos[env.qadr:env.qadr + 3] = [pad.pos[0], pad.pos[1], 3.0]
    env.data.qpos[env.qadr + 3:env.qadr + 7] = [1, 0, 0, 0]
    import mujoco
    mujoco.mj_forward(env.model, env.data)

    vis = PadVision()
    seen = vis.look(env.camera_frame(), np.zeros(3))
    if not seen:
        print("   NO MARKER DECODED - the camera or the pad textures are wrong.")
        print("   Try:  run.bat world")
        return 1
    ids = sorted(s.tag_id for s in seen)
    best = PadVision.pick(seen)
    print("   camera renders and the detector decodes: tag ids {}".format(ids))
    print("   pad-centre estimate from 3.00 m: ({:+.3f}, {:+.3f}) m".format(
        best.offset[0], best.offset[1]))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:
        traceback.print_exc()
        print("\n   Self-test failed. If packages are missing, run:  run.bat install")
        sys.exit(1)
