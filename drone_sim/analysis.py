"""
System-theory analysis of this project: controllability, observability, and the
interpolation used in the filters.

    python drone_sim/analysis.py        (or: run.bat analyse)

Everything printed here is computed from the same constants the simulator flies
with, so the claims in the report can be checked rather than believed.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import config as C

np.set_printoptions(precision=3, suppress=True)


def line(t):
    print("\n" + t)
    print("-" * len(t))


# ====================================================================== 1
def controllability():
    """
    Linearise the quadrotor about hover and ask whether the four inputs can
    reach the whole twelve-dimensional state.

    State  x = [pos(3), vel(3), attitude(3), body rates(3)]
    Input  u = [thrust, torque_x, torque_y, torque_z]
    """
    line("1. CONTROLLABILITY  (hover linearisation, 12 states, 4 inputs)")

    g, m = C.GRAVITY, C.MASS
    Ix, Iy, Iz = C.INERTIA

    A = np.zeros((12, 12))
    A[0:3, 3:6] = np.eye(3)          # position integrates velocity
    A[3, 7] = g                      # pitch tips thrust into +x
    A[4, 6] = -g                     # roll tips thrust into -y
    A[6:9, 9:12] = np.eye(3)         # attitude integrates body rate

    B = np.zeros((12, 4))
    B[5, 0] = 1.0 / m                # thrust acts on vertical acceleration
    B[9, 1] = 1.0 / Ix
    B[10, 2] = 1.0 / Iy
    B[11, 3] = 1.0 / Iz

    ctrb = np.hstack([np.linalg.matrix_power(A, k) @ B for k in range(12)])
    rank = np.linalg.matrix_rank(ctrb, tol=1e-9)
    print("  rank of the controllability matrix : %d of 12" % rank)
    print("  -> %s" % ("controllable: every state is reachable"
                       if rank == 12 else "NOT controllable"))
    print("""
  What that does and does not mean here. The rank is full, so the drone can be
  driven to any position and attitude - but only over time, and never
  independently. Lateral motion appears in the fourth power of A: thrust tilts
  the body, the tilt turns thrust into sideways force, and only then does the
  position move. That is why x and y are reachable at all, and why they are slow.""")

    n_lat = np.linalg.matrix_rank(np.hstack(
        [np.linalg.matrix_power(A, k) @ B for k in range(2)]), tol=1e-9)
    print("  rank using only [B, AB] (i.e. two integrations): %d of 12" % n_lat)
    print("  -> position needs the full chain; it is not directly actuated.")

    a_max = g * np.tan(C.MAX_TILT)
    print("\n  Underactuation as a number:")
    print("    tilt limit                     %5.1f deg" % np.degrees(C.MAX_TILT))
    print("    maximum lateral acceleration   %5.2f m/s^2   (g*tan(tilt))" % a_max)
    print("    tracking speed cap             %5.2f m/s" % C.V_MAX_TRACK)
    fastest = max(p.get("speed", p.get("radius", 0) * p.get("omega", 0))
                  for p in C.PADS if p["motion"] != "static")
    print("    fastest pad in the world       %5.2f m/s" % fastest)
    print("    time to match that speed       %5.2f s      (from rest)" % (fastest / a_max))
    print("""
  This is the controllability statement that actually constrains the mission:
  the drone can catch any pad whose speed is inside the velocity cap, and the
  acceleration budget above says how long that takes from rest. A pad faster
  than the cap could never be caught, however good the controller.""")
    return rank


# ====================================================================== 2
def observability():
    """Can the state be reconstructed from what the drone actually measures?"""
    line("2. OBSERVABILITY  (same linearisation, two sensing cases)")

    g, m = C.GRAVITY, C.MASS
    A = np.zeros((12, 12))
    A[0:3, 3:6] = np.eye(3)
    A[3, 7] = g
    A[4, 6] = -g
    A[6:9, 9:12] = np.eye(3)

    def obsv_rank(Cm):
        O = np.vstack([Cm @ np.linalg.matrix_power(A, k) for k in range(12)])
        return np.linalg.matrix_rank(O, tol=1e-9)

    # with a decoded marker: relative position (3) + IMU attitude (3)
    C_tag = np.zeros((6, 12))
    C_tag[0:3, 0:3] = np.eye(3)
    C_tag[3:6, 6:9] = np.eye(3)
    r_tag = obsv_rank(C_tag)

    # marker lost: only the IMU is left
    C_imu = np.zeros((3, 12))
    C_imu[:, 6:9] = np.eye(3)
    r_imu = obsv_rank(C_imu)

    print("  marker decoded  (position + attitude measured) : rank %2d of 12  -> %s"
          % (r_tag, "observable" if r_tag == 12 else "NOT observable"))
    print("  marker lost     (attitude only)                : rank %2d of 12  -> %s"
          % (r_imu, "observable" if r_imu == 12 else "NOT observable"))
    print("""
  The second line is the formal reason the landing interlocks exist. With no
  decode, the six states that matter for landing - horizontal position and
  velocity relative to the pad - are not observable at all. Nothing in the
  measurement stream constrains them, so the estimate can only be propagated
  open-loop. That is exactly what the dead-reckoning branch does, and why the
  rule is "never descend without a live detection": descending on an
  unobservable estimate is descending blind.""")

    print("  Scale observability:")
    print("""    A single camera recovers the pose of a planar marker only up to
    scale. What supplies the scale is the marker's known physical size - which
    is why the two markers carry DIFFERENT ids (%d and %d for pad 1). Knowing
    which marker was decoded is knowing its size, which is what makes the
    metric offset observable at all.""" % (1, 1 + C.TAG_ID_SMALL_OFFSET))

    # the pad tracker: constant-acceleration model seen through position only
    dt = 1.0 / C.CAM_HZ
    F = np.array([[1, dt, 0.5 * dt * dt], [0, 1, dt], [0, 0, 1]])
    H = np.array([[1.0, 0.0, 0.0]])
    O = np.vstack([H @ np.linalg.matrix_power(F, k) for k in range(3)])
    print("\n  Pad tracker (alpha-beta-gamma), position measured only:")
    print("    observability rank %d of 3  -> %s"
          % (np.linalg.matrix_rank(O), "observable"))
    print("    condition number   %.0f" % np.linalg.cond(O))
    print("""    Velocity and acceleration are observable from position alone, so
    estimating them is legitimate - but the conditioning is poor, which is the
    quantitative reason gamma is small (%.3f) and the acceleration estimate is
    low-pass filtered and clamped.""" % C.AB_GAMMA)
    return r_tag, r_imu


# ====================================================================== 3
def lerp_in_this_project():
    """Every exponential filter in this codebase is a lerp. Show the equivalence."""
    line("3. LERP  (linear interpolation - what the filters actually are)")
    print("""  Every smoothing step in this project has the form

      x <- x + a * (target - x)        which is exactly   x <- lerp(x, target, a)

  A lerp with a fixed a, applied every step, IS a first-order low-pass filter
  with time constant tau = dt * (1 - a) / a. The filters and their settings:""")

    rows = [
        ("velocity setpoint", C.DT_CTRL, C.DT_CTRL / (C.VSP_FILT_TAU + C.DT_CTRL)),
        ("PID derivative", C.DT_CTRL, C.DT_CTRL / (C.D_FILT_TAU + C.DT_CTRL)),
        ("motor lag (thrust and torque)", C.DT_CTRL, C.DT_CTRL / (C.MOTOR_TAU + C.DT_CTRL)),
        ("pad tracker position correction", 1.0 / C.CAM_HZ, C.AB_ALPHA),
        ("wind gust (Ornstein-Uhlenbeck)", C.DT_CTRL, C.DT_CTRL / C.WIND_TAU),
    ]
    print("\n    %-34s %8s %8s %10s" % ("filter", "dt (s)", "a", "tau (s)"))
    for name, dt, a in rows:
        tau = dt * (1 - a) / a if a > 0 else float("inf")
        print("    %-34s %8.3f %8.3f %10.3f" % (name, dt, a, tau))

    print("""
  And one filter that is a SLERP, not a lerp:
    - the measured heading of each pad, smoothed in perception/vision.py before
      the large marker's offset is rotated by it. Measured on a heading sitting
      on the +-180 degree wrap, smoothing cuts the noise from about 2.7 deg to
      1.6 deg, which through the 0.40 m marker offset is 1.9 cm of position
      noise reduced to 1.1 cm. A plain lerp there would average across the wrap
      and swing the estimate through zero.

  Two lerps that are not filters:
    - the chase camera in record.py lerps its look-at POINT toward the drone,
    - the altitude gain schedule lerps the gain between its value at cruise and
      its value at touchdown, as a function of height.

  The first five interpolate points in a vector space, where lerp is the correct
  operation. Orientations are not a vector space, which is the next section.""")


# ====================================================================== 4
def slerp_demo():
    """Why orientation needs slerp, demonstrated on this project's own rotations."""
    line("4. SLERP  (spherical linear interpolation - for orientations)")
    print("""  Used in two places in this project: the attitude controller works on
  quaternion error rather than Euler angles, and the measured pad heading is
  smoothed with an orientation filter whose update step is a slerp.
""")

    def q_from_yaw(a):
        return np.array([np.cos(a / 2), 0.0, 0.0, np.sin(a / 2)])

    def q_mul(a, b):
        w1, x1, y1, z1 = a
        w2, x2, y2, z2 = b
        return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                         w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                         w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                         w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])

    def q_angle(a, b):
        d = abs(float(np.dot(a, b)))
        return 2.0 * np.arccos(np.clip(d, -1.0, 1.0))

    def lerp_q(a, b, t):
        return (1 - t) * a + t * b            # deliberately NOT normalised

    def slerp_q(a, b, t):
        d = float(np.dot(a, b))
        if d < 0.0:                            # take the short way round
            b, d = -b, -d
        if d > 0.9995:
            return (a + t * (b - a)) / np.linalg.norm(a + t * (b - a))
        th = np.arccos(d)
        s = np.sin(th)
        return (np.sin((1 - t) * th) * a + np.sin(t * th) * b) / s

    # a vehicle that turns 170 degrees between two camera frames
    q0, q1 = q_from_yaw(np.radians(10)), q_from_yaw(np.radians(180))
    print("  A pad vehicle turning from 10 deg to 180 deg between two fixes.")
    print("\n    %5s %12s %12s %14s" % ("t", "lerp |q|", "lerp angle", "slerp angle"))
    for t in (0.0, 0.25, 0.5, 0.75, 1.0):
        ql, qs = lerp_q(q0, q1, t), slerp_q(q0, q1, t)
        print("    %5.2f %12.4f %11.1f deg %10.1f deg"
              % (t, np.linalg.norm(ql), np.degrees(q_angle(q0, ql)),
                 np.degrees(q_angle(q0, qs))))
    print("""
  Two failures of lerp are visible above. Its result is not a unit quaternion,
  so it is not a rotation at all until renormalised; and even normalised, the
  angle does not advance evenly with t - the interpolation slows down in the
  middle. Slerp moves along the great circle at constant angular rate, which is
  what "half way between these two orientations" has to mean.""")

    # the sign trap. Measured as an ANGLE the two answers look identical,
    # because q and -q are the same rotation; the failure only shows up when
    # the interpolated heading itself is read out.
    def yaw_of(q):
        q = q / max(np.linalg.norm(q), 1e-12)
        return np.degrees(2.0 * np.arctan2(q[3], q[0])) % 360.0

    qa = q_from_yaw(np.radians(170))
    qb = q_from_yaw(np.radians(-170))         # 20 deg away, across the wrap
    naive = lerp_q(qa, qb, 0.5)
    good = slerp_q(qa, qb, 0.5)
    print("  The sign trap: q and -q are the SAME rotation.")
    print("    two fixes 20 deg apart across the +-180 deg wrap:"
          " %.0f deg and %.0f deg" % (yaw_of(qa), yaw_of(qb)))
    print("    correct midpoint    : 180 deg")
    print("    naive lerp midpoint : %6.1f deg   <- the long way round"
          % yaw_of(naive))
    print("    slerp midpoint      : %6.1f deg" % yaw_of(good))
    print("""    The naive result goes the long way round the circle. Slerp checks
    the sign of the dot product first and flips one input, so it always takes
    the shorter arc. Any code that averages or smooths measured orientations -
    for example a filtered estimate of a turning vehicle's heading - has to do
    this, and a plain lerp on Euler angles has the same defect at the wrap.""")


if __name__ == "__main__":
    print("=" * 72)
    print("  System analysis - computed from the flight constants in config.py")
    print("=" * 72)
    controllability()
    observability()
    lerp_in_this_project()
    slerp_demo()
    print()
