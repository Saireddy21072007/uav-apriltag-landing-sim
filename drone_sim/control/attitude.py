"""
Inner flight control: acceleration -> desired orientation -> torque.

A quadrotor has four inputs (one thrust, three torques) and six degrees of
freedom, so lateral motion is not commanded directly - it is bought by tilting.
This module turns a desired acceleration vector into the orientation that
produces it and the torques that achieve that orientation:

    desired accel  ->  desired thrust direction  ->  desired ORIENTATION (quat)
                   ->  error quaternion  ->  body torque

The base paper does this last part in Euler angles. This implementation works
in unit quaternions, which changes three things:

  * The desired orientation is BUILT, not decomposed. Given the thrust
    direction and a yaw reference, the desired body axes are constructed
    directly and turned into a quaternion; no roll/pitch angles are ever
    extracted, so nothing is singular and nothing wraps.
  * The attitude error is the rotation that takes the current orientation to the
    desired one - q_err = q^-1 * q_des - and the torque is proportional to its
    rotation vector. That is a genuine "rotate about this axis by this much"
    command, not three independent angle errors.
  * Sign correction on q_err guarantees the shortest arc, so the drone never
    turns the long way round to reach an orientation 5 degrees away.

Two consequences the outer guidance loop still has to live with, and which a
world-frame-force simulator hides completely:

  * lateral acceleration is capped by MAX_TILT, and
  * every lateral correction tilts the camera, which swings the marker across
    the image and can push it out of frame at low altitude.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config as C
from control import quaternion as Q


def desired_orientation(acc_des, yaw_des=0.0):
    """
    Desired world acceleration (excluding gravity) -> (thrust vector, q_des).

    The thrust axis must point along a_des + g*z_up. That single vector fixes
    two of the three degrees of freedom of the orientation; the third, rotation
    about the thrust axis, is fixed by the yaw reference. Building the rotation
    matrix from those two facts and converting to a quaternion is the standard
    geometric construction, and it never passes through an Euler angle.
    """
    f = np.array([acc_des[0], acc_des[1], acc_des[2] + C.GRAVITY], float) * C.MASS
    f[2] = max(f[2], 0.25 * C.HOVER_THRUST)      # never command negative lift

    # limit the tilt by limiting how far the thrust axis may lean from vertical
    lean = np.linalg.norm(f[:2])
    max_lean = f[2] * np.tan(C.MAX_TILT)
    if lean > max_lean > 0.0:
        f[:2] *= max_lean / lean

    b3 = f / max(np.linalg.norm(f), 1e-9)        # desired body z, in world
    c1 = np.array([np.cos(yaw_des), np.sin(yaw_des), 0.0])   # yaw reference
    b2 = np.cross(b3, c1)
    n2 = np.linalg.norm(b2)
    if n2 < 1e-6:                                # thrust axis parallel to c1
        c1 = np.array([-np.sin(yaw_des), np.cos(yaw_des), 0.0])
        b2 = np.cross(b3, c1)
        n2 = np.linalg.norm(b2)
    b2 /= n2
    b1 = np.cross(b2, b3)

    R_des = np.column_stack((b1, b2, b3))
    return f, rot_to_quat(R_des)


def rot_to_quat(R):
    """
    Rotation matrix -> quaternion, via the largest-diagonal branch.

    Picking the branch by the largest term avoids dividing by something close to
    zero, which is what makes the naive single-formula conversion lose precision
    at large rotation angles.
    """
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return Q.normalize(np.array([w, x, y, z]))


def attitude_torque(q, omega, q_des):
    """
    Torque from the quaternion attitude error.

    q_err = q^-1 * q_des is the rotation, expressed in the BODY frame, that
    still has to be applied. Its rotation vector is the axis-times-angle the
    drone must turn through, sign-corrected to the short way round, so a simple
    proportional term on it plus rate damping is a complete attitude loop - no
    per-axis Euler errors, no wrap handling, no small-angle assumption.
    """
    q_err = Q.mul(Q.conj(q), q_des)
    e = Q.to_rotvec(q_err)                      # body-frame axis * angle
    gains = np.array([C.KP_ATT, C.KP_ATT, C.KP_YAW])
    damp = np.array([C.KD_RATE, C.KD_RATE, C.KD_YAW])
    inertia = np.array(C.INERTIA)
    return inertia * (gains * e - damp * np.asarray(omega, float)) * 10.0


class InnerLoop:
    """
    Velocity-tracking inner loop.

    Takes a velocity setpoint in the world frame and produces the thrust and
    torques that chase it. The outer guidance loop only ever speaks in velocity
    setpoints, exactly like the velocity API the base paper calls on the real
    M100 - so the two can be compared even though the attitude representation
    underneath is different.

    The horizontal channel carries a small integral term. A purely proportional
    velocity loop cannot hold station in a steady wind: holding against drag
    needs a standing lean, and the only way a P loop produces one is by keeping
    a standing velocity error of F_drag / (KP_VEL * MASS). Here that is about
    3.6 cm/s - the same order as the correction the vision loop asks for at
    touchdown height - so the wind is trimmed where it acts instead of being
    left for the outer loop to fight.
    """

    def __init__(self):
        self.yaw_des = 0.0
        self._vi = np.zeros(2)        # velocity-loop integral, m/s * s

    def __call__(self, state, vel_sp, yaw_des=None, dt=C.DT_CTRL):
        if yaw_des is not None:
            self.yaw_des = yaw_des
        vel = state["vel"]
        ev = np.asarray(vel_sp[:2], float) - np.asarray(vel[:2], float)
        acc = np.array([
            C.KP_VEL * ev[0] + C.KI_VEL * self._vi[0],
            C.KP_VEL * ev[1] + C.KI_VEL * self._vi[1],
            C.KP_VZ * (vel_sp[2] - vel[2]),
        ])
        # cap lateral demand at what the tilt limit can actually deliver
        a_lat = float(np.hypot(acc[0], acc[1]))
        a_max = C.GRAVITY * np.tan(C.MAX_TILT)
        if a_lat > a_max:
            acc[:2] *= a_max / a_lat
        else:
            # Integrate only when there is authority left to use it, so a
            # saturated manoeuvre cannot wind the trim up behind the limiter.
            self._vi = np.clip(self._vi + ev * dt, -C.VI_CLAMP, C.VI_CLAMP)
        acc[2] = float(np.clip(acc[2], -4.0, 6.0))

        f_des, q_des = desired_orientation(acc, self.yaw_des)

        q = state["quat"]
        # Thrust is the projection of the desired force onto the axis the drone
        # is CURRENTLY pointing along, not the length of the desired force.
        # Sizing it for an orientation the airframe has not reached yet leaves
        # surplus lift, and the drone climbs away from its altitude setpoint
        # whenever it manoeuvres hard.
        body_z = Q.to_rot(q)[:, 2]
        thrust = float(np.clip(float(np.dot(f_des, body_z)), 0.0, C.MAX_THRUST))
        torque = attitude_torque(q, state["omega"], q_des)
        return thrust, torque, q_des
