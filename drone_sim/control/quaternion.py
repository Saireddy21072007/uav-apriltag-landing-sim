"""
Quaternion algebra for attitude, including SLERP.

The base paper works in Euler angles: it composes R = Rz(a)Ry(t)Rx(b) and
controls roll, pitch and yaw as three separate numbers. That is readable, and
for a vehicle that never leaves a shallow hover it is adequate. This project
uses unit quaternions instead, for three reasons that matter here:

  * No gimbal lock and no wrap. Euler angles are singular at pitch = 90 deg and
    discontinuous at +-180 deg of yaw. A vehicle chasing a pad that drives a
    circle passes through the yaw wrap on nearly every lap, and every difference
    taken across that wrap is wrong by 360 degrees unless it is special-cased.
  * The attitude error is a rotation. The error between where the drone points
    and where it should point is itself a rotation, and the quaternion product
    gives it directly - no trigonometry and no small-angle assumption.
  * Shortest arc for free. q and -q are the same rotation, so checking the sign
    of one dot product guarantees the controller always turns the short way
    round. In Euler angles that has to be arranged by hand, per axis.

Convention throughout: q = [w, x, y, z], unit norm, rotating BODY vectors into
the WORLD frame - the same convention MuJoCo uses, so no conversion is needed
at the boundary.
"""
import numpy as np


def identity():
    return np.array([1.0, 0.0, 0.0, 0.0])


def normalize(q):
    n = float(np.linalg.norm(q))
    return identity() if n < 1e-12 else np.asarray(q, float) / n


def mul(a, b):
    """Hamilton product: the rotation b followed by the rotation a."""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def conj(q):
    """Conjugate, which for a unit quaternion is the inverse rotation."""
    w, x, y, z = q
    return np.array([w, -x, -y, -z])


def to_rot(q):
    """Rotation matrix that takes a body vector to the world frame."""
    w, x, y, z = normalize(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def to_euler(q):
    """Roll, pitch, yaw - used for logging and display only, never for control."""
    w, x, y, z = normalize(q)
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def from_axis_angle(axis, angle):
    axis = np.asarray(axis, float)
    n = float(np.linalg.norm(axis))
    if n < 1e-12:
        return identity()
    axis = axis / n
    s = np.sin(angle / 2.0)
    return np.array([np.cos(angle / 2.0), axis[0] * s, axis[1] * s, axis[2] * s])


def from_yaw(a):
    return np.array([np.cos(a / 2.0), 0.0, 0.0, np.sin(a / 2.0)])


def from_rotvec(v):
    """Small rotation from a rotation vector (axis * angle)."""
    v = np.asarray(v, float)
    a = float(np.linalg.norm(v))
    return identity() if a < 1e-12 else from_axis_angle(v / a, a)


def to_rotvec(q):
    """
    Rotation vector (axis * angle) of q, taking the SHORT way round.

    This is the function the attitude controller is built on: the vector part of
    the error quaternion, sign-corrected, is proportional to the axis and angle
    the drone has to turn through, and it is continuous everywhere a controller
    ever operates.
    """
    q = normalize(q)
    if q[0] < 0.0:                       # q and -q are the same rotation;
        q = -q                           # this one is the shorter arc
    v = q[1:]
    s = float(np.linalg.norm(v))
    if s < 1e-9:
        return 2.0 * v                   # small-angle limit
    angle = 2.0 * np.arctan2(s, q[0])
    return v * (angle / s)


def slerp(a, b, t):
    """
    Spherical linear interpolation between two orientations.

    Straight lerp on quaternion components fails twice: the result is not a unit
    quaternion, so it is not a rotation until renormalised, and even normalised
    the angle does not advance evenly with t. Worse, because q and -q are the
    same rotation, a naive lerp between two nearby orientations can travel the
    long way round the sphere - interpolating 170 deg to 190 deg through 0 deg
    instead of through 180 deg. Slerp fixes the sign first, then moves along the
    great circle at constant angular rate.
    """
    a, b = normalize(a), normalize(b)
    d = float(np.dot(a, b))
    if d < 0.0:                          # take the short way round
        b, d = -b, -d
    if d > 0.9995:                       # nearly parallel: lerp is safe here
        return normalize(a + t * (b - a))
    th = np.arccos(np.clip(d, -1.0, 1.0))
    s = np.sin(th)
    return (np.sin((1.0 - t) * th) * a + np.sin(t * th) * b) / s


def angle_between(a, b):
    """Angle of the rotation that takes a to b, radians."""
    d = abs(float(np.dot(normalize(a), normalize(b))))
    return 2.0 * np.arccos(np.clip(d, 0.0, 1.0))


class OrientationFilter:
    """
    A low-pass filter for orientation - the spherical counterpart of the lerp
    smoothers used everywhere else in this project.

    An exponential filter on a vector is x <- lerp(x, measurement, a). The same
    filter on an orientation is q <- slerp(q, measurement, a). Using lerp here
    would smooth a turning vehicle's heading the long way round every time it
    crossed the wrap.
    """

    def __init__(self, alpha=0.35):
        self.alpha = alpha
        self.q = None

    def reset(self):
        self.q = None

    def update(self, q_meas):
        q_meas = normalize(q_meas)
        self.q = q_meas.copy() if self.q is None else slerp(self.q, q_meas, self.alpha)
        return self.q

    @property
    def value(self):
        return identity() if self.q is None else self.q
