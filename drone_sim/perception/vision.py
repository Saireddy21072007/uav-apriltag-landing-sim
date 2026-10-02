"""
Perception: real camera frames -> real AprilTag detections -> pad-centre offset.

Nothing here is faked. The image comes from the MuJoCo offscreen renderer
looking through the drone's own camera at the textures painted on the pads;
pupil_apriltags decodes it; cv2.solvePnP recovers the tag pose from the corner
pixels and the known metric tag size. If the drone tilts, the tag moves in the
image and the pose changes accordingly, because it is the same image a real
camera in that pose would produce.

Two design choices worth stating:

  * Each pad carries two tags with DIFFERENT ids (large = k, small = k + 10).
    The detector therefore never has to guess which physical marker it decoded,
    and never needs the true altitude to disambiguate - a shortcut that quietly
    leaks ground truth into the perception layer.
  * The large tag is deliberately offset from the pad centre. Its pose is
    converted to a pad-centre offset using the tag's own measured yaw, so the
    correction uses measured data rather than a known pad orientation.
"""
import os
import sys
from dataclasses import dataclass

import cv2
import numpy as np
from pupil_apriltags import Detector

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config as C
from control import quaternion as Q

# OpenCV camera axes (x right, y down, z forward) expressed in the MuJoCo camera
# frame (x right, y up, z backwards). Getting this wrong flips the sign of every
# correction and the drone flies away from the pad.
CV_TO_MJ = np.diag([1.0, -1.0, -1.0])

K = np.array([[C.FOCAL_PX, 0.0, C.CX],
              [0.0, C.FOCAL_PX, C.CY],
              [0.0, 0.0, 1.0]])
DIST = np.zeros((5, 1))


@dataclass
class Sighting:
    pad_id: int
    tag_id: int
    is_large: bool
    tag_size: float
    offset: np.ndarray      # world-frame (x, y) from drone to PAD CENTRE, m
    tag_offset: np.ndarray  # world-frame (x, y) from drone to the tag itself, m
    height: float           # camera height above the pad surface, m
    pad_yaw: float          # measured pad heading, rad
    pixels: float           # apparent tag edge, px
    margin: float           # detector decision margin
    corners: np.ndarray


def _rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s], [s, c]])


def _att_to_rot(att):
    """
    Attitude estimate -> rotation matrix (body to world).

    Accepts a quaternion, which is what the IMU now reports. A three-element
    input is still understood as roll/pitch/yaw so that older calls and the
    unit checks keep working, but the flight path never uses that branch: going
    through Euler angles here would reintroduce exactly the wrap and
    singularity problems the quaternion attitude representation removes.
    """
    att = np.asarray(att, float)
    if att.size == 4:
        return Q.to_rot(att)
    r, p, y = att
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _yaw_of(q):
    """Heading of a yaw-only quaternion, radians."""
    return float(2.0 * np.arctan2(q[3], q[0]))


def _tag_object_points(size):
    """Tag corners in the tag's own frame, in the order pupil_apriltags returns."""
    h = size / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])


class PadVision:
    """Runs the detector on a camera frame and reports pad-centre offsets."""

    def __init__(self, nthreads=2, smooth_heading=True):
        self.detector = Detector(families=C.TAG_FAMILY, nthreads=nthreads,
                                 quad_decimate=1.0, refine_edges=True)
        self.frames = 0
        self.hits = 0
        # One orientation filter per pad, smoothing the measured pad heading.
        # This matters because the large marker sits TAG_LARGE_OFFSET from the
        # pad centre, and that offset is rotated by the measured heading: a few
        # degrees of yaw noise becomes a couple of centimetres of position noise
        # in the estimate the controller flies on. The filter is a SLERP, not a
        # lerp - a vehicle driving a circle crosses the +-180 degree wrap on
        # every lap, and averaging angles across that wrap is wrong by 360.
        self.smooth_heading = smooth_heading
        self._heading = {}

    @staticmethod
    def pad_of(tag_id):
        """Which pad a tag id belongs to, and whether it is the large marker."""
        if 1 <= tag_id <= len(C.PADS):
            return tag_id, True
        if C.TAG_ID_SMALL_OFFSET < tag_id <= C.TAG_ID_SMALL_OFFSET + len(C.PADS):
            return tag_id - C.TAG_ID_SMALL_OFFSET, False
        return None, None

    def look(self, frame, att_est, only_pad=None):
        """
        frame     : RGB image from the onboard camera
        att_est   : attitude estimate from the IMU, as a quaternion [w,x,y,z]
        only_pad  : if given, ignore every pad except this one

        Returns the sightings, best (largest apparent) first.
        """
        self.frames += 1
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY) if frame.ndim == 3 else frame
        raw = self.detector.detect(gray)
        R_wb = _att_to_rot(att_est)            # body -> world

        out = []
        for det in raw:
            pad_id, is_large = self.pad_of(int(det.tag_id))
            if pad_id is None:
                continue
            if only_pad is not None and pad_id != only_pad:
                continue
            if float(det.decision_margin) < C.DECISION_MARGIN_MIN:
                continue

            size = C.TAG_LARGE if is_large else C.TAG_SMALL
            corners = np.asarray(det.corners, dtype=np.float64)
            ok, rvec, tvec = cv2.solvePnP(_tag_object_points(size), corners, K, DIST,
                                          flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue

            # tag position, camera(OpenCV) -> camera(MuJoCo) -> body -> world
            p_cam = CV_TO_MJ @ tvec.reshape(3)
            p_world = R_wb @ p_cam
            if p_world[2] > -0.02:             # tag must be below the drone
                continue

            # tag orientation gives the pad heading, which is how the offset of
            # the large tag from the pad centre is rotated into the world frame
            R_ct, _ = cv2.Rodrigues(rvec)
            R_wt = R_wb @ CV_TO_MJ @ R_ct
            pad_yaw = float(np.arctan2(R_wt[1, 0], R_wt[0, 0]))

            if self.smooth_heading:
                filt = self._heading.setdefault(pad_id, Q.OrientationFilter(0.35))
                pad_yaw = _yaw_of(filt.update(Q.from_yaw(pad_yaw)))

            tag_xy = p_world[:2]
            if is_large:
                centre_xy = tag_xy - _rot_z(pad_yaw) @ np.array([C.TAG_LARGE_OFFSET, 0.0])
            else:
                centre_xy = tag_xy

            edge = float(np.linalg.norm(corners[0] - corners[1]))
            out.append(Sighting(
                pad_id=pad_id, tag_id=int(det.tag_id), is_large=is_large,
                tag_size=size, offset=centre_xy, tag_offset=tag_xy,
                height=float(-p_world[2]), pad_yaw=pad_yaw,
                pixels=edge, margin=float(det.decision_margin), corners=corners))

        out.sort(key=lambda s: -s.pixels)
        if out:
            self.hits += 1
        return out

    @staticmethod
    def pick(sightings):
        """
        Choose the sighting to fly on.

        The smaller tag wins whenever it is available: it is closer to the pad
        centre, it is fully in frame, and its pose is the more accurate one at
        the altitude where it can be seen at all. This is the multi-scale
        selection rule of the base paper, made explicit.
        """
        if not sightings:
            return None
        small = [s for s in sightings if not s.is_large]
        return small[0] if small else sightings[0]
