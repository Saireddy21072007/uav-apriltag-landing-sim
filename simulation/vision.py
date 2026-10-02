"""
Sensor models: the monocular AprilTag detector and the AirTag radio beacon.

AprilTag detector (paper section 2): a tag of edge length s seen from slant
range R projects to  px = f * s / R  pixels. The detector needs the tag to be
big enough to decode (>= MIN_TAG_PX) and small enough to fit in frame
(<= MAX_TAG_FRAC of the image), which is exactly why the paper puts TWO tag
sizes on one pad: the 0.30 m tag carries the approach, the 0.06 m tag carries
the last two metres where the big tag no longer fits in the field of view.

Positioning noise falls with apparent size, sigma = K * R / px, so the small
tag is also the more accurate source once the drone is close - the paper's
"data provided by the smaller tags were selected for more accurate positioning".
"""
from dataclasses import dataclass

import numpy as np

import config as C


@dataclass
class Detection:
    tag_id: int
    size: float          # which physical tag produced it, m
    offset: np.ndarray   # measured (tag - drone) in world x,y, m
    alt: float           # measured altitude from the tag geometry, m
    pixels: float        # apparent tag edge, px
    sigma: float         # 1-sigma of this measurement, m


class AprilTagCamera:
    """Downward-looking monocular camera running the AprilTag detector."""

    def __init__(self, seed=0):
        self.rng = np.random.default_rng(seed)
        self._acc = 0.0                    # frame-rate accumulator
        self.last = None                   # last Detection (None = no lock)
        self.fresh = False                 # True only on a newly decoded frame
        self.frames = 0
        self.hits = 0

    # ------------------------------------------------------------- geometry
    def _fov_halfwidth(self, h):
        """Ground half-extent visible at altitude h (narrow image axis), m."""
        return h * (C.IMG_H / 2.0) / C.FOCAL_PX

    def visible_range(self, size):
        """(min, max) altitude at which a tag of this edge length is usable, m."""
        h_max = C.FOCAL_PX * size / C.MIN_TAG_PX
        h_min = C.FOCAL_PX * size / (C.MAX_TAG_FRAC * C.IMG_H)
        return h_min, h_max

    # ------------------------------------------------------------- detection
    def _try_tag(self, drone_p, tag_p, size, tag_id):
        d = np.asarray(tag_p, float) - np.asarray(drone_p, float)[:2]
        h = max(drone_p[2], 1e-3)
        rng_ = float(np.hypot(np.linalg.norm(d), h))
        px = C.FOCAL_PX * size / rng_

        if px < C.MIN_TAG_PX:                       # too far / tag too small
            return None
        if px > C.MAX_TAG_FRAC * C.IMG_H:           # too close, tag overflows frame
            return None
        if np.linalg.norm(d) > self._fov_halfwidth(h):   # outside the footprint
            return None
        if self.rng.random() < C.DROPOUT_P:         # decode failure / motion blur
            return None

        sigma = C.DETECT_NOISE_K * rng_ / px
        meas = d + self.rng.normal(0.0, sigma, 2)
        alt = h + self.rng.normal(0.0, sigma)
        return Detection(tag_id, size, meas, alt, px, sigma)

    def sense(self, drone_p, tag, dt):
        """
        Run the detector at CAM_FPS. Returns a Detection or None.

        When both tags decode, the smaller one wins - that is the paper's
        multi-scale selection rule (Fig.3b).
        """
        self._acc += dt
        self.fresh = False
        if self._acc < 1.0 / C.CAM_FPS:
            return self.last
        self._acc = 0.0
        self.frames += 1

        big = self._try_tag(drone_p, tag.pos, C.TAG_BIG, tag.id)
        small = self._try_tag(drone_p, tag.pos, C.TAG_SMALL, tag.id)

        pick = small if (small is not None and drone_p[2] < C.TAG_SWITCH_ALT) else None
        if pick is None:
            pick = big if big is not None else small

        self.last = pick
        self.fresh = pick is not None
        if pick is not None:
            self.hits += 1
        return pick


class AirTagBeacon:
    """
    Coarse BLE/UWB fix from the AirTag itself.

    This is what makes an *address* flyable: before any camera can see a 30 cm
    tag, the radio already says roughly where address 7 is, so the drone knows
    which way to fly. Accuracy is metres, not centimetres - it hands over to
    vision as soon as the AprilTag decodes.
    """

    def __init__(self, seed=0):
        self.rng = np.random.default_rng(seed + 991)
        self._acc = 0.0
        self.fix = None

    def sense(self, drone_p, tag, dt):
        self._acc += dt
        if self._acc < 1.0 / C.BEACON_HZ:
            return self.fix
        self._acc = 0.0
        if np.linalg.norm(tag.pos - drone_p[:2]) > C.BEACON_RANGE:
            self.fix = None
        else:
            self.fix = tag.pos + self.rng.normal(0.0, C.BEACON_SIGMA, 2)
        return self.fix
