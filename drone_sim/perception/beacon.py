"""
The AirTag radio beacon.

This is the part that makes an ADDRESS flyable. Before any camera can decode a
42 cm marker, the radio already says roughly where pad 7 is, so the drone knows
which way to fly. It is metres-accurate, never centimetres: its job is to hand
over to vision, not to land.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config as C


class AirTagBeacon:
    def __init__(self, seed=0):
        self.rng = np.random.default_rng(seed + 4242)
        self._acc = 0.0
        self.fix = None

    def reset(self):
        self._acc = 0.0
        self.fix = None

    def sense(self, drone_xy, pad, dt):
        """A noisy fix at BEACON_HZ, or None when out of radio range."""
        self._acc += dt
        if self._acc < 1.0 / C.BEACON_HZ:
            return self.fix
        self._acc = 0.0
        if np.linalg.norm(pad.pos - np.asarray(drone_xy)) > C.BEACON_RANGE:
            self.fix = None
        else:
            self.fix = pad.pos + self.rng.normal(0.0, C.BEACON_SIGMA, 2)
        return self.fix
