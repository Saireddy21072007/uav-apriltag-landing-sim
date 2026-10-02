"""
The ten addressed landing targets.

Each target is an "AirTag address" (1..10): a street-style label the operator
types, a BLE/UWB beacon that gives a coarse fix anywhere in the arena, and a
physical landing pad carrying the two AprilTags of section 2.2 of the base
paper (a 0.30 m tag for high altitude, a 0.06 m tag for the last two metres).

Some pads sit on the ground, some sit on a moving ground vehicle - the paper's
"dynamic tracking" case.
"""
import math
import numpy as np

import config as C


class AirTag:
    """One addressed landing pad."""

    def __init__(self, tag_id, address, home, motion="static", **kw):
        self.id = tag_id                    # AprilTag family id, 1..10
        self.address = address              # what the operator types
        self.home = np.array(home, float)   # nominal pad position, m
        self.pos = self.home.copy()         # live position, m
        self.vel = np.zeros(2)              # live velocity, m/s
        self.motion = motion
        self.p = kw                         # motion parameters
        self._t = 0.0

    # ------------------------------------------------------------------ motion
    def step(self, dt):
        """Advance the pad. Ground vehicles keep moving while the drone chases."""
        self._t += dt
        t = self._t
        if self.motion == "static":
            new = self.home.copy()

        elif self.motion == "line":
            # back-and-forth along a heading, like a vehicle on a service road
            spd, hdg, half = self.p["speed"], self.p["heading"], self.p["half_len"]
            s = half * math.sin(spd * t / max(half, 1e-6))
            d = np.array([math.cos(hdg), math.sin(hdg)])
            new = self.home + s * d

        elif self.motion == "circle":
            r, w = self.p["radius"], self.p["omega"]
            new = self.home + r * np.array([math.cos(w * t), math.sin(w * t)])

        elif self.motion == "drift":
            # slow wandering pad (trolley nudged by people)
            a, w = self.p["amp"], self.p["omega"]
            new = self.home + a * np.array([math.sin(w * t), math.sin(0.7 * w * t + 1.1)])

        else:
            raise ValueError(self.motion)

        self.vel = (new - self.pos) / dt if dt > 0 else np.zeros(2)
        self.pos = new

    def reset(self):
        self._t = 0.0
        self.pos = self.home.copy()
        self.vel = np.zeros(2)

    @property
    def speed(self):
        return float(np.linalg.norm(self.vel))

    def __repr__(self):
        return f"<AirTag {self.id} '{self.address}' {self.motion}>"


def build_fleet(seed=0):
    """The ten addresses served by this pilot deployment."""
    rng = np.random.default_rng(seed)
    del rng  # layout is deterministic on purpose; kept for future randomisation

    return [
        AirTag(1,  "A-01  Block A rooftop pad",      (-20.0,  18.0), "static"),
        AirTag(2,  "A-02  Block A loading bay",      ( -8.0,  22.0), "drift",
               amp=0.8, omega=0.25),
        AirTag(3,  "B-03  Warehouse gate 2",         ( 14.0,  20.0), "static"),
        AirTag(4,  "B-04  Delivery van (route N)",   ( 22.0,   6.0), "line",
               speed=1.6, heading=math.radians(115), half_len=9.0),
        AirTag(5,  "C-05  Hospital helipad",         ( -4.0,   2.0), "static"),
        AirTag(6,  "C-06  Patrol rover (loop)",      (  6.0,  -6.0), "circle",
               radius=7.0, omega=0.16),
        AirTag(7,  "D-07  Campus parcel locker",     (-22.0,  -8.0), "static"),
        AirTag(8,  "D-08  Service cart (aisle 3)",   ( -9.0, -18.0), "line",
               speed=1.1, heading=math.radians(20), half_len=6.5),
        AirTag(9,  "E-09  Substation pad",           ( 18.0, -20.0), "static"),
        AirTag(10, "E-10  Harbour tender (moving)",  (  2.0, -16.0), "circle",
               radius=8.0, omega=0.15),
    ]


def resolve(fleet, text):
    """Map what the operator typed ('4', 'B-04', 'van') onto one AirTag."""
    q = str(text).strip().lower()
    if q.isdigit():
        for t in fleet:
            if t.id == int(q):
                return t
        return None
    for t in fleet:                       # exact address-code match, e.g. 'b-04'
        if t.address.lower().startswith(q):
            return t
    for t in fleet:                       # loose keyword match, e.g. 'van'
        if q and q in t.address.lower():
            return t
    return None
