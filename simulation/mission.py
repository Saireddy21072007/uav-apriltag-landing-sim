"""
The mission: fly to an AirTag address, track the pad, land on it.

State machine (mirrors the two experiments of section 4 of the base paper, with
the addressing layer added on top):

    TAKEOFF  climb to search altitude
    TRANSIT  fly to the AirTag radio fix at cruise speed        (radio guidance)
    SEARCH   expanding spiral when the radio fix was stale      (radio guidance)
    ALIGN    hold altitude, PID the vision offset to zero       (vision guidance)
    DESCEND  descend only while alignment holds; hold height if it degrades
    FINAL    below FINAL_ALT and inside FINAL_TOL -> cut motors
    LANDED / ABORTED

Guidance source is the whole point of the design: metres-accurate radio until
the AprilTag decodes, centimetres-accurate vision after that.
"""
import math

import numpy as np

import config as C
from controller import TargetTracker, altitude_controller
from drone import Drone, camera_to_body, body_to_world
from vision import AprilTagCamera, AirTagBeacon


STATES = ["TAKEOFF", "TRANSIT", "SEARCH", "ALIGN", "DESCEND", "FINAL",
          "LANDED", "ABORTED"]


class MissionLog:
    """Flat rows, one per control step - everything results.py plots."""

    def __init__(self):
        self.rows = []
        self.events = []          # (t, text) - phase changes, tag switch, holds
        self.summary = {}

    def add(self, **kw):
        self.rows.append(kw)

    def event(self, t, text):
        self.events.append((t, text))

    def col(self, name):
        return np.array([r[name] for r in self.rows])


class LandingMission:
    def __init__(self, fleet, target_id, seed=0, feedforward=True,
                 use_pid=True, gain_schedule=True, start=(0.0, 0.0, 0.0),
                 reset_fleet=True):
        self.fleet = fleet
        self.tag = next(t for t in fleet if t.id == target_id)
        if reset_fleet:                 # False when this is the second leg of a
            for t in fleet:             # multi-address sortie: the pads must keep
                t.reset()               # moving where the last leg left them
        self.drone = Drone(start, seed=seed)
        self.cam = AprilTagCamera(seed=seed)
        self.beacon = AirTagBeacon(seed=seed)
        self.tracker = TargetTracker(feedforward=feedforward,
                                     gain_schedule=gain_schedule)
        self.use_pid = use_pid          # False -> bare proportional command,
                                        #          which reproduces paper Fig.10
        self.state = "TAKEOFF"
        self.t = 0.0
        self.log = MissionLog()
        self.aligned_for = 0.0
        self.since_detection = 0.0
        self.search_t = 0.0
        self.last_good_fix = None       # last known pad position (any source)
        self.last_meas_off = None       # last vision offset, dead-reckoned when blind
        self.tag_size_in_use = None
        self._switched = False
        self.result = None

    # --------------------------------------------------------------- helpers
    def _measure(self, det):
        """
        Turn a detection into a world-frame offset, going through the section
        3.2 chain of the base paper: camera frame -> body frame -> world frame.
        """
        h = max(self.drone.altitude, 1e-3)
        v_cam = np.array([det.offset[0], -det.offset[1], h])   # camera axes
        v_body = camera_to_body(v_cam)                          # Fig.9 flip
        v_world = body_to_world(v_body, self.drone.yaw)
        return v_world[:2]

    def _set_state(self, s):
        if s != self.state:
            self.log.event(self.t, "%s -> %s" % (self.state, s))
            self.state = s

    # -------------------------------------------------------------- one step
    def step(self, dt=C.DT):
        self.tag.step(dt)
        for other in self.fleet:                 # the rest of the world moves too
            if other is not self.tag:
                other.step(dt)

        det = self.cam.sense(self.drone.p, self.tag, dt)
        fix = self.beacon.sense(self.drone.p, self.tag, dt)

        self.tracker.predict(dt)                 # pad estimate rolls forward always

        have_vision = det is not None
        if have_vision:
            self.since_detection = 0.0
            meas_off = self._measure(det)
            self.last_good_fix = self.drone.p[:2] + meas_off
            if self.cam.fresh:                   # correct only on a new frame
                self.tracker.observe(self.last_good_fix)
            if det.size == C.TAG_SMALL and not self._switched:
                self._switched = True
                self.log.event(self.t, "tag switch: 0.30 m -> 0.06 m at "
                                       "h=%.2f m" % self.drone.altitude)
            self.tag_size_in_use = det.size
            self.last_meas_off = meas_off.copy()
        else:
            self.since_detection += dt
            meas_off = None
            if fix is not None:
                self.last_good_fix = np.asarray(fix, float)
            self.tag_size_in_use = None
            # Dead-reckon the held offset: the pad keeps moving at its estimated
            # velocity and the drone keeps moving at its own, so the stale offset
            # is propagated instead of being treated as zero. Treating a missing
            # detection as "centred" is how a real vehicle lands on the grass.
            if self.last_meas_off is not None:
                self.last_meas_off = (self.last_meas_off
                                      + (self.tracker.target_velocity
                                         - self.drone.v[:2]) * dt)

        v_cmd = np.zeros(3)
        h = self.drone.altitude

        # ------------------------------------------------------------ TAKEOFF
        if self.state == "TAKEOFF":
            v_cmd[2] = altitude_controller(C.TAKEOFF_ALT, h)
            if h > C.TAKEOFF_ALT - 0.25:
                self._set_state("TRANSIT")

        # ------------------------------------------------------------ TRANSIT
        elif self.state == "TRANSIT":
            v_cmd[2] = altitude_controller(C.CRUISE_ALT, h)
            if have_vision:
                self.tracker.reset()
                self._set_state("ALIGN")
            elif self.last_good_fix is not None:
                err = self.last_good_fix - self.drone.p[:2]
                d = float(np.linalg.norm(err))
                v = 0.9 * err                     # simple P guidance on radio fix
                n = np.linalg.norm(v)
                if n > C.V_MAX_CRUISE:
                    v *= C.V_MAX_CRUISE / n
                v_cmd[:2] = v
                if d < 1.5:                       # over the radio fix, still blind
                    self.search_t = 0.0
                    self._set_state("SEARCH")
            else:
                v_cmd[:2] = np.array([1.0, 0.0])  # no radio yet: creep forward

        # ------------------------------------------------------------- SEARCH
        elif self.state == "SEARCH":
            v_cmd[2] = altitude_controller(C.CRUISE_ALT, h)
            self.search_t += dt
            if have_vision:
                self.tracker.reset()
                self._set_state("ALIGN")
            else:
                # expanding spiral around the last known fix
                w, r = 0.7, 0.6 + 0.35 * self.search_t
                centre = (self.last_good_fix if self.last_good_fix is not None
                          else self.drone.p[:2])
                want = centre + r * np.array([math.cos(w * self.search_t),
                                              math.sin(w * self.search_t)])
                v_cmd[:2] = np.clip(1.2 * (want - self.drone.p[:2]), -2.5, 2.5)
                if self.search_t > 25.0:
                    self._set_state("TRANSIT")    # go back to the radio fix

        # -------------------------------------------------------------- ALIGN
        elif self.state == "ALIGN":
            v_cmd[2] = altitude_controller(C.CRUISE_ALT, h)
            if not have_vision and self.since_detection > C.LOST_TIMEOUT:
                self._set_state("TRANSIT")
            elif meas_off is not None:
                v_cmd[:2] = self._horizontal(meas_off, dt)
                if np.linalg.norm(meas_off) < C.ALIGN_TOL:
                    self.aligned_for += dt
                else:
                    self.aligned_for = 0.0
                if self.aligned_for > C.ALIGN_HOLD:
                    self._set_state("DESCEND")

        # ------------------------------------------------------------ DESCEND
        elif self.state == "DESCEND":
            if not have_vision and self.since_detection > C.LOST_TIMEOUT:
                v_cmd[2] = altitude_controller(C.CRUISE_ALT, h)
                self._set_state("ALIGN")          # climb and re-acquire
            else:
                off = meas_off if meas_off is not None else self.last_meas_off
                off = off if off is not None else np.zeros(2)
                v_cmd[:2] = self._horizontal(off, dt)
                e = float(np.linalg.norm(off))
                # Approach cone: the offset allowed shrinks with altitude, so the
                # drone funnels onto the pad instead of descending on a bad guess.
                allowed = max(C.FINAL_TOL, C.CONE_SLOPE * h)
                if not have_vision:
                    v_cmd[2] = 0.0                # never descend blind
                elif e > C.DESCEND_ABORT:
                    v_cmd[2] = +0.5               # badly off -> climb back
                    self._note_pause()
                elif e > allowed:
                    v_cmd[2] = 0.0                # outside the cone -> hold height
                else:
                    quality = max(0.0, 1.0 - e / allowed)
                    v_cmd[2] = -np.clip(0.20 + 0.70 * quality, 0.0, C.V_MAX_VERT)
                    if h < 1.0:                   # ease off for final centring
                        v_cmd[2] = max(v_cmd[2], -0.30)
                if have_vision and h < C.FINAL_ALT and e < C.FINAL_TOL:
                    self._set_state("FINAL")

        # -------------------------------------------------------------- FINAL
        elif self.state == "FINAL":
            off = meas_off if meas_off is not None else self.last_meas_off
            off = off if off is not None else np.zeros(2)
            v_cmd[:2] = self._horizontal(off, dt)
            v_cmd[2] = -0.40                       # committed descent
            if float(np.linalg.norm(off)) > 3 * C.FINAL_TOL:
                self._set_state("DESCEND")         # drifted off during the commit
            if h <= 0.06:
                self.drone.cut_motors()
                self._set_state("LANDED")
                self._finish("LANDED")

        # ------------------------------------------------ battery / time guard
        if self.state not in ("LANDED", "ABORTED"):
            if self.t > C.MAX_MISSION_TIME or self.drone.battery_left <= 0.0:
                self._set_state("ABORTED")
                self._finish("ABORTED")

        self.drone.step(v_cmd, dt)
        self.t += dt

        true_off = self.tag.pos - self.drone.p[:2]
        self.log.add(t=self.t, x=self.drone.p[0], y=self.drone.p[1],
                     z=self.drone.p[2], vx=self.drone.v[0], vy=self.drone.v[1],
                     vz=self.drone.v[2], cvx=v_cmd[0], cvy=v_cmd[1], cvz=v_cmd[2],
                     tag_x=self.tag.pos[0], tag_y=self.tag.pos[1],
                     ex=true_off[0], ey=true_off[1],
                     err=float(np.linalg.norm(true_off)),
                     mex=(meas_off[0] if meas_off is not None else np.nan),
                     mey=(meas_off[1] if meas_off is not None else np.nan),
                     seen=1.0 if have_vision else 0.0,
                     tag_size=(self.tag_size_in_use or np.nan),
                     state=self.state)
        return self.state

    def _horizontal(self, off, dt):
        """PID guidance (paper eq.9), or the bare proportional law without it."""
        if self.use_pid:
            return self.tracker.step(off, dt, altitude=self.drone.altitude)
        v = C.KP * np.asarray(off, float) * 3.0
        n = np.linalg.norm(v)
        return v * (C.V_MAX_TRACK / n) if n > C.V_MAX_TRACK else v

    def _note_pause(self):
        if not self.log.events or "descent paused" not in self.log.events[-1][1]:
            self.log.event(self.t, "descent paused: offset exceeded 1.20 m")

    def _finish(self, outcome):
        final_off = self.tag.pos - self.drone.p[:2]
        self.result = {
            "outcome": outcome,
            "tag_id": self.tag.id,
            "address": self.tag.address,
            "moving": self.tag.motion != "static",
            "time_s": self.t,
            "error_m": float(np.linalg.norm(final_off)),
            "error_x": float(final_off[0]),
            "error_y": float(final_off[1]),
            "detect_rate": self.cam.hits / max(self.cam.frames, 1),
        }
        self.log.summary = self.result

    # ------------------------------------------------------------------- run
    def run(self, verbose=False):
        while self.state not in ("LANDED", "ABORTED"):
            self.step()
            if verbose and len(self.log.rows) % 250 == 0:
                print("  t=%6.1fs %-8s h=%5.2f m err=%5.2f m"
                      % (self.t, self.state, self.drone.altitude,
                         self.log.rows[-1]["err"]))
        return self.log
