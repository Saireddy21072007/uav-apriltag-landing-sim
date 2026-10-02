"""
The mission: fly to an AirTag address, track the pad, land on it.

    TAKEOFF   climb to search altitude
    TRANSIT   fly to the addressed pad's radio fix              (radio guidance)
    SEARCH    expanding spiral when the radio fix was stale     (radio guidance)
    ALIGN     hold altitude, drive the vision offset to zero    (vision guidance)
    DESCEND   descend only while inside the approach cone
    FINAL     committed descent, motors cut on contact
    LANDED / ABORTED

Guidance authority passes from radio to vision exactly once, at the moment the
addressed pad's AprilTag first decodes. The drone will not accept any other
pad's tag: the ten pads carry ten different marker ids, and at search altitude
two or three of them are in frame at the same time.
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import config as C
from control.attitude import InnerLoop
from control.guidance import Guidance
from perception.beacon import AirTagBeacon
from perception.vision import PadVision

STATES = ("TAKEOFF", "TRANSIT", "SEARCH", "ALIGN", "DESCEND", "FINAL",
          "LANDED", "ABORTED")


class MissionLog:
    def __init__(self):
        self.rows = []
        self.events = []
        self.summary = {}

    def add(self, **kw):
        self.rows.append(kw)

    def event(self, t, text):
        self.events.append((t, text))

    def col(self, name):
        return np.array([r[name] for r in self.rows])


class LandingMission:
    def __init__(self, env, pad_id, seed=0, feedforward=True, gain_schedule=True,
                 rate_normalised=True, wind=True, vision=None,
                 model_derivative=False, anti_windup=True, tracker="ct"):
        self.env = env
        self.env.arm()                # a divert after touchdown must re-arm
        self.pad = env.pad_by_id(pad_id)
        self.vision = vision if vision is not None else PadVision()
        self.beacon = AirTagBeacon(seed=seed)
        self.guide = Guidance(feedforward=feedforward, gain_schedule=gain_schedule,
                              rate_normalised=rate_normalised,
                              model_derivative=model_derivative,
                              anti_windup=anti_windup, tracker=tracker)
        self.inner = InnerLoop()
        self.wind = wind

        self.state = "TAKEOFF"
        # The environment clock keeps running across diverts, so mission time is
        # measured from when THIS mission started. Comparing the global clock
        # against the timeout aborts every mission after the first, even one
        # that is centimetres from touching down.
        self.t0 = float(env.t)
        self.log = MissionLog()
        self.aligned_for = 0.0
        self.go_arounds = 0           # bounded: see C.MAX_GO_AROUNDS
        self.since_vision = 99.0
        self.search_t = 0.0
        self.last_fix = None          # last known pad position, any source
        self.held_offset = None       # last vision offset, dead-reckoned
        self.sighting = None
        self.frame = None
        self.switched = False
        self.result = None
        self._cam_acc = 1.0
        self._vsp = np.zeros(3)       # low-passed velocity setpoint
        # Search altitude is relative to the surface the pad sits on. A pad on a
        # fifth-floor roof is approached from a height above THAT roof; using an
        # altitude above the street would put the drone only a metre above it.
        self.cruise_alt = self.pad.deck_height + C.CRUISE_ALT

    # ------------------------------------------------------------ helpers
    def _climb_back(self, h, rate):
        """
        A recovery climb, capped at the search altitude.

        Every climb in this state machine goes through here. The search altitude
        is already the height at which the large marker decodes, so climbing
        past it buys nothing - and an uncapped recovery climb ratchets the drone
        upwards a little on every attempt, which is how a configuration that
        keeps losing alignment ends up fifteen metres up, still nominally
        "descending", until the mission times out.
        """
        return min(rate, 1.1 * (self.cruise_alt - h))

    def _set(self, s):
        if s != self.state:
            self.log.event(self.env.t, "%s -> %s" % (self.state, s))
            self.state = s

    def _sense(self, dt):
        """Run the camera at CAM_HZ; everything else updates every step."""
        st = self.env.state()
        self.guide.predict(dt)

        self._cam_acc += dt
        fresh = False
        if self._cam_acc >= 1.0 / C.CAM_HZ:
            self._cam_acc = 0.0
            self.frame = self.env.camera_frame()
            att = self.env.imu()
            sightings = self.vision.look(self.frame, att, only_pad=self.pad.id)
            self.sighting = PadVision.pick(sightings)
            fresh = self.sighting is not None

        if fresh:
            self.since_vision = 0.0
            self.held_offset = self.sighting.offset.copy()
            self.guide.observe(st["pos"][:2] + self.sighting.offset)
            self.last_fix = st["pos"][:2] + self.sighting.offset
            if not self.sighting.is_large and not self.switched:
                self.switched = True
                self.log.event(self.env.t,
                               "tag hand-over %.2f m -> %.2f m at h=%.2f m"
                               % (C.TAG_LARGE, C.TAG_SMALL, self.env.altitude))
        else:
            self.since_vision += dt
            if self.held_offset is not None:
                # dead-reckon: the pad keeps moving and so does the drone
                self.held_offset = (self.held_offset
                                    + (self.guide.pad_velocity - st["vel"][:2]) * dt)
            fix = self.beacon.sense(st["pos"][:2], self.pad, dt)
            if fix is not None and self.since_vision > C.LOST_TIMEOUT:
                self.last_fix = np.asarray(fix, float)

        return st, (self.since_vision < 1e-6)

    # --------------------------------------------------------------- step
    def step(self):
        dt = C.DT_CTRL
        st, fresh = self._sense(dt)
        h = float(st["pos"][2])
        # Height above the surface THIS pad sits on. Every gate below is a
        # question about how far the drone still has to fall and how much
        # ground the camera can see, and both are set by the height above the
        # deck - not above the street. Using the absolute altitude leaves a
        # rooftop approach permanently in its "high and far away" regime: the
        # cone stays metres wide, the guidance keeps its slow cruise gain and
        # the descent never slows down, so the drone arrives at the parapet
        # fast and off-centre, loses the marker under its own nose and climbs
        # away again. That is the go-around loop, and it is only ever seen on
        # the roof addresses.
        deck = self.pad.deck_height
        agl = h - deck
        have_vision = self.since_vision < C.LOST_TIMEOUT and self.held_offset is not None
        live_vision = self.since_vision < 1.5 / C.CAM_HZ

        vel_sp = np.zeros(3)
        offset = self.held_offset if self.held_offset is not None else np.zeros(2)
        err = float(np.linalg.norm(offset))

        # ---------------------------------------------------------- TAKEOFF
        if self.state == "TAKEOFF":
            vel_sp[2] = np.clip(1.1 * (self.cruise_alt - h), -C.V_MAX_VERT, C.V_MAX_VERT)
            if h > self.cruise_alt - 0.4:
                self._set("TRANSIT")

        # ---------------------------------------------------------- TRANSIT
        elif self.state == "TRANSIT":
            vel_sp[2] = np.clip(1.1 * (self.cruise_alt - h), -C.V_MAX_VERT, C.V_MAX_VERT)
            if live_vision:
                self.guide.reset()
                self.guide.observe(st["pos"][:2] + offset)
                self._set("ALIGN")
            elif self.last_fix is not None:
                e = self.last_fix - st["pos"][:2]
                v = 0.8 * e
                n = np.linalg.norm(v)
                if n > C.V_MAX_CRUISE:
                    v *= C.V_MAX_CRUISE / n
                vel_sp[:2] = v
                if np.linalg.norm(e) < 1.2:
                    self.search_t = 0.0
                    self._set("SEARCH")

        # ----------------------------------------------------------- SEARCH
        elif self.state == "SEARCH":
            vel_sp[2] = np.clip(1.1 * (self.cruise_alt - h), -C.V_MAX_VERT, C.V_MAX_VERT)
            self.search_t += dt
            if live_vision:
                self.guide.reset()
                self.guide.observe(st["pos"][:2] + offset)
                self._set("ALIGN")
            else:
                w, r = 0.8, 0.5 + 0.30 * self.search_t
                centre = self.last_fix if self.last_fix is not None else st["pos"][:2]
                want = centre + r * np.array([math.cos(w * self.search_t),
                                              math.sin(w * self.search_t)])
                vel_sp[:2] = np.clip(1.1 * (want - st["pos"][:2]), -2.5, 2.5)
                if self.search_t > 30.0:
                    self._set("TRANSIT")

        # ------------------------------------------------------------ ALIGN
        elif self.state == "ALIGN":
            vel_sp[2] = np.clip(1.1 * (self.cruise_alt - h), -C.V_MAX_VERT, C.V_MAX_VERT)
            if not have_vision:
                self._set("TRANSIT")
            else:
                vel_sp[:2] = self.guide.velocity_setpoint(offset, dt, agl,
                                                     drone_vel=st["vel"])
                self.aligned_for = self.aligned_for + dt if err < C.ALIGN_TOL else 0.0
                if self.aligned_for > C.ALIGN_HOLD:
                    self._set("DESCEND")

        # ---------------------------------------------------------- DESCEND
        elif self.state == "DESCEND":
            tol = C.commit_tolerance(np.linalg.norm(self.guide.pad_velocity))
            # DECISION HEIGHT. A downward camera cannot see its own pad all
            # the way to the ground: the marker grows in the frame until it no
            # longer fits, and blind_altitude(err) is the height at which that
            # happens for the offset the drone is carrying. Below it an empty
            # frame is the geometry working as designed, so the drone lands on
            # the solution it already holds, dead-reckoned the rest of the way.
            #
            # Reading that empty frame as a lost target is what produced the
            # go-around loop: climb eight metres, re-acquire, fly back down,
            # meet the identical geometry, repeat until the mission times out.
            # Committing rather than going around was measured, not assumed -
            # allowing up to two go-arounds from the decision height gave
            # 12.3 cm mean error against 10.7 cm, and took a third longer,
            # because the second approach meets the same geometry as the first
            # while the pad has moved on meanwhile. (Both numbers are from the
            # build of that day, before the integral windup was found; the
            # comparison holds, the absolutes are now about three times better.)
            #
            # The trigger is the marker ACTUALLY going away, held for a few
            # frames - never a prediction that it is about to. The geometry
            # only answers the second question: was this loss expected here?
            # Below the decision height it was; above it the loss means
            # something else, and the re-acquire below handles it.
            lost_low = (self.since_vision > C.COMMIT_HOLD
                        and agl <= C.blind_altitude(err)
                        and self.held_offset is not None)
            if lost_low:
                self._set("FINAL")
            elif not have_vision:
                # Losing the marker higher up is a real re-acquire, but it must
                # not be unbounded: after MAX_GO_AROUNDS attempts a drone that
                # is low and holds a recent solution lands on it.
                self.go_arounds += 1
                if (self.go_arounds > C.MAX_GO_AROUNDS and agl < 1.0
                        and self.held_offset is not None):
                    self.log.event(self.env.t,
                                   "committing after %d re-acquire attempts"
                                   % self.go_arounds)
                    self._set("FINAL")
                else:
                    vel_sp[2] = self._climb_back(h, 0.6)
                    self._set("ALIGN")
            else:
                vel_sp[:2] = self.guide.velocity_setpoint(offset, dt, agl,
                                                     drone_vel=st["vel"])
                allowed = max(tol, C.CONE_SLOPE * agl)
                if not live_vision:
                    vel_sp[2] = 0.0                  # never descend blind
                elif err > C.DESCEND_ABORT:
                    vel_sp[2] = self._climb_back(h, 0.5)
                    self._note_pause()
                elif err > allowed:
                    vel_sp[2] = 0.0                  # outside the cone: hold
                else:
                    q = max(0.0, 1.0 - err / allowed)
                    vel_sp[2] = -np.clip(0.25 + 0.75 * q, 0.0, C.V_MAX_VERT)
                    if agl < 1.2:
                        vel_sp[2] = max(vel_sp[2], -0.45)
                if live_vision and agl < C.FINAL_ALT and err < tol:
                    self._set("FINAL")

        # ------------------------------------------------------------ FINAL
        elif self.state == "FINAL":
            vel_sp[:2] = self.guide.velocity_setpoint(offset, dt, agl,
                                                     drone_vel=st["vel"])
            vel_sp[2] = -C.FINAL_SINK
            # A go-around is only meaningful while the marker is still usable,
            # and only while the drone has any left to spend.
            if (agl > C.blind_altitude(err)
                    and self.go_arounds <= C.MAX_GO_AROUNDS
                    and err > 3 * C.commit_tolerance(
                        np.linalg.norm(self.guide.pad_velocity))):
                self._set("DESCEND")
            if self.env.landed_on(self.pad):
                self.env.cut_motors()
                self._set("LANDED")
                self._finish("LANDED")

        # ----------------------------------------------------- time guard
        if (self.state not in ("LANDED", "ABORTED")
                and self.env.t - self.t0 > C.MAX_MISSION_TIME):
            self._set("ABORTED")
            self._finish("ABORTED")

        a = dt / (C.VSP_FILT_TAU + dt)
        self._vsp += a * (vel_sp - self._vsp)
        thrust, torque, att_sp = self.inner(st, self._vsp)
        self.env.step(thrust, torque, wind=self.wind)

        true_off = self.pad.pos - st["pos"][:2]
        self.log.add(
            t=self.env.t, x=st["pos"][0], y=st["pos"][1], z=st["pos"][2],
            vx=st["vel"][0], vy=st["vel"][1], vz=st["vel"][2],
            roll=st["euler"][0], pitch=st["euler"][1], yaw=st["euler"][2],
            roll_sp=att_sp[0], pitch_sp=att_sp[1], thrust=thrust,
            pad_x=self.pad.pos[0], pad_y=self.pad.pos[1], pad_speed=self.pad.speed,
            ex=true_off[0], ey=true_off[1], err=float(np.linalg.norm(true_off)),
            mex=(offset[0] if have_vision else np.nan),
            mey=(offset[1] if have_vision else np.nan),
            seen=1.0 if live_vision else 0.0,
            tag=(0 if self.sighting is None or not live_vision
                 else (1 if self.sighting.is_large else 2)),
            vsp_x=vel_sp[0], vsp_y=vel_sp[1], vsp_z=vel_sp[2],
            state=self.state)
        return self.state

    def _note_pause(self):
        if not self.log.events or "descent paused" not in self.log.events[-1][1]:
            self.log.event(self.env.t, "descent paused: offset outside %.2f m"
                           % C.DESCEND_ABORT)

    def _finish(self, outcome):
        st = self.env.state()
        off = self.pad.pos - st["pos"][:2]
        self.result = {
            "outcome": outcome,
            "pad_id": self.pad.id,
            "address": self.pad.address,
            "moving": self.pad.moving,
            "time_s": round(float(self.env.t - self.t0), 2),
            "error_m": float(np.linalg.norm(off)),
            "error_x": float(off[0]),
            "error_y": float(off[1]),
            "touchdown_vz": float(st["vel"][2]),
            "detect_rate": self.vision.hits / max(self.vision.frames, 1),
            "handover": self.switched,
        }
        self.log.summary = self.result

    def run(self, verbose=False):
        while self.state not in ("LANDED", "ABORTED"):
            self.step()
            if verbose and len(self.log.rows) % 200 == 0:
                r = self.log.rows[-1]
                print("   t=%6.1f %-8s h=%5.2f err=%5.2f tilt=%4.1f deg"
                      % (r["t"], r["state"], r["z"], r["err"],
                         math.degrees(math.hypot(r["roll"], r["pitch"]))))
        return self.log
