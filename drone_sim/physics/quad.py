"""
The MuJoCo environment: the quadrotor, the moving pads, the sensors.

The important difference from a "floating brick" simulator is where the forces
are applied. Thrust acts along the drone's OWN z-axis through a body site, and
the only lateral authority is the horizontal component of that thrust. To move
sideways the vehicle has to tilt, and when it tilts the downward camera tilts
with it - which is the coupling that makes vision-based landing hard and which a
world-frame-force model quietly deletes.
"""
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config as C
from control import quaternion as Q
from world.build_world import ensure_world

WORLD_XML = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                         "world", "assets", "world.xml")


# ------------------------------------------------------------------ pads
class Pad:
    """One addressed landing pad and the way it moves."""

    def __init__(self, spec):
        self.id = spec["id"]
        self.address = spec["address"]
        self.motion = spec["motion"]
        self.kind = spec.get("kind", "ground")
        self.height = C.pad_height(spec)      # height of the surface it sits on
        self.home = np.array(spec["home"], float)
        self.p = spec
        self.pos = self.home.copy()
        self.vel = np.zeros(2)
        self.yaw = 0.0
        self._t = 0.0

    @property
    def moving(self):
        return self.motion != "static"

    @property
    def deck_height(self):
        """
        Height of the landing surface above the road, m.

        Three cases: the street itself, a vehicle deck, or a roof at whatever
        height the address book gives it. Everything downstream - the cruise
        altitude, the commit height, the touchdown test - is expressed relative
        to THIS, not to the ground, so a rooftop landing is the same manoeuvre
        as a street landing performed higher up.
        """
        if self.moving:
            return self.height + 0.34
        return self.height + 0.022

    def reset(self):
        self._t = 0.0
        self.pos = self.home.copy()
        self.vel = np.zeros(2)
        self.yaw = 0.0

    def step(self, dt):
        self._t += dt
        t = self._t
        if self.motion == "static":
            new = self.home.copy()
        elif self.motion == "line":
            spd, hdg, half = self.p["speed"], self.p["heading"], self.p["half"]
            s = half * np.sin(spd * t / max(half, 1e-6))
            new = self.home + s * np.array([np.cos(hdg), np.sin(hdg)])
        elif self.motion == "circle":
            r, w = self.p["radius"], self.p["omega"]
            new = self.home + r * np.array([np.cos(w * t) - 1.0, np.sin(w * t)])
        elif self.motion == "drift":
            a, w = self.p["amp"], self.p["omega"]
            new = self.home + a * np.array([np.sin(w * t), np.sin(0.7 * w * t + 1.1)])
        else:
            raise ValueError(self.motion)

        self.vel = (new - self.pos) / dt if dt > 0 else np.zeros(2)
        self.pos = new
        if np.linalg.norm(self.vel) > 0.05:      # vehicles point where they go
            self.yaw = float(np.arctan2(self.vel[1], self.vel[0]))

    @property
    def speed(self):
        return float(np.linalg.norm(self.vel))


def build_pads():
    return [Pad(s) for s in C.PADS]


# ------------------------------------------------------------ quaternions
def quat_to_rot(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quat_to_euler(q):
    w, x, y, z = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


# ------------------------------------------------------------------- env
class QuadEnv:
    def __init__(self, render_camera=True, seed=0):
        # The world is generated from config.py, so it must be regenerated
        # whenever config.py changes - otherwise the pad the camera sees and
        # the pad the mission is aiming at are in different places, and the
        # drone lands beautifully on the wrong one.
        if ensure_world():
            print("  world regenerated from config.py")
        self.model = mujoco.MjModel.from_xml_path(WORLD_XML)
        self.data = mujoco.MjData(self.model)
        self.rng = np.random.default_rng(seed)

        jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "drone_free")
        self.qadr = self.model.jnt_qposadr[jid]
        self.vadr = self.model.jnt_dofadr[jid]
        self.drone_bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "drone")

        self.act = {n: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
                    for n in ("thrust", "torque_x", "torque_y", "torque_z")}
        self.sens = {n: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, n)
                     for n in ("imu_quat", "imu_gyro", "imu_acc", "alt_range")}

        self.pads = build_pads()
        self._mocap = {}
        for p in self.pads:
            if p.moving:
                bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "veh_%d" % p.id)
                self._mocap[p.id] = int(self.model.body_mocapid[bid])

        self.renderer = None
        self._scene_opt = None
        if render_camera:
            self.renew_renderer()

        self._thrust = C.HOVER_THRUST
        self._torque = np.zeros(3)
        self._wind = np.zeros(2)
        self.armed = True
        self.t = 0.0
        self.reset()

    # ------------------------------------------------------------- reset
    def reset(self, start_xy=None, start_alt=0.28, seed=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        mujoco.mj_resetData(self.model, self.data)
        x, y = start_xy if start_xy is not None else C.HOME_XY
        self.data.qpos[self.qadr:self.qadr + 3] = [x, y, start_alt]
        self.data.qpos[self.qadr + 3:self.qadr + 7] = [1, 0, 0, 0]
        self.data.qvel[:] = 0.0
        for p in self.pads:
            p.reset()
        self._sync_pads()
        self._thrust = C.HOVER_THRUST
        self._torque = np.zeros(3)
        self._wind = np.zeros(2)
        self.armed = True
        self.t = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _sync_pads(self):
        for p in self.pads:
            if not p.moving:
                continue
            mid = self._mocap[p.id]
            self.data.mocap_pos[mid] = [p.pos[0], p.pos[1], 0.0]
            c, s = np.cos(p.yaw / 2), np.sin(p.yaw / 2)
            self.data.mocap_quat[mid] = [c, 0.0, 0.0, s]

    # -------------------------------------------------------------- step
    def step(self, thrust_cmd, torque_cmd, wind=True):
        """One control step: motor lag, wind, drag, physics, moving pads."""
        a = C.DT_CTRL / (C.MOTOR_TAU + C.DT_CTRL)
        if not self.armed:
            thrust_cmd, torque_cmd = 0.0, np.zeros(3)
        self._thrust += a * (float(np.clip(thrust_cmd, 0.0, C.MAX_THRUST)) - self._thrust)
        self._torque += a * (np.clip(np.asarray(torque_cmd, float), -6, 6) - self._torque)

        self.data.ctrl[self.act["thrust"]] = self._thrust
        self.data.ctrl[self.act["torque_x"]] = self._torque[0]
        self.data.ctrl[self.act["torque_y"]] = self._torque[1]
        self.data.ctrl[self.act["torque_z"]] = self._torque[2]

        # Ornstein-Uhlenbeck gusts around a steady breeze, turned into a drag
        # force on the airframe. Wind matters here: the only way to hold station
        # against it is to tilt, and tilting moves the tag in the image.
        if wind:
            k = C.DT_CTRL / C.WIND_TAU
            self._wind += -k * self._wind + self.rng.normal(0, C.WIND_SIGMA * np.sqrt(2 * k), 2)
            air = np.array(C.WIND_MEAN) + self._wind
        else:
            air = np.zeros(2)
        vel = self.data.qvel[self.vadr:self.vadr + 3].copy()
        drag = np.zeros(6)
        drag[:2] = -C.DRAG_COEF * (vel[:2] - air)
        drag[2] = -C.DRAG_COEF * vel[2]
        self.data.xfrc_applied[self.drone_bid, :3] = drag[:3]

        for _ in range(C.STEPS_PER_CTRL):
            mujoco.mj_step(self.model, self.data)

        for p in self.pads:
            p.step(C.DT_CTRL)
        self._sync_pads()
        self.t += C.DT_CTRL

    def cut_motors(self):
        self.armed = False

    def arm(self):
        """
        Re-enable the motors.

        Touchdown cuts them, and a reset turns them back on - but a DIVERT does
        neither: the drone is retasked in place, keeping its position and the
        moving pads' phase. Without re-arming here, a drone that has landed sits
        on the pad with its motors off while the new mission runs its state
        machine to a timeout.
        """
        self.armed = True

    # ------------------------------------------------------------- state
    def state(self):
        q = self.data.qpos[self.qadr + 3:self.qadr + 7].copy()
        return {
            "pos": self.data.qpos[self.qadr:self.qadr + 3].copy(),
            "vel": self.data.qvel[self.vadr:self.vadr + 3].copy(),
            "omega": self.data.qvel[self.vadr + 3:self.vadr + 6].copy(),
            "quat": q,
            "euler": quat_to_euler(q),      # logging and display only
            "R": quat_to_rot(q),
            "t": self.t,
        }

    @property
    def altitude(self):
        return float(self.data.qpos[self.qadr + 2])

    def landed_on(self, pad):
        """True when the drone is resting on that pad's deck."""
        st = self.state()
        return (abs(st["pos"][2] - (pad.deck_height + 0.21)) < 0.09
                and abs(st["vel"][2]) < 0.25)

    # ----------------------------------------------------------- sensors
    def imu(self):
        """
        Attitude estimate as a QUATERNION: truth perturbed by a small rotation.

        The error is applied as a rotation, not as noise added to three Euler
        angles. Adding noise in Euler space is not a rotation of the true
        attitude - near the pitch singularity it is not even close, and it
        cannot be composed. Perturbing by a small quaternion is the correct
        model of an AHRS error and keeps the estimate a valid orientation.
        """
        q_true = self.data.qpos[self.qadr + 3:self.qadr + 7].copy()
        dq = Q.from_rotvec(self.rng.normal(0.0, C.ATT_SIGMA, 3))
        return Q.normalize(Q.mul(q_true, dq))

    def rangefinder(self):
        """Downward range, metres. Negative means out of range."""
        sid = self.sens["alt_range"]
        adr = self.model.sensor_adr[sid]
        raw = float(self.data.sensordata[adr])
        if raw < 0 or raw > C.RANGE_MAX:
            return None
        return raw + float(self.rng.normal(0, C.RANGE_SIGMA))

    def renew_renderer(self):
        """
        (Re)build the onboard-camera renderer.

        MuJoCo's Renderer owns a GL context and frees it when it is collected,
        and that free takes THIS renderer's context down with it. A figure that
        opens a second renderer for a wide shot therefore leaves the onboard
        camera rendering pure black once it goes out of scope - with no error
        anywhere, because a black frame is a perfectly valid image that simply
        contains no marker. Anything that makes its own renderer calls this
        afterwards.
        """
        if self.renderer is not None:
            try:
                self.renderer.close()
            except Exception:
                pass
            self.renderer = None
        self.renderer = mujoco.Renderer(self.model, C.IMG_H, C.IMG_W)
        # Shadows cost far more than they inform: the onboard camera only has
        # to see a flat marker, and turning them off makes a frame about ten
        # times cheaper, which is what makes Monte Carlo runs practical.
        self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
        self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0

    def camera_frame(self):
        if self.renderer is None:
            return None
        self.renderer.update_scene(self.data, camera="onboard_cam")
        self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
        self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
        return self.renderer.render()

    def pad_by_id(self, pad_id):
        for p in self.pads:
            if p.id == pad_id:
                return p
        raise KeyError(pad_id)
