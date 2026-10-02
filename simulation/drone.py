"""
Quadrotor model (DJI M100 class, velocity-API level).

The base paper does not control motors directly: it calls the M100 velocity API,
and Fig.10 of the paper shows that this inner loop answers a step command with
overshoot and lag. We therefore model the airframe as a second-order velocity
servo per axis - that is exactly the plant the paper's PID is placed in front of,
so turning the PID off in this simulator reproduces the paper's Fig.10 and
turning it on reproduces Fig.12.
"""
import numpy as np

import config as C


class Drone:
    def __init__(self, pos=(0.0, 0.0, 0.0), seed=0):
        self.p = np.array(pos, float)      # world ENU position, m
        self.v = np.zeros(3)               # achieved velocity, m/s
        self.a = np.zeros(3)               # velocity rate (servo state), m/s^2
        self.yaw = 0.0                     # heading, rad (headless/NED API mode)
        self.rng = np.random.default_rng(seed)
        self.flight_time = 0.0
        self.armed = True

    # ------------------------------------------------------------------ plant
    def step(self, v_cmd, dt, wind=True):
        """Apply a body-frame velocity command for one step."""
        v_cmd = np.clip(np.asarray(v_cmd, float),
                        [-C.V_MAX_CRUISE, -C.V_MAX_CRUISE, -C.V_MAX_VERT],
                        [C.V_MAX_CRUISE, C.V_MAX_CRUISE, C.V_MAX_VERT])

        if not self.armed:
            v_cmd = np.zeros(3)

        # second-order velocity servo: wn sets the lag, zeta sets the overshoot
        wn = 1.0 / C.ACTUATOR_TAU
        self.a += (wn * wn * (v_cmd - self.v) - 2 * C.ACTUATOR_ZETA * wn * self.a) * dt
        self.v += self.a * dt

        disturb = np.zeros(3)
        if wind and self.p[2] > 0.05:
            gust = self.rng.normal(0.0, C.WIND_GUST, 2)
            disturb[:2] = np.array(C.WIND_MEAN) + gust
            disturb[:2] *= 0.35            # the airframe only partly follows wind

        self.p += (self.v + disturb) * dt
        self.p[2] = max(self.p[2], 0.0)
        if self.p[2] <= 0.0:
            self.v[2] = max(self.v[2], 0.0)
        self.flight_time += dt
        return self.p.copy()

    def cut_motors(self):
        self.armed = False

    @property
    def altitude(self):
        return float(self.p[2])

    @property
    def battery_left(self):
        return max(0.0, 1.0 - self.flight_time / C.BATTERY_S)


# ------------------------------------------------ paper section 3.2 transforms
def camera_to_body(v_cam):
    """
    Camera frame -> UAV body frame.

    Fig.9 of the paper: the downward camera has its optical centre as origin and
    its y-axis pointing opposite to the UAV body y-axis, with the optical axis
    (camera z) pointing down along the body -z. This is the one transform that
    turns a pixel-derived offset into something the flight API can accept.
    """
    x, y, z = v_cam
    return np.array([x, -y, -z])


def body_to_world(v_body, yaw):
    """Body (headless/NED-style) -> world ENU, using the current heading."""
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s, 0.0],
                  [s,  c, 0.0],
                  [0.0, 0.0, 1.0]])
    return R @ np.asarray(v_body, float)
