"""
Guidance: the velocity PID of the base paper, plus the two additions its own
results ask for.

Paper eq. (9):   V = kp*err + ki*integral + kd*(err - last_err)
with kp = 0.20, ki = 0.03, kd = 0.35.

Two things that PID alone cannot do, and that this file adds on top of it:

1. FEED-FORWARD. A moving pad is a ramp input, and a PID chasing a ramp always
   keeps a steady-state lag - which is exactly why Fig.16 of the paper never
   settles on zero but sits inside (-0.2 m, +0.5 m). Feeding the estimated pad
   velocity forward cancels that lag. The estimate comes from an alpha-beta
   tracker, not from raw differencing: vision noise is centimetres, and
   differentiating it at 30 Hz produces metres per second of nonsense.

2. GAIN SCHEDULING. kp = 0.20 means a five-second closed-loop time constant.
   That is safe at six metres and far too slow at half a metre, where the pad
   can leave the camera footprint in under a second. The gains are therefore
   scaled up as the drone descends.

Both are switchable, so results.py can plot the paper's configuration and ours
side by side.
"""
import numpy as np

import config as C


class PID:
    """
    One-axis PID.

    Two deliberate departures from eq. (9) as it is printed in the paper:

    * The paper writes the derivative as kd*(err - last_err), with no division
      by the sample period. That makes the derivative action depend on the loop
      rate - the same kd behaves completely differently at 10 Hz and at 50 Hz,
      and at our 50 Hz it does almost nothing, so the loop overshoots as badly
      as it does with no D term at all. We use the rate-normalised form
      kd*d(err)/dt, which is what makes kp = 0.20 / kd = 0.35 mean a derivative
      time of 1.75 s regardless of how fast the loop runs.
    * The derivative is low-pass filtered (D_FILT_TAU). Differentiating a
      centimetre-noisy vision measurement raw would inject metres per second of
      command noise.

    Set rate_normalised=False to get the literal printed form back - results.py
    uses it to show what the difference costs.
    """

    def __init__(self, kp=C.KP, ki=C.KI, kd=C.KD, i_clamp=C.I_CLAMP,
                 rate_normalised=True):
        self.kp, self.ki, self.kd, self.i_clamp = kp, ki, kd, i_clamp
        self.rate_normalised = rate_normalised
        self.integral = 0.0
        self.last_err = 0.0
        self.d_state = 0.0
        self.started = False

    def reset(self):
        self.integral = 0.0
        self.last_err = 0.0
        self.d_state = 0.0
        self.started = False

    def step(self, err, dt, gain=1.0):
        if not self.started:               # no derivative kick on lock-on
            self.last_err = err
            self.started = True
        # Conditional integration: the integral exists to trim out a steady
        # wind once the drone is over the pad. Letting it charge during a 20 m
        # approach only buys overshoot, so it is frozen outside a narrow band.
        if abs(err) < C.I_BAND:
            self.integral = float(np.clip(self.integral + err * dt,
                                          -self.i_clamp, self.i_clamp))
        raw = (err - self.last_err)
        if self.rate_normalised:
            raw = raw / max(dt, 1e-6)
            a = dt / (C.D_FILT_TAU + dt)   # first-order filter on the derivative
            self.d_state += a * (raw - self.d_state)
        else:
            self.d_state = raw
        self.last_err = err
        return gain * (self.kp * err + self.ki * self.integral
                       + self.kd * self.d_state)


class AlphaBeta:
    """
    Constant-velocity alpha-beta tracker for the pad.

    Predicts every control step, corrects only when the camera actually decodes
    a new frame. That distinction matters: re-feeding a held measurement at
    50 Hz would drag the velocity estimate towards zero.
    """

    def __init__(self, alpha=C.AB_ALPHA, beta=C.AB_BETA):
        self.alpha, self.beta = alpha, beta
        self.p = None
        self.v = np.zeros(2)
        self.since_update = 0.0

    def reset(self):
        self.p = None
        self.v = np.zeros(2)
        self.since_update = 0.0

    def predict(self, dt):
        if self.p is not None:
            self.p = self.p + self.v * dt
        self.since_update += dt
        return self.p, self.v

    def update(self, meas):
        meas = np.asarray(meas, float)
        if self.p is None:
            self.p, self.v = meas.copy(), np.zeros(2)
            self.since_update = 0.0
            return
        dt = max(self.since_update, 1e-3)
        resid = meas - self.p
        self.p = self.p + self.alpha * resid
        self.v = self.v + (self.beta / dt) * resid
        self.v = np.clip(self.v, -4.0, 4.0)
        self.since_update = 0.0


def altitude_gain(h):
    """
    Gain multiplier as a function of altitude.

    1.0 at cruise (the paper's tuning), rising to GAIN_LOW near the ground where
    the camera footprint is only tens of centimetres wide.
    """
    hi, lo = 4.0, 0.5
    if h >= hi:
        return 1.0
    if h <= lo:
        return C.GAIN_LOW
    f = (hi - h) / (hi - lo)
    return 1.0 + f * (C.GAIN_LOW - 1.0)


class TargetTracker:
    """
    Horizontal guidance: two PIDs on the measured offset, an alpha-beta estimate
    of the pad velocity, and the altitude gain schedule.
    """

    def __init__(self, feedforward=True, gain_schedule=True):
        self.px = PID()
        self.py = PID()
        self.feedforward = feedforward
        self.gain_schedule = gain_schedule
        self.filt = AlphaBeta()

    def reset(self):
        self.px.reset(); self.py.reset(); self.filt.reset()

    # ------------------------------------------------------- pad velocity
    def predict(self, dt):
        self.filt.predict(dt)

    def observe(self, tag_world_pos):
        """Call once per *fresh* vision fix of the pad, in world coordinates."""
        self.filt.update(tag_world_pos)

    @property
    def target_velocity(self):
        return self.filt.v.copy()

    # ------------------------------------------------------------ command
    def step(self, err_xy, dt, altitude=99.0, v_max=C.V_MAX_TRACK):
        g = altitude_gain(altitude) if self.gain_schedule else 1.0
        v = np.array([self.px.step(float(err_xy[0]), dt, g),
                      self.py.step(float(err_xy[1]), dt, g)])
        if self.feedforward:
            v = v + self.filt.v
        n = np.linalg.norm(v)
        if n > v_max:
            v = v * (v_max / n)
        return v


def altitude_controller(target_alt, current_alt, v_max=C.V_MAX_VERT):
    """Plain P loop on height - the paper gates descent by thresholds, not gains."""
    return float(np.clip(C.KP_Z * (target_alt - current_alt), -v_max, v_max))
