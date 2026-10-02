"""
Outer guidance: vision offset -> velocity setpoint.

The core is eq. (9) of the base paper at its published gains,

    V = kp*err + ki*integral + kd*(err - last_err),   kp=0.20 ki=0.03 kd=0.35

with five changes, each of which is switchable so the evaluation can show what
each one buys. A sixth - taking the derivative from the state estimate instead
of from the measurement - is implemented and switchable but NOT enabled: see the
note at the end.

  RATE-NORMALISED, FILTERED DERIVATIVE
      As printed, the derivative has no division by the sample period, so the
      same kd behaves completely differently at 10 Hz and at 100 Hz. Here the
      loop runs at 100 Hz, where the printed form is nearly inert. The
      rate-normalised form is used instead, low-pass filtered so that
      centimetre-level vision noise is not differentiated into metres per
      second of command.

  CONDITIONAL INTEGRATION, WITH A LEAK
      The integral exists to trim a steady wind while hovering over the pad.
      Allowed to charge during a twenty-metre transit it only buys overshoot,
      so it is frozen outside a narrow error band - and it bleeds out whenever
      it is pulling against the current error, because a charge accumulated
      while closing from one side is history, not trim. Nor is it scaled by the
      altitude schedule: that schedule speeds up the response as the camera's
      view of the ground shrinks, and rescaling a converged trim by three at
      touchdown height makes it the largest term in the loop.

  PAD-VELOCITY FEED-FORWARD
      A PID chasing a moving pad is a PID chasing a ramp, and it always keeps a
      steady-state lag - which is exactly the (-0.2 m, +0.5 m) band the paper
      reports as its tracking result. The pad velocity is estimated by an
      alpha-beta-gamma tracker and added to the PID output, cancelling the lag
      rather than tolerating it. The acceleration term is what makes the
      circling pads landable; see TargetFilter.

  ALTITUDE GAIN SCHEDULING
      kp = 0.20 is a five-second closed-loop time constant. That is right at
      eight metres and far too slow at half a metre, where the camera sees
      23 cm of ground and a moving pad leaves the frame in a fraction of a
      second.

MEASURED AND NOT ADOPTED: the model-based derivative.

    The offset is pad position minus drone position, so its derivative is
    exactly the pad velocity minus the drone velocity, and both are already
    estimated. Differencing the measurement instead differentiates the vision
    noise. That reasoning is sound, and while a saturated integral was
    dominating the loop the noise really was the largest term in it - but once
    the integral was fixed the noise stopped mattering, and the model-based
    form measured WORSE: 8.2 cm against 7.3 cm mean over twelve missions on the
    moving addresses, with a larger worst case. It is kept switchable
    (model_derivative=True) so the comparison can be re-run, and left off,
    because a term that cannot show a benefit does not belong in the loop.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config as C


class PID:
    def __init__(self, kp=C.KP, ki=C.KI, kd=C.KD, rate_normalised=True,
                 anti_windup=True):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.rate_normalised = rate_normalised
        self.anti_windup = anti_windup
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.last_err = 0.0
        self.d_state = 0.0
        self.started = False

    def step(self, err, dt, gain=1.0, deriv=None):
        """
        One PID update. `deriv`, when given, is the error's rate of change
        obtained from the state estimate instead of by differencing the
        measurement - see Guidance.velocity_setpoint.
        """
        if not self.started:
            self.last_err = err
            self.started = True
        if abs(err) < C.I_BAND:
            self.integral = float(np.clip(self.integral + err * dt,
                                          -C.I_CLAMP, C.I_CLAMP))
        if self.anti_windup and self.integral * err < 0.0:
            self.integral *= max(0.0, 1.0 - dt / C.I_LEAK_TAU)
        if deriv is not None:
            a = dt / (C.D_FILT_TAU + dt)
            self.d_state += a * (float(deriv) - self.d_state)
        else:
            raw = err - self.last_err
            if self.rate_normalised:
                raw /= max(dt, 1e-6)
                a = dt / (C.D_FILT_TAU + dt)
                self.d_state += a * (raw - self.d_state)
            else:
                self.d_state = raw
        self.last_err = err
        # The altitude schedule speeds up the RESPONSE as the camera's view of
        # the ground shrinks. It has no business rescaling the accumulated
        # trim, which is an estimate of a standing disturbance and is already
        # in the right units: scaled by three at touchdown height, a trim that
        # was correct at altitude becomes the largest term in the loop.
        if self.anti_windup:
            return (gain * (self.kp * err + self.kd * self.d_state)
                    + self.ki * self.integral)
        return gain * (self.kp * err + self.ki * self.integral
                       + self.kd * self.d_state)


class TargetFilter:
    """
    Tracker for the pad: alpha-beta (constant velocity) or alpha-beta-gamma
    (constant acceleration).

    Predicts every control step, corrects only on frames that actually decoded.
    That distinction matters: re-feeding a held measurement at 100 Hz would drag
    the velocity estimate towards zero, and re-feeding it as a fresh measurement
    would make the estimate jump.

    Why the gamma term earns its place: a pad on a vehicle driving a curve has a
    constantly turning velocity, and a constant-velocity filter lags it by a
    fixed amount no matter how long it watches. Measured in this simulator, that
    lag parks the drone about 0.2 m from the pad centre and it never becomes
    small enough to commit to touchdown - the drone circles above the pad until
    the mission times out. Estimating acceleration removes the lag; the estimate
    is low-pass filtered and clamped, because differentiating twice through
    centimetre-level vision noise is otherwise a good way to produce nonsense.
    """

    def __init__(self, alpha=C.AB_ALPHA, beta=C.AB_BETA, gamma=C.AB_GAMMA,
                 order="ct"):
        self.alpha, self.beta, self.gamma = alpha, beta, gamma
        self.order = order
        self.reset()

    def reset(self):
        self.p = None
        self.v = np.zeros(2)
        self.a = np.zeros(2)
        self.w = 0.0                 # estimated turn rate, rad/s
        self.since_update = 0.0

    def _turn_rate(self):
        """
        Turn rate implied by the velocity and acceleration already estimated.

        For ANY planar motion the part of the acceleration perpendicular to the
        velocity is exactly the turn:

            omega = (v x a)_z / |v|^2

        so no new measurement is needed - it falls out of the two states the
        filter already carries. Below a fifth of a metre per second the angle of
        a velocity vector is mostly noise, so the turn rate is taken as zero and
        the filter behaves as the constant-acceleration one.
        """
        s2 = float(self.v @ self.v)
        if s2 < C.OMEGA_MIN_SPEED ** 2:
            return 0.0
        cross = float(self.v[0] * self.a[1] - self.v[1] * self.a[0])
        w = cross / s2

        # ... but only to the extent that the acceleration really IS the turn.
        # |cross| / (|v||a|) is |sin| of the angle between them: 1 for a pad
        # driving a steady circle, 0 for one speeding up or slowing down in a
        # straight line. Without this the model is actively harmful on the pads
        # that shuttle back and forth: at the end of each run the vehicle stops
        # and reverses, |v| passes through zero, and a cross product divided by
        # |v|^2 reports a violent corner where there is only a straight stop.
        # Measured, that cost those two addresses 4.7 cm -> 13.7 cm.
        speed = np.sqrt(s2)
        a_mag = float(np.linalg.norm(self.a))
        if a_mag < 1e-6:
            return 0.0
        # A soft weight is not enough on its own: on a straight pad the TRUE
        # perpendicularity is zero, so everything measured there is noise in the
        # estimated acceleration, and noise times a small weight is still a turn
        # rate the filter will act on. Below the deadband no turn is believed at
        # all; above it the weight ramps to one, so a pad driving a real circle
        # (perpendicularity ~ 1) is tracked at full strength.
        perpendicularity = abs(cross) / (speed * a_mag)
        if perpendicularity < C.OMEGA_PERP_MIN:
            return 0.0
        ramp = (perpendicularity - C.OMEGA_PERP_MIN) / (1.0 - C.OMEGA_PERP_MIN)
        w *= min(1.0, ramp)
        return float(np.clip(w, -C.OMEGA_MAX, C.OMEGA_MAX))

    @staticmethod
    def _rot(v, ang):
        c, s = np.cos(ang), np.sin(ang)
        return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])

    def predict(self, dt):
        if self.p is not None:
            if self.order == "ct":
                self.w += (dt / (C.OMEGA_TAU + dt)) * (self._turn_rate() - self.w)
                ang = self.w * dt
                if abs(ang) < 1e-4:                  # straight: same as "abg"
                    self.p = self.p + self.v * dt + 0.5 * self.a * dt * dt
                    self.v = self.v + self.a * dt
                else:
                    # Exact integral of a velocity that rotates at w: the target
                    # travels along the ARC, not along the tangent. This is the
                    # whole difference - a constant-acceleration model can only
                    # extrapolate a straight line plus a fixed bend, and a pad
                    # driving a steady circle curves away from it continuously.
                    c, s = np.cos(ang), np.sin(ang)
                    vx, vy = self.v
                    self.p = self.p + np.array([(s * vx - (1.0 - c) * vy) / self.w,
                                                ((1.0 - c) * vx + s * vy) / self.w])
                    self.v = self._rot(self.v, ang)
                    self.a = self._rot(self.a, ang)   # centripetal turns with it
            elif self.order == "abg":
                self.p = self.p + self.v * dt + 0.5 * self.a * dt * dt
                self.v = self.v + self.a * dt
            else:
                self.p = self.p + self.v * dt
        self.since_update += dt

    def update(self, meas):
        meas = np.asarray(meas, float)
        if self.p is None:
            self.p = meas.copy()
            self.v = np.zeros(2)
            self.a = np.zeros(2)
            self.since_update = 0.0
            return
        dt = max(self.since_update, 1e-3)
        resid = meas - self.p
        self.p = self.p + self.alpha * resid
        self.v = np.clip(self.v + (self.beta / dt) * resid, -4.0, 4.0)
        if self.order in ("abg", "ct"):
            a_new = self.a + (2.0 * self.gamma / (dt * dt)) * resid
            self.a = np.clip(0.75 * self.a + 0.25 * a_new, -2.5, 2.5)
        self.since_update = 0.0

    @property
    def lead_velocity(self):
        """
        Velocity to feed forward.

        On a curving path the command is worth aiming slightly ahead: the loop
        itself has lag, and LEAD_TIME of lead cancels most of what remains. On a
        turn that lead is a ROTATION of the velocity, not an addition to it -
        adding acceleration for a quarter of a second points the drone at the
        tangent, which is off the outside of the corner.
        """
        if self.order == "ct" and abs(self.w) > 1e-4:
            return self._rot(self.v, self.w * C.LEAD_TIME)
        return self.v + self.a * C.LEAD_TIME


def altitude_gain(h):
    if h >= C.GAIN_HI_ALT:
        return 1.0
    if h <= C.GAIN_LO_ALT:
        return C.GAIN_LOW
    f = (C.GAIN_HI_ALT - h) / (C.GAIN_HI_ALT - C.GAIN_LO_ALT)
    return 1.0 + f * (C.GAIN_LOW - 1.0)


class Guidance:
    def __init__(self, feedforward=True, gain_schedule=True, rate_normalised=True,
                 tracker="ct", model_derivative=False, anti_windup=True):
        self.px = PID(rate_normalised=rate_normalised, anti_windup=anti_windup)
        self.py = PID(rate_normalised=rate_normalised, anti_windup=anti_windup)
        self.filt = TargetFilter(order=tracker)
        self.feedforward = feedforward
        self.gain_schedule = gain_schedule
        self.model_derivative = model_derivative

    def reset(self):
        self.px.reset(); self.py.reset(); self.filt.reset()

    def predict(self, dt):
        self.filt.predict(dt)

    def observe(self, pad_world_xy):
        self.filt.update(pad_world_xy)

    @property
    def pad_velocity(self):
        return self.filt.v.copy()

    @property
    def pad_lead_velocity(self):
        return self.filt.lead_velocity

    def velocity_setpoint(self, offset_xy, dt, altitude, v_max=C.V_MAX_TRACK,
                          drone_vel=None):
        """
        MODEL-BASED DERIVATIVE
            The offset is pad position minus drone position, so its rate of
            change is exactly the pad velocity minus the drone velocity - and
            both are already estimated, the first by the tracker and the second
            by the drone's own state. Differencing the measurement instead
            differentiates the vision noise: at 25 Hz, a centimetre of jitter
            becomes a quarter of a metre per second, and after the low-pass it
            is still larger than the whole proportional term for a ten
            centimetre offset. The measured loop therefore does not converge on
            a small offset at all, it random-walks around it, and the drone
            only centres itself by loitering until the noise averages out.
            Taking the derivative from the state estimate removes the noise at
            its source rather than filtering it afterwards.
        """
        g = altitude_gain(altitude) if self.gain_schedule else 1.0
        d = None
        if self.model_derivative and drone_vel is not None:
            d = self.filt.v - np.asarray(drone_vel, float)[:2]
        v = np.array([self.px.step(float(offset_xy[0]), dt, g,
                                   None if d is None else d[0]),
                      self.py.step(float(offset_xy[1]), dt, g,
                                   None if d is None else d[1])])
        if self.feedforward:
            v = v + self.filt.lead_velocity
        n = float(np.linalg.norm(v))
        if n > v_max:
            v *= v_max / n
        return v
