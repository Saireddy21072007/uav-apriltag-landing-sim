"""
Central configuration for the AirTag-addressed autonomous landing simulator.

Every number that a reviewer might question lives here, with the source of the
value in the comment: [PAPER] = taken from the base paper (Zhou Li et al.,
"UAV Autonomous Landing Technology Based on AprilTags Vision Positioning
Algorithm", 38th Chinese Control Conference, 2019), [DJI] = DJI M100 datasheet
class values, [MODEL] = our modelling assumption.
"""

# ----------------------------------------------------------------- simulation
DT = 0.02                 # [MODEL] integration step, s (50 Hz control loop)
MAX_MISSION_TIME = 240.0  # [MODEL] hard stop for one mission, s

# ----------------------------------------------------------------- the world
WORLD_SIZE = 60.0         # [MODEL] square arena side, m (x,y in +-30 m)
WIND_MEAN = (0.18, -0.12) # [MODEL] steady wind pushing the airframe, m/s
WIND_GUST = 0.25          # [MODEL] std-dev of gust noise, m/s

# ----------------------------------------------------------------- the drone
CRUISE_ALT = 6.0          # [MODEL] search altitude, m. Chosen so the 0.30 m
                          #         tag is decodable while searching (see vision.py)
TAKEOFF_ALT = 6.0
V_MAX_CRUISE = 5.0        # [DJI]  M100 horizontal speed cap used in API mode, m/s
V_MAX_TRACK = 3.5         # [MODEL] speed cap once vision is locked, m/s.
                          #         Must exceed the fastest pad (2.0 m/s) or the
                          #         drone can never close on a moving target.
V_MAX_VERT = 1.2          # [DJI]  vertical speed cap, m/s
ACTUATOR_TAU = 0.35       # [PAPER Fig.10] velocity-loop lag that causes the
                          #                overshoot the paper fixes with PID, s
ACTUATOR_ZETA = 0.55      # [PAPER Fig.10] damping -> the visible overshoot
BATTERY_S = 1200.0        # [DJI]  usable endurance, s

# ------------------------------------------------------- PID (paper eq. (9))
KP = 0.20                 # [PAPER] proportional gain
KI = 0.03                 # [PAPER] integral gain
KD = 0.35                 # [PAPER] derivative gain
D_FILT_TAU = 0.15         # [MODEL] low-pass on the derivative term, s
AB_ALPHA = 0.35           # [MODEL] alpha-beta tracker, position correction
AB_BETA = 0.08            # [MODEL] alpha-beta tracker, velocity correction. Too
                          #         small and the estimate lags on a curving pad.
I_CLAMP = 1.2             # [MODEL] anti-windup clamp on the integral term, m*s
I_BAND = 0.6              # [MODEL] the integral only charges inside this error
                          #         band, so it trims wind without winding up
                          #         during the long approach
KP_Z = 0.55               # [MODEL] vertical loop is a simple P controller
GAIN_LOW = 4.0            # [MODEL] gain multiplier at touchdown altitude. The
                          #         paper's fixed gains are a 5 s time constant,
                          #         too slow when the footprint is 30 cm wide.

# --------------------------------------------------------- monocular camera
IMG_W, IMG_H = 640, 480   # [PAPER] Microsoft HD3000 class VGA stream
FOCAL_PX = 525.0          # [MODEL] focal length in pixels (~60 deg HFOV)
CAM_FPS = 30.0            # [MODEL] vision update rate, Hz
MIN_TAG_PX = 22.0         # [MODEL] smallest tag edge the detector can decode, px
MAX_TAG_FRAC = 0.75       # [MODEL] tag must occupy < 75 % of the frame to be seen
DETECT_NOISE_K = 0.9      # [MODEL] noise scale: sigma = K * range / tag_pixels, m
DROPOUT_P = 0.03          # [MODEL] probability a good frame still fails to decode

# ------------------------------------------ landing target (paper section 2.2)
TAG_BIG = 0.30            # [PAPER] large tag edge length, m  -> high altitude
TAG_SMALL = 0.06          # [PAPER] small tag edge length, m  -> close to ground
TAG_SWITCH_ALT = 1.2      # [PAPER Table 1, point C ~1.02 m] altitude at which the
                          #        small tag becomes the primary source, m

# ---------------------------------------------- AirTag (BLE/UWB) coarse fix
BEACON_HZ = 2.0           # [MODEL] AirTag beacon update rate, Hz
BEACON_SIGMA = 2.5        # [MODEL] 1-sigma error of the BLE/UWB fix, m
BEACON_RANGE = 45.0       # [MODEL] radio range, m

# ------------------------------------------------------- mission thresholds
ALIGN_TOL = 0.60          # [MODEL] offset that unlocks the descent, m
CONE_SLOPE = 0.25         # [MODEL] approach cone: while descending the offset
                          #         must stay below CONE_SLOPE * altitude, so the
                          #         tolerance tightens as the drone gets lower.
ALIGN_HOLD = 0.8          # [MODEL] the offset must stay inside ALIGN_TOL this long, s
DESCEND_ABORT = 1.20      # [MODEL] offset that forces a climb-back, m
FINAL_ALT = 0.35          # [MODEL] altitude at which the motors are cut, m
FINAL_TOL = 0.12          # [MODEL] offset required before the motors are cut, m
LOST_TIMEOUT = 1.5        # [MODEL] vision loss tolerated before re-searching, s
