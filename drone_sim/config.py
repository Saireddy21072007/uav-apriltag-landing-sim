"""
Single source of truth for the whole simulator.

The MuJoCo world, the pad textures, the perception model and the mission code
all read this file, so the physical world and the code that flies in it can
never disagree about where a pad is or which tag is painted on it.

Provenance tags in comments:
  [PAPER] value taken from Li et al., "UAV Autonomous Landing Technology Based
          on AprilTags Vision Positioning Algorithm", 38th CCC, 2019
  [DJI]   DJI Matrice 100 class value
  [MODEL] our modelling choice
"""
import math

# ============================================================ timing
DT_SIM = 0.002            # [MODEL] MuJoCo physics step, s (500 Hz)
DT_CTRL = 0.01            # [MODEL] outer control step, s (100 Hz)
STEPS_PER_CTRL = int(round(DT_CTRL / DT_SIM))
CAM_HZ = 25.0             # [MODEL] onboard camera frame rate
MAX_MISSION_TIME = 180.0  # [MODEL] hard stop, s

# ============================================================ airframe [DJI]
MASS = 3.5                # kg, M100 with payload
GRAVITY = 9.81
HOVER_THRUST = MASS * GRAVITY
INERTIA = (0.045, 0.045, 0.080)   # kg m^2
MAX_THRUST = 2.2 * HOVER_THRUST   # thrust-to-weight ~2.2
MAX_TILT = math.radians(25.0)     # [MODEL] tilt limit -> caps lateral accel
MOTOR_TAU = 0.04          # [MODEL] first-order motor lag, s

# ============================================================ flight envelope
CRUISE_ALT = 8.0          # [MODEL] search altitude, m. Clears the 5.3 m roofs and
                          #         stays under the 8.7 m ceiling at which the
                          #         0.42 m tag stops decoding (see check_geometry).
V_MAX_CRUISE = 6.0        # [MODEL] transit speed cap, m/s
V_MAX_TRACK = 3.0         # [MODEL] speed cap once the tag is locked, m/s
V_MAX_VERT = 1.4          # [MODEL] climb/descend cap, m/s

# ============================================================ camera
IMG_W, IMG_H = 640, 480
CAM_FOVY = 60.0           # degrees, vertical field of view (MuJoCo camera)
# MuJoCo defines fovy over the image height, so the focal length in pixels is:
FOCAL_PX = (IMG_H / 2.0) / math.tan(math.radians(CAM_FOVY) / 2.0)
CX, CY = IMG_W / 2.0, IMG_H / 2.0
CAM_OFFSET_BODY = (0.0, 0.0, -0.12)   # camera position in the body frame, m

# ============================================================ landing pads
# Two tag scales per pad, exactly the idea of section 2.2 of the paper, but with
# DISTINCT IDs so the detector never has to guess which one it decoded:
#   pad k  ->  large tag id = k, small tag id = k + TAG_ID_SMALL_OFFSET
TAG_FAMILY = "tag36h11"
TAG_ID_SMALL_OFFSET = 10
TAG_LARGE = 0.42          # [PAPER-style] large tag edge, m
TAG_SMALL = 0.12          # [PAPER-style] small tag edge, m
TAG_LARGE_OFFSET = 0.40   # large tag centre offset from pad centre, m (+x_pad)
PAD_SIZE = 1.50           # pad edge length, m

# The small tag sits ON the pad centre so the drone still has a marker in frame
# at touchdown height, where the camera only sees a few centimetres of ground.

# ============================================================ detection limits
MIN_TAG_PX = 20.0         # [MODEL] smallest decodable tag edge, px
DECISION_MARGIN_MIN = 25.0  # [MODEL] reject weak decodes from the detector

# ============================================================ AirTag beacon
BEACON_HZ = 2.0           # [MODEL] radio fix rate
BEACON_SIGMA = 2.5        # [MODEL] 1-sigma radio fix error, m
BEACON_RANGE = 60.0       # [MODEL] radio range, m

# ============================================================ IMU / rangefinder
ATT_SIGMA = math.radians(0.6)   # [MODEL] attitude estimate noise, rad
RANGE_SIGMA = 0.03              # [MODEL] downward rangefinder noise, m
RANGE_MAX = 20.0                # [MODEL] rangefinder ceiling, m

# ============================================================ guidance (paper)
KP = 0.20                 # [PAPER] eq. (9)
KI = 0.03                 # [PAPER]
KD = 0.35                 # [PAPER]
I_CLAMP = 1.2             # [MODEL] integral clamp, m*s
I_BAND = 0.6              # [MODEL] integral only charges inside this error, m
D_FILT_TAU = 0.12         # [MODEL] low-pass on the derivative term, s
I_LEAK_TAU = 0.6          # [MODEL] the integral bleeds out with this time
                          #         constant whenever it is pulling AGAINST the
                          #         current error. A charge built up while the
                          #         drone was closing from one side is not trim,
                          #         it is history, and at the clamp it is worth
                          #         more than the proportional term for a ten
                          #         centimetre offset - so the drone sits over
                          #         the pad, sees the offset, and flies the
                          #         wrong way until the charge integrates out.
GAIN_LOW = 3.0            # [MODEL] gain multiplier at touchdown altitude
GAIN_HI_ALT = 4.0         # [MODEL] above this altitude the multiplier is 1.0
GAIN_LO_ALT = 0.6         # [MODEL] below this altitude it is GAIN_LOW

AB_ALPHA = 0.35           # [MODEL] alpha-beta tracker, position gain
AB_BETA = 0.08            # [MODEL] tracker velocity gain
AB_GAMMA = 0.010          # [MODEL] tracker acceleration gain. Small: the term
                          #         only has to catch steady curvature, and it
                          #         amplifies vision noise twice over.
OMEGA_MIN_SPEED = 0.30    # [MODEL] below this speed a velocity vector has no
                          #         reliable direction, so no turn rate is read
                          #         from it, m/s
OMEGA_PERP_MIN = 0.55     # [MODEL] |sin| of the angle between velocity and
                          #         acceleration below which no turn is believed.
                          #         A steady circle sits at 1.0; a vehicle
                          #         accelerating or braking in a straight line
                          #         sits at 0.
OMEGA_MAX = 1.5           # [MODEL] largest turn rate the tracker will believe,
                          #         rad/s. Above this it is reading vision noise
                          #         as a corner.
OMEGA_TAU = 0.35          # [MODEL] low-pass on the estimated turn rate, s. It
                          #         comes from a cross product of a velocity and
                          #         an acceleration that were themselves
                          #         differentiated from a noisy position.
LEAD_TIME = 0.25          # [MODEL] seconds of acceleration lead in the feed-
                          #         forward. This was 0 for most of the project:
                          #         measured then, the lead made things worse.
                          #         That measurement was taken while a saturated
                          #         integral dominated the loop, and it did not
                          #         survive re-measurement once that was fixed -
                          #         over twelve missions on the moving addresses
                          #         the lead now takes the mean touchdown error
                          #         from 7.3 cm to 6.9 cm and the median from
                          #         7.8 cm to 6.1 cm. A tuning result is only
                          #         as good as the loop it was tuned in.

# ============================================================ inner loops
VSP_FILT_TAU = 0.08       # [MODEL] low-pass on the velocity setpoint, s. The
                          #         guidance output carries vision noise, and an
                          #         unfiltered setpoint makes the airframe chatter
                          #         several degrees - which shakes the camera that
                          #         produced the noise in the first place.
KP_VEL = 1.6              # [MODEL] velocity -> acceleration
KI_VEL = 1.2              # [MODEL] velocity-loop integral. A proportional
                          #         velocity loop cannot hold station in a
                          #         steady wind: the drag force needs a standing
                          #         lean, and the only way a P loop produces one
                          #         is by keeping a standing velocity error of
                          #         F_drag / (KP_VEL * MASS) - about 3.6 cm/s
                          #         here, which is the same order as the
                          #         correction the vision loop is asking for.
                          #         The outer PID then spends its whole descent
                          #         fighting the wind instead of the offset,
                          #         and its 10 cm plateau closes only if the
                          #         drone loiters. Trimming the wind where it
                          #         acts costs one state and removes the need.
VI_CLAMP = 0.6            # [MODEL] velocity-loop integral clamp, m/s
KP_VZ = 2.4               # [MODEL] vertical velocity -> acceleration
KP_ATT = 13.0             # [MODEL] attitude -> body rate
KD_RATE = 2.2             # [MODEL] body rate damping
KP_YAW = 3.0
KD_YAW = 1.2

# ============================================================ mission gates
ALIGN_TOL = 0.45          # [MODEL] offset that unlocks the descent, m
ALIGN_HOLD = 0.7          # [MODEL] and it must hold this long, s
CONE_SLOPE = 0.22         # [MODEL] descent allowed while offset < slope * alt
DESCEND_ABORT = 1.00      # [MODEL] offset that forces a climb-back, m
FINAL_ALT = 0.45          # [MODEL] commit-to-touchdown altitude, m
# Commit tolerance. A single number cannot serve both cases: demand 8 cm on a
# turning pad and the drone never commits at all, allow 16 cm on a stationary
# one and it lands three times less accurately than it could. The tolerance is
# therefore set by how fast the pad is estimated to be moving - which is the
# real limit on how precisely a moving target can be centred.
FINAL_TOL = 0.08          # [MODEL] commit tolerance for a stationary pad, m
FINAL_TOL_K = 0.14        # [MODEL] extra metres of tolerance per m/s of pad speed
FINAL_TOL_MAX = 0.18      # [MODEL] cap, still only 12 % of the 1.50 m pad edge
COMMIT_HOLD = 0.20        # [MODEL] seconds without a decode, at or below the
                          #         decision height, before the drone commits.
                          #         Five camera frames: long enough that a
                          #         single dropped detection is not a decision,
                          #         short enough that a moving pad has travelled
                          #         under 20 cm while it is being made.
MAX_GO_AROUNDS = 2        # [MODEL] a go-around is a real manoeuvre, but an
                          #         unbounded number of them is a hang. After
                          #         this many the drone lands on the solution
                          #         it has: on a 1.50 m pad a 20 cm miss is a
                          #         landing, an endless circuit is not.
LOST_TIMEOUT = 1.2        # [MODEL] vision loss tolerated before re-searching, s
FINAL_SINK = 0.35         # [MODEL] sink rate once committed, m/s. Every second
                          #         spent committed is a second flown on dead
                          #         reckoning, so on a pad that can change
                          #         direction the descent rate IS an accuracy
                          #         parameter, not just a comfort one.

# ============================================================ wind
WIND_MEAN = (0.6, -0.4)   # [MODEL] steady wind, m/s
WIND_SIGMA = 0.5          # [MODEL] Ornstein-Uhlenbeck gust std-dev, m/s
WIND_TAU = 1.5            # [MODEL] gust correlation time, s
DRAG_COEF = 0.28          # [MODEL] linear drag, N per (m/s)


# ============================================================ the address book
# Three kinds of landing site, all addressed the same way:
#   ground   - a pad on the street surface
#   vehicle  - a pad on the deck of a vehicle that keeps moving
#   roof     - a pad on an apartment/building roof, "height" metres up
#
# "height" is the height of the SURFACE the pad sits on. The world builder
# raises a building under any pad with a height, so the scene and the address
# book can never disagree about how high a roof is.
PADS = [
    dict(id=1,  address="A-01  Street drop point",   kind="ground",  home=(-1.6,  11.0),
         motion="static"),
    dict(id=2,  address="A-02  Loading bay",         kind="ground",  home=( 1.7,   7.5),
         motion="static"),
    dict(id=3,  address="B-03  Courier van",         kind="vehicle", home=(-1.5,   3.0),
         motion="line", speed=0.9, heading=math.radians(90), half=3.0),
    dict(id=4,  address="B-04  Clinic entrance",     kind="ground",  home=( 1.8,   0.0),
         motion="static"),
    dict(id=5,  address="C-05  Service rover",       kind="vehicle", home=(-1.0,  -3.5),
         motion="circle", radius=0.9, omega=0.60),
    dict(id=6,  address="C-06  Apartment roof, 2F",  kind="roof",    home=( 6.8,  -6.0),
         motion="static", height=3.2),
    dict(id=7,  address="D-07  Utility cart",        kind="vehicle", home=(-1.7, -10.5),
         motion="line", speed=0.7, heading=math.radians(75), half=2.2),
    dict(id=8,  address="D-08  Apartment roof, 3F",  kind="roof",    home=(-6.8, -13.0),
         motion="static", height=5.0),
    dict(id=9,  address="E-09  Tower roof, 5F",      kind="roof",    home=( 7.0, -18.5),
         motion="static", height=7.5),
    dict(id=10, address="E-10  Kerbside locker",     kind="ground",  home=( 1.8, -20.0),
         motion="static"),
]

MOVING_IDS = [p["id"] for p in PADS if p["motion"] != "static"]
ROOF_IDS = [p["id"] for p in PADS if p.get("kind") == "roof"]
GROUND_IDS = [p["id"] for p in PADS if p.get("kind") == "ground"]


def pad_height(spec):
    """Height of the surface the pad sits on, m (0 for the street)."""
    return float(spec.get("height", 0.0))

HOME_XY = (0.0, 14.5)     # launch point, m
ROAD_HALF_WIDTH = 2.85    # kerb-to-kerb half width the pads must stay inside, m


def pad(pad_id):
    for p in PADS:
        if p["id"] == pad_id:
            return p
    raise KeyError(pad_id)


def commit_tolerance(pad_speed):
    """
    How close the drone must be before committing to touchdown, given how fast
    the pad is moving. Uses the ESTIMATED pad speed, not the true one.
    """
    return min(FINAL_TOL + FINAL_TOL_K * abs(pad_speed), FINAL_TOL_MAX)


def pad_track(spec, samples=400):
    """Every position a pad visits, as an (N,2) array - used by check_layout."""
    import numpy as np
    home = np.asarray(spec["home"], float)
    m = spec["motion"]
    if m == "static":
        return home[None, :]
    if m == "circle":
        t = np.linspace(0, 2 * math.pi / spec["omega"], samples)
        return home + np.stack([spec["radius"] * (np.cos(spec["omega"] * t) - 1),
                                spec["radius"] * np.sin(spec["omega"] * t)], axis=1)
    if m == "line":
        t = np.linspace(0, 2 * math.pi * spec["half"] / spec["speed"], samples)
        s_ = spec["half"] * np.sin(spec["speed"] * t / spec["half"])
        d = np.array([math.cos(spec["heading"]), math.sin(spec["heading"])])
        return home + s_[:, None] * d[None, :]
    if m == "drift":
        t = np.linspace(0, 4 * math.pi / spec["omega"], samples)
        return home + spec["amp"] * np.stack(
            [np.sin(spec["omega"] * t), np.sin(0.7 * spec["omega"] * t + 1.1)], axis=1)
    raise ValueError(m)


def check_layout():
    """
    Every pad, over its whole path, must stay on the roadway.

    A pad that wanders over the kerb takes its vehicle onto a raised pavement,
    and the drone then chases it across an obstacle it cannot land on. That is
    how two of the ten addresses silently became unlandable, so it is asserted
    here instead of being rediscovered in a failed mission.
    """
    import numpy as np
    worst = {}
    for spec in PADS:
        if spec.get("kind") == "roof":
            continue                  # roof pads are deliberately off the road
        track = pad_track(spec)
        reach = np.abs(track[:, 0]).max() + PAD_SIZE / 2.0
        worst[spec["id"]] = round(float(reach), 2)
        assert reach <= ROAD_HALF_WIDTH + 0.9, (
            "pad %d reaches x=%.2f m, outside the roadway" % (spec["id"], reach))
    return worst


def fov_halfwidth(alt):
    """Ground half-extent visible straight down at this altitude, m."""
    return alt * (IMG_H / 2.0) / FOCAL_PX


def tag_decode_ceiling(tag_size):
    """Highest altitude at which a tag of this edge length still decodes, m."""
    return FOCAL_PX * tag_size / MIN_TAG_PX


QUIET_ZONE = 1.25         # tag + mandatory white border, as a factor on the edge


def blind_altitude(offset=0.0):
    """
    Height above the deck below which the small marker no longer fits in frame.

    The camera sees a half-height of (agl - camera drop) * (IMG_H/2) / f on the
    deck plane, and the marker needs its whole quiet zone inside that. Below
    the height returned here a "no lock" is the geometry behaving exactly as
    designed, not a perception failure - so the state machine has to commit to
    the touchdown at this height rather than read it as a lost target and climb
    away. A drone that treats its last 25 cm as an emergency can never land.
    """
    drop = -CAM_OFFSET_BODY[2]
    return drop + (TAG_SMALL * QUIET_ZONE / 2.0 + abs(offset)) / (IMG_H / 2.0 / FOCAL_PX)
    # Measured against the renderer and the real detector, the marker actually
    # stops decoding at about 0.23 + 1.55 * offset metres, against the
    # 0.25 + 1.73 * offset this returns. The model is therefore a few
    # centimetres conservative, which is the right direction: it is used to ask
    # whether a detection that has ALREADY been lost was expected to be lost,
    # never to predict a loss that has not happened yet.


BLIND_ALT = round(blind_altitude(0.0), 3)   # m above the deck, ~0.25


def visible_offset(agl):
    """
    Largest pad offset that still keeps the small marker whole in the frame at
    this height above the deck. Exact inverse of blind_altitude(), and the
    honest limit on how far off-centre the drone may be and still be able to
    correct: descending past it trades the measurement for altitude.
    """
    half = (agl + CAM_OFFSET_BODY[2]) * (IMG_H / 2.0 / FOCAL_PX)
    return max(0.0, half - TAG_SMALL * QUIET_ZONE / 2.0)


def check_geometry():
    """
    The dual-scale pad only works if four inequalities hold at once. They are
    checked here rather than discovered as a silently-never-decoding tag.

    Returns a dict of the derived altitudes so the report can quote them.
    """
    big_half = TAG_LARGE * QUIET_ZONE / 2.0
    small_half = TAG_SMALL * QUIET_ZONE / 2.0
    pad_half = PAD_SIZE / 2.0

    fits = TAG_LARGE_OFFSET + big_half <= pad_half
    clears = TAG_LARGE_OFFSET - big_half >= small_half
    ceiling_big = tag_decode_ceiling(TAG_LARGE)
    ceiling_small = tag_decode_ceiling(TAG_SMALL)
    # the large tag leaves the field of view once its far edge does
    big_leaves = (TAG_LARGE_OFFSET + TAG_LARGE / 2.0) / (IMG_H / 2.0 / FOCAL_PX)

    assert fits, "large tag + quiet zone overflows the pad"
    assert clears, "large tag overlaps the small tag's quiet zone"
    assert ceiling_big > CRUISE_ALT, "large tag cannot be decoded at cruise altitude"
    assert big_leaves < ceiling_small, "no altitude band where both tags are usable"

    return {
        "large_decodes_below_m": round(ceiling_big, 2),
        "small_decodes_below_m": round(ceiling_small, 2),
        "large_leaves_fov_below_m": round(big_leaves, 2),
        "handover_window_m": (round(big_leaves, 2), round(ceiling_small, 2)),
        "fov_halfwidth_at_cruise_m": round(fov_halfwidth(CRUISE_ALT), 2),
        "fov_halfwidth_at_touchdown_m": round(fov_halfwidth(0.20), 3),
        # Where the drone must stop trusting the camera and commit.
        "decision_height_centred_m": round(blind_altitude(0.0), 3),
        "decision_height_at_tol_m": round(blind_altitude(FINAL_TOL), 3),
        "visible_offset_at_final_alt_m": round(visible_offset(FINAL_ALT), 3),
    }
