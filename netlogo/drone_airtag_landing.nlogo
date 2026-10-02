;; =====================================================================
;;  Vision-guided autonomous landing on addressed AirTag targets
;;  NetLogo implementation of the methodology in ../drone_sim
;;
;;  Base paper: Zhou Li, Yang Chen, Hao Lu, Huaiyu Wu, Lei Cheng,
;;  "UAV Autonomous Landing Technology Based on AprilTags Vision
;;  Positioning Algorithm", 38th Chinese Control Conference, 2019.
;;
;;  Same methodology as the Python/MuJoCo build, on a second platform:
;;    * ten addressed pads - street, vehicle decks, apartment roofs
;;    * a dual-scale marker, large tag offset from the pad centre
;;    * radio (AirTag) guidance until the addressed tag is decoded, then
;;      vision guidance, with authority passing exactly once
;;    * eq. (9) PID at the paper's published gains, plus the five
;;      changes this project adds
;;    * QUATERNION attitude control - no Euler angle anywhere in the
;;      control path
;;    * a decision height derived from the marker geometry
;;
;;  What differs, and why it is honest: NetLogo renders no camera, so
;;  the DETECTOR is modelled rather than run. A tag is decodable when
;;  its apparent edge exceeds MIN-TAG-PX and its quiet zone lies inside
;;  the field of view - the same two inequalities the Python build
;;  asserts in config.check_geometry() - and the measurement noise is
;;  calibrated to the error that build measures against ground truth
;;  (7.15 cm mean over 90 random poses). Everything downstream of the
;;  measurement is the same algorithm, line for line.
;; =====================================================================

breed [uavs uav]
breed [pads pad]
breed [fovs fov]
breed [captions caption]

globals [
  ;; ---- timing -------------------------------------------------------
  DT CAM-EVERY cam-count sim-t mission-t
  ;; ---- airframe (DJI Matrice 100 class) -----------------------------
  MASS GRAV HOVER-THRUST MAX-THRUST MAX-TILT JX JY JZ
  ;; ---- camera and markers -------------------------------------------
  IMG-W IMG-H FOCAL-PX CAM-DROP MIN-TAG-PX QUIET-ZONE
  TAG-LARGE TAG-SMALL TAG-LARGE-OFFSET PAD-SIZE
  VIS-SIGMA-A VIS-SIGMA-B
  ;; ---- guidance: eq. (9) and the five changes -----------------------
  KP KI KD I-CLAMP I-BAND D-FILT-TAU I-LEAK-TAU
  GAIN-LOW GAIN-HI-ALT GAIN-LO-ALT
  AB-ALPHA AB-BETA AB-GAMMA LEAD-TIME
  OMEGA-MAX OMEGA-TAU OMEGA-PERP-MIN OMEGA-MIN-SPEED
  ;; ---- inner loops --------------------------------------------------
  KP-VEL KI-VEL VI-CLAMP KP-VZ KP-ATT KD-RATE KP-YAW KD-YAW VSP-FILT-TAU
  ;; ---- flight envelope ----------------------------------------------
  CRUISE-ALT V-MAX-CRUISE V-MAX-TRACK V-MAX-VERT
  ;; ---- mission gates ------------------------------------------------
  ALIGN-TOL ALIGN-HOLD CONE-SLOPE DESCEND-ABORT FINAL-ALT FINAL-SINK
  FINAL-TOL FINAL-TOL-K FINAL-TOL-MAX LOST-TIMEOUT COMMIT-HOLD
  MAX-GO-AROUNDS MAX-MISSION-TIME
  ;; ---- beacon -------------------------------------------------------
  BEACON-SIGMA BEACON-RANGE beacon-clock last-fix-x last-fix-y have-fix?
  ;; ---- wind ---------------------------------------------------------
  WIND-MEAN-X WIND-MEAN-Y WIND-SIGMA WIND-TAU DRAG-COEF wind-x wind-y
  ;; ---- mission ------------------------------------------------------
  target-id mission-state go-arounds armed? cruise-target
  aligned-for since-vision held-x held-y have-held? switched? live? fresh?
  search-t result-text
  ;; ---- guidance state -----------------------------------------------
  int-x int-y last-err-x last-err-y d-state-x d-state-y pid-started?
  tp-x tp-y tv-x tv-y ta-x ta-y turn-w t-since have-track?
  vi-x vi-y vsp-x vsp-y vsp-z vspf-x vspf-y vspf-z
  ;; ---- the drone's own state ----------------------------------------
  ;; Held in globals rather than in a turtle: every controller below
  ;; reads and writes all of it every step, and threading that through
  ;; "of" and "ask" in NetLogo buys nothing but places to make a context
  ;; mistake. The turtle exists to be drawn.
  dr-x dr-y dr-z dr-vx dr-vy dr-vz dr-q dr-wx dr-wy dr-wz dr-thrust
  yaw-des head-filt
  ;; ---- the rendered onboard camera panel -----------------------------
  CAM-X0 CAM-X1 CAM-Y0 CAM-Y1 cam-patches
  ;; ---- reporting ----------------------------------------------------
  meas-off true-off tilt-deg marker-text fov-half
]

pads-own [ pad-id pad-kind deck mot home-x home-y spd hdg halfspan rad om
           px py pvx pvy ]

;; =====================================================================
;;  QUATERNION ALGEBRA
;;  q = [w x y z], unit norm, rotating BODY vectors into the WORLD frame
;;  - the same convention as the Python build, and as MuJoCo.
;;
;;  The base paper composes R = Rz(a)Ry(t)Rx(b) and controls three Euler
;;  angles separately. Nothing below ever forms an Euler angle: the
;;  desired orientation is BUILT from the thrust direction, the error is
;;  a quaternion product, and the torque is proportional to that error's
;;  rotation vector. No singularity, no wrap, and the shortest arc comes
;;  from the sign of a single dot product.
;; =====================================================================

to-report deg2rad [d] report d * pi / 180 end
to-report rad2deg [r] report r * 180 / pi end
to-report sin-r [r] report sin (rad2deg r) end
to-report cos-r [r] report cos (rad2deg r) end

to-report acos-r [d]
  let c d
  if c > 1 [ set c 1 ]
  if c < -1 [ set c -1 ]
  report deg2rad (acos c)
end

;; Standard atan2, in radians. NetLogo's own "atan x y" reports a
;; HEADING: degrees clockwise from the positive y axis. Mixing the two
;; conventions silently mirrors every rotation, so the conversion is
;; done once, here, and nowhere else.
to-report atan2-r [y x]
  if x = 0 and y = 0 [ report 0 ]
  let d 90 - (atan x y)
  while [d > 180] [ set d d - 360 ]
  while [d <= -180] [ set d d + 360 ]
  report deg2rad d
end

to-report q-identity report (list 1 0 0 0) end

to-report v-norm [v] report sqrt (sum (map [ a -> a * a ] v)) end

to-report q-normalize [q]
  let n v-norm q
  if n < 1.0e-12 [ report q-identity ]
  report map [ a -> a / n ] q
end

to-report q-conj [q]
  report (list (item 0 q) (0 - item 1 q) (0 - item 2 q) (0 - item 3 q))
end

;; Hamilton product: the rotation b followed by the rotation a.
to-report q-mul [a b]
  let w1 item 0 a   let x1 item 1 a   let y1 item 2 a   let z1 item 3 a
  let w2 item 0 b   let x2 item 1 b   let y2 item 2 b   let z2 item 3 b
  report (list
    (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)
    (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2)
    (w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2)
    (w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2))
end

;; Third column of the rotation matrix: the body z axis in world axes -
;; the direction the rotors actually push along.
to-report q-body-z [q]
  let w item 0 q   let x item 1 q   let y item 2 q   let z item 3 q
  report (list (2 * (x * z + w * y))
               (2 * (y * z - w * x))
               (1 - 2 * (x * x + y * y)))
end

to-report q-from-yaw [psi]
  report (list (cos-r (psi / 2)) 0 0 (sin-r (psi / 2)))
end

to-report q-from-rotvec [v]
  let a v-norm v
  if a < 1.0e-12 [ report q-identity ]
  let s (sin-r (a / 2)) / a
  report (list (cos-r (a / 2))
               ((item 0 v) * s) ((item 1 v) * s) ((item 2 v) * s))
end

;; Rotation vector (axis * angle) of q, taking the SHORT way round.
;; This is the function the attitude controller is built on.
to-report q-to-rotvec [q0]
  let q q-normalize q0
  if item 0 q < 0 [ set q map [ a -> 0 - a ] q ]
  let v (list (item 1 q) (item 2 q) (item 3 q))
  let s v-norm v
  if s < 1.0e-9 [ report map [ a -> 2 * a ] v ]
  let ang 2 * (atan2-r s (item 0 q))
  report map [ a -> a * (ang / s) ] v
end

;; Spherical linear interpolation. A plain lerp fails twice: the result
;; is not a unit quaternion, and because q and -q are the same rotation
;; it can travel the long way round - interpolating 170 degrees to 190
;; degrees through zero instead of through 180.
to-report q-slerp [qa qb t]
  let a q-normalize qa
  let b q-normalize qb
  let d (item 0 a) * (item 0 b) + (item 1 a) * (item 1 b) +
        (item 2 a) * (item 2 b) + (item 3 a) * (item 3 b)
  if d < 0 [ set b map [ z -> 0 - z ] b   set d 0 - d ]
  if d > 0.9995 [
    report q-normalize (map [ [p r] -> p + t * (r - p) ] a b) ]
  let th acos-r d
  let s sin-r th
  report (map [ [p r] -> ((sin-r ((1 - t) * th)) * p +
                          (sin-r (t * th)) * r) / s ] a b)
end

;; Build a quaternion from three orthonormal body axes, given as the
;; columns b1 b2 b3. Shepperd's method: take the branch whose divisor is
;; largest, so it is never ill-conditioned.
to-report axes-to-q [b1 b2 b3]
  let r00 item 0 b1   let r10 item 1 b1   let r20 item 2 b1
  let r01 item 0 b2   let r11 item 1 b2   let r21 item 2 b2
  let r02 item 0 b3   let r12 item 1 b3   let r22 item 2 b3
  let tr r00 + r11 + r22
  if tr > 0 [
    let s 2 * sqrt (tr + 1)
    report q-normalize (list (0.25 * s) ((r21 - r12) / s)
                             ((r02 - r20) / s) ((r10 - r01) / s)) ]
  if r00 > r11 and r00 > r22 [
    let s 2 * sqrt (1 + r00 - r11 - r22)
    report q-normalize (list ((r21 - r12) / s) (0.25 * s)
                             ((r01 + r10) / s) ((r02 + r20) / s)) ]
  if r11 > r22 [
    let s 2 * sqrt (1 + r11 - r00 - r22)
    report q-normalize (list ((r02 - r20) / s) ((r01 + r10) / s)
                             (0.25 * s) ((r12 + r21) / s)) ]
  let s 2 * sqrt (1 + r22 - r00 - r11)
  report q-normalize (list ((r10 - r01) / s) ((r02 + r20) / s)
                           ((r12 + r21) / s) (0.25 * s))
end

to-report v-cross [a b]
  report (list ((item 1 a) * (item 2 b) - (item 2 a) * (item 1 b))
               ((item 2 a) * (item 0 b) - (item 0 a) * (item 2 b))
               ((item 0 a) * (item 1 b) - (item 1 a) * (item 0 b)))
end

to-report v-dot [a b]
  report (item 0 a) * (item 0 b) + (item 1 a) * (item 1 b) +
         (item 2 a) * (item 2 b)
end

to-report v-scale [v k] report map [ a -> a * k ] v end

to-report v-unit [v]
  let n v-norm v
  if n < 1.0e-9 [ report (list 0 0 1) ]
  report v-scale v (1 / n)
end

to-report clamp [x lo hi]
  if x < lo [ report lo ]
  if x > hi [ report hi ]
  report x
end

;; =====================================================================
;;  CONSTANTS - the single source of truth, mirroring ../drone_sim/config.py
;;  Provenance: [PAPER] from Li et al. 2019, [DJI] Matrice 100 class,
;;  [MODEL] this project's modelling choice.
;; =====================================================================

to setup-constants
  ;; ---- timing. The Python build runs control at 100 Hz over 500 Hz
  ;; physics; here one tick is one control step at 50 Hz, with the camera
  ;; at 25 Hz as before. Halving the control rate changes nothing in the
  ;; algorithm - every gain below is either rate-normalised or expressed
  ;; as a time constant.
  set DT 0.02
  set CAM-EVERY 2

  ;; ---- airframe [DJI]
  set MASS 3.5
  set GRAV 9.81
  set HOVER-THRUST MASS * GRAV
  set MAX-THRUST 2.2 * HOVER-THRUST
  set MAX-TILT deg2rad 25
  set JX 0.045   set JY 0.045   set JZ 0.080

  ;; ---- camera
  set IMG-W 640
  set IMG-H 480
  ;; MuJoCo defines the field of view over the image HEIGHT, so
  ;; f = (h/2) / tan(fovy/2) with fovy = 60 degrees.
  set FOCAL-PX (IMG-H / 2) / (tan 30)
  set CAM-DROP 0.12
  set MIN-TAG-PX 20
  set QUIET-ZONE 1.25

  ;; ---- markers [PAPER-style, with distinct ids per scale]
  set TAG-LARGE 0.42
  set TAG-SMALL 0.12
  set TAG-LARGE-OFFSET 0.40
  set PAD-SIZE 1.50

  ;; ---- the modelled detector's error. Calibrated against the Python
  ;; build, which measures its real detector against MuJoCo ground truth
  ;; over 90 random poses: 7.15 cm mean, 9.02 cm on the large tag and
  ;; 3.99 cm on the small one. Error grows with slant range because the
  ;; same pixel of corner error subtends more ground further away.
  set VIS-SIGMA-A 0.010
  set VIS-SIGMA-B 0.011

  ;; ---- guidance, eq. (9) at the paper's published gains [PAPER]
  set KP 0.20
  set KI 0.03
  set KD 0.35
  set I-CLAMP 1.2
  set I-BAND 0.6
  set D-FILT-TAU 0.12
  set I-LEAK-TAU 0.6
  set GAIN-LOW 3.0
  set GAIN-HI-ALT 4.0
  set GAIN-LO-ALT 0.6
  set AB-ALPHA 0.35
  set AB-BETA 0.08
  set AB-GAMMA 0.010
  set LEAD-TIME 0.25
  set OMEGA-MAX 1.5
  set OMEGA-TAU 0.35
  set OMEGA-PERP-MIN 0.55
  set OMEGA-MIN-SPEED 0.30

  ;; ---- inner loops
  set KP-VEL 1.6
  set KI-VEL 1.2
  set VI-CLAMP 0.6
  set KP-VZ 2.4
  set KP-ATT 13.0
  set KD-RATE 2.2
  set KP-YAW 3.0
  set KD-YAW 1.2
  set VSP-FILT-TAU 0.08

  ;; ---- flight envelope
  set CRUISE-ALT 8.0
  set V-MAX-CRUISE 6.0
  set V-MAX-TRACK 3.0
  set V-MAX-VERT 1.4

  ;; ---- mission gates
  set ALIGN-TOL 0.45
  set ALIGN-HOLD 0.7
  set CONE-SLOPE 0.22
  set DESCEND-ABORT 1.00
  set FINAL-ALT 0.45
  set FINAL-SINK 0.35
  set FINAL-TOL 0.08
  set FINAL-TOL-K 0.14
  set FINAL-TOL-MAX 0.18
  set LOST-TIMEOUT 1.2
  set COMMIT-HOLD 0.20
  set MAX-GO-AROUNDS 2
  set MAX-MISSION-TIME 180

  ;; ---- AirTag radio beacon
  set BEACON-SIGMA 2.5
  set BEACON-RANGE 60

  ;; ---- the camera panel occupies the right-hand strip of the world
  set CAM-X0 14   set CAM-X1 29
  set CAM-Y0 -8   set CAM-Y1 7

  ;; ---- wind
  set WIND-MEAN-X 0.6
  set WIND-MEAN-Y -0.4
  set WIND-SIGMA 0.5
  set WIND-TAU 1.5
  set DRAG-COEF 0.28
end

;; Height of the surface a pad sits on. Everything downstream - the
;; search altitude, the approach cone, the gain schedule, the commit
;; height, the touchdown test - is measured against THIS and never
;; against the street. Measured against absolute altitude instead, a
;; rooftop approach keeps its cruise-altitude gains and its wide early
;; cone all the way down to the parapet, arrives fast and off-centre,
;; loses the marker under its own nose and climbs away - for ever.
to-report deck-of [p] report [deck] of p end

to-report commit-tolerance [pad-speed]
  report min (list (FINAL-TOL + FINAL-TOL-K * abs pad-speed) FINAL-TOL-MAX)
end

;; =====================================================================
;;  THE ADDRESS BOOK - the same ten addresses as the Python build
;; =====================================================================

to make-pads
  make-pad  1 "ground"  -1.6   11.0 "static"  0 0 0 0 0 0
  make-pad  2 "ground"   1.7    7.5 "static"  0 0 0 0 0 0
  make-pad  3 "vehicle" -1.5    3.0 "line"    0.9 90  3.0 0 0 0
  make-pad  4 "ground"   1.8    0.0 "static"  0 0 0 0 0 0
  make-pad  5 "vehicle" -1.0   -3.5 "circle"  0 0 0 0.9 0.60 0
  make-pad  6 "roof"     6.8   -6.0 "static"  0 0 0 0 0 3.2
  make-pad  7 "vehicle" -1.7  -10.5 "line"    0.7 75  2.2 0 0 0
  make-pad  8 "roof"    -6.8  -13.0 "static"  0 0 0 0 0 5.0
  make-pad  9 "roof"     7.0  -18.5 "static"  0 0 0 0 0 7.5
  make-pad 10 "ground"   1.8  -20.0 "static"  0 0 0 0 0 0
end

to make-pad [i kind hx hy m s h hs r w height]
  create-pads 1 [
    set pad-id i
    set pad-kind kind
    set home-x hx   set home-y hy
    set mot m       set spd s   set hdg deg2rad h   set halfspan hs
    set rad r       set om w
    ;; a vehicle deck stands above the road; a roof stands at "height"
    ifelse kind = "vehicle" [ set deck height + 0.34 ] [ set deck height + 0.022 ]
    set px hx   set py hy   set pvx 0   set pvy 0
    setxy hx hy
    set shape "square"
    set size ifelse-value (kind = "vehicle") [ 1.8 ] [ 1.5 ]
    set color ifelse-value (kind = "ground") [ orange ]
              [ ifelse-value (kind = "vehicle") [ sky ] [ lime ] ]
    set label (word i)
    set label-color white
  ]
end

to move-pads
  ask pads [
    let ox px
    let oy py
    if mot = "line" [
      ;; back and forth along a fixed heading, so the vehicle stops and
      ;; REVERSES at each end - the case that breaks a naive turn model
      let s halfspan * (sin-r (spd * sim-t / halfspan))
      set px home-x + s * (cos-r hdg)
      set py home-y + s * (sin-r hdg)
    ]
    if mot = "circle" [
      set px home-x + rad * ((cos-r (om * sim-t)) - 1)
      set py home-y + rad * (sin-r (om * sim-t))
    ]
    set pvx (px - ox) / DT
    set pvy (py - oy) / DT
    setxy px py
    if (abs pvx > 0.01) or (abs pvy > 0.01) [ set heading (atan pvx pvy) ]
  ]
end

to-report pad-speed-of [p] report sqrt (([pvx] of p) ^ 2 + ([pvy] of p) ^ 2) end

;; =====================================================================
;;  PERCEPTION - the detector, modelled
;;
;;  NetLogo renders no image, so instead of running a real AprilTag
;;  detector this asks the two questions that decide whether a real one
;;  would have succeeded, and they are exactly the inequalities the
;;  Python build asserts in config.check_geometry():
;;
;;    1. is the tag big enough in the image?   f * size / range >= 20 px
;;    2. does it FIT in the frame?             offset + quiet zone
;;                                             <= (agl - cam drop) * h/2f
;;
;;  The second one is the whole reason the decision height exists: a
;;  downward camera cannot see its own pad to the ground, because the
;;  marker outgrows the frame. Modelling it is not a shortcut - it is
;;  the part that matters.
;; =====================================================================

to-report fov-halfwidth [camera-height]
  report camera-height * (IMG-H / 2) / FOCAL-PX
end

;; Height above the deck below which a marker seen from this far
;; off-centre no longer fits in the frame. The exact inverse of
;; fov-halfwidth, and the drone's decision height.
to-report blind-altitude [off]
  report CAM-DROP + ((TAG-SMALL * QUIET-ZONE / 2) + abs off) / ((IMG-H / 2) / FOCAL-PX)
end

;; Would a tag of this edge length, whose centre lies this far from the
;; camera's ground track, decode from this camera height?
to-report tag-visible? [tag-size off camera-height]
  if camera-height <= 0.02 [ report false ]
  let slant sqrt (off * off + camera-height * camera-height)
  let apparent FOCAL-PX * tag-size / slant
  if apparent < MIN-TAG-PX [ report false ]
  report (off + tag-size * QUIET-ZONE / 2) <= (fov-halfwidth camera-height)
end

to sense
  set fresh? false
  set cam-count cam-count + 1
  if cam-count < CAM-EVERY [
    set since-vision since-vision + DT
    dead-reckon
    stop
  ]
  set cam-count 0

  let p pad-with-id target-id
  let camera-height dr-z - ([deck] of p) - CAM-DROP
  let off-x ([px] of p) - dr-x
  let off-y ([py] of p) - dr-y
  let centre-off sqrt (off-x * off-x + off-y * off-y)

  ;; the large marker sits TAG-LARGE-OFFSET from the pad centre, along
  ;; the pad's own +x axis, so it is a different distance off-axis
  let pyaw ifelse-value (pad-speed-of p > 0.05)
             [ atan2-r ([pvy] of p) ([pvx] of p) ] [ 0 ]
  let lx ([px] of p) + TAG-LARGE-OFFSET * (cos-r pyaw)
  let ly ([py] of p) + TAG-LARGE-OFFSET * (sin-r pyaw)
  let large-off sqrt ((lx - dr-x) ^ 2 + (ly - dr-y) ^ 2)

  let saw-small? tag-visible? TAG-SMALL centre-off camera-height
  let saw-large? tag-visible? TAG-LARGE large-off camera-height

  ;; The smaller tag wins whenever it is available: it is closer to the
  ;; pad centre and its pose is the more accurate one at the altitude
  ;; where it can be seen at all. This is the base paper's multi-scale
  ;; selection rule, made explicit.
  ifelse saw-small? [
    let slant sqrt (centre-off ^ 2 + camera-height ^ 2)
    let sg VIS-SIGMA-A + VIS-SIGMA-B * slant
    set held-x off-x + random-normal 0 sg
    set held-y off-y + random-normal 0 sg
    set have-held? true
    set since-vision 0
    set fresh? true
    set marker-text "0.12 m marker"
    if not switched? [ set switched? true ]
  ] [
    ifelse saw-large? [
      let slant sqrt (large-off ^ 2 + camera-height ^ 2)
      let sg VIS-SIGMA-A + VIS-SIGMA-B * slant
      ;; the large tag gives the position of the TAG, not of the pad; the
      ;; correction is its known offset, rotated by the measured pad
      ;; heading - and a few degrees of heading noise becomes centimetres
      ;; of position error through that 0.40 m lever arm, which is what
      ;; the SLERP filter below is for
      let tx (lx - dr-x) + random-normal 0 sg
      let ty (ly - dr-y) + random-normal 0 sg
      let yaw-meas pyaw + random-normal 0 (deg2rad 2.7)
      set head-filt q-slerp head-filt (q-from-yaw yaw-meas) 0.35
      let yf yaw-of head-filt
      set held-x tx - TAG-LARGE-OFFSET * (cos-r yf)
      set held-y ty - TAG-LARGE-OFFSET * (sin-r yf)
      set have-held? true
      set since-vision 0
      set fresh? true
      set marker-text "0.42 m marker"
    ] [
      set since-vision since-vision + DT
      dead-reckon
      set marker-text ifelse-value (mission-state = "FINAL")
        [ "committed - marker wider than frame" ] [ "NO LOCK" ]
    ]
  ]

  ;; the radio fix keeps arriving whether or not the camera sees anything
  set beacon-clock beacon-clock + DT * CAM-EVERY
  if beacon-clock >= 0.5 [
    set beacon-clock 0
    if centre-off < BEACON-RANGE [
      set last-fix-x ([px] of p) + random-normal 0 BEACON-SIGMA
      set last-fix-y ([py] of p) + random-normal 0 BEACON-SIGMA
      set have-fix? true
    ]
  ]

  if fresh? [
    track-observe (dr-x + held-x) (dr-y + held-y)
    set last-fix-x dr-x + held-x
    set last-fix-y dr-y + held-y
    set have-fix? true
  ]
end

;; Yaw of a yaw-only quaternion, in radians.
to-report yaw-of [q]
  report 2 * (atan2-r (item 3 q) (item 0 q))
end

;; While a detection is missing, the last measured offset is propagated
;; using the estimated pad velocity and the drone's own velocity, so a
;; one-frame dropout does not reset the loop.
to dead-reckon
  if have-held? [
    set held-x held-x + (tv-x - dr-vx) * DT
    set held-y held-y + (tv-y - dr-vy) * DT
  ]
end

to-report pad-with-id [i] report one-of pads with [pad-id = i] end

;; =====================================================================
;;  TARGET TRACKER
;;  Alpha-beta-gamma (constant acceleration) with a COORDINATED-TURN
;;  prediction. Predicts every control step, corrects only on frames that
;;  actually decoded - re-feeding a held measurement at the control rate
;;  would drag the velocity estimate towards zero.
;; =====================================================================

to track-reset
  set have-track? false
  set tp-x 0   set tp-y 0
  set tv-x 0   set tv-y 0
  set ta-x 0   set ta-y 0
  set turn-w 0
  set t-since 0
end

;; Turn rate implied by the velocity and acceleration already estimated.
;; For ANY planar motion the part of the acceleration perpendicular to
;; the velocity IS the turn:   omega = (v x a)_z / |v|^2
;; so no new measurement is needed. But it is believed only when the
;; acceleration really is perpendicular: |sin| of the angle between them
;; is 1 for a pad driving a steady circle and 0 for one braking in a
;; straight line. Without that deadband the model is actively harmful on
;; the pads that shuttle back and forth - at each end the vehicle stops
;; and reverses, |v| passes through zero, and a cross product divided by
;; |v|^2 reports a violent corner where there is only a straight stop.
to-report turn-rate
  let s2 tv-x * tv-x + tv-y * tv-y
  if s2 < OMEGA-MIN-SPEED * OMEGA-MIN-SPEED [ report 0 ]
  let cross tv-x * ta-y - tv-y * ta-x
  let a-mag sqrt (ta-x * ta-x + ta-y * ta-y)
  if a-mag < 1.0e-6 [ report 0 ]
  let perp (abs cross) / ((sqrt s2) * a-mag)
  if perp < OMEGA-PERP-MIN [ report 0 ]
  let ramp (perp - OMEGA-PERP-MIN) / (1 - OMEGA-PERP-MIN)
  if ramp > 1 [ set ramp 1 ]
  report clamp (cross / s2 * ramp) (0 - OMEGA-MAX) OMEGA-MAX
end

to track-predict
  if have-track? [
    ifelse tracker-coordinated-turn? [
      set turn-w turn-w + (DT / (OMEGA-TAU + DT)) * ((turn-rate) - turn-w)
      let ang turn-w * DT
      ifelse abs ang < 1.0e-4 [
        set tp-x tp-x + tv-x * DT + 0.5 * ta-x * DT * DT
        set tp-y tp-y + tv-y * DT + 0.5 * ta-y * DT * DT
        set tv-x tv-x + ta-x * DT
        set tv-y tv-y + ta-y * DT
      ] [
        ;; Exact integral of a velocity that rotates at omega: the target
        ;; travels along the ARC, not along the tangent. A constant-
        ;; acceleration model can only extrapolate a straight line plus a
        ;; fixed bend, and a pad driving a steady circle curves away from
        ;; that continuously.
        let c cos-r ang
        let s sin-r ang
        set tp-x tp-x + (s * tv-x - (1 - c) * tv-y) / turn-w
        set tp-y tp-y + ((1 - c) * tv-x + s * tv-y) / turn-w
        let nvx c * tv-x - s * tv-y
        let nvy s * tv-x + c * tv-y
        set tv-x nvx   set tv-y nvy
        let nax c * ta-x - s * ta-y
        let nay s * ta-x + c * ta-y
        set ta-x nax   set ta-y nay
      ]
    ] [
      set tp-x tp-x + tv-x * DT + 0.5 * ta-x * DT * DT
      set tp-y tp-y + tv-y * DT + 0.5 * ta-y * DT * DT
      set tv-x tv-x + ta-x * DT
      set tv-y tv-y + ta-y * DT
    ]
  ]
  set t-since t-since + DT
end

to track-observe [mx my]
  ifelse not have-track? [
    set tp-x mx   set tp-y my
    set tv-x 0    set tv-y 0
    set ta-x 0    set ta-y 0
    set turn-w 0
    set have-track? true
  ] [
    let dt2 max (list t-since 0.001)
    let rx mx - tp-x
    let ry my - tp-y
    set tp-x tp-x + AB-ALPHA * rx
    set tp-y tp-y + AB-ALPHA * ry
    set tv-x clamp (tv-x + (AB-BETA / dt2) * rx) (0 - 4) 4
    set tv-y clamp (tv-y + (AB-BETA / dt2) * ry) (0 - 4) 4
    let anx ta-x + (2 * AB-GAMMA / (dt2 * dt2)) * rx
    let a-new-y ta-y + (2 * AB-GAMMA / (dt2 * dt2)) * ry
    set ta-x clamp (0.75 * ta-x + 0.25 * anx) (0 - 2.5) 2.5
    set ta-y clamp (0.75 * ta-y + 0.25 * a-new-y) (0 - 2.5) 2.5
  ]
  set t-since 0
end

;; Velocity to feed forward. On a turn the lead is a ROTATION of the
;; velocity, not an addition to it - adding acceleration for a quarter of
;; a second points the drone at the tangent, off the outside of the corner.
to-report lead-vx
  if tracker-coordinated-turn? and abs turn-w > 1.0e-4 [
    report (cos-r (turn-w * LEAD-TIME)) * tv-x - (sin-r (turn-w * LEAD-TIME)) * tv-y ]
  report tv-x + ta-x * LEAD-TIME
end

to-report lead-vy
  if tracker-coordinated-turn? and abs turn-w > 1.0e-4 [
    report (sin-r (turn-w * LEAD-TIME)) * tv-x + (cos-r (turn-w * LEAD-TIME)) * tv-y ]
  report tv-y + ta-y * LEAD-TIME
end

;; =====================================================================
;;  GUIDANCE - eq. (9) of the base paper, at its published gains, with
;;  five changes. Each is switchable from the interface so the ablation
;;  can show what it buys.
;;
;;    V = kp*err + ki*integral + kd*(err - last_err)
;; =====================================================================

to-report altitude-gain [agl]
  if not gain-schedule? [ report 1 ]
  if agl >= GAIN-HI-ALT [ report 1 ]
  if agl <= GAIN-LO-ALT [ report GAIN-LOW ]
  let f (GAIN-HI-ALT - agl) / (GAIN-HI-ALT - GAIN-LO-ALT)
  report 1 + f * (GAIN-LOW - 1)
end

to reset-pid
  set int-x 0   set int-y 0
  set last-err-x 0   set last-err-y 0
  set d-state-x 0    set d-state-y 0
  set pid-started? false
end

;; One PID axis. Returns the velocity command for that axis.
to-report pid-step [err which g]
  let integral ifelse-value (which = 0) [ int-x ] [ int-y ]
  let last-e   ifelse-value (which = 0) [ last-err-x ] [ last-err-y ]
  let dstate   ifelse-value (which = 0) [ d-state-x ] [ d-state-y ]
  if not pid-started? [ set last-e err ]

  ;; CONDITIONAL INTEGRATION: the integral exists to trim a steady wind
  ;; while the drone holds station over the pad. Allowed to charge during
  ;; a twenty-metre transit it only buys overshoot.
  if abs err < I-BAND [
    set integral clamp (integral + err * DT) (0 - I-CLAMP) I-CLAMP ]

  ;; ...WITH A LEAK. A charge built up while closing in from one side is
  ;; history, not trim. Left to unwind on its own it is the largest term
  ;; in the loop: the drone hovers over the pad, measures its ten
  ;; centimetre offset correctly, and flies the other way. On stationary
  ;; addresses this single rule is worth 12.5 cm -> 1.9 cm.
  if anti-windup? and integral * err < 0 [
    set integral integral * (max (list 0 (1 - DT / I-LEAK-TAU))) ]

  ;; RATE-NORMALISED, FILTERED DERIVATIVE. As printed the derivative has
  ;; no division by the sample period, so the same kd behaves completely
  ;; differently at every loop rate; low-pass filtered so that
  ;; centimetre-level vision noise is not differentiated into metres per
  ;; second of command.
  let raw (err - last-e) / (max (list DT 1.0e-6))
  let a DT / (D-FILT-TAU + DT)
  set dstate dstate + a * (raw - dstate)

  ifelse which = 0
    [ set int-x integral   set last-err-x err   set d-state-x dstate ]
    [ set int-y integral   set last-err-y err   set d-state-y dstate ]

  ;; The altitude schedule speeds up the RESPONSE as the camera's view of
  ;; the ground shrinks. It has no business rescaling the accumulated
  ;; trim, which is an estimate of a standing disturbance and is already
  ;; in the right units.
  ifelse anti-windup?
    [ report g * (KP * err + KD * dstate) + KI * integral ]
    [ report g * (KP * err + KI * integral + KD * dstate) ]
end

to-report guidance-vx [ox oy agl]
  let g altitude-gain agl
  let v pid-step ox 0 g
  if feed-forward? [ set v v + lead-vx ]
  report v
end

to-report guidance-vy [ox oy agl]
  let g altitude-gain agl
  let v pid-step oy 1 g
  if feed-forward? [ set v v + lead-vy ]
  set pid-started? true
  report v
end

;; Apply the shared speed cap to a velocity pair.
to set-track-setpoint [ox oy agl]
  let cx guidance-vx ox oy agl
  let cy guidance-vy ox oy agl
  let n sqrt (cx * cx + cy * cy)
  if n > V-MAX-TRACK [ set cx cx * V-MAX-TRACK / n   set cy cy * V-MAX-TRACK / n ]
  set vsp-x cx
  set vsp-y cy
end

;; =====================================================================
;;  INNER LOOP - velocity setpoint to thrust and torque, through a
;;  desired ORIENTATION that is built, never decomposed.
;; =====================================================================

to inner-loop
  ;; ---- velocity loop, with a small integral term.
  ;; A purely proportional velocity loop cannot hold station in a steady
  ;; wind: holding against drag needs a standing lean, and the only way a
  ;; P loop produces one is by keeping a standing velocity error of
  ;; F_drag / (kp_vel * m) - about 3.6 cm/s here, the same order as the
  ;; correction the vision loop is asking for at touchdown height.
  let evx vspf-x - dr-vx
  let evy vspf-y - dr-vy
  let ax KP-VEL * evx + KI-VEL * vi-x
  let ay KP-VEL * evy + KI-VEL * vi-y
  let az KP-VZ * (vspf-z - dr-vz)

  let a-lat sqrt (ax * ax + ay * ay)
  let a-max GRAV * (tan (rad2deg MAX-TILT))
  ifelse a-lat > a-max [
    set ax ax * a-max / a-lat
    set ay ay * a-max / a-lat
  ] [
    ;; integrate only where there is authority left to use it, so a
    ;; saturated manoeuvre cannot wind the trim up behind the limiter
    set vi-x clamp (vi-x + evx * DT) (0 - VI-CLAMP) VI-CLAMP
    set vi-y clamp (vi-y + evy * DT) (0 - VI-CLAMP) VI-CLAMP
  ]
  set az clamp az (0 - 4) 6

  ;; ---- desired force, and the orientation that produces it
  let fx MASS * ax
  let fy MASS * ay
  let fz MASS * (az + GRAV)
  if fz < 0.25 * HOVER-THRUST [ set fz 0.25 * HOVER-THRUST ]
  let lean sqrt (fx * fx + fy * fy)
  let max-lean fz * (tan (rad2deg MAX-TILT))
  if lean > max-lean and max-lean > 0 [
    set fx fx * max-lean / lean
    set fy fy * max-lean / lean
  ]
  let f (list fx fy fz)

  ;; A quadrotor can only push along its own +z, so the desired body z
  ;; axis IS the force direction. Three cross products and a normalise -
  ;; no trigonometry, and nothing singular.
  let b3 v-unit f
  let c1 (list (cos-r yaw-des) (sin-r yaw-des) 0)
  let b2 v-unit (v-cross b3 c1)
  let b1 v-cross b2 b3
  let q-des axes-to-q b1 b2 b3

  ;; ---- attitude error IS a rotation:  q_err = q^-1 * q_des
  let q-err q-mul (q-conj dr-q) q-des
  let erot q-to-rotvec q-err
  ;; proportional on the rotation vector, plus rate damping. A complete
  ;; attitude loop, valid at any attitude, with no small-angle assumption.
  let alpha-x 10 * (KP-ATT * (item 0 erot) - KD-RATE * dr-wx)
  let alpha-y 10 * (KP-ATT * (item 1 erot) - KD-RATE * dr-wy)
  let alpha-z 10 * (KP-YAW * (item 2 erot) - KD-YAW * dr-wz)

  ;; ---- thrust is the projection of the desired force onto the axis the
  ;; drone is CURRENTLY pointing along, not the length of the desired
  ;; force. Sizing it for an orientation the airframe has not reached yet
  ;; leaves surplus lift, and the drone climbs away from its altitude
  ;; setpoint whenever it manoeuvres hard.
  let bz q-body-z dr-q
  set dr-thrust clamp (v-dot f bz) 0 MAX-THRUST

  integrate alpha-x alpha-y alpha-z
end

;; =====================================================================
;;  PHYSICS - rigid body, integrated with the same quaternion state
;; =====================================================================

to integrate [alpha-x alpha-y alpha-z]
  if not armed? [ set dr-thrust 0 ]

  ;; body rates, then the quaternion kinematics  q' = 0.5 * q (x) [0, w]
  set dr-wx dr-wx + alpha-x * DT
  set dr-wy dr-wy + alpha-y * DT
  set dr-wz dr-wz + alpha-z * DT
  let qd q-mul dr-q (list 0 dr-wx dr-wy dr-wz)
  set dr-q q-normalize ((map [ [a b] -> a + 0.5 * b * DT ] dr-q qd))

  ;; wind: an Ornstein-Uhlenbeck gust about a steady mean
  set wind-x wind-x + (DT / WIND-TAU) * (WIND-MEAN-X - wind-x) +
             WIND-SIGMA * sqrt (2 * DT / WIND-TAU) * random-normal 0 1
  set wind-y wind-y + (DT / WIND-TAU) * (WIND-MEAN-Y - wind-y) +
             WIND-SIGMA * sqrt (2 * DT / WIND-TAU) * random-normal 0 1
  let wx-eff ifelse-value wind? [ wind-x ] [ 0 ]
  let wy-eff ifelse-value wind? [ wind-y ] [ 0 ]

  let bz q-body-z dr-q
  let ax (dr-thrust / MASS) * (item 0 bz) + DRAG-COEF * (wx-eff - dr-vx) / MASS
  let ay (dr-thrust / MASS) * (item 1 bz) + DRAG-COEF * (wy-eff - dr-vy) / MASS
  let az (dr-thrust / MASS) * (item 2 bz) - GRAV - DRAG-COEF * dr-vz / MASS

  set dr-vx dr-vx + ax * DT
  set dr-vy dr-vy + ay * DT
  set dr-vz dr-vz + az * DT
  set dr-x dr-x + dr-vx * DT
  set dr-y dr-y + dr-vy * DT
  set dr-z dr-z + dr-vz * DT

  ;; contact with whatever surface is underneath
  let floor-h surface-under dr-x dr-y
  if dr-z <= floor-h + 0.21 [
    set dr-z floor-h + 0.21
    if dr-vz < 0 [ set dr-vz 0 ]
    set dr-vx dr-vx * 0.5
    set dr-vy dr-vy * 0.5
  ]
end

;; Height of the surface beneath a point: a roof if we are over one,
;; a vehicle deck if we are over one, otherwise the road.
to-report surface-under [x y]
  let h 0
  ask pads [
    if pad-kind = "roof" [
      if (abs (x - px) < 2.2) and (abs (y - py) < 2.2) [
        if deck > h [ set h deck ] ] ]
    if pad-kind = "vehicle" [
      if (abs (x - px) < 0.75) and (abs (y - py) < 0.75) [
        if deck > h [ set h deck ] ] ]
  ]
  report h
end

;; =====================================================================
;;  THE MISSION
;;
;;    TAKEOFF   climb to search altitude
;;    TRANSIT   fly to the addressed pad's radio fix      (radio guidance)
;;    SEARCH    expanding spiral when the fix was stale    (radio guidance)
;;    ALIGN     hold altitude, drive the vision offset to zero  (vision)
;;    DESCEND   descend only while inside the approach cone
;;    FINAL     committed descent, motors cut on contact
;;    LANDED / ABORTED
;;
;;  Guidance authority passes from radio to vision exactly once, at the
;;  moment the addressed pad's marker first decodes. The drone will not
;;  accept any other pad's marker: the ten pads carry ten different ids.
;; =====================================================================

;; NetLogo runs this by itself when the model is opened, so the world is
;; already built before anyone presses anything.
to startup
  setup
end

to setup
  clear-all
  setup-constants
  if fixed-seed? [ random-seed seed ]
  draw-street
  make-pads
  draw-buildings
  setup-camera-panel
  setup-camera-caption
  create-uavs 1 [
    set shape "default"
    set color white
    set size 2.2
  ]
  create-fovs 1 [
    set shape "circle 2"
    set color grey
    set size 1
  ]
  set sim-t 0
  set wind-x WIND-MEAN-X
  set wind-y WIND-MEAN-Y
  set dr-x 0   set dr-y 14.5   set dr-z 0.28
  set dr-vx 0  set dr-vy 0     set dr-vz 0
  set dr-q q-identity
  set dr-wx 0  set dr-wy 0     set dr-wz 0
  set dr-thrust HOVER-THRUST
  set yaw-des 0
  set head-filt q-identity
  set result-text "-"
  set target-id 0
  set mission-state "IDLE"
  reset-ticks
  fly-to start-address
  refresh-display
end

;; The street, drawn once. Roadway, kerbs, and a block under every pad
;; that sits on a roof - the building is generated FROM the address
;; book's height, so the scene and the mission can never disagree about
;; how high a roof is.
to draw-street
  ;; the street only - the right-hand strip belongs to the camera panel
  ask patches with [pxcor < CAM-X0 - 1] [
    set pcolor ifelse-value (abs pxcor <= 3) [ 36 ] [ 56 ]
  ]
  ask patches with [abs pxcor = 3] [ set pcolor 35 ]
end

to draw-buildings
  ask pads with [pad-kind = "roof"] [
    ask patches in-radius 2.2 [ set pcolor 4 ]
  ]
end

;; ---------------------------------------------------------------------
;;  Retasking. Deliberately does NOT reset the world: the vehicles keep
;;  driving and the drone keeps its position and velocity, so pressing a
;;  number mid-flight is a DIVERT, not a restart. Three defects lived
;;  only on this path in the Python build and none were visible to a
;;  batch evaluation that flies every mission from a fresh reset.
;; ---------------------------------------------------------------------
to fly-to [i]
  ;; Pressing an address before SETUP is the natural thing to try, and
  ;; it used to fail with "OF expected ... but got NOBODY" because there
  ;; were no pads yet. Build the world first instead of complaining.
  if not any? pads [ setup ]
  set target-id i
  set armed? true                    ;; a divert after touchdown must re-arm
  set mission-state "TAKEOFF"
  set mission-t 0                    ;; ...and time out on ITS OWN clock
  set go-arounds 0
  set aligned-for 0
  set since-vision 99
  set have-held? false
  set switched? false
  set have-fix? false
  set search-t 0
  set result-text "-"
  set head-filt q-identity
  reset-pid
  track-reset
  set vi-x 0   set vi-y 0
  set vsp-x 0  set vsp-y 0  set vsp-z 0
  set vspf-x 0 set vspf-y 0 set vspf-z 0
  ;; Search altitude is relative to the surface the pad sits on. A pad on
  ;; a fifth-floor roof is approached from a height above THAT roof.
  set cruise-target ([deck] of (pad-with-id i)) + CRUISE-ALT
end

to fly-to-1 fly-to 1 end
to fly-to-2 fly-to 2 end
to fly-to-3 fly-to 3 end
to fly-to-4 fly-to 4 end
to fly-to-5 fly-to 5 end
to fly-to-6 fly-to 6 end
to fly-to-7 fly-to 7 end
to fly-to-8 fly-to 8 end
to fly-to-9 fly-to 9 end
to fly-to-10 fly-to 10 end

;; ---------------------------------------------------------------------
;;  One control step
;; ---------------------------------------------------------------------
to go
  set sim-t sim-t + DT
  set mission-t mission-t + DT
  move-pads
  track-predict
  sense
  fly-state-machine
  smooth-setpoint
  inner-loop
  refresh-display
  ;; the panel is a display, not part of the control loop - rendering it
  ;; every fourth step keeps the view smooth without doing 256 ray casts
  ;; fifty times a second
  if (ticks mod 4) = 0 [ render-camera  update-camera-caption ]
  tick
end

;; A low-pass on the velocity setpoint. The guidance output carries
;; vision noise, and an unfiltered setpoint makes the airframe chatter
;; several degrees - which shakes the camera that produced the noise.
to smooth-setpoint
  let a DT / (VSP-FILT-TAU + DT)
  set vspf-x vspf-x + a * (vsp-x - vspf-x)
  set vspf-y vspf-y + a * (vsp-y - vspf-y)
  set vspf-z vspf-z + a * (vsp-z - vspf-z)
end

to-report have-vision? report (since-vision < LOST-TIMEOUT) and have-held? end
to-report live-vision? report since-vision < 1.5 * DT * CAM-EVERY end

;; Every climb in this state machine goes through here. The search
;; altitude is already the height at which the large marker decodes, so
;; climbing past it buys nothing - and an uncapped recovery climb
;; ratchets the drone upwards a little on every attempt.
to-report climb-back [rate]
  report min (list rate (1.1 * (cruise-target - dr-z)))
end

to fly-state-machine
  let p pad-with-id target-id
  let deck-h [deck] of p
  let agl dr-z - deck-h
  let ox ifelse-value have-held? [ held-x ] [ 0 ]
  let oy ifelse-value have-held? [ held-y ] [ 0 ]
  let err sqrt (ox * ox + oy * oy)
  set meas-off err
  set true-off sqrt ((([px] of p) - dr-x) ^ 2 + (([py] of p) - dr-y) ^ 2)
  set vsp-x 0   set vsp-y 0   set vsp-z 0

  if mission-state = "TAKEOFF" [
    set vsp-z clamp (1.1 * (cruise-target - dr-z)) (0 - V-MAX-VERT) V-MAX-VERT
    if dr-z > cruise-target - 0.4 [ set mission-state "TRANSIT" ]
  ]

  if mission-state = "TRANSIT" [
    set vsp-z clamp (1.1 * (cruise-target - dr-z)) (0 - V-MAX-VERT) V-MAX-VERT
    ifelse live-vision? [
      track-reset
      track-observe (dr-x + ox) (dr-y + oy)
      reset-pid
      set mission-state "ALIGN"
    ] [
      if have-fix? [
        let ex last-fix-x - dr-x
        let ey last-fix-y - dr-y
        let cx 0.8 * ex
        let cy 0.8 * ey
        let n sqrt (cx * cx + cy * cy)
        if n > V-MAX-CRUISE [ set cx cx * V-MAX-CRUISE / n   set cy cy * V-MAX-CRUISE / n ]
        set vsp-x cx   set vsp-y cy
        if sqrt (ex * ex + ey * ey) < 1.2 [
          set search-t 0
          set mission-state "SEARCH" ]
      ]
    ]
  ]

  if mission-state = "SEARCH" [
    set vsp-z clamp (1.1 * (cruise-target - dr-z)) (0 - V-MAX-VERT) V-MAX-VERT
    set search-t search-t + DT
    ifelse live-vision? [
      track-reset
      track-observe (dr-x + ox) (dr-y + oy)
      reset-pid
      set mission-state "ALIGN"
    ] [
      let r 0.5 + 0.30 * search-t
      let wx last-fix-x + r * (cos-r (0.8 * search-t))
      let wy last-fix-y + r * (sin-r (0.8 * search-t))
      set vsp-x clamp (1.1 * (wx - dr-x)) (0 - 2.5) 2.5
      set vsp-y clamp (1.1 * (wy - dr-y)) (0 - 2.5) 2.5
      if search-t > 30 [ set mission-state "TRANSIT" ]
    ]
  ]

  if mission-state = "ALIGN" [
    set vsp-z clamp (1.1 * (cruise-target - dr-z)) (0 - V-MAX-VERT) V-MAX-VERT
    ifelse not have-vision? [
      set mission-state "TRANSIT"
    ] [
      set-track-setpoint ox oy agl
      ifelse err < ALIGN-TOL [ set aligned-for aligned-for + DT ] [ set aligned-for 0 ]
      if aligned-for > ALIGN-HOLD [ set mission-state "DESCEND" ]
    ]
  ]

  if mission-state = "DESCEND" [
    let tol commit-tolerance (pad-speed-of p)
    ;; DECISION HEIGHT. blind-altitude(err) is the height at which a
    ;; marker seen from this far off-centre stops fitting in the frame.
    ;; Below it the camera has nothing left to give, so the drone lands
    ;; on the solution it already holds. The trigger is the marker
    ;; ACTUALLY going away, held for a few frames - never a prediction
    ;; that it is about to. The geometry only answers the second
    ;; question: was this loss expected here?
    let lost-low? (since-vision > COMMIT-HOLD) and
                  (agl <= blind-altitude err) and have-held?
    ifelse lost-low? [
      set mission-state "FINAL"
    ] [
      ifelse not have-vision? [
        ;; losing the marker higher up is a real re-acquire, but it must
        ;; not be unbounded: an endless circuit is not a safety feature
        set go-arounds go-arounds + 1
        ifelse (go-arounds > MAX-GO-AROUNDS) and (agl < 1.0) and have-held? [
          set mission-state "FINAL"
        ] [
          set vsp-z climb-back 0.6
          set mission-state "ALIGN"
        ]
      ] [
        set-track-setpoint ox oy agl
        let allowed max (list tol (CONE-SLOPE * agl))
        ifelse not live-vision? [
          set vsp-z 0                            ;; never descend blind
        ] [
          ifelse err > DESCEND-ABORT [
            set vsp-z climb-back 0.5
          ] [
            ifelse err > allowed [
              set vsp-z 0                        ;; outside the cone: hold
            ] [
              let q max (list 0 (1 - err / allowed))
              set vsp-z 0 - (clamp (0.25 + 0.75 * q) 0 V-MAX-VERT)
              if agl < 1.2 [ set vsp-z max (list vsp-z (0 - 0.45)) ]
            ]
          ]
        ]
        if live-vision? and agl < FINAL-ALT and err < tol [
          set mission-state "FINAL" ]
      ]
    ]
  ]

  if mission-state = "FINAL" [
    set-track-setpoint ox oy agl
    set vsp-z 0 - FINAL-SINK
    ;; a go-around is only meaningful while the marker is still usable,
    ;; and only while the drone has any left to spend
    if (agl > blind-altitude err) and (go-arounds <= MAX-GO-AROUNDS) and
       (err > 3 * (commit-tolerance (pad-speed-of p))) [
      set mission-state "DESCEND" ]
    if (abs (agl - 0.21) < 0.09) and (abs dr-vz < 0.25) [
      set armed? false
      set mission-state "LANDED"
      set result-text (word "LANDED  " (precision (100 * true-off) 1)
                            " cm  in " (precision mission-t 1) " s")
    ]
  ]

  if (mission-state != "LANDED") and (mission-state != "ABORTED")
     and (mission-t > MAX-MISSION-TIME) [
    set mission-state "ABORTED"
    set result-text (word "ABORTED after " (precision mission-t 0) " s")
  ]

  if mission-state = "LANDED" or mission-state = "ABORTED" [
    set vsp-x 0   set vsp-y 0   set vsp-z 0
  ]
end

;; ---------------------------------------------------------------------
;;  Display
;; ---------------------------------------------------------------------
to refresh-display
  let bz q-body-z dr-q
  set tilt-deg rad2deg (acos-r (item 2 bz))
  let p pad-with-id target-id
  let camera-height dr-z - ([deck] of p) - CAM-DROP
  set fov-half fov-halfwidth (max (list camera-height 0))

  ask uavs [
    setxy dr-x dr-y
    ;; size carries the altitude: the drone looks bigger when it is high,
    ;; the way it would in a top-down view
    set size 1.2 + 0.16 * dr-z
    set label (word (precision dr-z 1) " m")
    set label-color white
  ]
  ;; The camera footprint, to scale. Watching this circle shrink below
  ;; the marker is the decision height, visible.
  ask fovs [
    setxy dr-x dr-y
    set size max (list 0.2 (2 * fov-half))
    set color ifelse-value live-vision? [ green ] [ red ]
  ]
  ask pads [
    set color ifelse-value (pad-id = target-id) [ yellow ]
      [ ifelse-value (pad-kind = "ground") [ orange ]
        [ ifelse-value (pad-kind = "vehicle") [ sky ] [ lime ] ] ]
  ]
end

;; ---------------------------------------------------------------------
;;  Interface reporters
;; ---------------------------------------------------------------------
to-report state-text report mission-state end
to-report address-text
  if target-id = 0 [ report "-" ]
  report (word target-id "  " (address-name target-id))
end
to-report address-name [i]
  if i = 1 [ report "A-01 Street drop point" ]
  if i = 2 [ report "A-02 Loading bay" ]
  if i = 3 [ report "B-03 Courier van" ]
  if i = 4 [ report "B-04 Clinic entrance" ]
  if i = 5 [ report "C-05 Service rover" ]
  if i = 6 [ report "C-06 Apartment roof, 2F" ]
  if i = 7 [ report "D-07 Utility cart" ]
  if i = 8 [ report "D-08 Apartment roof, 3F" ]
  if i = 9 [ report "E-09 Tower roof, 5F" ]
  report "E-10 Kerbside locker"
end
to-report altitude-m report precision dr-z 2 end
to-report agl-m
  if target-id = 0 [ report 0 ]
  report precision (dr-z - ([deck] of (pad-with-id target-id))) 2
end
to-report offset-m report precision meas-off 3 end
to-report true-offset-cm report precision (100 * true-off) 1 end
to-report tilt-d report precision tilt-deg 1 end
to-report lock-text report marker-text end
to-report decision-height-m
  report precision (blind-altitude meas-off) 3
end
to-report mission-time report precision mission-t 1 end
to-report pad-speed-now
  if target-id = 0 [ report 0 ]
  report precision (pad-speed-of (pad-with-id target-id)) 2
end

;; =====================================================================
;;  THE ONBOARD CAMERA, RENDERED
;;
;;  NetLogo has one view, so the right-hand strip of the world is given
;;  over to the drone's camera and every patch in it is treated as a
;;  PIXEL. For each one: turn the pixel into a ray in the camera frame,
;;  rotate that ray into the world with the drone's actual attitude
;;  quaternion, intersect it with the surface the pad sits on, and colour
;;  the pixel by what is at the point it hits.
;;
;;  That is ray casting, and it is the same pinhole model the perception
;;  code uses - f = (h/2)/tan(fovy/2), camera looking along -body z. So
;;  the panel is not an illustration of what the drone sees. It is what
;;  the drone sees.
;;
;;  Watch it on the descent. High up, both markers sit small in the
;;  middle. Lower, the large marker slides to the edge and is cut off -
;;  that is the hand-over to the small one. Lower still the small marker
;;  outgrows the frame too, and the height at which that happens is the
;;  decision height: vision has nothing left to give and the drone
;;  commits on the solution it already holds.
;; =====================================================================

to setup-camera-panel
  set cam-patches patches with [pxcor >= CAM-X0 and pxcor <= CAM-X1 and
                               pycor >= CAM-Y0 and pycor <= CAM-Y1]
  ask cam-patches [ set pcolor 2 ]
  ;; a one-patch frame around the panel so it reads as a separate display
  ask patches with [(pxcor >= CAM-X0 - 1) and (pxcor <= CAM-X1 + 1) and
                    (pycor >= CAM-Y0 - 1) and (pycor <= CAM-Y1 + 1) and
                    not (pxcor >= CAM-X0 and pxcor <= CAM-X1 and
                         pycor >= CAM-Y0 and pycor <= CAM-Y1)] [ set pcolor 6 ]
end

;; First column of the rotation matrix: the body x axis, in world axes.
to-report q-body-x [q]
  let w item 0 q   let x item 1 q   let y item 2 q   let z item 3 q
  report (list (1 - 2 * (y * y + z * z))
               (2 * (x * y + w * z))
               (2 * (x * z - w * y)))
end

;; Second column: the body y axis, in world axes.
to-report q-body-y [q]
  let w item 0 q   let x item 1 q   let y item 2 q   let z item 3 q
  report (list (2 * (x * y - w * z))
               (1 - 2 * (x * x + z * z))
               (2 * (y * z + w * x)))
end

to render-camera
  let p pad-with-id target-id
  let deck-h [deck] of p
  let padx [px] of p
  let pady [py] of p
  let pyaw 0
  if pad-speed-of p > 0.05 [ set pyaw atan2-r ([pvy] of p) ([pvx] of p) ]
  let cy cos-r pyaw
  let sy sin-r pyaw

  let bx-axis q-body-x dr-q
  let by-axis q-body-y dr-q
  let bz-axis q-body-z dr-q

  ;; the camera hangs CAM-DROP below the body origin, along -body z
  let camx dr-x - CAM-DROP * (item 0 bz-axis)
  let camy dr-y - CAM-DROP * (item 1 bz-axis)
  let camz dr-z - CAM-DROP * (item 2 bz-axis)

  let span-x (CAM-X1 - CAM-X0)
  let span-y (CAM-Y1 - CAM-Y0)
  let half-w IMG-W / 2
  let half-h IMG-H / 2

  ask cam-patches [
    ;; this pixel, in image coordinates
    let u ((pxcor - CAM-X0) / span-x) * IMG-W
    let v ((CAM-Y1 - pycor) / span-y) * IMG-H

    ;; the ray it corresponds to, in the camera (body) frame. The camera
    ;; looks along -body z, so the ray is (x, y, -1) scaled by 1/f.
    let rx (u - half-w) / FOCAL-PX
    let ry (0 - (v - half-h)) / FOCAL-PX

    ;; rotate that ray into the world
    let dx-w rx * (item 0 bx-axis) + ry * (item 0 by-axis) - (item 0 bz-axis)
    let dy-w rx * (item 1 bx-axis) + ry * (item 1 by-axis) - (item 1 bz-axis)
    let dz-w rx * (item 2 bx-axis) + ry * (item 2 by-axis) - (item 2 bz-axis)

    ifelse dz-w > -0.02 [
      set pcolor 103                      ;; ray points at the sky
    ] [
      ;; where it meets the plane the pad sits on
      let t (deck-h - camz) / dz-w
      ifelse t <= 0 [
        set pcolor 103
      ] [
        let hx camx + t * dx-w
        let hy camy + t * dy-w
        ;; that point, in the pad's own frame
        let ex hx - padx
        let ey hy - pady
        let lx ex * cy + ey * sy
        let ly (0 - ex) * sy + ey * cy
        set pcolor pixel-colour lx ly
      ]
    ]
  ]

  ;; Crosshair: where the camera is actually pointing. Addressed
  ;; directly - asking every patch in the world whether it belongs to the
  ;; panel is a membership test against a 256-patch agentset for each of
  ;; the world's 1700 patches, twice a tick, and it brings the model to a
  ;; halt.
  let mx round ((CAM-X0 + CAM-X1) / 2)
  let my round ((CAM-Y0 + CAM-Y1) / 2)
  ask patch mx my       [ set pcolor 25 ]
  ask patch (mx - 1) my [ set pcolor 25 ]
  ask patch (mx + 1) my [ set pcolor 25 ]
  ask patch mx (my - 1) [ set pcolor 25 ]
  ask patch mx (my + 1) [ set pcolor 25 ]
end

;; What is painted at this point of the pad, in the pad's own frame?
to-report pixel-colour [lx ly]
  ;; the small marker, at the pad centre
  if (abs lx <= TAG-SMALL / 2) and (abs ly <= TAG-SMALL / 2) [ report 0 ]
  ;; the large marker, TAG-LARGE-OFFSET along the pad's +x axis
  if (abs (lx - TAG-LARGE-OFFSET) <= TAG-LARGE / 2) and
     (abs ly <= TAG-LARGE / 2) [ report 0 ]
  ;; the pad itself, and its painted border
  if (abs lx <= PAD-SIZE / 2) and (abs ly <= PAD-SIZE / 2) [
    if (abs lx > PAD-SIZE / 2 - 0.09) or (abs ly > PAD-SIZE / 2 - 0.09) [
      report 95 ]                          ;; blue deck edging
    report 9.9 ]                           ;; white deck
  report 35                                ;; the surface around it
end

;; A readable caption under the panel, drawn with a label-only turtle.
to setup-camera-caption
  create-captions 1 [
    setxy ((CAM-X0 + CAM-X1) / 2) (CAM-Y0 - 2)
    set shape "square"
    set size 0.1
    set color black
    set label-color white
    set label "onboard camera"
  ]
end

to update-camera-caption
  ask captions [
    set label (word "camera  " (precision (dr-z - ([deck] of (pad-with-id target-id))) 2)
                    " m above deck   -   " marker-text)
  ]
end


;; ---------------------------------------------------------------------
;;  Checking the camera without a display.
;;
;;  The panel is ray cast, so it can be verified by counting pixels: park
;;  the drone squarely over a pad at a known height and ask how much of
;;  the frame the markers and the pad fill. Those counts follow from the
;;  pinhole geometry alone, so if they match what the lens equation says
;;  they should be, the render is right.
;; ---------------------------------------------------------------------
to-report cam-dark   report count cam-patches with [pcolor = 0] end
to-report cam-pad    report count cam-patches with [pcolor = 9.9] end
to-report cam-total  report count cam-patches end

to park-over-pad [i h]
  set target-id i
  let p pad-with-id i
  set dr-x [px] of p
  set dr-y [py] of p
  set dr-z ([deck] of p) + h
  set dr-q q-identity
  set dr-vx 0  set dr-vy 0  set dr-vz 0
  render-camera
end

;; Marker pixels and pad pixels against height above the deck. The marker
;; must grow as the drone descends, and the pad must fill the frame
;; before the drone touches down - that filling is the decision height.
to-report camera-geometry-table
  let out []
  foreach (list 8 4 2 1 0.6 0.45 0.3 0.2) [ h ->
    park-over-pad 4 h
    set out lput (list (precision h 2) cam-dark cam-pad cam-total) out
  ]
  report out
end
@#$#@#$#@
GRAPHICS-WINDOW
265
10
1185
705
-1
-1
16.0
1
10
1
1
1
0
0
0
1
-12
30
-23
17
1
1
1
ticks
30.0

TEXTBOX
12
10
352
32
AirTag-addressed autonomous landing
14
0
1

TEXTBOX
12
34
352
68
Press a number to land there. Pressing one in flight is a DIVERT, not a restart.
10
0
1

BUTTON
12
72
92
104
setup
setup
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
100
72
180
104
go
go
T
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
188
72
268
104
step
go
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
12
116
126
148
1 street
fly-to-1
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
134
116
248
148
2 loading
fly-to-2
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
12
154
126
186
3 van
fly-to-3
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
134
154
248
186
4 clinic
fly-to-4
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
12
192
126
224
5 rover
fly-to-5
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
134
192
248
224
6 roof 2F
fly-to-6
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
12
230
126
262
7 cart
fly-to-7
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
134
230
248
262
8 roof 3F
fly-to-8
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
12
268
126
300
9 tower 5F
fly-to-9
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
134
268
248
300
10 kerb
fly-to-10
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

SLIDER
12
312
248
345
start-address
start-address
1
10
1
1
1
NIL
HORIZONTAL

SLIDER
12
350
248
383
seed
seed
0
50
1
1
1
NIL
HORIZONTAL

SWITCH
12
390
126
423
fixed-seed?
fixed-seed?
0
1
-1000

SWITCH
134
390
248
423
wind?
wind?
0
1
-1000

SWITCH
12
428
248
461
feed-forward?
feed-forward?
0
1
-1000

SWITCH
12
466
248
499
gain-schedule?
gain-schedule?
0
1
-1000

SWITCH
12
504
248
537
anti-windup?
anti-windup?
0
1
-1000

SWITCH
12
542
248
575
tracker-coordinated-turn?
tracker-coordinated-turn?
0
1
-1000

MONITOR
12
586
128
631
state
state-text
0
1
11

MONITOR
132
586
248
631
address
address-text
0
1
11

MONITOR
12
636
128
681
alt (m)
altitude-m
2
1
11

MONITOR
132
636
248
681
above deck (m)
agl-m
2
1
11

MONITOR
12
686
128
731
offset (m)
offset-m
3
1
11

MONITOR
132
686
248
731
true miss (cm)
true-offset-cm
1
1
11

MONITOR
12
736
128
781
decision h (m)
decision-height-m
3
1
11

MONITOR
132
736
248
781
tilt (deg)
tilt-d
1
1
11

MONITOR
12
786
248
831
marker
lock-text
0
1
11

MONITOR
12
836
128
881
mission t (s)
mission-time
1
1
11

MONITOR
132
836
248
881
pad speed (m/s)
pad-speed-now
2
1
11

PLOT
256
586
596
731
offset to pad centre
time (s)
m
0.0
60.0
0.0
1.0
true
true
"" ""
PENS
"measured" 1.0 0 -13345367 true "" "plot offset-m"
"true" 1.0 0 -2674135 true "" "plot true-off"

PLOT
256
736
596
881
height above the deck
time (s)
m
0.0
60.0
0.0
12.0
true
true
"" ""
PENS
"agl" 1.0 0 -16777216 true "" "plot agl-m"
"decision height" 1.0 0 -955883 true "" "plot decision-height-m"
@#$#@#$#@
## WHAT IS IT?

A NetLogo implementation of the landing methodology in the Python/MuJoCo
build that sits beside it in this project, and of the base paper it
extends:

  Zhou Li, Yang Chen, Hao Lu, Huaiyu Wu, Lei Cheng, "UAV Autonomous
  Landing Technology Based on AprilTags Vision Positioning Algorithm",
  38th Chinese Control Conference, 2019, pp. 8148-8153.

Ten AirTag addresses are laid out along a street: four on the road
surface, three on the decks of vehicles that keep driving, and three on
building roofs at 3.2 m, 5.0 m and 7.5 m. Press a number and the drone
flies there and lands on it. Pressing a number while it is already flying
retasks it in place - a divert, not a restart.

## HOW IT WORKS

Guidance authority passes from radio to vision exactly once, when the
addressed pad's marker first decodes. Before that the drone steers on a
noisy AirTag radio fix; after it, on the marker.

The control law is eq. (9) of the base paper at its published gains
(kp = 0.20, ki = 0.03, kd = 0.35) with five changes, each switchable from
the interface so you can see what it buys:

  * a rate-normalised, low-pass filtered derivative
  * conditional integration WITH A LEAK, and a trim the altitude schedule
    does not rescale
  * pad-velocity feed-forward from an alpha-beta-gamma tracker
  * altitude gain scheduling, measured against the height above the
    surface the pad sits on rather than above the street
  * a coordinated-turn prediction in the tracker, gated on how
    perpendicular the estimated acceleration actually is

Attitude is controlled in UNIT QUATERNIONS throughout. The base paper
composes R = Rz(a)Ry(t)Rx(b) and controls three Euler angles; nothing in
the control path here ever forms an Euler angle. The desired orientation
is built from the thrust direction, the error is the quaternion product
q^-1 * q_des, and the torque is proportional to that error's rotation
vector - so nothing is singular, nothing wraps, and the shortest arc
comes from the sign of one dot product.

## THINGS TO NOTICE

The green circle around the drone is the camera's real footprint on the
deck plane. Watch it shrink as the drone descends. When it closes inside
the marker, vision can no longer help - that is the DECISION HEIGHT, and
the drone commits to the touchdown on the solution it already holds
rather than climbing away to re-acquire. Reading that empty frame as a
lost target instead produces an endless circuit, which is exactly the
defect this project found and fixed in the Python build.

## THINGS TO TRY

Turn ANTI-WINDUP? off and watch a stationary landing. The drone will
hover over the pad, measuring its offset correctly, and drift the wrong
way: the integral term saturated while closing in from one side, and
until it unwinds it is the largest term in the loop.

Turn FEED-FORWARD? off and try address 5, the circling rover. A PID
chasing a moving pad is a PID chasing a ramp, and it keeps a standing
lag that never enters the landing tolerance.

Turn TRACKER-COORDINATED-TURN? off and compare address 5 against
addresses 3 and 7. The turn model is what address 5 needs; 3 and 7 shuttle
in a straight line and do not.

## HOW IT DIFFERS FROM THE PYTHON BUILD

NetLogo renders no camera, so the DETECTOR is modelled rather than run:
a tag decodes when its apparent edge exceeds 20 px and its quiet zone
fits inside the field of view - the same two inequalities the Python
build asserts - with noise calibrated to the error that build measures
against MuJoCo ground truth. Everything downstream of the measurement is
the same algorithm.

Control runs at 50 Hz here against 100 Hz there, over direct rigid-body
integration rather than MuJoCo. Every gain is either rate-normalised or
expressed as a time constant, so the algorithm is unchanged.

@#$#@#$#@
default
true
0
Polygon -7500403 true true 150 5 40 250 150 205 260 250

circle
false
0
Circle -7500403 true true 0 0 300

circle 2
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240

square
false
0
Rectangle -7500403 true true 30 30 270 270
@#$#@#$#@
NetLogo 6.4.0
@#$#@#$#@

@#$#@#$#@

@#$#@#$#@
<experiments>
  <experiment name="validate" repetitions="1" runMetricsEveryStep="false">
    <setup>setup</setup>
    <go>go</go>
    <timeLimit steps="3000"/>
    <exitCondition>member? mission-state (list "LANDED" "ABORTED")</exitCondition>
    <metric>mission-state</metric>
    <metric>true-offset-cm</metric>
    <metric>mission-time</metric>
    <metric>altitude-m</metric>
    <enumeratedValueSet variable="start-address">
      <value value="1"/>
      <value value="3"/>
      <value value="4"/>
      <value value="5"/>
      <value value="6"/>
      <value value="9"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="seed"><value value="1"/></enumeratedValueSet>
    <enumeratedValueSet variable="fixed-seed?"><value value="true"/></enumeratedValueSet>
    <enumeratedValueSet variable="wind?"><value value="true"/></enumeratedValueSet>
    <enumeratedValueSet variable="feed-forward?"><value value="true"/></enumeratedValueSet>
    <enumeratedValueSet variable="gain-schedule?"><value value="true"/></enumeratedValueSet>
    <enumeratedValueSet variable="anti-windup?"><value value="true"/></enumeratedValueSet>
    <enumeratedValueSet variable="tracker-coordinated-turn?"><value value="true"/></enumeratedValueSet>
  </experiment>
  <experiment name="camcheck" repetitions="1" runMetricsEveryStep="false">
    <setup>setup</setup>
    <go>go</go>
    <timeLimit steps="1"/>
    <metric>camera-geometry-table</metric>
    <enumeratedValueSet variable="start-address"><value value="4"/></enumeratedValueSet>
    <enumeratedValueSet variable="seed"><value value="1"/></enumeratedValueSet>
    <enumeratedValueSet variable="fixed-seed?"><value value="true"/></enumeratedValueSet>
    <enumeratedValueSet variable="wind?"><value value="true"/></enumeratedValueSet>
    <enumeratedValueSet variable="feed-forward?"><value value="true"/></enumeratedValueSet>
    <enumeratedValueSet variable="gain-schedule?"><value value="true"/></enumeratedValueSet>
    <enumeratedValueSet variable="anti-windup?"><value value="true"/></enumeratedValueSet>
    <enumeratedValueSet variable="tracker-coordinated-turn?"><value value="true"/></enumeratedValueSet>
  </experiment>
</experiments>
@#$#@#$#@

@#$#@#$#@
default
0.0
-0.2 0 0.0 1.0
0.0 1 1.0 0.0
0.2 0 0.0 1.0
link direction
true
0
Line -7500403 true 150 150 90 180
Line -7500403 true 150 150 210 180
@#$#@#$#@
0
@#$#@#$#@
