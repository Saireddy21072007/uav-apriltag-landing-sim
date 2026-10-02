# MATLAB implementation

The same methodology as `../drone_sim` (Python + MuJoCo), on a second
platform. Same base paper, same addressing layer, same dual-scale marker,
same eq. (9) PID with the same five changes, same quaternion attitude
control, same decision height.

Tested on MATLAB R2024a. No toolboxes required — base MATLAB only.

## Run it

```matlab
run_batch                       % all ten addresses x four seeds, prints a table
run_batch(0:1)                  % quicker: two seeds
run_demo3d                      % 3D view + onboard camera  <- the good one
run_demo3d([5], 0)              % just the circling rover
run_demo3d([6 3 9], 1, 'x.gif') % ...and save the animation
run_demo                        % simpler top-down 2D view
run_ablation                    % what each addition to the paper's PID buys
res = simulate_landing(5, 0);   % one mission, one address, one seed
```

## Files

| file | what it is |
|---|---|
| `droneconfig.m` | every constant and the ten-address book — the single source of truth |
| `quatlib.m` | quaternion algebra: product, conjugate, rotation vector, SLERP, axes→quaternion |
| `simulate_landing.m` | one mission: perception, tracker, guidance, inner loop, physics, state machine |
| `run_batch.m` | Monte Carlo over every address |
| `run_demo3d.m` | 3D world view plus a synthetic onboard camera — closest to MuJoCo |
| `run_demo.m` | simpler top-down 2D animation |
| `run_ablation.m` | the controller ladder, on moving and on stationary addresses |

## Results

Ten addresses × two seeds:

```
    1  A-01  Street drop point    ground  2/2   1.05 cm   32.3 s
    2  A-02  Loading bay          ground  2/2   2.13 cm   28.4 s
    3  B-03  Courier van          vehicle 2/2   3.31 cm   30.4 s
    4  B-04  Clinic entrance      ground  2/2   1.80 cm   29.1 s
    5  C-05  Service rover        vehicle 2/2  16.38 cm   29.8 s
    6  C-06  Apartment roof, 2F   roof    2/2   1.72 cm   32.2 s
    7  D-07  Utility cart         vehicle 2/2   2.22 cm   39.0 s
    8  D-08  Apartment roof, 3F   roof    2/2   3.67 cm   38.0 s
    9  E-09  Tower roof, 5F       roof    2/2   1.38 cm   39.2 s
   10  E-10  Kerbside locker      ground  2/2   1.09 cm   27.7 s

  20/20 landed (100.0 %)   mean 3.48 cm   static 1.84 cm   moving 7.30 cm
```

Against the Python/MuJoCo build: **2.88 cm mean, 1.20 static, 6.82 moving**.
Two independent implementations, different languages, different physics,
agreeing to within about a centimetre — and agreeing on *which* address is
hardest (address 5, the only pad that drives a circle).

## The ablation, reproduced independently

```
  MOVING addresses  (can it land at all?)
     paper PID only                 0/ 6 landed       -          -
     + integral anti-windup         0/ 6 landed       -          -
     + pad-velocity feed-forward    5/ 6 landed   12.29 cm   46.3 s
     + altitude gain scheduling     6/ 6 landed    6.89 cm   32.5 s
     + coordinated-turn tracker     6/ 6 landed    7.30 cm   33.1 s

  STATIONARY addresses  (how accurately?)
     paper PID only                 6/ 6 landed   12.63 cm   30.8 s
     + integral anti-windup         6/ 6 landed    2.37 cm   30.3 s
     + pad-velocity feed-forward    6/ 6 landed    1.95 cm   30.3 s
     + altitude gain scheduling     6/ 6 landed    1.57 cm   30.3 s
     + coordinated-turn tracker     6/ 6 landed    1.66 cm   29.9 s
```

The Python build measures the paper's PID alone at **12.5 cm** on
stationary addresses; MATLAB gets **12.63 cm** from an independent
implementation. Anti-windup is the dominant correction on both platforms,
and on both the paper's fixed-gain PID lands **none** of the moving
addresses.

One honest disagreement: the coordinated-turn tracker helps on the Python
build (6.5 → 4.6 cm on moving addresses, validated on hold-out seeds) and
very slightly *hurts* here (6.89 → 7.30 cm). Six missions is far too few
to call either way — it is reported rather than tidied away.

## The 3D view

`run_demo3d` is the one to show. Two panels, side by side:

**Left — the world in 3D.** Buildings raised from the address book's roof
heights, pads with their two markers drawn to scale, and the drone as
four rotor arms rotated by the *actual attitude quaternion* the
controller is flying — when it tilts to accelerate, you see it tilt,
because on a quadrotor lateral motion is bought with attitude. The yellow
cone is the camera's field of view projected along the camera's own axis
onto the surface the pad sits on; it swings when the drone tilts, because
the camera is bolted to the airframe. The footprint outline turns red
when the marker is not decoding.

The drone is drawn at four times life size (`DRONE_SCALE`). It is the only
thing in the view that is not to scale — at 0.52 m across it would be two
pixels. The attitude it is drawn at is the real one.

**Right — the onboard camera.** The pad and both markers projected through
the same pinhole model the perception code uses: world point into the body
frame, camera looking along −body z, focal length `f = (h/2)/tan(fovy/2)`.

That panel is where the method explains itself. High up, both markers sit
small in the middle of the frame. As the drone descends the large marker
drifts toward the edge and is cut off — that is the hand-over to the small
one. Lower still, the small marker outgrows the frame too, and that height
is the **decision height**: vision has nothing left to give, and the drone
commits on the solution it holds. Watching the `LOCK` indicator change to
`COMMITTED — marker wider than frame` is the whole argument in one frame.

## What is modelled rather than run

MATLAB renders no camera here, so the **detector is modelled**. A tag
decodes when both of these hold:

1. it is big enough in the image — `f · size / range ≥ 20 px`
2. it **fits** in the frame — `offset + quiet zone ≤ (agl − camdrop) · h/2f`

These are exactly the two inequalities the Python build asserts in
`config.check_geometry()`, and the measurement noise is calibrated to what
that build measures against MuJoCo ground truth (7.15 cm mean over 90
random poses; 9.02 cm on the large tag, 3.99 cm on the small one).

The second inequality is not a simplification — it is the point. A
downward camera cannot see its own pad all the way to the ground, because
the marker outgrows the frame, and the height where that happens is the
decision height the whole landing turns on.

Everything downstream of the measurement — tracker, guidance, attitude
control, state machine — is the same algorithm as the Python build.

Control runs at 100 Hz over direct rigid-body integration rather than
MuJoCo's 500 Hz contact solver. Ground contact is a simple surface
constraint instead of a full contact model, which is why touchdown
speeds are not directly comparable between the two builds.
