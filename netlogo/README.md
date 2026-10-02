# NetLogo implementation

The same methodology as `../drone_sim` (Python + MuJoCo) and `../matlab`,
on an agent-based platform.

## Just open it

**Double-click `run2.bat`.** That is the whole procedure. NetLogo opens,
the model loads, and the world builds itself. Then:

1. press **GO**
2. press any address number, **1** to **10**

Orange = street, blue = vehicle, green = roof. Press another number while
it is flying and the drone diverts in place.

NetLogo itself is at `%LOCALAPPDATA%\NetLogo\NetLogo-6.4.0-64`, a folder
Windows hides - which is why you could not find it by browsing. The
launcher finds it for you. Nothing was installed system-wide: no admin,
no registry, no Start-menu entry. Deleting that folder removes NetLogo.

## What you are looking at

The world is split in two, the same way the MuJoCo build's panel is.

**Left - the street, from above.** Ten addressed pads, three of them on
vehicles driving their own routes, three on buildings. The circle around
the drone is the camera's footprint on the surface the pad sits on; it is
green while the marker is decoding and red when it is not.

**Right - the drone's own camera.** Every patch in that panel is a PIXEL.
For each one the model turns the pixel into a ray in the camera frame,
rotates it into the world with the drone's actual attitude quaternion,
intersects it with the surface the pad sits on, and colours it by what is
painted at the point it hits. That is ray casting, through the same
pinhole model the perception code uses - `f = (h/2)/tan(fovy/2)`, camera
looking along -body z. The panel is not a drawing of what the drone sees.
It is what the drone sees.

Watch it descend. High up both markers sit small in the middle of the
frame. Lower, the large marker slides to the edge and is cut off - that
is the hand-over to the small one. Lower still the small marker outgrows
the frame too, and the height where that happens is the **decision
height**: vision has nothing left to give, so the drone commits on the
solution it already holds instead of climbing away to re-acquire.

That last part is measurable, not just visible. The `camcheck` experiment
parks the drone over a pad at known heights and counts pixels:

| height above deck | marker pixels | pad pixels (of 256) |
|---|---|---|
| 8.00 m | 0 | 1 |
| 2.00 m | 3 | 52 |
| 1.00 m | 31 | 188 |
| 0.60 m | 52 | 199 |
| 0.45 m | 43 | 208 |
| **0.20 m** | **219** | 32 |

The marker goes from invisible to flooding the frame, and it crosses over
at about 0.25 m - which is exactly what `blind-altitude` predicts from the
marker size, the field of view and the camera's mounting offset.

## Verified

NetLogo 6.4.0 was installed and the model run headless. Six addresses,
one seed, covering all three kinds of landing site:

| address | kind | miss | time | landed at |
|---|---|---|---|---|
| 1 A-01 Street drop point | ground | 3.6 cm | 32.5 s | 0.21 m |
| 3 B-03 Courier van | vehicle | 2.7 cm | 40.5 s | 0.55 m |
| 4 B-04 Clinic entrance | ground | 2.0 cm | 25.7 s | 0.21 m |
| 5 C-05 Service rover | vehicle, circling | 7.3 cm | 30.9 s | 0.55 m |
| 6 C-06 Apartment roof, 2F | roof 3.2 m | 0.9 cm | 35.8 s | 3.43 m |
| 9 E-09 Tower roof, 5F | roof 7.5 m | 1.7 cm | 37.4 s | 7.73 m |

**6/6 landed, mean 3.0 cm.** The touchdown altitudes are the check that
matters: 7.73 m is the 7.52 m tower roof plus 0.21 m of skid, 3.43 m is
the 3.22 m apartment roof, 0.55 m is the 0.34 m vehicle deck. The drone
landed *on* the surfaces it was addressed to, not through them.

Three platforms, three independent implementations, same answer - and all
three agree that address 5, the only pad that drives a circle, is hardest:

| | mean | static | moving | worst address |
|---|---|---|---|---|
| Python + MuJoCo | 2.88 cm | 1.20 | 6.82 | 5 |
| MATLAB | 3.48 cm | 1.84 | 7.30 | 5 |
| NetLogo | 3.03 cm | 2.05 | 5.00 | 5 |

### Run it yourself, without the GUI

```
run_headless.bat              # the "validate" experiment
run_headless.bat myexp        # any other BehaviorSpace experiment
```

Results land in `netlogo_results.csv`. Set `NETLOGO_HOME` if NetLogo is
installed somewhere other than `%LOCALAPPDATA%\NetLogo\NetLogo-6.4.0-64`.

### Two bugs the headless run caught

Both would have stopped the model dead on opening, and neither was
visible to a structural check:

- **`dx` and `dy` are NetLogo primitives** (a turtle's heading
  increments). Using them as local names is not a warning, it is a
  compile error: *"There is already a primitive reporter called DX"*.
- **`e` is a NetLogo constant** (2.718...). The attitude error vector was
  called `e`, which shadowed it.

Renamed to `off-x`, `off-y` and `erot`. Worth knowing before writing any
NetLogo of your own.

## The interface

**Ten address buttons.** Orange = street, blue = vehicle, green = roof.
Press one in flight and the drone **diverts** — the vehicles keep driving
and it keeps its momentum. It is not a restart.

**Five switches**, so the ablation is something you can do live rather
than read about:

| switch | off means |
|---|---|
| `anti-windup?` | the integral keeps a stale charge — watch a stationary landing drift the wrong way |
| `feed-forward?` | no pad-velocity feed-forward — the moving addresses become unlandable |
| `gain-schedule?` | one gain at every altitude |
| `tracker-coordinated-turn?` | constant-acceleration tracking only — costs address 5 most |
| `wind?` | no wind at all |

**The green circle** around the drone is the camera's real footprint on
the deck plane. Watch it shrink during the descent. When it closes inside
the marker, vision can no longer help — that is the **decision height**,
plotted live against the drone's height above the deck in the lower plot.

## Why an agent-based platform at all

The addressing layer is the part that benefits. Ten pads, three of them
on vehicles driving their own routes, each carrying its own markers and
its own radio beacon, and a drone that must accept exactly one of them —
that is naturally a multi-agent model, and NetLogo makes the vehicles,
the pads and the drone first-class agents rather than rows in an array.

## What is modelled rather than run

NetLogo renders no camera, so the **detector is modelled**, exactly as in
the MATLAB build. A tag decodes when:

1. it is big enough in the image — `f · size / range ≥ 20 px`
2. it **fits** in the frame — `offset + quiet zone ≤ (agl − camdrop) · h/2f`

These are the two inequalities the Python build asserts in
`config.check_geometry()`; the noise is calibrated to what that build
measures against MuJoCo ground truth. Everything downstream of the
measurement is the same algorithm.

Control runs at 50 Hz here against 100 Hz in Python. Every gain is either
rate-normalised or expressed as a time constant, so the algorithm is
unchanged.

## Quaternions

`code_part1.nls` is a complete quaternion library in NetLogo: Hamilton
product, conjugate, body-z extraction, rotation vector with shortest-arc
sign correction, SLERP, and Shepperd's axes→quaternion construction.

Two NetLogo-specific traps worth knowing about, both handled in the code:

- **`atan` is not `atan2`.** NetLogo's `atan x y` reports a *heading* —
  degrees clockwise from the positive y axis. Standard `atan2` measures
  counter-clockwise from the positive x axis. Mixing the two silently
  mirrors every rotation. The conversion is done once, in `atan2-r`.
- **All trigonometry is in degrees.** The physics is in radians, so
  `sin-r` / `cos-r` wrap the conversion rather than scattering
  `* 180 / pi` through the control law.

## Files

| file | what it is |
|---|---|
| `drone_airtag_landing.nlogo` | the model — open this |
| `code_part1.nls` | header, globals, quaternion algebra |
| `code_part2.nls` | constants, address book, pad motion, the modelled detector |
| `code_part3.nls` | tracker, guidance, inner loop, physics |
| `code_part4.nls` | mission state machine, setup, go, display, reporters |
| `code_part5.nls` | the ray-cast onboard camera panel |
| `build_nlogo.py` | reassembles and revalidates the `.nlogo` from the parts |
| `run2.bat` | **open this** - starts NetLogo with the model loaded |
| `OPEN_NETLOGO.bat` | the same thing, with more diagnostics |
| `run_headless.bat` | runs the model with no GUI, results to CSV |
