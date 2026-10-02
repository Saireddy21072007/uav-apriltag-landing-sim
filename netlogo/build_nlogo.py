"""
Assemble drone_airtag_landing.nlogo from the code parts and a generated
interface, then validate the structure.

A .nlogo file is twelve sections separated by a line of "@#$#@#$#@":
  1 code   2 interface   3 info   4 shapes   5 version   6 preview
  7 system dynamics   8 behaviorspace   9 hubnet   10 link shapes
  11 model settings   12 (empty)

Every widget block is a fixed number of lines, and a miscount is the one
way to produce a file NetLogo will not open. The widgets are therefore
emitted from templates and counted back on the way out.
"""
import io
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SEP = "@#$#@#$#@"
NL = chr(10)

WIDGET_LINES = {          # lines AFTER the type keyword
    "BUTTON": 15, "SLIDER": 13, "SWITCH": 9, "MONITOR": 9, "TEXTBOX": 8,
}


def button(x, y, w, h, label, code, forever=False):
    return "\n".join(["BUTTON", str(x), str(y), str(x + w), str(y + h),
                      label, code, "T" if forever else "NIL", "1", "T",
                      "OBSERVER", "NIL", "NIL", "NIL", "NIL", "1"])


def slider(x, y, w, h, name, lo, hi, val, step):
    return "\n".join(["SLIDER", str(x), str(y), str(x + w), str(y + h),
                      name, name, str(lo), str(hi), str(val), str(step),
                      "1", "NIL", "HORIZONTAL"])


def switch(x, y, w, h, name, on=True):
    return "\n".join(["SWITCH", str(x), str(y), str(x + w), str(y + h),
                      name, name, "0" if on else "1", "1", "-1000"])


def monitor(x, y, w, h, label, reporter, decimals=2, font=11):
    return "\n".join(["MONITOR", str(x), str(y), str(x + w), str(y + h),
                      label, reporter, str(decimals), "1", str(font)])


def textbox(x, y, w, h, text, font=11, colour=0):
    return "\n".join(["TEXTBOX", str(x), str(y), str(x + w), str(y + h),
                      text, str(font), str(colour), "1"])


def graphics_window():
    # left top right bottom, two deprecated -1s, patch size, shapes-on,
    # font size, three unused, wrapX, wrapY, unused, world bounds,
    # update mode, update mode, tick counter visible, label, frame rate
    return "\n".join(["GRAPHICS-WINDOW",
                      "265", "10", "1185", "705",
                      "-1", "-1", "16.0", "1", "10", "1", "1", "1",
                      "0", "0", "0", "1",
                      "-12", "30", "-23", "17",
                      "1", "1", "1", "ticks", "30.0"])


def plot(x, y, w, h, name, xlab, ylab, xmax, ymin, ymax, pens):
    lines = ["PLOT", str(x), str(y), str(x + w), str(y + h), name, xlab, ylab,
             "0.0", str(xmax), str(ymin), str(ymax), "true", "true", '"" ""',
             "PENS"]
    lines += pens
    return "\n".join(lines)


def interface():
    parts = [graphics_window()]

    parts.append(textbox(12, 10, 340, 22,
                         "AirTag-addressed autonomous landing", 14, 0))
    parts.append(textbox(12, 34, 340, 34,
                         "Press a number to land there. Pressing one in "
                         "flight is a DIVERT, not a restart.", 10, 0))

    parts.append(button(12, 72, 80, 32, "setup", "setup"))
    parts.append(button(100, 72, 80, 32, "go", "go", forever=True))
    parts.append(button(188, 72, 80, 32, "step", "go"))

    # the ten addresses, two columns of five
    names = ["1 street", "2 loading", "3 van", "4 clinic", "5 rover",
             "6 roof 2F", "7 cart", "8 roof 3F", "9 tower 5F", "10 kerb"]
    for i in range(10):
        col, row = i % 2, i // 2
        x = 12 + col * 122
        y = 116 + row * 38
        parts.append(button(x, y, 114, 32, names[i], "fly-to-%d" % (i + 1)))

    parts.append(slider(12, 312, 236, 33, "start-address", 1, 10, 1, 1))
    parts.append(slider(12, 350, 236, 33, "seed", 0, 50, 1, 1))
    parts.append(switch(12, 390, 114, 33, "fixed-seed?", True))
    parts.append(switch(134, 390, 114, 33, "wind?", True))
    parts.append(switch(12, 428, 236, 33, "feed-forward?", True))
    parts.append(switch(12, 466, 236, 33, "gain-schedule?", True))
    parts.append(switch(12, 504, 236, 33, "anti-windup?", True))
    parts.append(switch(12, 542, 236, 33, "tracker-coordinated-turn?", True))

    parts.append(monitor(12, 586, 116, 45, "state", "state-text", 0, 11))
    parts.append(monitor(132, 586, 116, 45, "address", "address-text", 0, 11))
    parts.append(monitor(12, 636, 116, 45, "alt (m)", "altitude-m", 2, 11))
    parts.append(monitor(132, 636, 116, 45, "above deck (m)", "agl-m", 2, 11))
    parts.append(monitor(12, 686, 116, 45, "offset (m)", "offset-m", 3, 11))
    parts.append(monitor(132, 686, 116, 45, "true miss (cm)",
                         "true-offset-cm", 1, 11))
    parts.append(monitor(12, 736, 116, 45, "decision h (m)",
                         "decision-height-m", 3, 11))
    parts.append(monitor(132, 736, 116, 45, "tilt (deg)", "tilt-d", 1, 11))
    parts.append(monitor(12, 786, 236, 45, "marker", "lock-text", 0, 11))
    parts.append(monitor(12, 836, 116, 45, "mission t (s)",
                         "mission-time", 1, 11))
    parts.append(monitor(132, 836, 116, 45, "pad speed (m/s)",
                         "pad-speed-now", 2, 11))

    parts.append(plot(256, 586, 340, 145, "offset to pad centre", "time (s)",
                      "m", 60.0, 0.0, 1.0,
                      ['"measured" 1.0 0 -13345367 true "" "plot offset-m"',
                       '"true" 1.0 0 -2674135 true "" "plot true-off"']))
    parts.append(plot(256, 736, 340, 145, "height above the deck", "time (s)",
                      "m", 60.0, 0.0, 12.0,
                      ['"agl" 1.0 0 -16777216 true "" "plot agl-m"',
                       '"decision height" 1.0 0 -955883 true "" '
                       '"plot decision-height-m"']))
    return "\n\n".join(parts)


INFO = """## WHAT IS IT?

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
"""

BEHAVIORSPACE = """<experiments>
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
</experiments>"""


SHAPES = """default
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
Rectangle -7500403 true true 30 30 270 270"""

LINK_SHAPES = """default
0.0
-0.2 0 0.0 1.0
0.0 1 1.0 0.0
0.2 0 0.0 1.0
link direction
true
0
Line -7500403 true 150 150 90 180
Line -7500403 true 150 150 210 180"""


def build():
    code = []
    for part in ("code_part1.nls", "code_part2.nls", "code_part3.nls",
                 "code_part4.nls", "code_part5.nls"):
        with io.open(os.path.join(HERE, part), encoding="utf-8") as f:
            code.append(f.read().rstrip())
    sections = [
        "\n\n".join(code),      # 1 code
        interface(),            # 2 interface
        INFO,                   # 3 info
        SHAPES,                 # 4 turtle shapes
        "NetLogo 6.4.0",        # 5 version
        "",                     # 6 preview commands
        "",                     # 7 system dynamics
        BEHAVIORSPACE,          # 8 behaviorspace
        "",                     # 9 hubnet
        LINK_SHAPES,            # 10 link shapes
        "0",                    # 11 model settings
        "",                     # 12 trailing
    ]
    out = ("\n" + SEP + "\n").join(sections)
    path = os.path.join(HERE, "drone_airtag_landing.nlogo")
    with io.open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(out)
    return path, out


def validate(text):
    problems = []
    parts = text.split(SEP)
    if len(parts) != 12:
        problems.append("expected 12 sections, found %d" % len(parts))

    # every widget block must have exactly the right number of lines
    lines = parts[1].strip().split("\n")
    i = 0
    widgets = 0
    while i < len(lines):
        kind = lines[i].strip()
        if not kind:
            i += 1
            continue
        if kind in WIDGET_LINES:
            need = WIDGET_LINES[kind]
            block = [l for l in lines[i + 1:i + 1 + need]]
            if len(block) < need:
                problems.append("%s at line %d is truncated" % (kind, i))
            i += need + 1
            widgets += 1
        elif kind == "GRAPHICS-WINDOW":
            i += 26
            widgets += 1
        elif kind == "PLOT":
            j = i + 1
            while j < len(lines) and lines[j].strip() != "PENS":
                j += 1
            k = j + 1
            while k < len(lines) and lines[k].strip().startswith('"'):
                k += 1
            i = k
            widgets += 1
        else:
            problems.append("unknown widget %r at line %d" % (kind, i))
            i += 1

    # A `let` may not shadow a global, an own-variable or a primitive.
    # NetLogo rejects the model outright: "There is already a PADS-OWN
    # variable called DECK". Nothing but the real compiler catches it, so
    # it is checked here too.
    code = parts[0]
    nocomment = NL.join(l.split(";")[0] for l in code.split(NL))
    declared = set()
    g = re.search(r"globals\s*\[(.*?)\]", nocomment, re.S)
    if g:
        declared |= set(re.findall(r"[A-Za-z][\w?\-]*", g.group(1)))
    for own in re.finditer(r"(\w+)-own\s*\[(.*?)\]", nocomment, re.S):
        declared |= set(re.findall(r"[A-Za-z][\w?\-]*", own.group(2)))
    PRIMS = {"dx", "dy", "e", "pi", "who", "color", "size", "shape", "label",
             "heading", "xcor", "ycor", "breed", "ticks", "timer", "pcolor",
             "pxcor", "pycor", "end", "self", "myself", "nobody"}
    for m in re.finditer(r"let\s+([A-Za-z][\w?\-]*)", nocomment):
        name = m.group(1)
        if name in declared:
            problems.append("`let %s` shadows a declared variable" % name)
        elif name in PRIMS:
            problems.append("`let %s` shadows a NetLogo primitive" % name)

    code = parts[0]
    stripped = "\n".join(l.split(";")[0] for l in code.split("\n"))
    if stripped.count("[") != stripped.count("]"):
        problems.append("unbalanced [ ] in code: %d vs %d"
                        % (stripped.count("["), stripped.count("]")))
    if stripped.count("(") != stripped.count(")"):
        problems.append("unbalanced ( ) in code: %d vs %d"
                        % (stripped.count("("), stripped.count(")")))
    opens = len([l for l in stripped.split("\n")
                 if l.strip().startswith("to ") or l.strip().startswith("to-report ")])
    closes = len([l for l in stripped.split("\n") if l.strip() == "end"])
    inline = len([l for l in stripped.split("\n")
                  if (l.strip().startswith("to ") or l.strip().startswith("to-report "))
                  and l.strip().endswith(" end")])
    if opens != closes + inline:
        problems.append("to/end mismatch: %d procedures, %d end, %d inline"
                        % (opens, closes, inline))
    return widgets, problems


if __name__ == "__main__":
    path, text = build()
    widgets, problems = validate(text)
    print("  wrote %s  (%.1f KB)" % (os.path.basename(path), len(text) / 1024))
    print("  sections: %d   widgets: %d" % (len(text.split(SEP)), widgets))
    if problems:
        print("  PROBLEMS:")
        for p in problems:
            print("    - %s" % p)
        sys.exit(1)
    print("  structure OK")
