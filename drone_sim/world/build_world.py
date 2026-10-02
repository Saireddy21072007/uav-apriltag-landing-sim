"""
Generates the MuJoCo world and the pad textures from config.py.

    python world/build_world.py

Two artefacts come out of this:

  assets/pad_<id>.png   one texture per landing pad, carrying REAL tag36h11
                        markers at two scales (large id = k, small id = k+10)
  assets/world.xml      the scene: urban street, ten pads, five of them on
                        vehicles, and the quadrotor

The point of generating rather than hand-writing the XML is that config.PADS is
the only place a pad position or a tag id is written down. The physics world,
the perception code and the mission logic cannot drift apart.
"""
import hashlib
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config as C

HERE = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.join(HERE, "assets")
TEX_PX = 1024                      # texture resolution, px per pad edge

_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)


# --------------------------------------------------------------- textures
def _tag_image(tag_id, px):
    """
    A tag36h11 marker with the mandatory white quiet zone around it.

    The marker is rotated by 180 degrees so that the tag's own coordinate frame
    (which solvePnP recovers) lines up with the pad frame. A tag decodes to the
    same id at any rotation, so this costs nothing - but without it the pose of
    the large tag comes back yawed by 180 degrees, and the correction from tag
    position to pad centre is then applied in exactly the wrong direction.
    """
    core = cv2.aruco.generateImageMarker(_DICT, tag_id, px)
    core = cv2.rotate(core, cv2.ROTATE_180)
    q = max(2, px // 8)            # quiet zone, at least one module wide
    return cv2.copyMakeBorder(core, q, q, q, q, cv2.BORDER_CONSTANT, value=255)


def _paste(canvas, patch, cx_px, cy_px):
    h, w = patch.shape[:2]
    x0, y0 = int(cx_px - w / 2), int(cy_px - h / 2)
    canvas[y0:y0 + h, x0:x0 + w] = patch if patch.ndim == 3 else \
        cv2.cvtColor(patch, cv2.COLOR_GRAY2BGR)


def make_pad_texture(pad_id, path):
    """
    One pad, seen from above, drawn into a square texture.

    Pad frame: +x_pad points right in the texture, +y_pad points UP in the
    texture.

    Two texture-mapping facts, both learned the hard way:

      * Do NOT flip the image. A vertical flip mirrors the marker, and a
        mirrored tag36h11 is undecodable - the detector just returns nothing,
        with no error to say why.
      * MuJoCo lays a 2d texture onto the top face rotated by 180 degrees, so
        the image is rotated by 180 here to compensate. That is a rotation, not
        a mirror, so the tags still decode - but without it the large tag ends
        up on the opposite side of the pad from where config says it is, and
        every pad-centre correction comes out with twice the offset, in the
        wrong direction.
    """
    m2px = TEX_PX / C.PAD_SIZE
    img = np.full((TEX_PX, TEX_PX, 3), 245, np.uint8)

    # deck markings: border, corner ticks, address number
    cv2.rectangle(img, (0, 0), (TEX_PX - 1, TEX_PX - 1), (28, 150, 220), 26)
    cv2.rectangle(img, (40, 40), (TEX_PX - 41, TEX_PX - 41), (170, 170, 170), 3)
    for cx, cy in ((60, 60), (TEX_PX - 60, 60), (60, TEX_PX - 60),
                   (TEX_PX - 60, TEX_PX - 60)):
        cv2.line(img, (cx - 34, cy), (cx + 34, cy), (28, 150, 220), 6)
        cv2.line(img, (cx, cy - 34), (cx, cy + 34), (28, 150, 220), 6)

    # large tag, offset from the pad centre along +x_pad
    big_px = int(C.TAG_LARGE * m2px)
    big = _tag_image(pad_id, big_px)
    _paste(img, big, TEX_PX / 2 + C.TAG_LARGE_OFFSET * m2px, TEX_PX / 2)

    # small tag, exactly on the pad centre - this is the one that survives to
    # touchdown height, where the camera sees barely 20 cm of ground
    small_px = int(C.TAG_SMALL * m2px)
    small = _tag_image(pad_id + C.TAG_ID_SMALL_OFFSET, small_px)
    _paste(img, small, TEX_PX / 2, TEX_PX / 2)

    # human-readable address, kept clear of both tags
    label = C.pad(pad_id)["address"].split("  ")[0]
    cv2.putText(img, label, (58, TEX_PX - 74), cv2.FONT_HERSHEY_SIMPLEX,
                1.5, (60, 60, 60), 4, cv2.LINE_AA)
    cv2.putText(img, "%d" % pad_id, (TEX_PX - 210, TEX_PX - 74),
                cv2.FONT_HERSHEY_SIMPLEX, 2.6, (28, 150, 220), 7, cv2.LINE_AA)

    cv2.imwrite(path, img)
    return path


# -------------------------------------------------------------------- XML
def _pad_body(p, on_vehicle):
    """
    XML for one landing pad: on the street, on a roof, or on a vehicle.

    A roof pad also emits the building that holds it up, sized from the pad's
    own height. Generating the building from the pad rather than placing both by
    hand means the scene and the address book cannot disagree about how high a
    roof is - the same rule the rest of this file follows.
    """
    x, y = p["home"]
    pid = p["id"]
    half = C.PAD_SIZE / 2.0
    height = C.pad_height(p)

    if p.get("kind") == "roof":
        bh = height / 2.0
        bw = half + 0.9              # the roof overhangs the pad on every side
        return f"""
    <body name="block_{pid}" pos="{x:.3f} {y:.3f} 0.0">
      <geom name="block_{pid}_wall" type="box" size="{bw:.3f} {bw:.3f} {bh:.3f}"
            pos="0 0 {bh:.3f}" material="apartment_mat" contype="1" conaffinity="1"/>
      <geom name="block_{pid}_parapet_n" type="box" size="{bw:.3f} 0.08 0.16"
            pos="0 {bw - 0.08:.3f} {height + 0.16:.3f}" material="parapet_mat"/>
      <geom name="block_{pid}_parapet_s" type="box" size="{bw:.3f} 0.08 0.16"
            pos="0 {-(bw - 0.08):.3f} {height + 0.16:.3f}" material="parapet_mat"/>
      <geom name="block_{pid}_parapet_e" type="box" size="0.08 {bw:.3f} 0.16"
            pos="{bw - 0.08:.3f} 0 {height + 0.16:.3f}" material="parapet_mat"/>
      <geom name="block_{pid}_parapet_w" type="box" size="0.08 {bw:.3f} 0.16"
            pos="{-(bw - 0.08):.3f} 0 {height + 0.16:.3f}" material="parapet_mat"/>
      <geom name="pad_{pid}_deck" type="box" size="{half:.3f} {half:.3f} 0.010"
            pos="0 0 {height + 0.012:.3f}"
            material="pad_mat_{pid}" contype="1" conaffinity="1" friction="1.2 0.05 0.005"/>
    </body>"""

    if not on_vehicle:
        return f"""
    <body name="pad_{pid}" pos="{x:.3f} {y:.3f} 0.012">
      <geom name="pad_{pid}_deck" type="box" size="{half:.3f} {half:.3f} 0.010"
            material="pad_mat_{pid}" contype="1" conaffinity="1" friction="1.2 0.05 0.005"/>
    </body>"""
    # Moving pads ride a mocap body: the vehicle is driven kinematically by the
    # simulator, but still collides properly with the drone's legs.
    #
    # The cab sits clear of the deck footprint on purpose. A cab overlapping the
    # deck is not a cosmetic problem: it stands proud of the landing surface, so
    # the drone touches down on the cab roof instead of the pad and the landing
    # detector never fires.
    return f"""
    <body name="veh_{pid}" mocap="true" pos="{x:.3f} {y:.3f} 0.0">
      <geom name="veh_{pid}_bed" type="box" size="1.30 0.60 0.16" pos="0 0 0.16"
            material="veh_body_mat" contype="1" conaffinity="1"/>
      <geom name="veh_{pid}_cab" type="box" size="0.34 0.55 0.26" pos="-1.18 0 0.58"
            material="veh_glass" contype="1" conaffinity="1"/>
      <geom name="veh_{pid}_wfl" type="cylinder" size="0.15 0.06" pos="0.85 0.62 0.15"
            euler="1.5708 0 0" material="tyre_mat"/>
      <geom name="veh_{pid}_wfr" type="cylinder" size="0.15 0.06" pos="0.85 -0.62 0.15"
            euler="1.5708 0 0" material="tyre_mat"/>
      <geom name="veh_{pid}_wrl" type="cylinder" size="0.15 0.06" pos="-0.95 0.62 0.15"
            euler="1.5708 0 0" material="tyre_mat"/>
      <geom name="veh_{pid}_wrr" type="cylinder" size="0.15 0.06" pos="-0.95 -0.62 0.15"
            euler="1.5708 0 0" material="tyre_mat"/>
      <geom name="pad_{pid}_deck" type="box" size="{half:.3f} {half:.3f} 0.010" pos="0 0 0.33"
            material="pad_mat_{pid}" contype="1" conaffinity="1" friction="1.2 0.05 0.005"/>
    </body>"""


def build_xml(path):
    pad_assets = "\n".join(
        f"""    <texture name="pad_tex_{p['id']}" type="2d" file="pad_{p['id']}.png"/>
    <material name="pad_mat_{p['id']}" texture="pad_tex_{p['id']}" texrepeat="1 1"
              shininess="0.05" specular="0.05" reflectance="0.0"/>"""
        for p in C.PADS)

    bodies = "\n".join(_pad_body(p, p["motion"] != "static") for p in C.PADS)

    hx, hy = C.HOME_XY
    cx, cy, cz = C.CAM_OFFSET_BODY
    ixx, iyy, izz = C.INERTIA

    xml = f"""<mujoco model="airtag_addressed_landing">
  <!-- GENERATED BY world/build_world.py - edit config.py, not this file -->

  <compiler angle="radian" autolimits="true" balanceinertia="true" meshdir="." texturedir="."/>
  <option gravity="0 0 -{C.GRAVITY}" timestep="{C.DT_SIM}" integrator="RK4" density="1.2" viscosity="0.0"/>

  <default>
    <joint damping="0.002"/>
    <geom condim="4" friction="1.0 0.05 0.005" solref="0.012 1" solimp="0.9 0.95 0.001"/>
  </default>

  <asset>
    <texture name="sky" type="skybox" builtin="gradient"
             rgb1="0.52 0.62 0.75" rgb2="0.16 0.20 0.28" width="512" height="512"/>
    <texture name="street_tex" type="2d" file="street_tex.png"/>
    <material name="street_mat" texture="street_tex" texrepeat="1 14" shininess="0.05" specular="0.03"/>
    <material name="asphalt_mat" rgba="0.30 0.30 0.31 1" shininess="0.02"/>
    <material name="pavement_mat" rgba="0.58 0.56 0.52 1" shininess="0.04"/>
    <texture name="brick_tex" type="2d" file="brick_tex.png"/>
    <material name="brick_mat" texture="brick_tex" texrepeat="3 8" shininess="0.03"/>
    <texture name="stucco_tex" type="2d" file="stucco_tex.png"/>
    <material name="stucco_mat" texture="stucco_tex" texrepeat="3 8" shininess="0.03"/>

    <material name="roof_sheet" rgba="0.42 0.40 0.38 1" shininess="0.2"/>
    <material name="apartment_mat" rgba="0.72 0.66 0.56 1" shininess="0.05"/>
    <material name="parapet_mat" rgba="0.52 0.48 0.42 1" shininess="0.05"/>
    <material name="kerb_mat" rgba="0.62 0.60 0.56 1"/>
    <material name="veh_body_mat" rgba="0.78 0.30 0.16 1" shininess="0.5"/>
    <material name="veh_glass" rgba="0.12 0.15 0.20 1" shininess="0.9" specular="0.7"/>
    <material name="tyre_mat" rgba="0.08 0.08 0.09 1"/>
    <material name="drone_shell" rgba="0.16 0.18 0.20 1" shininess="0.4" specular="0.3"/>
    <material name="drone_accent" rgba="0.10 0.11 0.12 1" shininess="0.5"/>
    <material name="motor_mat" rgba="0.35 0.38 0.42 1" shininess="0.8" specular="0.6"/>
    <material name="prop_mat" rgba="0.12 0.14 0.16 0.5"/>
    <material name="led_red" rgba="1.0 0.25 0.25 1" emission="1.0"/>
    <material name="led_green" rgba="0.25 1.0 0.45 1" emission="1.0"/>
    <material name="lens_mat" rgba="0.02 0.02 0.05 1" shininess="1.0" specular="0.9"/>
{pad_assets}
  </asset>

  <!-- MuJoCo scales the near/far clipping planes by stat.extent, so extent is
       pinned here: without it the near plane sits about a metre in front of the
       camera and the pad vanishes exactly when the drone is about to touch it. -->
  <statistic extent="10" center="0 0 2"/>

  <visual>
    <!-- the offscreen buffer has to be at least as large as any render we ask
         for, including the wide overview shots used in the report figures -->
    <global offwidth="1600" offheight="1200"/>
    <headlight ambient="0.62 0.62 0.62" diffuse="0.65 0.65 0.63" specular="0.12 0.12 0.12"/>
    <quality shadowsize="4096" offsamples="8"/>
    <map znear="0.004" zfar="15"/>
  </visual>

  <worldbody>
    <light name="sun" directional="true" pos="6 6 25" dir="-0.3 -0.4 -1"
           diffuse="0.65 0.64 0.60" specular="0.15 0.15 0.15" castshadow="true"/>

    <!-- Ground is plain asphalt; the road surface is a strip down the middle.
         Tiling the road texture over the whole plane produced a field of
         crosswalks, which is both wrong and visually confusing in the report. -->
    <geom name="ground" type="plane" size="60 60 0.1" material="asphalt_mat" pos="0 0 0"/>
    <geom name="roadway" type="box" size="2.9 32 0.006" pos="0 -3 0.006" material="street_mat"/>
    <geom name="pavement_l" type="box" size="1.6 32 0.09" pos="-4.75 -3 0.09" material="pavement_mat"/>
    <geom name="pavement_r" type="box" size="1.6 32 0.09" pos=" 4.75 -3 0.09" material="pavement_mat"/>
    <geom name="kerb_left"  type="box" size="0.22 32 0.10" pos="-3.1 -3 0.10" material="kerb_mat"/>
    <geom name="kerb_right" type="box" size="0.22 32 0.10" pos=" 3.1 -3 0.10" material="kerb_mat"/>

    <!-- background terraces. They stop short of the generated apartment
         blocks so that a roof pad is never buried inside a wall. -->
    <body name="terrace_left" pos="-8.0 6.0 0">
      <geom name="tl_wall" type="box" size="1.6 12.0 2.3" pos="0 0 2.3" material="brick_mat"/>
      <geom name="tl_roof" type="box" size="1.75 12.0 0.12" pos="0 0 4.7" material="roof_sheet"/>
    </body>
    <body name="terrace_right" pos="8.0 7.0 0">
      <geom name="tr_wall" type="box" size="1.6 10.0 2.6" pos="0 0 2.6" material="stucco_mat"/>
      <geom name="tr_roof" type="box" size="1.75 10.0 0.12" pos="0 0 5.3" material="roof_sheet"/>
    </body>

    <!-- ================= the ten addressed landing pads ================= -->
{bodies}

    <!-- ============================ the drone =========================== -->
    <body name="drone" pos="{hx:.3f} {hy:.3f} 0.28">
      <freejoint name="drone_free"/>
      <inertial pos="0 0 0" mass="{C.MASS}" diaginertia="{ixx} {iyy} {izz}"/>

      <geom name="fuselage" type="box" size="0.17 0.10 0.045" material="drone_shell"
            contype="1" conaffinity="1"/>
      <geom name="canopy" type="box" size="0.12 0.075 0.020" pos="0 0 0.055" material="drone_accent"/>

      <geom name="arm_fl" type="capsule" size="0.018" fromto="0.09 0.07 0 0.235 0.235 0" material="drone_shell"/>
      <geom name="arm_fr" type="capsule" size="0.018" fromto="0.09 -0.07 0 0.235 -0.235 0" material="drone_shell"/>
      <geom name="arm_rl" type="capsule" size="0.018" fromto="-0.09 0.07 0 -0.235 0.235 0" material="drone_shell"/>
      <geom name="arm_rr" type="capsule" size="0.018" fromto="-0.09 -0.07 0 -0.235 -0.235 0" material="drone_shell"/>

      <geom name="mot_fl" type="cylinder" size="0.030 0.028" pos="0.235 0.235 0.028" material="motor_mat"/>
      <geom name="mot_fr" type="cylinder" size="0.030 0.028" pos="0.235 -0.235 0.028" material="motor_mat"/>
      <geom name="mot_rl" type="cylinder" size="0.030 0.028" pos="-0.235 0.235 0.028" material="motor_mat"/>
      <geom name="mot_rr" type="cylinder" size="0.030 0.028" pos="-0.235 -0.235 0.028" material="motor_mat"/>

      <geom name="prop_fl" type="cylinder" size="0.150 0.004" pos="0.235 0.235 0.062" material="prop_mat" contype="0" conaffinity="0"/>
      <geom name="prop_fr" type="cylinder" size="0.150 0.004" pos="0.235 -0.235 0.062" material="prop_mat" contype="0" conaffinity="0"/>
      <geom name="prop_rl" type="cylinder" size="0.150 0.004" pos="-0.235 0.235 0.062" material="prop_mat" contype="0" conaffinity="0"/>
      <geom name="prop_rr" type="cylinder" size="0.150 0.004" pos="-0.235 -0.235 0.062" material="prop_mat" contype="0" conaffinity="0"/>

      <geom name="led_front" type="sphere" size="0.016" pos="0.19 0 -0.01" material="led_green"/>
      <geom name="led_rear" type="sphere" size="0.016" pos="-0.19 0 -0.01" material="led_red"/>

      <!-- landing gear: four legs, these are what actually touch the pad -->
      <geom name="leg_fl" type="capsule" size="0.011" fromto="0.14 0.11 -0.045 0.18 0.15 -0.20" material="drone_accent"/>
      <geom name="leg_fr" type="capsule" size="0.011" fromto="0.14 -0.11 -0.045 0.18 -0.15 -0.20" material="drone_accent"/>
      <geom name="leg_rl" type="capsule" size="0.011" fromto="-0.14 0.11 -0.045 -0.18 0.15 -0.20" material="drone_accent"/>
      <geom name="leg_rr" type="capsule" size="0.011" fromto="-0.14 -0.11 -0.045 -0.18 -0.15 -0.20" material="drone_accent"/>
      <geom name="skid_l" type="capsule" size="0.013" fromto="-0.20 0.15 -0.21 0.20 0.15 -0.21" material="drone_accent"
            contype="1" conaffinity="1" friction="1.4 0.1 0.01"/>
      <geom name="skid_r" type="capsule" size="0.013" fromto="-0.20 -0.15 -0.21 0.20 -0.15 -0.21" material="drone_accent"
            contype="1" conaffinity="1" friction="1.4 0.1 0.01"/>

      <!-- downward camera. euler 0 0 0 means the camera looks along body -z,
           i.e. straight down when the drone is level. The 180-degree roll that
           a naive setup uses points it at the sky. -->
      <geom name="cam_housing" type="box" size="0.030 0.030 0.018" pos="{cx} {cy} {cz + 0.02}" material="drone_accent"/>
      <geom name="cam_lens" type="cylinder" size="0.018 0.006" pos="{cx} {cy} {cz}" material="lens_mat"/>
      <camera name="onboard_cam" pos="{cx} {cy} {cz}" euler="0 0 0" fovy="{C.CAM_FOVY}"/>

      <!-- actuation frame: forces and torques are applied in THIS site's frame,
           i.e. in the body frame, so thrust always points along the drone's own
           z-axis and the vehicle has to tilt in order to translate -->
      <site name="thrust_site" pos="0 0 0" size="0.01"/>
      <site name="imu" pos="0 0 0" size="0.01"/>
      <site name="rangefinder" pos="0 0 -0.05" size="0.01" zaxis="0 0 -1"/>
    </body>
  </worldbody>

  <actuator>
    <general name="thrust" site="thrust_site" gear="0 0 1 0 0 0" ctrllimited="true" ctrlrange="0 {C.MAX_THRUST:.1f}"/>
    <general name="torque_x" site="thrust_site" gear="0 0 0 1 0 0" ctrllimited="true" ctrlrange="-6 6"/>
    <general name="torque_y" site="thrust_site" gear="0 0 0 0 1 0" ctrllimited="true" ctrlrange="-6 6"/>
    <general name="torque_z" site="thrust_site" gear="0 0 0 0 0 1" ctrllimited="true" ctrlrange="-2 2"/>
  </actuator>

  <sensor>
    <framequat name="imu_quat" objtype="site" objname="imu"/>
    <gyro name="imu_gyro" site="imu"/>
    <accelerometer name="imu_acc" site="imu"/>
    <rangefinder name="alt_range" site="rangefinder"/>
  </sensor>
</mujoco>
"""
    with open(path, "w", encoding="utf-8") as f:
        f.write(xml)
    return path


STAMP = os.path.join(ASSETS, "world.stamp")


def layout_signature():
    """
    Fingerprint of everything the generated world is built from.

    config.py is the single source of truth for the simulator - but the world
    it describes is generated ONTO DISK, and a freshly edited coordinate and a
    stale world.xml disagree in silence. The drone then flies to the marker
    where the picture puts it, lands neatly on it, and the touchdown error is
    measured against where the config says the pad is: a textbook landing five
    metres from the pad, reported as a success. Hashing the two files that
    generate the world turns that whole class of mistake into a rebuild.
    """
    h = hashlib.sha1()
    for f in (os.path.abspath(C.__file__), os.path.abspath(__file__)):
        with open(f, "rb") as fh:
            h.update(fh.read())
    return h.hexdigest()


def ensure_world(verbose=False):
    """
    Rebuild the world if it is missing, or was generated from a different
    config. Returns True if anything was regenerated. Costs about half a
    second, which is far less than one silently wrong landing.
    """
    want = layout_signature()
    have = None
    if os.path.exists(STAMP) and os.path.exists(os.path.join(ASSETS, "world.xml")):
        with open(STAMP, encoding="utf-8") as f:
            have = f.read().strip()
    if have == want:
        return False
    main(quiet=not verbose)
    with open(STAMP, "w", encoding="utf-8") as f:
        f.write(want)
    return True


def main(quiet=False):
    say = (lambda *a: None) if quiet else print
    say("  tag geometry:", C.check_geometry())
    say("  pad reach (m):", C.check_layout())
    C.check_geometry()
    C.check_layout()
    os.makedirs(ASSETS, exist_ok=True)
    for p in C.PADS:
        make_pad_texture(p["id"], os.path.join(ASSETS, "pad_%d.png" % p["id"]))
    say("  wrote %d pad textures" % len(C.PADS))
    build_xml(os.path.join(ASSETS, "world.xml"))
    say("  wrote assets/world.xml")


if __name__ == "__main__":
    main()
    with open(STAMP, "w", encoding="utf-8") as f:
        f.write(layout_signature())
