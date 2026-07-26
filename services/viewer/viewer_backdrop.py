# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Compositing viewer: the robot standing in front of the live camera image.

Uses the passive MuJoCo viewer, which maps a window reliably on this host, and
its update_texture() method, which pushes new pixels into a named texture on the
GPU each frame (locking internally). A large plane is placed behind the robot
with that texture on it; every frame its pixels are replaced with the latest
camera image. MuJoCo draws the 3D robot in front of the plane natively, so the
robot appears to stand in the room the camera sees.

The plane is a fixed wall a few metres behind the robot, and the camera pose is
fixed, so the wall fills the view and the robot walks in front of it. The
original robot scene is wrapped, not modified: a temp scene includes it and adds
the backdrop plane, so nothing in the fetched model files changes.
"""

from __future__ import annotations

import logging
import os
import tempfile
import time

import cv2
import mujoco
import mujoco.viewer
import numpy as np
from edgebot import topics
from edgebot.bus import Subscriber

log = logging.getLogger("viewer")

ROBOT = os.environ.get("ROBOT", "g1")
SCENES = {
    "g1": "/models/mujoco_menagerie/unitree_g1/scene.xml",
    "h1": "/models/mujoco_menagerie/unitree_h1/scene.xml",
    "t1": "/models/mujoco_menagerie/booster_t1/scene.xml",
    "g1_walker": "/models/g1_walker/scene.xml",
}

BACKDROP_TEX = "camera_backdrop"
# Texture matches the D455 colour frame aspect (4:3) to avoid resampling smear.
# Sized generously; the frame is fit to exactly these dimensions each update.
TEX_W, TEX_H = 848, 640  # 4:3-ish, matches 640x480 without horizontal stretch

# Camera framing. Azimuth 180 puts the camera behind the robot so we see its
# back; the robot walks along +x away from the camera. Elevation near level
# so it looks planted on the floor rather than seen from above.
CAM_DISTANCE = float(os.environ.get("CAM_DISTANCE", "3.0"))
CAM_ELEVATION = float(os.environ.get("CAM_ELEVATION", "-3.0"))
CAM_AZIMUTH = float(os.environ.get("CAM_AZIMUTH", "90.0"))
CAM_SMOOTHING = 0.08

# Lower the look-at point from the hip toward the floor so the feet stay in
# frame and the robot reads as standing on the ground. Larger = look lower.
LOOK_AT_DROP = float(os.environ.get("LOOK_AT_DROP", "0.5"))

# Frame the robot to this fraction of the detected person's height, once,
# when the first person is seen. Then the distance is frozen.
TARGET_HEIGHT_FRAC = float(os.environ.get("TARGET_HEIGHT_FRAC", "0.8"))
# Robot pixel height as a fraction of the view at the default distance,
# measured empirically; used to convert desired fraction into a distance.
ROBOT_FRAC_AT_REF = float(os.environ.get("ROBOT_FRAC_AT_REF", "0.55"))
REF_DISTANCE = 3.0

# Backdrop plane geometry. It sits behind the robot, facing the camera.
BACKDROP_Y = float(os.environ.get("BACKDROP_Y", "3.5"))     # metres behind origin
BACKDROP_Z = float(os.environ.get("BACKDROP_Z", "1.3"))     # centre height (robot-ish)
BACKDROP_HW = float(os.environ.get("BACKDROP_HW", "16.0"))  # half-width (over-fills at any zoom)
BACKDROP_HH = float(os.environ.get("BACKDROP_HH", "9.0"))   # half-height (over-fills at any zoom)


def build_scene(scene_path: str) -> str:
    """Wrap the robot scene with a textured backdrop plane. Returns a path.

    The robot scene lives under a read-only mount and its meshes are referenced
    relative to it (meshdir="assets"). To add a backdrop without editing those
    files, the whole model directory is copied to a writable temp location and
    the wrapper is written there, so both the include and the relative mesh
    paths resolve.
    """
    import shutil

    scene_dir = os.path.dirname(os.path.abspath(scene_path))
    scene_file = os.path.basename(scene_path)

    tmp_dir = tempfile.mkdtemp(prefix="viewer_model_")
    dst = os.path.join(tmp_dir, "model")
    shutil.copytree(scene_dir, dst)

    xml = f"""
<mujoco model="with_backdrop">
  <include file="{scene_file}"/>
  <visual>
    <global offwidth="{TEX_W}" offheight="{TEX_H}"/>
  </visual>
  <asset>
    <texture name="{BACKDROP_TEX}" type="2d" builtin="flat"
             rgb1="0.1 0.15 0.2" width="{TEX_W}" height="{TEX_H}"/>
    <material name="{BACKDROP_TEX}_mat" texture="{BACKDROP_TEX}"
              texrepeat="1 1" texuniform="false" emission="1.0"/>
  </asset>
  <worldbody>
    <geom name="{BACKDROP_TEX}_screen" type="plane"
          pos="0 {BACKDROP_Y} {BACKDROP_Z}"
          size="{BACKDROP_HW} {BACKDROP_HH} 0.1"
          zaxis="0 -1 0"
          material="{BACKDROP_TEX}_mat"
          contype="0" conaffinity="0"/>
  </worldbody>
</mujoco>
"""
    path = os.path.join(dst, "_viewer_backdrop.xml")
    with open(path, "w") as fh:
        fh.write(xml)
    return path


class CameraFrame:
    def __init__(self) -> None:
        self.rgb: np.ndarray | None = None

    def update(self, jpeg: bytes) -> None:
        arr = np.frombuffer(jpeg, dtype=np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is not None:
            self.rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)




def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")

    scene = SCENES[ROBOT]
    log.info("loading %s", scene)
    wrapped = build_scene(scene)
    model = mujoco.MjModel.from_xml_path(wrapped)
    data = mujoco.MjData(model)

    tex_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TEXTURE, BACKDROP_TEX)
    if tex_id < 0:
        raise SystemExit("backdrop texture not found")
    adr = int(model.tex_adr[tex_id])
    tw = int(model.tex_width[tex_id])
    th = int(model.tex_height[tex_id])
    tex_size = tw * th * 3

    # Hide the robot's own world (floor, table, props) so only the robot stands
    # over the camera backdrop. Any geom in the world body other than our
    # backdrop plane is made fully transparent. Robot-body geoms are untouched.
    backdrop_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{BACKDROP_TEX}_screen")
    if backdrop_geom < 0:
        raise SystemExit("backdrop geom not found")
    hidden = 0
    for gid in range(model.ngeom):
        if gid == backdrop_geom:
            continue
        if model.geom_bodyid[gid] == 0:  # world body
            model.geom_rgba[gid, 3] = 0.0
            hidden += 1
    log.info("hid %d world geom(s), keeping robot + backdrop", hidden)

    camera = CameraFrame()
    state_sub = Subscriber([topics.ROBOT_STATE])
    frame_sub = Subscriber([topics.CAMERA_FRAME])
    obstacle_sub = Subscriber([topics.PERCEPTION_OBSTACLES])
    look_at = np.zeros(3)

    with mujoco.viewer.launch_passive(
        model, data, show_left_ui=False, show_right_ui=False
    ) as viewer:
        viewer.cam.distance = CAM_DISTANCE
        viewer.cam.elevation = CAM_ELEVATION
        viewer.cam.azimuth = CAM_AZIMUTH

        log.info("compositing viewer up on DISPLAY=%s", os.environ.get("DISPLAY"))
        stale_since: float | None = None
        frames_applied = 0
        zoom_locked = False  # True once distance is set from first person
        last_frame_log = time.perf_counter()

        while viewer.is_running():
            frame_start = time.perf_counter()

            msg = state_sub.drain()
            if msg is not None:
                _, payload = msg
                qpos = np.asarray(payload["qpos"], dtype=np.float64)
                if qpos.shape[0] == model.nq:
                    data.qpos[:] = qpos
                    mujoco.mj_forward(model, data)
                    stale_since = None
                else:
                    log.warning("qpos size %d != model nq %d, ignoring", qpos.shape[0], model.nq)
            elif stale_since is None:
                stale_since = frame_start
            elif frame_start - stale_since > 3.0:
                log.warning("no state from sim for 3 s")
                stale_since = frame_start

            fmsg = frame_sub.drain()
            if fmsg is not None:
                _, fpayload = fmsg
                camera.update(fpayload["jpeg"])
                if camera.rgb is not None:
                    # Fit the frame to the texture, flip so it's upright on the
                    # plane, and write it into the model's texture buffer, then
                    # push to the GPU via the viewer (which locks internally).
                    img = cv2.resize(camera.rgb, (tw, th))
                    img = np.flipud(img)
                    with viewer.lock():
                        model.tex_data[adr: adr + tex_size] = np.ascontiguousarray(
                            img, dtype=np.uint8
                        ).reshape(-1)
                    viewer.update_texture(tex_id)
                    frames_applied += 1

            # One-shot framing: when the first person is detected, set the
            # camera distance so the robot is TARGET_HEIGHT_FRAC of that
            # person's on-screen height, then freeze it.
            omsg = obstacle_sub.drain()
            if omsg is not None and not zoom_locked:
                _, opayload = omsg
                people = [o for o in opayload.get("obstacles", [])
                          if o.get("class_id", 0) == 0 and o.get("height")]
                if people:
                    person_h = max(p["height"] for p in people)  # tallest = nearest
                    target_frac = TARGET_HEIGHT_FRAC * person_h
                    target_frac = max(0.1, min(0.95, target_frac))
                    dist = REF_DISTANCE * (ROBOT_FRAC_AT_REF / target_frac)
                    viewer.cam.distance = float(max(2.6, min(8.0, dist)))
                    zoom_locked = True
                    log.info("framed: person_h=%.2f -> robot %.0f%% -> dist=%.2f",
                             person_h, target_frac * 100, viewer.cam.distance)

            if frame_start - last_frame_log >= 5.0:
                log.info("backdrop frames applied: %d", frames_applied)
                last_frame_log = frame_start

            target = data.qpos[:3].copy()
            target[2] -= LOOK_AT_DROP  # aim lower so feet stay in frame
            look_at += (target - look_at) * CAM_SMOOTHING
            viewer.cam.lookat[:] = look_at

            viewer.sync()
            time.sleep(max(0.0, (1 / 60) - (time.perf_counter() - frame_start)))

    log.info("viewer closed")
    state_sub.close()
    frame_sub.close()
    obstacle_sub.close()
    try:
        os.unlink(wrapped)
    except OSError:
        pass


if __name__ == "__main__":
    main()
