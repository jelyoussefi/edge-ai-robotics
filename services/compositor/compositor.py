# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Compositor: fuses real and virtual, displays the result.

Receives everything over the bus, owns no camera:
  - RGB frames from the source (CAMERA_RGB), with a capture timestamp.
  - Depth frames from the source (CAMERA_DEPTH), same timestamp as their RGB.
  - Detections from perception (DETECTIONS), tagged with the RGB frame's
    timestamp they were computed on.
  - Robot state from the sim (ROBOT_STATE).

It pairs depth with RGB by timestamp, turns detections into measured obstacles
(distance from the paired depth at each box centre), sends those obstacles to the
sim for avoidance, renders the virtual robot over the RGB frame, draws the boxes
and distances, and displays. Compositing and display are one process so no image
is produced onto the bus here; only obstacles go back out.

To keep display fluid, the incoming frames are drained "latest wins": only the
most recent RGB and depth are kept, so a slow render never builds a backlog.
"""

from __future__ import annotations

import json
import logging
import os
import time

import cv2
import mujoco
import numpy as np
from edgebot import topics
from edgebot.bus import Publisher, Subscriber

log = logging.getLogger("compositor")

ROBOT = os.environ.get("ROBOT", "g1")
SCENES = {
    "g1": "/models/mujoco_menagerie/unitree_g1/scene.xml",
    "h1": "/models/mujoco_menagerie/unitree_h1/scene.xml",
    "t1": "/models/booster_t1/scene.xml",
    "g1_walker": "/models/g1_walker/scene.xml",
}

WINDOW_W = int(os.environ.get("WINDOW_W", "1280"))
WINDOW_H = int(os.environ.get("WINDOW_H", "720"))
WINDOW_NAME = "Edge AI Robotics"

# COCO class id -> readable label, for the classes the detector reports.
COCO_NAMES = {
    0: "person", 24: "backpack", 26: "handbag", 28: "suitcase",
    39: "bottle", 41: "cup", 56: "chair", 57: "couch", 59: "bed",
    60: "table", 63: "laptop", 73: "book",
}

CAM_DISTANCE = float(os.environ.get("CAM_DISTANCE", "1.0"))
# The robot starts this many metres ahead of the camera and walks away.
START_AHEAD = float(os.environ.get("START_AHEAD", "1.0"))
CAM_ELEVATION = float(os.environ.get("CAM_ELEVATION", "-3.0"))
# Azimuth 0: virtual camera behind the robot looking along +x (its forward
# direction), so the robot is seen from behind, walking away from the camera.
CAM_AZIMUTH = float(os.environ.get("CAM_AZIMUTH", "0.0"))
LOOK_AT_DROP = float(os.environ.get("LOOK_AT_DROP", "0.5"))
CAM_SMOOTHING = 0.08
HFOV_DEG = float(os.environ.get("HFOV_DEG", "89.7"))
WAITKEY_MS = int(os.environ.get("WAITKEY_MS", "1"))
# Max age difference to accept a depth frame as matching an RGB timestamp.
PAIR_TOLERANCE_S = float(os.environ.get("PAIR_TOLERANCE_S", "0.2"))


def load_calibration() -> dict | None:
    path = os.environ.get("CAMERA_CALIBRATION", "/config/camera_calibration.json")
    if not os.path.exists(path):
        return None
    try:
        with open(path) as fh:
            calib = json.load(fh)
        log.info("calibration: vfov=%.1f pitch=%.1f height=%.2f",
                 calib.get("vfov_deg", 0), calib.get("pitch_deg", 0),
                 calib.get("camera_height_m", 0))
        return calib
    except (OSError, ValueError):
        return None


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")

    if not os.environ.get("DISPLAY"):
        raise SystemExit("compositor: no DISPLAY set; cannot open a window.")

    scene = SCENES[ROBOT]
    log.info("loading %s", scene)
    model = mujoco.MjModel.from_xml_path(scene)
    data = mujoco.MjData(model)

    for gid in range(model.ngeom):
        if model.geom_bodyid[gid] == 0:
            model.geom_rgba[gid, 3] = 0.0

    model.vis.global_.offwidth = WINDOW_W
    model.vis.global_.offheight = WINDOW_H

    calib = load_calibration()
    if calib and calib.get("vfov_deg"):
        model.vis.global_.fovy = float(calib["vfov_deg"])
    cam_height = float(calib["camera_height_m"]) if calib and calib.get("camera_height_m") else float(os.environ.get("CAM_HEIGHT", "1.2"))
    # The real camera is tilted DOWN by this pitch. The virtual camera matches it
    # so the floors line up. Clamped to a safe range: too steep an angle flips the
    # view past vertical and the robot appears upside down.
    _raw_pitch = abs(float(calib["pitch_deg"])) if calib and calib.get("pitch_deg") else float(os.environ.get("CAM_PITCH", "12"))
    cam_pitch = float(np.clip(_raw_pitch, 0.0, 45.0))

    renderer = mujoco.Renderer(model, height=WINDOW_H, width=WINDOW_W)
    seg_renderer = mujoco.Renderer(model, height=WINDOW_H, width=WINDOW_W)
    seg_renderer.enable_segmentation_rendering()

    cam = mujoco.MjvCamera()
    cam.distance = CAM_DISTANCE
    cam.azimuth = CAM_AZIMUTH
    # Tilt the virtual camera down by the real pitch, so the virtual ground plane
    # matches the real floor. This is what plants the robot's feet on the ground.
    cam.elevation = -cam_pitch

    pub = Publisher()
    # Camera streams on a low-HWM socket so ZeroMQ drops old frames and we always
    # get the freshest RGB/depth. Small messages (state, detections) on a normal
    # socket so none are dropped.
    cam_sub = Subscriber([topics.CAMERA_RGB, topics.CAMERA_DEPTH], rcvhwm=2)
    sub = Subscriber([topics.ROBOT_STATE, topics.DETECTIONS])

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
    log.info("compositor window %dx%d on DISPLAY=%s", WINDOW_W, WINDOW_H, os.environ.get("DISPLAY"))

    bg = np.zeros((WINDOW_H, WINDOW_W, 3), np.uint8)
    bg_t = 0.0
    depth_buf = None    # (t, HxW uint16, scale) latest depth
    detections = []     # latest bbox+conf+label
    det_t = 0.0
    show_overlay = False  # start hidden; press i to show
    robot_scale = float(os.environ.get("ROBOT_SCALE", "1.0"))  # +/- resize
    # Initial camera values, captured for the 'z' reset key.
    init_distance = cam.distance
    init_azimuth = cam.azimuth
    init_pitch = cam_pitch
    init_height = cam_height
    init_scale = robot_scale
    frames = 0
    last_log = time.perf_counter()

    while True:
        # Drain camera streams (low HWM already keeps these fresh), then the
        # small-message socket. Latest wins for the frames.
        while (msg := cam_sub.recv(0)) is not None:
            topic, payload = msg
            if topic == topics.CAMERA_RGB:
                arr = np.frombuffer(payload["jpeg"], dtype=np.uint8)
                img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if img is not None:
                    if img.shape[0] != WINDOW_H or img.shape[1] != WINDOW_W:
                        img = cv2.resize(img, (WINDOW_W, WINDOW_H))
                    bg = img
                    bg_t = payload.get("t", time.time())
            elif topic == topics.CAMERA_DEPTH:
                d = np.frombuffer(payload["depth"], dtype=np.uint16).reshape(
                    payload["h"], payload["w"])
                depth_buf = (payload.get("t", time.time()), d, payload.get("scale", 0.001))
        while (msg := sub.recv(0)) is not None:
            topic, payload = msg
            if topic == topics.DETECTIONS:
                detections = payload.get("detections", [])
                det_t = payload.get("t", time.time())
            elif topic == topics.ROBOT_STATE:
                qpos = np.asarray(payload["qpos"], dtype=np.float64)
                if qpos.shape[0] == model.nq:
                    data.qpos[:] = qpos
                    mujoco.mj_forward(model, data)

        # Distance for each detection, from the depth frame paired by timestamp.
        # cx/cy are normalised; depth is at its own resolution.
        obstacles = []
        depth_ok = depth_buf is not None and abs(depth_buf[0] - det_t) <= PAIR_TOLERANCE_S
        for d in detections:
            cxn, cyn = d.get("cx", 0.5), d.get("cy", 0.5)
            rng = None
            if depth_ok:
                _, dmap, scale = depth_buf
                dh, dw = dmap.shape
                px = min(dw - 1, max(0, int(cxn * dw)))
                py = min(dh - 1, max(0, int(cyn * dh)))
                raw = int(dmap[py, px])
                if raw > 0:
                    rng = raw * scale
            bearing = (cxn - 0.5) * HFOV_DEG
            obstacles.append({
                "cx": cxn, "cy": cyn, "w": d.get("w", 0.0), "height": d.get("h", 0.0),
                "score": d.get("score", 0.0), "class_id": d.get("class_id", 0),
                "range_m": rng, "bearing_deg": bearing,
                "measured": rng is not None, "camera": 0,
            })

        obstacles.sort(key=lambda o: o["range_m"] if o["range_m"] is not None else float("inf"))
        pub.send(topics.PERCEPTION_OBSTACLES, {"obstacles": obstacles, "stamp": time.time()})

        # Aim horizontally at camera height: the lookat point is at the robot's
        # x,y but at the camera's height H (not the robot's feet). With elevation
        # Fixed camera at the real D455 pose (height H, tilted down by the real
        # pitch). The lookat points at the GROUND (z=0) a fixed distance ahead, so
        # the virtual ground plane matches the real floor and the robot, standing
        # on z=0, is always on the floor, not floating up at table height.
        # Ground distance the camera naturally looks at, from its height and pitch.
        p = np.radians(max(1.0, cam_pitch))
        ground_ahead = cam_height / np.tan(p)
        cam.lookat[:] = np.array([ground_ahead, 0.0, 0.0])
        # Put the camera at world height H by setting the orbital distance so that
        # distance * sin(pitch) = H (camera_z = lookat_z + distance*sin(p) = H).
        cam.distance = cam_height / np.sin(p)

        # Camera world position (fixed), for the real distance readout.
        az, el = np.radians(cam.azimuth), np.radians(cam.elevation)
        fwd = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
        cam_pos = cam.lookat - cam.distance * fwd
        robot_dist = float(np.linalg.norm(data.qpos[:3] - cam_pos))

        renderer.update_scene(data, cam)
        robot = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
        seg_renderer.update_scene(data, cam)
        mask = seg_renderer.render()[:, :, 0] >= 0

        # Resize the rendered robot by robot_scale, anchored at the feet, so the
        # robot grows/shrinks in place without moving where it stands or touching
        # physics. Scaling the image (not the model) keeps it cheap and stable.
        if abs(robot_scale - 1.0) > 1e-3 and mask.any():
            ys, xs = np.where(mask)
            # Anchor at the bottom-centre of the robot (its feet on the floor).
            ax = int((xs.min() + xs.max()) / 2)
            ay = int(ys.max())
            M = np.float32([[robot_scale, 0, ax * (1 - robot_scale)],
                            [0, robot_scale, ay * (1 - robot_scale)]])
            robot = cv2.warpAffine(robot, M, (WINDOW_W, WINDOW_H))
            mask = cv2.warpAffine(mask.astype(np.uint8), M, (WINDOW_W, WINDOW_H)) > 0

        out = np.where(mask[:, :, None], robot, bg)

        # Overlay (boxes, labels, distances, robot readout) is toggled with 'i'.
        if show_overlay:
            for o in obstacles:
                cxn, cyn = o["cx"], o["cy"]
                wn = o["w"] if o["w"] else o["height"] * 0.5  # real width from detection
                hn = o["height"]
                x1 = int((cxn - wn / 2) * WINDOW_W)
                y1 = int((cyn - hn / 2) * WINDOW_H)
                x2 = int((cxn + wn / 2) * WINDOW_W)
                y2 = int((cyn + hn / 2) * WINDOW_H)
                cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # Label: class name, confidence, then distance if measured.
                name = COCO_NAMES.get(o["class_id"], "obj")
                conf = int(o["score"] * 100)
                if o["range_m"] is not None:
                    label = f"{name} {conf}% {o['range_m']:.2f}m"
                else:
                    label = f"{name} {conf}%"
                cv2.putText(out, label, (x1, max(18, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.putText(out, f"robot: {robot_dist:.2f}m", (12, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 255), 2)

        cv2.imshow(WINDOW_NAME, out)

        frames += 1
        now = time.perf_counter()
        if now - last_log >= 5.0:
            fps = frames / (now - last_log)
            age = time.time() - bg_t if bg_t else -1
            log.info("composited %.1f fps, bg age %.2fs, depth paired=%s",
                     fps, age, depth_ok)
            frames = 0
            last_log = now

        key = cv2.waitKey(WAITKEY_MS) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("i"):
            show_overlay = not show_overlay
            log.info("overlay %s", "on" if show_overlay else "off")
        # z resets all camera adjustments to their initial values (recovers from
        # a bad tweak, e.g. an upside-down view).
        elif key == ord("z"):
            cam.distance = init_distance
            cam.azimuth = init_azimuth
            cam_pitch = init_pitch
            cam.elevation = -cam_pitch
            cam_height = init_height
            robot_scale = init_scale
            # Also send the robot back to its start pose (the sim handles CMD_RESET).
            pub.send(topics.CMD_RESET, {"stamp": time.time()})
            log.info("reset: camera to initial + robot to start pose")
        # Live camera-pose tuning for placing the robot. Arrow-like keys:
        #   a/d  azimuth (rotate view around robot)
        #   w/s  distance (closer / farther)
        #   r/f  elevation (tilt up / down)
        elif key == ord("a"):
            cam.azimuth -= 5
            log.info("azimuth %.0f", cam.azimuth)
        elif key == ord("d"):
            cam.azimuth += 5
            log.info("azimuth %.0f", cam.azimuth)
        elif key == ord("w"):
            cam.distance = max(0.3, cam.distance - 0.1)
            log.info("distance %.2f", cam.distance)
        elif key == ord("s"):
            cam.distance += 0.1
            log.info("distance %.2f", cam.distance)
        elif key == ord("k"):
            cam_pitch = max(0.0, cam_pitch - 2)
            cam.elevation = -cam_pitch
            log.info("cam_pitch %.0f (less down)", cam_pitch)
        elif key == ord("j"):
            cam_pitch = min(45.0, cam_pitch + 2)
            cam.elevation = -cam_pitch
            log.info("cam_pitch %.0f (more down)", cam_pitch)
        # t/g raise/lower the virtual camera height, to match the real camera or
        # fine-tune where the robot's feet meet the floor.
        elif key == ord("t"):
            cam_height += 0.05
            log.info("cam_height %.2f", cam_height)
        elif key == ord("g"):
            cam_height = max(0.1, cam_height - 0.05)
            log.info("cam_height %.2f", cam_height)
        # +/- resize the robot (bigger / smaller), anchored at its feet.
        elif key in (ord("+"), ord("=")):
            robot_scale = min(3.0, robot_scale + 0.1)
            log.info("robot_scale %.2f", robot_scale)
        elif key in (ord("-"), ord("_")):
            robot_scale = max(0.3, robot_scale - 0.1)
            log.info("robot_scale %.2f", robot_scale)
        # l / r turn the ROBOT itself left / right by 5 degrees (operator control).
        elif key == ord("l"):
            pub.send(topics.CMD_TURN, {"wz": np.radians(5)})
            log.info("robot turn left 5deg")
        elif key == ord("r"):
            pub.send(topics.CMD_TURN, {"wz": -np.radians(5)})
            log.info("robot turn right 5deg")
        if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
            break

    renderer.close()
    seg_renderer.close()
    cv2.destroyAllWindows()
    pub.close()
    sub.close()
    cam_sub.close()


if __name__ == "__main__":
    main()
