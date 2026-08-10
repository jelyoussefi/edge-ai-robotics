# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Perception: a pure detector.

Receives RGB frames from the compositor over the bus, runs person/object
detection, and sends back only bounding boxes, confidence, and class labels.
It has no camera and no depth: distance is computed by the compositor, which
owns the depth stream. Perception is stateless and knows nothing about the
scene beyond the pixels it is handed.
"""

from __future__ import annotations

import json
import logging
import os
import signal
import time

import cv2
import numpy as np

from edgebot import topics
# Resolution the obstacle silhouettes are published at. A quarter of the camera
# frame is plenty: the mask only decides which floor cells are occupied, and the
# world grid it feeds is coarser still.
# Resolution of the union silhouette put on the bus. 160x120 was chosen to keep
# the message tiny, and it cost accuracy where it matters most: one cell is 8 px
# at 720p, so the boundary between two objects that touch in the IMAGE is
# quantised by 8 px and the straddling cells are set. Deprojected, those cells
# land on the floor between the two objects. 320x240 packs to 9600 bytes, still
# nothing beside a JPEG frame, and halves the error.
MASK_W = int(os.environ.get("MASK_W", "320"))
MASK_H = int(os.environ.get("MASK_H", "240"))

from edgebot.bus import Publisher, Subscriber

from detector import Detector

log = logging.getLogger("perception")

CONFIG_PATH = os.environ.get("PERCEPTION_CONFIG", "/config/streams.json")


class _Frame:
    """Minimal stand-in for the detector's frame input: only colour, no depth."""

    def __init__(self, color: np.ndarray) -> None:
        self.color = color
        self.depth = None
        self.depth_scale = 1.0
        self.has_depth = False
        self.intrinsics = None


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")

    with open(CONFIG_PATH) as fh:
        spec = json.load(fh)[0]
    # The threshold lives in the stream config, but a demo needs to move it
    # without editing a committed file and it is the single knob that decides
    # whether a weakly recognised obstacle exists at all: a nappe-covered
    # coffee table measured 0.13 on this scene, so 0.25 erased it and 0.10
    # keeps it. PERCEPTION_CONF wins when it is set.
    _conf = float(spec.get("confidence", 0.4))
    _env_conf = os.environ.get("PERCEPTION_CONF", "").strip()
    if _env_conf:
        _conf = float(_env_conf)
    detector = Detector(
        spec["model"], spec.get("device", "CPU"),
        conf=_conf,
    )
    log.info("detection threshold %.2f (%s)", _conf,
             "PERCEPTION_CONF" if _env_conf else CONFIG_PATH)

    pub = Publisher()
    sub = Subscriber([topics.CAMERA_RGB])
    running = True

    def stop(*_: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    log.info("perception ready (pure detector), waiting for RGB from the source")
    count = 0
    last_log = time.perf_counter()
    infer_ms = 0.0

    while running:
        msg = sub.recv(100)
        if msg is None:
            continue
        _, payload = msg
        frame_t = payload.get("t", time.time())  # echo this so results align
        arr = np.frombuffer(payload["jpeg"], dtype=np.uint8)
        color = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if color is None:
            continue

        t0 = time.perf_counter()
        dets = detector.infer(_Frame(color))
        infer_ms = (time.perf_counter() - t0) * 1e3

        # Send bbox + confidence + label, tagged with the RGB frame's timestamp
        # so the compositor can pair them with the matching depth frame.
        detections = [
            {"cx": d.cx, "cy": d.cy, "w": d.width, "h": d.height,
             "score": d.score, "class_id": d.class_id}
            for d in dets
        ]
        pub.send(topics.DETECTIONS, {"detections": detections, "t": frame_t})

        # The silhouettes, when the model provides them. Downscaled and packed
        # into bits: the consumer wants to know which floor pixels are occupied,
        # not the mask at full resolution, and this keeps the message tiny.
        mask = getattr(detector, "mask", None)
        if mask is not None:
            small = cv2.resize(mask.astype(np.uint8), (MASK_W, MASK_H),
                               interpolation=cv2.INTER_NEAREST)
            body = {"w": MASK_W, "h": MASK_H, "t": frame_t,
                    "bits": np.packbits(small.astype(bool)).tobytes()}
            # Instance labels alongside the union, PNG-encoded. Added as extra
            # keys rather than replacing `bits`: every existing consumer reads
            # the boolean union and must keep working untouched. A label map is
            # almost all zeros, so PNG takes it down to a few kB.
            inst = getattr(detector, "inst_map", None)
            if inst is not None and inst.any():
                inst_small = cv2.resize(inst, (MASK_W, MASK_H),
                                        interpolation=cv2.INTER_NEAREST)
                ok, buf = cv2.imencode(".png", inst_small)
                if ok:
                    body["inst_png"] = buf.tobytes()
                    body["inst_meta"] = [
                        {"class_id": c, "score": sc}
                        for c, sc in getattr(detector, "inst_meta", [])]
            pub.send(topics.OBSTACLE_MASK, body)

        count += 1
        now = time.perf_counter()
        if now - last_log >= 30.0:
            log.info("%d detection(s) | %.0fms/frame | processed %d",
                     len(detections), infer_ms, count)
            last_log = now

    pub.close()
    sub.close()


if __name__ == "__main__":
    main()
