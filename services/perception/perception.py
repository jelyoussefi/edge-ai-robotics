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
    detector = Detector(
        spec["model"], spec.get("device", "CPU"),
        conf=float(spec.get("confidence", 0.4)),
    )

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
