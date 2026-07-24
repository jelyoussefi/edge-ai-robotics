# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Perception service.

Runs one detector per configured stream and publishes the people it finds.

The stream list is a JSON file with the same shape the Robotics AI Suite
multicam-demo uses, so the same config drives either. A stream source is any
OpenCV-readable path: an mp4 for development, /dev/video-rs-color-N once the
D457 cameras are up. Nothing else in the service changes between the two.
"""

from __future__ import annotations

import json
import logging
import os
import signal
import threading
import time
from collections import deque

import cv2
from edgebot import topics
from edgebot.bus import Publisher

from detector import Detector

log = logging.getLogger("perception")

CONFIG_PATH = os.environ.get("PERCEPTION_CONFIG", "/config/streams.json")
PUBLISH_HZ = float(os.environ.get("PERCEPTION_PUBLISH_HZ", "15"))
LOOP_VIDEO = os.environ.get("LOOP_VIDEO", "1") == "1"


class Stream(threading.Thread):
    """One camera or file, one detector, its own thread.

    Frames are held in a one-slot buffer that the reader overwrites, so a slow
    inference never builds a backlog of stale frames. Latest frame always wins.
    """

    def __init__(self, index: int, spec: dict) -> None:
        super().__init__(daemon=True)
        self.index = index
        self.source = spec["source"]
        self.vfov = float(spec.get("vfov_deg", 65.0))
        self.detector = Detector(
            spec["model"],
            spec.get("device", "CPU"),
            conf=float(spec.get("confidence", 0.4)),
        )
        self.people: list[dict] = []
        self.fps = 0.0
        self.infer_ms = 0.0
        self.running = True
        self._buf: deque = deque(maxlen=1)
        self._ready = threading.Event()
        self._lock = threading.Lock()

    def _reader(self) -> None:
        cap = cv2.VideoCapture(self.source)
        if not cap.isOpened():
            log.error("stream %d: cannot open %s", self.index, self.source)
            self.running = False
            return
        log.info("stream %d: reading %s", self.index, self.source)

        while self.running:
            ok, frame = cap.read()
            if not ok:
                if LOOP_VIDEO and cap.get(cv2.CAP_PROP_FRAME_COUNT) > 0:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                log.warning("stream %d: source ended", self.index)
                break
            self._buf.append(frame)
            self._ready.set()
        cap.release()

    def run(self) -> None:
        threading.Thread(target=self._reader, daemon=True).start()
        frames = 0
        started = time.perf_counter()

        while self.running:
            if not self._ready.wait(timeout=1.0):
                continue
            self._ready.clear()
            if not self._buf:
                continue
            frame = self._buf[-1]

            t0 = time.perf_counter()
            detections = self.detector.infer(frame, self.vfov)
            infer_ms = (time.perf_counter() - t0) * 1e3

            with self._lock:
                self.people = [
                    {
                        "cx": d.cx,
                        "cy": d.cy,
                        "height": d.height,
                        "score": d.score,
                        "range_m": d.range_m,
                        "camera": self.index,
                    }
                    for d in detections
                ]
                self.infer_ms = infer_ms
                frames += 1
                elapsed = time.perf_counter() - started
                self.fps = frames / elapsed if elapsed > 0 else 0.0

    def snapshot(self) -> tuple[list[dict], float, float]:
        with self._lock:
            return list(self.people), self.fps, self.infer_ms

    def stop(self) -> None:
        self.running = False


def load_config(path: str) -> list[dict]:
    if not os.path.exists(path):
        raise SystemExit(f"no stream config at {path!r}")
    # The suite writes these files with a .js extension and // comments.
    with open(path) as fh:
        text = "\n".join(line for line in fh if not line.lstrip().startswith("//"))
    return json.loads(text)


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")

    specs = load_config(CONFIG_PATH)
    log.info("starting %d stream(s)", len(specs))
    streams = [Stream(i, spec) for i, spec in enumerate(specs)]
    for s in streams:
        s.start()

    pub = Publisher()
    running = True

    def stop(*_: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    dt = 1.0 / PUBLISH_HZ
    last_log = time.perf_counter()

    while running:
        loop_start = time.perf_counter()

        people: list[dict] = []
        per_stream = []
        for s in streams:
            found, fps, infer_ms = s.snapshot()
            people.extend(found)
            per_stream.append({"camera": s.index, "fps": fps, "infer_ms": infer_ms, "device": s.detector.device})

        # Nearest first, so a consumer can just take people[0].
        people.sort(key=lambda p: p["range_m"])

        pub.send(
            topics.PERCEPTION_PEOPLE,
            {"people": people, "streams": per_stream, "stamp": time.time()},
        )

        if time.perf_counter() - last_log >= 5.0:
            summary = ", ".join(f"cam{p['camera']} {p['fps']:.1f}fps/{p['infer_ms']:.0f}ms" for p in per_stream)
            log.info("%d person(s) | %s", len(people), summary)
            last_log = time.perf_counter()

        time.sleep(max(0.0, dt - (time.perf_counter() - loop_start)))

    for s in streams:
        s.stop()
    pub.close()
    log.info("stopping")


if __name__ == "__main__":
    main()
