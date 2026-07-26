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
from rs_source import open_source

log = logging.getLogger("perception")

CONFIG_PATH = os.environ.get("PERCEPTION_CONFIG", "/config/streams.json")
PUBLISH_HZ = float(os.environ.get("PERCEPTION_PUBLISH_HZ", "15"))
LOOP_VIDEO = os.environ.get("LOOP_VIDEO", "1") == "1"
# Publish the colour frame of this stream as a backdrop, or -1 to disable.
BACKDROP_CAMERA = int(os.environ.get("BACKDROP_CAMERA", "0"))
BACKDROP_HZ = float(os.environ.get("BACKDROP_HZ", "15"))
BACKDROP_WIDTH = int(os.environ.get("BACKDROP_WIDTH", "1280"))


class Stream(threading.Thread):
    """One camera or file, one detector, its own thread.

    Frames are held in a one-slot buffer that the reader overwrites, so a slow
    inference never builds a backlog of stale frames. Latest frame always wins.
    """

    def __init__(self, index: int, spec: dict) -> None:
        super().__init__(daemon=True)
        self.index = index
        self.spec = spec
        self.detector = Detector(
            spec["model"],
            spec.get("device", "CPU"),
            conf=float(spec.get("confidence", 0.4)),
        )
        self.obstacles: list[dict] = []
        self.latest_color = None  # BGR frame for the backdrop
        self.fps = 0.0
        self.infer_ms = 0.0
        self.has_depth = False
        self.running = True
        self._buf: deque = deque(maxlen=1)
        self._ready = threading.Event()
        self._lock = threading.Lock()

    def _reader(self) -> None:
        try:
            source = open_source(self.spec)
        except Exception as exc:  # noqa: BLE001 - want the reason in the log
            log.error("stream %d: cannot open source: %s", self.index, exc)
            self.running = False
            return

        while self.running:
            frame = source.read()
            if frame is None:
                log.warning("stream %d: source ended", self.index)
                break
            self.has_depth = frame.has_depth
            self._buf.append(frame)
            self._ready.set()
        source.close()

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
            detections = self.detector.infer(frame)
            infer_ms = (time.perf_counter() - t0) * 1e3

            with self._lock:
                # Copy, don't reference: pyrealsense reuses the same underlying
                # buffer for every frame, so storing the reference would make the
                # backdrop freeze on the first frame. The copy is a snapshot.
                self.latest_color = frame.color.copy()
                self.obstacles = [
                    {
                        "cx": d.cx,
                        "cy": d.cy,
                        "height": d.height,
                        "score": d.score,
                        "range_m": d.range_m if d.measured else None,
                        "bearing_deg": d.bearing_deg,
                        "class_id": d.class_id,
                        "measured": d.measured,
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
            return list(self.obstacles), self.fps, self.infer_ms

    def color_frame(self):
        with self._lock:
            return None if self.latest_color is None else self.latest_color

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
    last_backdrop = 0.0
    backdrop_dt = 1.0 / BACKDROP_HZ if BACKDROP_HZ > 0 else 0.0
    backdrop_count = 0

    while running:
        loop_start = time.perf_counter()

        obstacles: list[dict] = []
        per_stream = []
        for s in streams:
            found, fps, infer_ms = s.snapshot()
            obstacles.extend(found)
            per_stream.append(
                {"camera": s.index, "fps": fps, "infer_ms": infer_ms,
                 "device": s.detector.device, "depth": s.has_depth}
            )

        # Nearest first; unknown range (None) sorts last.
        obstacles.sort(key=lambda o: o["range_m"] if o["range_m"] is not None else float("inf"))

        pub.send(
            topics.PERCEPTION_OBSTACLES,
            {"obstacles": obstacles, "streams": per_stream, "stamp": time.time()},
        )

        # Backdrop: publish one stream's colour frame as JPEG, at its own rate.
        now = time.perf_counter()
        if backdrop_dt and BACKDROP_CAMERA >= 0 and (now - last_backdrop) >= backdrop_dt:
            if 0 <= BACKDROP_CAMERA < len(streams):
                color = streams[BACKDROP_CAMERA].color_frame()
                if color is not None:
                    h0, w0 = color.shape[:2]
                    if w0 > BACKDROP_WIDTH:
                        scale = BACKDROP_WIDTH / w0
                        color = cv2.resize(color, (BACKDROP_WIDTH, int(h0 * scale)))
                    ok, buf = cv2.imencode(".jpg", color, [cv2.IMWRITE_JPEG_QUALITY, 90])
                    if ok:
                        h1, w1 = color.shape[:2]
                        pub.send(
                            topics.CAMERA_FRAME,
                            {"jpeg": buf.tobytes(), "w": w1, "h": h1,
                             "camera": BACKDROP_CAMERA, "stamp": time.time()},
                        )
                        backdrop_count += 1
            last_backdrop = now

        if time.perf_counter() - last_log >= 5.0:
            nearest = next((o["range_m"] for o in obstacles if o["range_m"] is not None), None)
            near_str = f"{nearest:.2f}m" if nearest is not None else "n/a"
            summary = ", ".join(
                f"cam{p['camera']} {p['fps']:.1f}fps/{p['infer_ms']:.0f}ms{'/D' if p['depth'] else ''}"
                for p in per_stream
            )
            log.info("%d obstacle(s), nearest %s | %s | backdrop sent %d",
                     len(obstacles), near_str, summary, backdrop_count)
            last_log = time.perf_counter()

        time.sleep(max(0.0, dt - (time.perf_counter() - loop_start)))

    for s in streams:
        s.stop()
    pub.close()
    log.info("stopping")


if __name__ == "__main__":
    main()
