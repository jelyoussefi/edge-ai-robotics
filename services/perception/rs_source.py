# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""RealSense D457 source.

Wraps a pyrealsense2 pipeline and hands out color frames with their aligned
depth. Depth is aligned to color so a pixel in the detection maps to the same
pixel in the depth image, which is what lets a bounding box be turned into a
real distance.

Falls back to a plain video file when no camera is present, so the perception
pipeline still runs on the sample clip during development. In that mode depth is
unavailable and the caller is told so per frame.
"""

from __future__ import annotations

import logging

import cv2
import numpy as np

log = logging.getLogger("rs_source")


class Frame:
    """One synchronised capture."""

    __slots__ = ("color", "depth", "depth_scale", "intrinsics")

    def __init__(self, color, depth, depth_scale, intrinsics):
        self.color = color  # HxWx3 BGR uint8
        self.depth = depth  # HxW uint16, or None on a video source
        self.depth_scale = depth_scale  # metres per depth unit
        self.intrinsics = intrinsics  # dict with fx, fy, ppx, ppy, hfov_deg, or None

    @property
    def has_depth(self) -> bool:
        return self.depth is not None


class RealSenseSource:
    """Aligned color + depth from a D457 (or any RealSense) over pyrealsense2."""

    def __init__(
        self,
        serial: str | None,
        width: int,
        height: int,
        fps: int,
        depth_width: int | None = None,
        depth_height: int | None = None,
        depth_fps: int | None = None,
    ) -> None:
        import pyrealsense2 as rs

        self._rs = rs
        self.pipeline = rs.pipeline()
        config = rs.config()
        if serial:
            config.enable_device(serial)

        # Colour drives the backdrop, so it may be high resolution. Depth is
        # aligned to colour afterwards, so it need not match: it runs at its own
        # resolution (default 848x480@30), which avoids the D455's inability to
        # run both sensors at 720p together. Alignment maps depth onto colour.
        dw = depth_width or 848
        dh = depth_height or 480
        dfps = depth_fps or 30
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        config.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, dfps)

        profile = self.pipeline.start(config)

        # Align depth into the color frame so pixels correspond.
        self.align = rs.align(rs.stream.color)

        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_stream.get_intrinsics()
        hfov = float(np.degrees(2 * np.arctan2(intr.width / 2.0, intr.fx)))
        vfov = float(np.degrees(2 * np.arctan2(intr.height / 2.0, intr.fy)))
        self.intrinsics = {
            "fx": intr.fx,
            "fy": intr.fy,
            "ppx": intr.ppx,
            "ppy": intr.ppy,
            "hfov_deg": hfov,
            "vfov_deg": vfov,
        }

        dev = profile.get_device()
        name = dev.get_info(rs.camera_info.name) if dev else "RealSense"
        log.info(
            "%s started %dx%d@%d, depth scale %.4f m/unit, HFOV %.1f deg",
            name, width, height, fps, self.depth_scale, hfov,
        )

    def read(self) -> Frame | None:
        frames = self.pipeline.wait_for_frames()
        frames = self.align.process(frames)
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        if not color or not depth:
            return None
        return Frame(
            # Copy out of pyrealsense's recycled buffers immediately, so no
            # frame shares memory that the next wait_for_frames() overwrites.
            np.asanyarray(color.get_data()).copy(),
            np.asanyarray(depth.get_data()).copy(),
            self.depth_scale,
            self.intrinsics,
        )

    def close(self) -> None:
        self.pipeline.stop()


class VideoSource:
    """Fallback source: a video file, no depth.

    Lets the pipeline run on the sample clip when no camera is attached. The
    detector then reports distance as unknown for every obstacle, and the
    behaviour treats unknown distance as far.
    """

    def __init__(self, path: str, loop: bool = True) -> None:
        self.path = path
        self.loop = loop
        self.cap = cv2.VideoCapture(path)
        if not self.cap.isOpened():
            raise SystemExit(f"cannot open video source {path!r}")
        log.info("video source %s (no depth)", path)

    def read(self) -> Frame | None:
        ok, color = self.cap.read()
        if not ok:
            if self.loop and self.cap.get(cv2.CAP_PROP_FRAME_COUNT) > 0:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, color = self.cap.read()
            if not ok:
                return None
        return Frame(color, None, 0.0, None)

    def close(self) -> None:
        self.cap.release()


def open_source(spec: dict):
    """Pick a source from a stream spec.

    A spec with "serial" or type "realsense" opens the camera. Anything else
    with a "source" path opens that as video. This is what lets one config
    format drive both the camera demo and the video development path.
    """
    kind = spec.get("type", "").lower()
    if kind == "realsense" or "serial" in spec:
        return RealSenseSource(
            spec.get("serial"),
            int(spec.get("width", 848)),
            int(spec.get("height", 480)),
            int(spec.get("fps", 30)),
            depth_width=spec.get("depth_width"),
            depth_height=spec.get("depth_height"),
            depth_fps=spec.get("depth_fps"),
        )
    return VideoSource(spec["source"], loop=spec.get("loop", True))
