# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Source: the single camera owner.

Reads the RealSense (colour + depth, synchronised at the same resolution and fps
per Intel's validated config) and broadcasts each on its own bus channel with a
shared capture timestamp. Colour goes out as JPEG on CAMERA_RGB; depth goes out
as raw bytes on CAMERA_DEPTH with the same timestamp, so the compositor can pair
a depth frame with its RGB frame and with the detections computed from it.

Being the only process that opens the camera avoids RealSense's single-owner
contention. Colour and depth share resolution and fps so the sensor delivers
them synchronously (mixed modes desync and stall the colour stream).
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
from edgebot.camera import stream_mode, stream_name
from edgebot.bus import Publisher

log = logging.getLogger("source")

CONFIG_PATH = os.environ.get("SOURCE_CONFIG", "/config/streams.d455.json")
JPEG_QUALITY = int(os.environ.get("JPEG_QUALITY", "80"))
# Broadcast rates. The camera runs at its native fps; the source throttles
# what it puts on the bus to limit transport load. RGB drives the backdrop
# so it gets the higher rate; depth is only for distances/avoidance so it can
# be slower, which sharply cuts the largest payload on the bus.
RGB_HZ = float(os.environ.get("RGB_HZ", "25"))
DEPTH_HZ = float(os.environ.get("DEPTH_HZ", "10"))


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")

    import pyrealsense2 as rs
    with open(CONFIG_PATH) as fh:
        spec = json.load(fh)[0]
    # STREAM_RES wins over the JSON's width/height/fps. The JSON still owns the
    # serial, the model and the confidence -- things that do not change with
    # resolution -- but the resolution itself has to come from one place that
    # `calibrate` reads too, or the intrinsics on disk stop describing the
    # frames on the bus. See common/edgebot/camera.py.
    w, h, fps = stream_mode()
    if (w, h) != (spec.get("width"), spec.get("height")):
        log.info("STREAM_RES=%s overrides %s (%sx%s)", stream_name(),
                 CONFIG_PATH, spec.get("width"), spec.get("height"))

    pipeline = rs.pipeline()
    config = rs.config()
    if spec.get("serial"):
        config.enable_device(spec["serial"])
    # Colour and depth at the SAME resolution and fps: the sensor delivers them
    # synchronously, avoiding the desync that stalls colour.
    config.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
    config.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
    profile = pipeline.start(config)
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    log.info("source: camera %dx%d@%d colour + depth (scale %.4f)", w, h, fps, depth_scale)

    # Align depth to colour so a pixel (u,v) in the colour image maps to the same
    # point in the depth image. The D455's colour and depth sensors are physically
    # offset; without alignment, distances read at colour pixels are wrong. Safe
    # here because colour and depth share resolution and fps (the mismatch that
    # previously stalled the stream is gone).
    align = rs.align(rs.stream.color)

    # Depth post-processing. Glossy or dark surfaces reflect the projected IR
    # pattern away from the sensor, so stereo returns nothing there: a shiny tiled
    # floor comes back full of holes even though it is the flattest, most
    # unambiguous surface in the room. These filters run on the SDK side and cost
    # very little, and they are what makes downstream floor detection usable.
    #   spatial   edge-preserving smoothing, with its own small hole filling
    #   temporal  reuses recent valid values for a pixel that drops out, which is
    #             exactly the specular-reflection case on a fixed camera
    #   hole      fills what remains from the neighbours
    filters = []
    if os.environ.get("DEPTH_FILTERS", "1") == "1":
        spatial = rs.spatial_filter()
        spatial.set_option(rs.option.filter_magnitude, 2)
        spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
        spatial.set_option(rs.option.filter_smooth_delta, 20)
        spatial.set_option(rs.option.holes_fill, 2)
        temporal = rs.temporal_filter()
        temporal.set_option(rs.option.filter_smooth_alpha, 0.4)
        temporal.set_option(rs.option.filter_smooth_delta, 20)
        hole = rs.hole_filling_filter()
        hole.set_option(rs.option.holes_fill, 1)   # 1 = from the nearest valid
        filters = [spatial, temporal, hole]
        log.info("depth filters: spatial + temporal + hole filling")

    pub = Publisher()
    running = True

    def stop(*_: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    count = 0
    last_log = time.perf_counter()
    rgb_interval = 1.0 / RGB_HZ
    depth_interval = 1.0 / DEPTH_HZ
    last_rgb = 0.0
    last_depth = 0.0

    while running:
        try:
            frames = pipeline.wait_for_frames()
        except Exception:  # noqa: BLE001
            continue
        # Align depth to the colour frame so their pixels correspond.
        frames = align.process(frames)
        now = time.perf_counter()
        # One capture timestamp shared by both channels, so the compositor can
        # pair depth with RGB and with the detections computed from that RGB.
        stamp = time.time()

        if (now - last_rgb) >= rgb_interval:
            cf = frames.get_color_frame()
            if cf:
                color = np.asanyarray(cf.get_data())
                ok, buf = cv2.imencode(".jpg", color, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                if ok:
                    pub.send(topics.CAMERA_RGB,
                             {"jpeg": buf.tobytes(), "w": w, "h": h, "t": stamp})
            last_rgb = now

        if (now - last_depth) >= depth_interval:
            df = frames.get_depth_frame()
            for flt in filters:
                df = flt.process(df)
            if df:
                depth = np.asanyarray(df.get_data())  # uint16, native Z16
                pub.send(topics.CAMERA_DEPTH,
                         {"depth": depth.tobytes(), "w": w, "h": h,
                          "scale": depth_scale, "t": stamp})
            last_depth = now

        count += 1
        now = time.perf_counter()
        if now - last_log >= 30.0:
            log.info("broadcast %d frame pairs, %.1f fps", count, count / (now - last_log))
            count = 0
            last_log = now

    pipeline.stop()
    pub.close()


if __name__ == "__main__":
    main()
