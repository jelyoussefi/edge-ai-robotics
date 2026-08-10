#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Record a clip of exactly what `source` would have opened.

THE EXTENSION IS .db3, NOT .bag. librealsense switched its recorder to the
rosbag2 sqlite3 container, and 2.58.3 in this image refuses anything else
outright: "Output file must have .db3 extension". It is still the same
record-and-replay facility everyone calls a bag, and this file keeps calling it
one in prose; only the suffix moved.

For developing without the camera. Everything downstream -- compositor,
navigator, the probes -- runs unchanged against a recording, but only if the
recording is of the same thing the live source produces. Two conditions, and
both are checked here rather than assumed:

  THE RASTER. config/camera_calibration.json describes ONE room at ONE
  resolution. fx, fy, ppx, ppy are pixel quantities in a particular raster, and
  1280x720 is not an enlarged 640x480 -- different aspect, different sensor
  crop, 19.9 % different focal length on this D455. A bag recorded at another
  raster makes every distance, the floor polygon and every footprint wrong
  while the picture still composites and the overlay still paints something.
  NOTHING downstream can detect it. So the mode comes from the same
  stream_mode() the source uses, over the same streams.d455.json, and the
  profiles the device actually negotiated are read back and compared. A
  mismatch is an ERROR here, at the only moment it can still be fixed.

  THE STREAMS. Colour and depth, same resolution, same fps, from the same
  serial. The D455 desyncs and stalls colour on mixed modes, and a bag missing
  depth plays back as a camera that stopped reporting distances.

Runs in the `source` container, because that is the only one allowed to open
the RealSense -- privileged, with /dev mounted. `make record` does that; a bare
python3 on the host will not find the device.

SIZE. Roughly 8 GB per minute at 720p30: 1280x720 of BGR8 plus 1280x720 of Z16
is 4.4 MB per frame pair, 30 times a second. The estimate is printed before the
first frame, not after the disk fills.
"""
from __future__ import annotations

import argparse
import json
import logging
import os
import shutil
import signal
import sys
import time

from edgebot.camera import stream_mode, stream_name

log = logging.getLogger("record")

CONFIG_PATH = os.environ.get("SOURCE_CONFIG", "/config/streams.d455.json")
# BGR8 colour + Z16 depth, per frame pair, at the recorded raster.
BYTES_PER_PAIR_PX = 3 + 2


def main() -> int:
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(name)s %(levelname)s %(message)s")
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--out", default="/data/scene.db3")
    ap.add_argument("--force", action="store_true",
                    help="overwrite an existing file instead of refusing")
    args = ap.parse_args()

    import pyrealsense2 as rs

    with open(CONFIG_PATH) as fh:
        spec = json.load(fh)[0]
    # The SAME mode the source asks for, from the SAME place. Not a copy of the
    # numbers: a copy is a second thing to keep in step, and the whole failure
    # this guards against is two places disagreeing about the raster.
    w, h, fps = stream_mode()

    if not args.out.endswith(".db3"):
        log.error("the recorder needs a .db3 file, not %s. librealsense %s "
                  "writes the rosbag2 sqlite3 container and rejects any other "
                  "suffix inside pipeline.start(), i.e. AFTER the camera is "
                  "open and the estimate is printed. Refused here instead.",
                  os.path.basename(args.out), rs.__version__)
        return 1

    if os.path.exists(args.out) and not args.force:
        log.error("%s already exists (%.1f GB). Pass --force to overwrite, or "
                  "choose another OUT -- a bag is expensive to re-record and "
                  "this one may be the only copy of that scene.",
                  args.out, os.path.getsize(args.out) / 1e9)
        return 1

    est = w * h * BYTES_PER_PAIR_PX * fps * args.seconds
    free = shutil.disk_usage(os.path.dirname(args.out) or ".").free
    log.info("recording %.0f s of %s (%dx%d@%d) to %s", args.seconds,
             stream_name(), w, h, fps, args.out)
    log.info("expect about %.1f GB (%.1f GB per minute); %.1f GB free on that "
             "filesystem", est / 1e9, est / 1e9 * 60.0 / max(1e-9, args.seconds),
             free / 1e9)
    if est > free * 0.9:
        log.error("that will not fit: %.1f GB needed, %.1f GB free. Record a "
                  "shorter clip or free space first.", est / 1e9, free / 1e9)
        return 1

    pipeline = rs.pipeline()
    config = rs.config()
    if spec.get("serial"):
        config.enable_device(spec["serial"])
    config.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
    config.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
    # Raw frames only. No align, no spatial/temporal/hole filters: those are
    # the source's business and it applies them on playback exactly as it does
    # live. Baking them into the bag would freeze one filter configuration into
    # every future run and make DEPTH_FILTERS=0 unmeasurable.
    config.enable_record_to_file(args.out)

    profile = pipeline.start(config)
    _cp = profile.get_stream(rs.stream.color).as_video_stream_profile()
    _dp = profile.get_stream(rs.stream.depth).as_video_stream_profile()
    log.info("RealSense negotiated: colour %dx%d@%d %s | depth %dx%d@%d %s",
             _cp.width(), _cp.height(), _cp.fps(), _cp.format(),
             _dp.width(), _dp.height(), _dp.fps(), _dp.format())
    bad = False
    for name, p in (("colour", _cp), ("depth", _dp)):
        if (p.width(), p.height(), p.fps()) != (w, h, fps):
            bad = True
            log.error("the %s stream came back as %dx%d@%d, NOT the %dx%d@%d "
                      "asked for. A bag at the wrong raster makes every "
                      "distance downstream wrong and nothing can detect it "
                      "later -- this recording would be a trap.",
                      name, p.width(), p.height(), p.fps(), w, h, fps)
    _i = _cp.get_intrinsics()
    log.info("colour intrinsics being recorded: %dx%d fx=%.1f fy=%.1f "
             "ppx=%.1f ppy=%.1f", _i.width, _i.height, _i.fx, _i.fy,
             _i.ppx, _i.ppy)
    if bad:
        pipeline.stop()
        os.unlink(args.out)
        log.error("recording aborted and %s removed.", args.out)
        return 1

    running = True

    def stop(*_: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    t0 = time.time()
    n = 0
    said = 0.0
    while running and time.time() - t0 < args.seconds:
        try:
            pipeline.wait_for_frames()
        except Exception as exc:              # noqa: BLE001
            log.warning("frame wait failed (%s)", exc)
            continue
        n += 1
        el = time.time() - t0
        if el - said >= 5.0:
            said = el
            log.info("%3.0f s / %3.0f s, %d frame pairs, %.1f GB on disk",
                     el, args.seconds, n,
                     os.path.getsize(args.out) / 1e9 if
                     os.path.exists(args.out) else 0.0)
    pipeline.stop()

    el = max(1e-9, time.time() - t0)
    size = os.path.getsize(args.out) if os.path.exists(args.out) else 0
    log.info("done: %.1f s, %d frame pairs (%.1f fps), %.2f GB, %s",
             el, n, n / el, size / 1e9, args.out)
    # Not a formality. A bag that recorded at a different rate than it claims
    # plays back at that rate, and the demo's timings are all frame-paced.
    if abs(n / el - fps) > 0.15 * fps:
        log.error("recorded at %.1f fps against %d asked for. Playback will "
                  "run the whole stack at that rate.", n / el, fps)
    return 0


if __name__ == "__main__":
    sys.exit(main())
