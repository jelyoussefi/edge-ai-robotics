#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Check the segmentation model on its own.

Runs the detector over one image, or over the live camera, and writes an
annotated picture showing each object's mask, box, class and score. Nothing else
runs: no bus, no simulator, no compositing. When a footprint looks wrong in the
demo the question is always the same, is the mask wrong or is the projection
wrong, and this answers the first half without the second in the way.

    make seg-test                      one frame from the camera
    make seg-test SEG_ARGS="--image /data/shot.png"
    make seg-test SEG_ARGS="--frames 20 --out /data/seg"

Exit status is 1 when nothing is detected, so it can be used as a smoke test.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, "/opt/edgebot")
from edgebot import topics

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# The 80 COCO names, so the report says "chair" rather than "class 56".
COCO = (
    "person bicycle car motorcycle airplane bus train truck boat traffic_light "
    "fire_hydrant stop_sign parking_meter bench bird cat dog horse sheep cow "
    "elephant bear zebra giraffe backpack umbrella handbag tie suitcase frisbee "
    "skis snowboard sports_ball kite baseball_bat baseball_glove skateboard "
    "surfboard tennis_racket bottle wine_glass cup fork knife spoon bowl banana "
    "apple sandwich orange broccoli carrot hot_dog pizza donut cake chair couch "
    "potted_plant bed dining_table toilet tv laptop mouse remote keyboard "
    "cell_phone microwave oven toaster sink refrigerator book clock vase "
    "scissors teddy_bear hair_drier toothbrush"
).split()


def _palette(n: int) -> np.ndarray:
    """Distinct colours, so two adjacent objects never share one."""
    hues = np.linspace(0, 179, max(1, n), endpoint=False).astype(np.uint8)
    hsv = np.stack([hues, np.full_like(hues, 220), np.full_like(hues, 255)], 1)
    return cv2.cvtColor(hsv[None], cv2.COLOR_HSV2BGR)[0]


def annotate(frame: np.ndarray, dets, mask) -> np.ndarray:
    """Draw the masks, boxes and labels onto a copy of the frame."""
    out = frame.copy()
    h, w = out.shape[:2]
    if mask is not None and mask.any():
        m = mask if mask.shape == (h, w) else cv2.resize(
            mask.astype(np.uint8), (w, h), interpolation=cv2.INTER_NEAREST).astype(bool)
        out[m] = (0.45 * out[m].astype(np.float32)
                  + 0.55 * np.array([0, 200, 255], np.float32)).astype(np.uint8)
    colours = _palette(len(dets) or 1)
    for i, d in enumerate(dets):
        c = tuple(int(v) for v in colours[i % len(colours)])
        x1 = int((d.cx - d.width / 2) * w)
        y1 = int((d.cy - d.height / 2) * h)
        x2 = int((d.cx + d.width / 2) * w)
        y2 = int((d.cy + d.height / 2) * h)
        cv2.rectangle(out, (x1, y1), (x2, y2), c, 2)
        name = COCO[d.class_id] if 0 <= d.class_id < len(COCO) else str(d.class_id)
        rng = "" if not np.isfinite(d.range_m) else f" {d.range_m:.1f}m"
        cv2.putText(out, f"{name} {d.score:.2f}{rng}", (x1, max(14, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 2, cv2.LINE_AA)
    return out


def report(dets, mask, frame_shape) -> None:
    """Say what was found, and whether the masks are usable."""
    h, w = frame_shape[:2]
    if not dets:
        print("  nothing detected")
        return
    print(f"  {len(dets)} detection(s):")
    for d in dets:
        name = COCO[d.class_id] if 0 <= d.class_id < len(COCO) else str(d.class_id)
        rng = "no depth" if not np.isfinite(d.range_m) else f"{d.range_m:.2f} m"
        print(f"    {name:14s} score {d.score:.2f}  box {d.width * w:4.0f}x"
              f"{d.height * h:4.0f} px  {rng}")
    if mask is None:
        print("  no masks: the model is a plain detector, not a -seg one")
        return
    box_px = sum(d.width * w * d.height * h for d in dets)
    mask_px = int(mask.sum())
    print(f"  masks cover {mask_px} px, boxes cover {box_px:.0f} px "
          f"({100.0 * mask_px / max(1.0, box_px):.0f}% of the box area)")
    if mask_px == 0:
        print("  the masks are EMPTY though the model reports prototypes: the "
              "coefficients or the crop are wrong, not the detections")


def _from_bus(count: int, timeout: float):
    """Take frames from the running source service rather than the camera.

    Opening the RealSense directly needs a privileged container and, worse,
    fails outright while the demo holds the device: a camera has one client.

    Uses the project's own Subscriber rather than a hand-rolled ZeroMQ socket.
    The first attempt did the latter and connected to BUS_PUB, which is the
    broker's XSUB side where publishers push; subscribers belong on BUS_SUB.
    Reusing the class makes that impossible to get wrong.
    """
    from edgebot.bus import Subscriber

    sub = Subscriber([topics.CAMERA_RGB, topics.CAMERA_DEPTH])
    out, colour, depth, scale = [], None, None, 0.001
    deadline = time.time() + timeout
    while len(out) < count and time.time() < deadline:
        msg = sub.recv(200)
        if msg is None:
            continue
        topic, payload = msg
        if topic == topics.CAMERA_RGB:
            colour = cv2.imdecode(np.frombuffer(payload["jpeg"], np.uint8),
                                  cv2.IMREAD_COLOR)
        elif topic == topics.CAMERA_DEPTH:
            depth = np.frombuffer(payload["depth"], np.uint16).reshape(
                payload["h"], payload["w"])
            scale = float(payload.get("scale", 0.001))
        # Only once both have arrived: the detector wants depth for its ranges,
        # and colour and depth are published on separate schedules.
        if colour is not None and depth is not None:
            out.append(_BusFrame(colour, depth, scale))
            colour = depth = None
    return out


class _BusFrame:
    """The little of a camera frame the detector needs."""

    def __init__(self, colour, depth=None, scale=0.001):
        self.color, self.depth = colour, depth
        self.depth_scale = scale
        self.has_depth = depth is not None
        hfov = 2 * np.degrees(np.arctan(colour.shape[1] / (2 * 385.9)))
        self.intrinsics = {"hfov_deg": hfov}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default=os.environ.get(
        "SEG_MODEL", "/assets/models/yolo11m-seg/FP16/yolo11m-seg.xml"))
    ap.add_argument("--device", default=os.environ.get("SEG_DEVICE", "NPU"))
    ap.add_argument("--image", default="", help="run on this file instead of the camera")
    ap.add_argument("--frames", type=int, default=1, help="camera frames to grab")
    ap.add_argument("--out", default="/data/seg_test", help="output prefix")
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--timeout", type=float, default=15.0,
                    help="seconds to wait for frames on the bus")
    args = ap.parse_args()

    from detector import Detector

    if not os.path.exists(args.model):
        print(f"  {args.model} is missing. Fetch it with: make build")
        return 1
    det = Detector(args.model, args.device, conf=args.conf)
    if getattr(det, "proto_port", None) is None:
        print("  this model has no mask prototypes: it is a detector, not -seg")

    frames = []
    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            print(f"  cannot read {args.image}")
            return 1
        frames.append(_BusFrame(img, None, 0.001))
    else:
        print(f"  waiting up to {args.timeout:.0f} s for camera frames on the "
              f"bus ...")
        frames = _from_bus(args.frames, args.timeout)
        if not frames:
            print("  nothing arrived. The source service publishes the frames, "
                  "so the stack has to be running: start it in another "
                  "terminal with 'make', then run this again. To work from a "
                  "file instead, pass --image.")
            return 1

    found = 0
    for i, f in enumerate(frames):
        dets = det.infer(f)
        found += len(dets)
        print(f"frame {i + 1}/{len(frames)}:")
        report(dets, getattr(det, "mask", None), f.color.shape)
        path = f"{args.out}_{i:02d}.png"
        cv2.imwrite(path, annotate(f.color, dets, getattr(det, "mask", None)))
        print(f"  wrote {path}")

    return 0 if found else 1


if __name__ == "__main__":
    raise SystemExit(main())
