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
import json
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


class MaskAccumulator:
    """Union of the detector's silhouettes over several frames.

    CONTOURS ONLY. The bounding box is never used, and that is the whole point:
    measured on this scene the couch box is 546x294 px while its silhouette is
    6.7 % of the frame -- mask/box = 38.5 %, so the rectangle is 61.5 % empty
    and it swallows the coffee table almost entirely (99.3 % of the table's box
    in x, 100 % in y). The two silhouettes do not touch. Subtracting boxes would
    therefore remove the table's floor as well as the couch's, and remove a
    great deal of real floor besides.

    Accumulated over frames because the camera is fixed and weak detections
    flicker: the dining table is present in roughly a quarter of frames, so one
    pass is a coin toss. The union of N passes is not.
    """

    def __init__(self, shape) -> None:
        self.h, self.w = shape[:2]
        self.union = np.zeros((self.h, self.w), bool)
        self.per_class: dict[int, np.ndarray] = {}
        self.scores: dict[int, list] = {}
        self.frames = 0

    def add(self, det, dets, keep) -> None:
        self.frames += 1
        masks = getattr(det, "det_masks", None) or []
        for k, d in enumerate(dets):
            if keep and d.class_id not in keep:
                continue
            m = masks[k] if k < len(masks) else None
            if m is None:                     # no prototype for this detection
                continue
            if m.shape != (self.h, self.w):
                m = cv2.resize(m.astype(np.uint8), (self.w, self.h),
                               interpolation=cv2.INTER_NEAREST).astype(bool)
            self.union |= m
            prev = self.per_class.get(d.class_id)
            self.per_class[d.class_id] = m if prev is None else (prev | m)
            self.scores.setdefault(d.class_id, []).append(float(d.score))

    def write(self, path, classes_prefix="", manifest_path="") -> None:
        cv2.imwrite(path, self.union.astype(np.uint8) * 255)
        print(f"  wrote {path}: contours over {self.frames} frame(s), "
              f"{100.0 * self.union.mean():.2f} % of the frame")
        rows = []
        for cid in sorted(self.per_class):
            m = self.per_class[cid]
            sc = sorted(self.scores[cid])
            name = COCO[cid] if 0 <= cid < len(COCO) else str(cid)
            rows.append({"class_id": int(cid), "name": name,
                         "detections": len(sc), "frames": self.frames,
                         "score_min": round(sc[0], 3),
                         "score_med": round(sc[len(sc) // 2], 3),
                         "score_max": round(sc[-1], 3),
                         "mask_px": int(m.sum()),
                         "mask_pct": round(100.0 * m.mean(), 3)})
            # detections, not frames: several books can appear in one frame,
            # so this count can legitimately exceed the frame count.
            print(f"    {name:14s} id {cid:2d}  {len(sc):3d} det over "
                  f"{self.frames} frames  "
                  f"score {sc[0]:.2f}/{sc[len(sc) // 2]:.2f}/{sc[-1]:.2f} "
                  f"(min/med/max)  mask {100.0 * m.mean():5.2f} % of frame")
            if classes_prefix:
                cv2.imwrite(f"{classes_prefix}-{cid:02d}.png",
                            m.astype(np.uint8) * 255)
        if manifest_path:
            with open(manifest_path, "w") as fh:
                json.dump({"frames": self.frames,
                           "union_px": int(self.union.sum()),
                           "union_pct": round(100.0 * self.union.mean(), 3),
                           "classes": rows}, fh, indent=1)
            print(f"  wrote {manifest_path}")


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
    ap.add_argument("--mask-out", default="", dest="mask_out",
                    help="write the union of the silhouette CONTOURS here as an "
                         "8-bit PNG, for `make calibrate` to subtract from the "
                         "floor. Never a bounding box.")
    ap.add_argument("--mask-classes-out", default="", dest="mask_classes_out",
                    help="prefix for one PNG per class, used to attribute floor "
                         "pixels to a particular object")
    ap.add_argument("--manifest", default="",
                    help="write the per-class report as JSON here")
    ap.add_argument("--images", default="",
                    help="glob of frames to accumulate over, e.g. "
                         "/data/calib-frame-*.png. The camera is fixed, so the "
                         "union of several passes catches flickering objects "
                         "that a single frame misses.")
    ap.add_argument("--classes", default="",
                    help="comma-separated COCO class ids to keep in the mask; "
                         "empty means every detection")
    ap.add_argument("--timeout", type=float, default=15.0,
                    help="seconds to wait for frames on the bus")
    args = ap.parse_args()

    from detector import Detector

    if not os.path.exists(args.model):
        print(f"  {args.model} is missing. Fetch it with: make build")
        return 1
    det = Detector(args.model, args.device, conf=args.conf,
                   keep_masks=bool(args.mask_out))
    if getattr(det, "proto_port", None) is None:
        print("  this model has no mask prototypes: it is a detector, not -seg")

    frames = []
    if args.images:
        import glob as _glob
        for path in sorted(_glob.glob(args.images)):
            img = cv2.imread(path)
            if img is not None:
                frames.append(_BusFrame(img, None, 0.001))
        if not frames:
            print(f"  no frames matched {args.images}")
            return 1
        print(f"  {len(frames)} frame(s) matched {args.images}")
    elif args.image:
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

    keep = {int(c) for c in args.classes.split(",") if c.strip()}
    acc = MaskAccumulator(frames[0].color.shape) if args.mask_out else None
    found = 0
    for i, f in enumerate(frames):
        dets = det.infer(f)
        found += len(dets)
        if acc is not None:
            acc.add(det, dets, keep)
        print(f"frame {i + 1}/{len(frames)}:")
        report(dets, getattr(det, "mask", None), f.color.shape)
        path = f"{args.out}_{i:02d}.png"
        cv2.imwrite(path, annotate(f.color, dets, getattr(det, "mask", None)))
        print(f"  wrote {path}")

    if acc is not None:
        acc.write(args.mask_out, args.mask_classes_out, args.manifest)

    return 0 if found else 1


if __name__ == "__main__":
    raise SystemExit(main())
