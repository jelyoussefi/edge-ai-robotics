# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""YOLO11 object detection on OpenVINO, with depth from a RealSense frame.

Two responsibilities kept separate:
  - the YOLO part says WHAT is in the frame and WHERE in the image (a box)
  - the depth part says HOW FAR that box is, read from the aligned depth image

The YOLO11 output tensor has the same [1, 4+nc, anchors] layout as YOLOv8, so
the decode below is unchanged from the v8 version. Only the exported weights
differ. The letterbox round trip and NMS are covered by tests/test_postprocess.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass

import cv2
import numpy as np
import openvino as ov

log = logging.getLogger("detector")

# COCO classes worth avoiding. Kept broad on purpose: a mobile robot should
# steer around chairs and bags, not only people. Empty set means keep all.
OBSTACLE_CLASSES = {
    0,   # person
    24,  # backpack
    26,  # handbag
    28,  # suitcase
    56,  # chair
    57,  # couch
    59,  # bed
    60,  # dining table
    39,  # bottle
    41,  # cup
    63,  # laptop
    73,  # book
}

# Distance returned when depth is genuinely unknown (video source, or the
# depth pixels in the box were all invalid). The behaviour treats this as far.
UNKNOWN_RANGE = float("inf")


@dataclass
class Obstacle:
    cx: float          # box centre, normalised 0..1 across the frame
    cy: float
    width: float       # normalised
    height: float
    score: float
    range_m: float     # measured from aligned depth, or inf if unknown
    bearing_deg: float # +right / -left of camera centre, from HFOV
    class_id: int
    measured: bool     # True if range_m came from real depth


class Detector:
    """One detector. Runs YOLO on the color image, samples depth per box."""

    def __init__(self, model_path: str, device: str, conf: float = 0.4,
                 nms: float = 0.45, keep_masks: bool = False) -> None:
        self.conf = conf
        self.nms = nms

        core = ov.Core()
        available = core.available_devices
        if device not in available:
            log.warning("device %s not available (have %s), using CPU", device, available)
            device = "CPU"

        model = core.read_model(model_path)
        self.compiled = core.compile_model(model, device)
        self.input_port = self.compiled.input(0)
        self.output_port = self.compiled.output(0)
        # A -seg model has a second output, the mask prototypes: (1, 32, h, w).
        # Its first output carries 32 extra channels per anchor, the coefficients
        # that combine those prototypes into one mask. Detect this rather than
        # requiring the caller to say which kind of model it loaded, so the same
        # code runs either way and a plain detector still works.
        self.proto_port = None
        self.nm = 0
        if len(self.compiled.outputs) > 1:
            for port in list(self.compiled.outputs)[1:]:
                shape = list(port.partial_shape)
                if len(shape) == 4 and shape[1].is_static:
                    self.proto_port = port
                    self.nm = int(shape[1].get_length())
                    break
        if self.proto_port is not None:
            log.info("segmentation model: %d mask prototypes, silhouettes "
                     "instead of boxes", self.nm)
        _, _, self.net_h, self.net_w = self.input_port.shape
        self.device = device
        # Per-detection silhouettes, off by default. The runtime only needs the
        # union and a list of boolean images per frame would be pure cost; the
        # calibration assist needs them to attribute floor pixels to the couch
        # rather than to the table. Nothing changes when this is False.
        self.keep_masks = keep_masks
        self.det_masks: list | None = None
        log.info("%s compiled for %s, input %dx%d", model_path, device, self.net_w, self.net_h)

    def _letterbox(self, frame: np.ndarray) -> tuple[np.ndarray, float, int, int]:
        h, w = frame.shape[:2]
        scale = min(self.net_w / w, self.net_h / h)
        new_w, new_h = int(round(w * scale)), int(round(h * scale))
        resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        canvas = np.full((self.net_h, self.net_w, 3), 114, dtype=np.uint8)
        pad_x, pad_y = (self.net_w - new_w) // 2, (self.net_h - new_h) // 2
        canvas[pad_y : pad_y + new_h, pad_x : pad_x + new_w] = resized

        blob = canvas.transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        return blob, scale, pad_x, pad_y

    @staticmethod
    def _sample_depth(depth: np.ndarray, scale: float, x1: int, y1: int, x2: int, y2: int) -> float:
        """Median valid depth over the central region of a box, in metres.

        A central patch avoids the box edges, which straddle the object border
        and pick up background depth. Median rejects the salt-and-pepper zero
        pixels the sensor leaves on edges and dark surfaces. Returns inf if no
        valid depth remains.
        """
        h, w = depth.shape
        # Shrink to the middle half of the box.
        bw, bh = x2 - x1, y2 - y1
        sx1 = max(0, int(x1 + 0.25 * bw))
        sy1 = max(0, int(y1 + 0.25 * bh))
        sx2 = min(w, int(x2 - 0.25 * bw))
        sy2 = min(h, int(y2 - 0.25 * bh))
        if sx2 <= sx1 or sy2 <= sy1:
            return UNKNOWN_RANGE

        patch = depth[sy1:sy2, sx1:sx2]
        valid = patch[patch > 0]
        if valid.size == 0:
            return UNKNOWN_RANGE
        return float(np.median(valid) * scale)

    def _add_mask(self, protos, coeff, box, scale, pad_x, pad_y, fw, fh) -> None:
        """Add one object's silhouette to the combined mask.

        The prototypes are a small basis, 32 images at a quarter of the network
        input; a detection's mask is their weighted sum through a sigmoid. It is
        computed in the network's letterboxed frame, then cropped to the box and
        mapped back to the camera frame, undoing the letterbox padding and scale.

        Cropping to the box matters: the prototype combination responds weakly
        all over the image, and without the crop a chair's mask would smear onto
        every other chair-like patch in the scene.
        """
        p = np.squeeze(protos)                    # (nm, ph, pw)
        nmk, ph, pw = p.shape
        m = 1.0 / (1.0 + np.exp(-(coeff @ p.reshape(nmk, -1)).reshape(ph, pw)))
        # Box in letterboxed coordinates, then in prototype coordinates.
        x1, y1, x2, y2 = box
        sx, sy = pw / float(self.net_w), ph / float(self.net_h)
        bx1 = int(np.clip((x1 * scale + pad_x) * sx, 0, pw - 1))
        by1 = int(np.clip((y1 * scale + pad_y) * sy, 0, ph - 1))
        bx2 = int(np.clip((x2 * scale + pad_x) * sx, bx1 + 1, pw))
        by2 = int(np.clip((y2 * scale + pad_y) * sy, by1 + 1, ph))
        crop = np.zeros_like(m, dtype=bool)
        crop[by1:by2, bx1:bx2] = m[by1:by2, bx1:bx2] > 0.5
        # Prototypes -> letterboxed input -> original frame.
        big = cv2.resize(crop.astype(np.uint8), (self.net_w, self.net_h),
                         interpolation=cv2.INTER_NEAREST)
        ix1, iy1 = int(round(pad_x)), int(round(pad_y))
        ix2 = int(round(self.net_w - pad_x)); iy2 = int(round(self.net_h - pad_y))
        inner = big[max(0, iy1):max(iy1 + 1, iy2), max(0, ix1):max(ix1 + 1, ix2)]
        if inner.size == 0:
            return
        painted = cv2.resize(inner, (fw, fh),
                             interpolation=cv2.INTER_NEAREST).astype(bool)
        self.mask |= painted
        if self.keep_masks and self.det_masks is not None:
            self.det_masks.append(painted)
        return painted

    def infer(self, frame) -> list[Obstacle]:
        """frame carries colour and optional depth. Uses frame.color for detection and
        frame.depth (if present) for distance."""
        color = frame.color
        frame_h, frame_w = color.shape[:2]
        blob, scale, pad_x, pad_y = self._letterbox(color)

        result = self.compiled([blob])
        raw = result[self.output_port]
        protos = result[self.proto_port] if self.proto_port is not None else None

        preds = np.squeeze(raw).T  # [anchors, 4 + nc (+ nm)]
        # With a seg model the last nm columns are mask coefficients, not
        # classes; including them in the argmax would invent classes that do not
        # exist and wreck the scores.
        nc = preds.shape[1] - 4 - self.nm
        coeffs = preds[:, 4 + nc:] if self.nm else None
        class_scores = preds[:, 4:4 + nc]
        class_ids = np.argmax(class_scores, axis=1)
        scores = class_scores[np.arange(len(class_ids)), class_ids]

        keep = scores > self.conf
        if OBSTACLE_CLASSES:
            keep &= np.isin(class_ids, list(OBSTACLE_CLASSES))
        if not np.any(keep):
            return []

        boxes_xywh = preds[keep, :4]
        if coeffs is not None:
            coeffs = coeffs[keep]
        scores = scores[keep]
        class_ids = class_ids[keep]

        x, y, w, h = boxes_xywh.T
        boxes_xyxy = np.stack(
            [
                (x - w / 2 - pad_x) / scale,
                (y - h / 2 - pad_y) / scale,
                (x + w / 2 - pad_x) / scale,
                (y + h / 2 - pad_y) / scale,
            ],
            axis=1,
        )

        idx = cv2.dnn.NMSBoxes(
            [[float(b[0]), float(b[1]), float(b[2] - b[0]), float(b[3] - b[1])] for b in boxes_xyxy],
            scores.astype(float).tolist(),
            self.conf,
            self.nms,
        )
        if len(idx) == 0:
            return []

        hfov = frame.intrinsics["hfov_deg"] if frame.intrinsics else 65.0

        # One combined silhouette for the whole frame. Per-object masks are not
        # needed downstream and a single boolean image is far cheaper to send
        # than a list of them.
        self.mask = (np.zeros((frame_h, frame_w), bool)
                     if protos is not None else None)
        # Instance labels beside the union: 0 is nothing, k is the k-th mask
        # painted. The union alone cannot say where one object ends and the
        # next begins, which is exactly the question a coffee table standing in
        # front of a couch asks. uint8 caps this at 255 instances, far above
        # anything this detector returns after NMS.
        self.inst_map = (np.zeros((frame_h, frame_w), np.uint8)
                         if protos is not None else None)
        # (class_id, score) for label k, at index k-1. Filled in PAINTING
        # order, which is not the order of `results`: those are sorted by range
        # before being returned, so pairing by position would mislabel them.
        self.inst_meta: list[tuple[int, float]] = []
        self.det_masks = [] if self.keep_masks else None

        results: list[Obstacle] = []
        for i in np.asarray(idx).reshape(-1):
            x1, y1, x2, y2 = [int(round(v)) for v in boxes_xyxy[i]]
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(frame_w, x2), min(frame_h, y2)

            cx_norm = ((x1 + x2) / 2) / frame_w
            bearing = (cx_norm - 0.5) * hfov  # + right, - left

            _painted = None
            if protos is not None and x2 > x1 and y2 > y1:
                _painted = self._add_mask(protos, coeffs[i], (x1, y1, x2, y2),
                                          scale, pad_x, pad_y, frame_w, frame_h)
                if _painted is not None and self.inst_map is not None:
                    # Later masks overwrite earlier ones where they overlap.
                    # An arbitrary rule, but a deterministic one, and overlap
                    # is a handful of pixels on a silhouette boundary.
                    _k = len(self.inst_meta) + 1
                    if _k <= 255:
                        self.inst_map[_painted] = _k
                        self.inst_meta.append((int(class_ids[i]),
                                               float(scores[i])))
            if self.keep_masks and self.det_masks is not None and _painted is None:
                # Index alignment with `results` matters more than compactness:
                # the caller pairs mask[k] with obstacle[k].
                self.det_masks.append(None)

            if frame.has_depth:
                rng = self._sample_depth(frame.depth, frame.depth_scale, x1, y1, x2, y2)
                measured = rng != UNKNOWN_RANGE
            else:
                rng = UNKNOWN_RANGE
                measured = False

            results.append(
                Obstacle(
                    cx=cx_norm,
                    cy=float(((y1 + y2) / 2) / frame_h),
                    width=float((x2 - x1) / frame_w),
                    height=float((y2 - y1) / frame_h),
                    score=float(scores[i]),
                    range_m=rng,
                    bearing_deg=float(bearing),
                    class_id=int(class_ids[i]),
                    measured=measured,
                )
            )

        results.sort(key=lambda o: o.range_m)
        return results
