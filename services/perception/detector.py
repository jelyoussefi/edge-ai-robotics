# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""YOLOv8 person detection on OpenVINO.

Deliberately small. The preprocessing, async queue depth and device string all
follow the pattern used by the Robotics AI Suite multicam-demo, so a model that
runs there runs here unchanged.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass

import cv2
import numpy as np
import openvino as ov

log = logging.getLogger("detector")

PERSON_CLASS_ID = 0  # COCO
PERSON_HEIGHT_M = 1.70  # assumed, only used for the monocular range estimate


@dataclass
class Detection:
    cx: float  # box centre, normalised 0..1 across the frame
    cy: float
    width: float  # normalised
    height: float
    score: float
    range_m: float  # rough, see estimate_range


class Detector:
    """Single-stream detector. One instance per camera."""

    def __init__(self, model_path: str, device: str, conf: float = 0.4, nms: float = 0.45) -> None:
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
        _, _, self.net_h, self.net_w = self.input_port.shape
        self.device = device
        log.info("%s compiled for %s, input %dx%d", model_path, device, self.net_w, self.net_h)

    def _letterbox(self, frame: np.ndarray) -> tuple[np.ndarray, float, int, int]:
        """Resize preserving aspect ratio, pad to the network input size."""
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
    def estimate_range(box_height_px: float, frame_height_px: int, vfov_deg: float) -> float:
        """Rough distance from apparent person height.

        Pinhole, assuming the person is standing and fully in frame. This is an
        estimate, not a measurement: it is wrong for a seated or partly occluded
        person. Milestone 3 replaces it with actual depth from the D457.
        """
        if box_height_px <= 1:
            return 0.0
        focal_px = frame_height_px / (2.0 * np.tan(np.radians(vfov_deg) / 2.0))
        return float(PERSON_HEIGHT_M * focal_px / box_height_px)

    def infer(self, frame: np.ndarray, vfov_deg: float = 65.0) -> list[Detection]:
        frame_h, frame_w = frame.shape[:2]
        blob, scale, pad_x, pad_y = self._letterbox(frame)

        raw = self.compiled([blob])[self.output_port]

        # YOLOv8 emits [1, 4 + num_classes, num_anchors]. Transpose so each row
        # is one candidate box.
        preds = np.squeeze(raw).T
        scores = preds[:, 4 + PERSON_CLASS_ID]
        keep = scores > self.conf
        if not np.any(keep):
            return []

        boxes_xywh = preds[keep, :4]
        scores = scores[keep]

        # Centre form to corner form, undo the letterbox.
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

        results: list[Detection] = []
        for i in np.asarray(idx).reshape(-1):
            x1, y1, x2, y2 = boxes_xyxy[i]
            box_h = max(1.0, y2 - y1)
            results.append(
                Detection(
                    cx=float(((x1 + x2) / 2) / frame_w),
                    cy=float(((y1 + y2) / 2) / frame_h),
                    width=float((x2 - x1) / frame_w),
                    height=float(box_h / frame_h),
                    score=float(scores[i]),
                    range_m=self.estimate_range(box_h, frame_h, vfov_deg),
                )
            )

        results.sort(key=lambda d: d.range_m)
        return results
