# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Read a ROS PointCloud2 and turn it into the footprints the navigator uses.

Kept out of the bridge node so it can be tested without ROS installed. The
parsing is deliberately written against the wire format rather than against
sensor_msgs_py, which only exists inside a ROS environment: the same code then
runs in a unit test here and in the container there.
"""
from __future__ import annotations

import struct

import numpy as np

# sensor_msgs/PointField datatype constants.
_DTYPES = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}


def read_xyz(fields, point_step: int, row_step: int, width: int, height: int,
             data: bytes, is_bigendian: bool = False) -> np.ndarray:
    """Extract the x, y, z columns of a PointCloud2 as an (N, 3) array.

    `fields` is a sequence of objects with `name`, `offset` and `datatype`,
    which is what rclpy gives, or of plain tuples in a test.

    Only the three position fields are read. A labelled cloud also carries an
    intensity or label column, and skipping it keeps this independent of which
    extra fields a given producer chose to attach.
    """
    offsets = {}
    for f in fields:
        name = f[0] if isinstance(f, tuple) else f.name
        offset = f[1] if isinstance(f, tuple) else f.offset
        dtype = f[2] if isinstance(f, tuple) else f.datatype
        if name in ("x", "y", "z"):
            offsets[name] = (offset, _DTYPES.get(dtype, "f4"))
    if len(offsets) != 3:
        return np.zeros((0, 3), np.float32)

    n = width * height
    if n == 0 or not data:
        return np.zeros((0, 3), np.float32)
    # Rows can be padded, so index by row rather than assuming a flat buffer.
    raw = np.frombuffer(data, dtype=np.uint8)
    if row_step and height > 1 and row_step != width * point_step:
        raw = raw.reshape(height, row_step)[:, :width * point_step].reshape(-1)
    raw = raw[:n * point_step].reshape(n, point_step)

    order = ">" if is_bigendian else "<"
    out = np.empty((n, 3), np.float32)
    for i, axis in enumerate(("x", "y", "z")):
        off, dt = offsets[axis]
        size = np.dtype(dt).itemsize
        col = raw[:, off:off + size].tobytes()
        out[:, i] = np.frombuffer(col, dtype=order + dt).astype(np.float32)
    return out


def footprints(points: np.ndarray, cell: float = 0.10, margin: float = 0.20,
               min_cells: int = 4, max_range: float = 8.0):
    """Ground rectangles covering clusters of obstacle points.

    The cloud is flattened onto the ground and binned; connected bins become one
    obstacle. Binning rather than clustering in 3D because the navigator wants a
    footprint, and two boxes stacked on a shelf are one obstacle to a robot on
    the floor.

    Points are expected in the ROS body frame: x forward, y left, z up, which is
    the same convention the rest of this project uses.

    Returns (x0, x1, y0, y1) rectangles in metres, already grown by `margin`.
    """
    import cv2

    if points.size == 0:
        return []
    ok = np.isfinite(points).all(axis=1)
    p = points[ok]
    p = p[(np.abs(p[:, 0]) < max_range) & (np.abs(p[:, 1]) < max_range)]
    if p.shape[0] < min_cells:
        return []

    x0, y0 = p[:, 0].min(), p[:, 1].min()
    nx = int((p[:, 0].max() - x0) / cell) + 3
    ny = int((p[:, 1].max() - y0) / cell) + 3
    if nx * ny > 4_000_000:
        return []
    grid = np.zeros((ny, nx), np.uint8)
    ix = ((p[:, 0] - x0) / cell).astype(np.int32) + 1
    iy = ((p[:, 1] - y0) / cell).astype(np.int32) + 1
    grid[iy, ix] = 1

    n, labels, stats, _ = cv2.connectedComponentsWithStats(grid, 8)
    out = []
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < min_cells:
            continue
        cx, cy = stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP]
        w, h = stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
        out.append((round(float(x0 + (cx - 1) * cell) - margin, 3),
                    round(float(x0 + (cx - 1 + w) * cell) + margin, 3),
                    round(float(y0 + (cy - 1) * cell) - margin, 3),
                    round(float(y0 + (cy - 1 + h) * cell) + margin, 3)))
    return out
