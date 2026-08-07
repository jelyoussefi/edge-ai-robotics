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
    cols = _read_columns(fields, point_step, row_step, width, height, data,
                         is_bigendian, ("x", "y", "z"))
    if cols is None:
        return np.zeros((0, 3), np.float32)
    out = np.empty((cols["x"].shape[0], 3), np.float32)
    for i, axis in enumerate(("x", "y", "z")):
        out[:, i] = cols[axis].astype(np.float32)
    return out


def _raw_rows(point_step: int, row_step: int, width: int, height: int,
              data: bytes):
    """The cloud as an (N, point_step) view of bytes, or None if it is empty.

    Split out of _read_columns for clip_xy, which needs the rows themselves and
    not parsed columns: it republishes points verbatim rather than rebuilding
    them. A view, not a copy -- the caller indexes it and copies only what it
    keeps.
    """
    n = width * height
    if n == 0 or not data:
        return None
    # Rows can be padded, so index by row rather than assuming a flat buffer.
    raw = np.frombuffer(data, dtype=np.uint8)
    if row_step and height > 1 and row_step != width * point_step:
        raw = raw.reshape(height, row_step)[:, :width * point_step].reshape(-1)
    return raw[:n * point_step].reshape(n, point_step)


def clip_xy(fields, point_step: int, row_step: int, width: int, height: int,
            data: bytes, is_bigendian: bool, x_min: float, x_max: float,
            y_min: float, y_max: float):
    """Keep only the points whose x and y fall inside a rectangle.

    Returns `(data, count)` ready to hand to a new PointCloud2, or None when the
    cloud carries no x/y columns to test. None rather than empty bytes on
    purpose: the caller then republishes the cloud uncut, because for a consumer
    that clusters obstacles "I could not read this" must not look like "there is
    nothing here".

    Rows are copied **verbatim**, which is why this works on bytes rather than
    on the (N, 3) array read_xyz returns. The consumer is Intel's node, not us:
    it reads a 4-field point (LiDAR_data_4D_t), and a cloud rebuilt from parsed
    xyz would quietly drop the fourth column.
    """
    rows = _raw_rows(point_step, row_step, width, height, data)
    if rows is None:
        return b"", 0
    cols = _read_columns(fields, point_step, row_step, width, height, data,
                         is_bigendian, ("x", "y"))
    if cols is None:
        return None
    x = cols["x"].astype(np.float32)
    y = cols["y"].astype(np.float32)
    # isfinite as well as the bounds: a NaN compares false against both ends, so
    # it would be dropped anyway, but saying it here means the count reported as
    # "outside the arena" is not quietly inflated by invalid returns.
    keep = (np.isfinite(x) & np.isfinite(y)
            & (x >= x_min) & (x <= x_max) & (y >= y_min) & (y <= y_max))
    return rows[keep].tobytes(), int(keep.sum())


def _read_columns(fields, point_step: int, row_step: int, width: int,
                  height: int, data: bytes, is_bigendian: bool, names):
    """Named columns of a PointCloud2 as a dict of 1-D arrays, or None.

    Split out of read_xyz when the labelled cloud arrived: that one carries a
    `label` column beside x, y, z, and the alternative was a second copy of the
    row-padding and endianness handling that took two tries to get right.

    None rather than empty arrays when a field is missing, so the caller can
    tell "this producer does not publish that column" apart from "the cloud was
    empty this frame".
    """
    offsets = {}
    for f in fields:
        name = f[0] if isinstance(f, tuple) else f.name
        offset = f[1] if isinstance(f, tuple) else f.offset
        dtype = f[2] if isinstance(f, tuple) else f.datatype
        if name in names:
            offsets[name] = (offset, _DTYPES.get(dtype, "f4"))
    if len(offsets) != len(names):
        return None

    raw = _raw_rows(point_step, row_step, width, height, data)
    if raw is None:
        return None

    order = ">" if is_bigendian else "<"
    out = {}
    for name in names:
        off, dt = offsets[name]
        size = np.dtype(dt).itemsize
        col = raw[:, off:off + size].tobytes()
        out[name] = np.frombuffer(col, dtype=order + dt)
    return out


def read_xyz_label(fields, point_step: int, row_step: int, width: int,
                   height: int, data: bytes, is_bigendian: bool = False,
                   label_field: str = "label"):
    """Extract (N, 3) positions and the (N,) integer label column.

    Their `labeled_points` cloud is the node's primary product: every point of
    the input carries the class the segmentation gave it, ground included.
    `obstacle_points` is a filtered view of the same thing, which is why the
    ground can only be read here.

    Returns (zeros((0, 3)), zeros((0,))) when the cloud has no label column, so
    a producer that publishes only x, y, z degrades to "no ground" rather than
    raising in a subscriber callback.
    """
    cols = _read_columns(fields, point_step, row_step, width, height, data,
                         is_bigendian, ("x", "y", "z", label_field))
    if cols is None:
        return np.zeros((0, 3), np.float32), np.zeros((0,), np.int64)
    out = np.empty((cols["x"].shape[0], 3), np.float32)
    for i, axis in enumerate(("x", "y", "z")):
        out[:, i] = cols[axis].astype(np.float32)
    return out, cols[label_field].astype(np.int64)


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


def floor_polygon(points: np.ndarray, cell: float = 0.10,
                  max_range: float = 8.0, close_cells: int = 2,
                  min_area_cells: int = 50, tol_frac: float = 0.01):
    """World-plane outline of the largest ground region.

    The counterpart of footprints() for the ground class: same binning, same
    connectivity, but the biggest component is kept and traced instead of every
    component being boxed. Returning an outline rather than a mask is what makes
    it comparable with the compositor's `roi`, which is also a polygon in
    (forward, lateral) metres.

    Ground returns are speckled -- a depth frame drops points on dark or shiny
    floor, and the raw grid comes out full of one-cell holes that would each
    become an interior contour. MORPH_CLOSE at `close_cells` fills them before
    tracing. It cannot invent floor beyond the outer boundary, which is the
    property that matters when this is about to be compared against ours.

    Only the OUTER boundary is returned, matching what the compositor sends and
    for the same reason: a simple polygon cannot express a hole, and obstacles
    standing away from the walls are carried separately as footprints.
    """
    import cv2

    if points.size == 0:
        return []
    ok = np.isfinite(points).all(axis=1)
    p = points[ok]
    p = p[(np.abs(p[:, 0]) < max_range) & (np.abs(p[:, 1]) < max_range)]
    if p.shape[0] < min_area_cells:
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
    if close_cells > 0:
        k = np.ones((2 * close_cells + 1, 2 * close_cells + 1), np.uint8)
        grid = cv2.morphologyEx(grid, cv2.MORPH_CLOSE, k)

    n, labels, stats, _ = cv2.connectedComponentsWithStats(grid, 8)
    if n <= 1:
        return []
    big = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if int(stats[big, cv2.CC_STAT_AREA]) < min_area_cells:
        return []

    cnts, _ = cv2.findContours((labels == big).astype(np.uint8),
                               cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return []
    c = max(cnts, key=cv2.contourArea)
    poly = cv2.approxPolyDP(c, tol_frac * cv2.arcLength(c, True), True)
    if len(poly) < 3:
        return []
    # Contour coordinates are (column, row) = (x bin, y bin), in that order.
    return [(round(float(x0 + (int(u) - 1) * cell), 3),
             round(float(y0 + (int(v) - 1) * cell), 3))
            for u, v in poly.reshape(-1, 2)]
