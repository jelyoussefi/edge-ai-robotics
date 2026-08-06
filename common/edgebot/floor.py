# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Shared floor-mask geometry.

Lives here rather than in either service because both need exactly the same
answer: the red overlay shown while calibrating has to be the floor the demo
actually walks on, and two copies of this that drifted apart would show one
thing and use another.
"""
from __future__ import annotations

import os

import numpy as np

# cv2 is imported inside the functions that need it, not here. The sim service
# uses signed_area and shrink, which are plain geometry, and its image has no
# OpenCV: a module-level import made it fail to start at all.

# STRAIGHTEN=0 keeps the raw pixel-accurate boundary.
STRAIGHTEN = os.environ.get("STRAIGHTEN", "1") != "0"

# Tolerance of the polygon approximation, as a fraction of each contour's own
# perimeter. Relative rather than absolute so a small region is not flattened
# into a triangle while a large one keeps its ragged edge.
STRAIGHTEN_TOL = float(os.environ.get("STRAIGHTEN_TOL", "0.012"))


def straighten(mask: np.ndarray, tol_frac: float | None = None) -> np.ndarray:
    """Replace the ragged border of a floor mask with straight edges.

    Stereo depth is noisiest exactly where the floor meets a wall or a piece of
    furniture, so the raw boundary wanders several pixels from one column to the
    next even though the real edge is a straight line. Approximating each
    region's contour with a polygon snaps that wobble onto the line it is
    sampling: closer to the truth, and much cleaner to look at.

    Holes are approximated as well and punched back out, so a table leg standing
    in the middle of the floor is not swallowed by its own enclosing contour.
    """
    if not STRAIGHTEN:
        return mask
    import cv2
    tol = STRAIGHTEN_TOL if tol_frac is None else tol_frac
    m = mask.astype(np.uint8)
    cnts, hier = cv2.findContours(m, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return mask
    out = np.zeros_like(m)
    holes = np.zeros_like(m)
    for i, c in enumerate(cnts):
        if len(c) < 4 or cv2.contourArea(c) < 40:
            continue
        poly = cv2.approxPolyDP(c, tol * cv2.arcLength(c, True), True)
        is_hole = hier is not None and hier[0][i][3] >= 0
        cv2.fillPoly(holes if is_hole else out, [poly], 1)
    out[holes > 0] = 0
    return out.astype(bool)


def polygon_from_mask(mask: np.ndarray, to_world, tol_frac: float | None = None,
                      min_area_frac: float = 0.02):
    """World-space polygon of the largest floor region.

    Takes the image-space mask, keeps its biggest connected region, simplifies
    the contour, and projects each vertex onto the ground plane with the
    caller's `to_world(u, v) -> (forward, lateral)`. Projecting the CONTOUR
    rather than re-deriving a shape from depth statistics keeps the polygon
    faithful to the floor that was calibrated and painted, edges included.

    Returns a list of (forward, lateral) in metres, or None.
    """
    import cv2
    m = mask.astype(np.uint8)
    if m.sum() < min_area_frac * m.size:
        return None
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    tol = STRAIGHTEN_TOL if tol_frac is None else tol_frac
    poly = cv2.approxPolyDP(c, tol * cv2.arcLength(c, True), True)
    pts = []
    for p in poly.reshape(-1, 2):
        w = to_world(float(p[0]), float(p[1]))
        if w is not None:
            pts.append(w)
    return pts if len(pts) >= 3 else None


def signed_area(poly) -> float:
    """Twice the signed area. Positive means counter-clockwise in (x, y)."""
    s = 0.0
    for i in range(len(poly)):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % len(poly)]
        s += x1 * y2 - x2 * y1
    return s


def shrink(poly, margin: float, cell: float = 0.02):
    """Move the boundary `margin` metres inward, whatever the shape.

    Rasterised and eroded rather than offset edge by edge. Offsetting works on a
    convex outline but is unstable the moment the polygon has a reflex corner,
    which is exactly what subtracting an obstacle produces: at a narrow spike
    the two offset edges are nearly parallel and their intersection lands metres
    away. Observed in the field as a walkable floor suddenly reported as
    2.1-6.4 m ahead and -8.3-1.1 m across, wider than the room.

    Erosion cannot do that. The cost is a resampled boundary, at `cell`
    resolution, which is well below the accuracy the floor itself has.
    """
    import cv2
    if len(poly) < 3 or margin <= 0:
        return list(poly)
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    pad = margin + 4 * cell
    x0, y0 = min(xs) - pad, min(ys) - pad
    w = int(np.ceil((max(xs) - x0 + pad) / cell))
    h = int(np.ceil((max(ys) - y0 + pad) / cell))
    if w < 3 or h < 3 or w * h > 4_000_000:
        return list(poly)
    img = np.zeros((h, w), np.uint8)
    pts = np.array([[(px - x0) / cell, (py - y0) / cell] for px, py in poly],
                   np.int32)
    cv2.fillPoly(img, [pts], 1)
    k = 2 * int(round(margin / cell)) + 1
    img = cv2.erode(img, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
    cnts, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return []
    c = max(cnts, key=cv2.contourArea)
    c = cv2.approxPolyDP(c, STRAIGHTEN_TOL * cv2.arcLength(c, True), True)
    return [(float(p[0][0]) * cell + x0, float(p[0][1]) * cell + y0) for p in c]


def box_footprints(detections, to_world, dw: int, dh: int, margin: float,
                   min_score: float = 0.45):
    """Ground rectangles occupied by detected objects, from their boxes.

    The bottom edge of a box is where the object meets the floor, so projecting
    its two bottom corners through the ground plane gives the footprint
    directly, with no need for the measured depth. That also sidesteps the trap
    of projecting the whole silhouette: the top of a tall object, projected on
    the plane, would land metres behind where the object actually stands.

    Each footprint is grown by `margin` on every side. Returns a list of
    (x_min, x_max, y_min, y_max) in world metres.
    """
    out = []
    for d in detections or []:
        if float(d.get("score", 1.0)) < min_score:
            continue
        cx, cy = float(d.get("cx", 0.5)), float(d.get("cy", 0.5))
        w, h = float(d.get("w", 0.0)), float(d.get("h", 0.0))
        u1 = (cx - w / 2.0) * dw
        u2 = (cx + w / 2.0) * dw
        v_bottom = (cy + h / 2.0) * dh
        # Slightly inside the box: the very bottom row straddles the contact
        # edge and often reads as background.
        v_bottom = min(dh - 1.0, v_bottom - 0.01 * h * dh)
        p1 = to_world(u1, v_bottom)
        p2 = to_world(u2, v_bottom)
        if p1 is None or p2 is None:
            continue
        xs = (p1[0], p2[0])
        ys = (p1[1], p2[1])
        # A box gives width but says nothing about depth: both bottom corners
        # sit at the same range. Assume the object is as deep as it is wide,
        # which is right for a stool or a person and errs the safe way for a
        # table seen end-on.
        span = max(ys) - min(ys)
        cx_w = (min(xs) + max(xs)) / 2.0
        out.append((cx_w - span / 2.0 - margin, cx_w + span / 2.0 + margin,
                    min(ys) - margin, max(ys) + margin))
    return out


def clear_of_boxes(mask, boxes, project, dw: int = 0, dh: int = 0):
    """Remove from a floor mask every pixel standing inside a footprint.

    Done in world coordinates rather than by drawing the boxes back into the
    image: a rectangle on the ground is not a rectangle on screen, and the
    margin has to be a real distance rather than a pixel count that would mean
    something different at each range.

    `project` takes arrays of pixel coordinates and returns (forward, lateral)
    arrays.
    """
    if not boxes:
        return mask
    ys, xs = np.nonzero(mask)
    if xs.size == 0:
        return mask
    out = mask.copy()
    # Projected in one array operation. Calling to_world per pixel took 247 ms
    # for a 134k-pixel floor, which is a visible stall even a couple of times a
    # second; the same maths on whole arrays is a fraction of a millisecond.
    fwd, lat = project(xs.astype(np.float64), ys.astype(np.float64))
    blocked = np.zeros(xs.shape, bool)
    for x0, x1, y0, y1 in boxes:
        blocked |= (fwd >= x0) & (fwd <= x1) & (lat >= y0) & (lat <= y1)
    out[ys[blocked], xs[blocked]] = False
    return out


def occupied_cells(mask, depth_m, rays, cell: float, margin: float,
                   bounds=(0.0, 10.0, -5.0, 5.0), passable: float = 0.0):
    """Ground cells covered by the obstacle silhouettes, plus a margin.

    Obstacle pixels are projected with their MEASURED depth, not with the ground
    plane: the plane is only true for floor pixels, and using it would throw a
    stool's seat metres away instead of onto its own feet. Every point of an
    object, at whatever height, lands on the same ground cell as its base, which
    is exactly the footprint the robot has to avoid.

    Working in world coordinates rather than in the image means the margin is a
    real distance. Dilating the image mask instead would grow a far obstacle far
    less than a near one, for no reason other than perspective.

    Returns (grid, extent) with grid[j, i] True where occupied.
    """
    import cv2
    x0, x1, y0, y1 = bounds
    nx = max(1, int((x1 - x0) / cell))
    ny = max(1, int((y1 - y0) / cell))
    grid = np.zeros((ny, nx), np.uint8)
    if mask is None or depth_m is None:
        return grid.astype(bool), bounds

    if mask.shape != depth_m.shape:
        mask = cv2.resize(mask.astype(np.uint8),
                          (depth_m.shape[1], depth_m.shape[0]),
                          interpolation=cv2.INTER_NEAREST).astype(bool)
    xr, yr = rays
    sel = mask & (depth_m > 0.2) & (depth_m < 15.0)
    if not sel.any():
        return grid.astype(bool), bounds
    Z = depth_m[sel]
    # Camera frame is x right, y down, z forward; world x is forward and world y
    # is to the left, hence the sign on the lateral term.
    fwd = Z
    lat = -xr[sel] * Z
    ix = ((fwd - x0) / cell).astype(np.int32)
    iy = ((lat - y0) / cell).astype(np.int32)
    ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    grid[iy[ok], ix[ok]] = 1

    if margin > 0:
        k = 2 * int(round(margin / cell)) + 1
        grid = cv2.dilate(grid, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))

    # Close the gaps the robot cannot use. A stool's legs project to two
    # separate patches with free floor between them, and that floor is real, but
    # a 0.35 m slot is not a passage for a 0.44 m robot. Flood filling does not
    # help here: the slot is open towards the back of the grid, where the sensor
    # sees nothing, so it is not an enclosed void. A morphological close with a
    # kernel the width of the robot seals anything narrower than the robot and
    # leaves genuine openings alone.
    if passable > 0:
        k = max(3, 2 * int(round(passable / cell / 2)) + 1)
        grid = cv2.morphologyEx(
            grid, cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
    return grid.astype(bool), bounds


def mask_footprints(mask, project, margin: float, min_px: int = 60,
                    max_depth: float = 12.0):
    """Ground rectangles from the segmentation masks, not from the boxes.

    For each object, the lowest mask pixel in every column is where that part of
    it meets the floor, and projecting those contact points through the ground
    plane gives the real footprint. A box cannot do this: it is a rectangle in
    the IMAGE, so its bottom edge is one distance for the whole object, and a
    table seen at an angle comes out as deep as it is long. That is where the
    2.6 x 4.6 m obstacles came from.

    Returns a list of (x_min, x_max, y_min, y_max) in world metres, already
    grown by `margin`.
    """
    import cv2
    if mask is None or not mask.any():
        return []
    n, labels = cv2.connectedComponents(mask.astype(np.uint8))
    h, w = mask.shape
    out = []
    for i in range(1, n):
        comp = labels == i
        if int(comp.sum()) < min_px:
            continue
        cols = np.nonzero(comp.any(axis=0))[0]
        if cols.size == 0:
            continue
        # Lowest set pixel per column: the contact line with the floor.
        rows = (h - 1) - np.argmax(comp[::-1, :][:, cols], axis=0)
        fwd, lat = project(cols.astype(np.float64), rows.astype(np.float64))
        ok = np.isfinite(fwd) & np.isfinite(lat) & (fwd > 0.2) & (fwd < max_depth)
        if ok.sum() < 3:
            continue
        fwd, lat = fwd[ok], lat[ok]
        # Percentiles rather than min and max: a few stray mask pixels along a
        # wall would otherwise stretch the footprint across the room.
        x0, x1 = np.percentile(fwd, 2), np.percentile(fwd, 98)
        y0, y1 = np.percentile(lat, 2), np.percentile(lat, 98)
        out.append((round(float(x0) - margin, 3), round(float(x1) + margin, 3),
                    round(float(y0) - margin, 3), round(float(y1) + margin, 3)))
    return out
