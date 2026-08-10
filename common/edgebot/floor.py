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


def _largest_rect(occ):
    """Largest all-True axis-aligned rectangle in a boolean grid.

    Histogram method, O(rows x cols). Returns (r0, r1, c0, c1) inclusive, or
    None when the grid is empty.
    """
    rows, cols = occ.shape
    heights = np.zeros(cols, np.int32)
    best = None
    best_area = 0
    for r in range(rows):
        heights = np.where(occ[r], heights + 1, 0)
        stack = []
        for c in range(cols + 1):
            h = heights[c] if c < cols else 0
            start = c
            while stack and stack[-1][1] >= h:
                sc, sh = stack.pop()
                area = int(sh) * (c - sc)
                if area > best_area and sh > 0:
                    best_area = area
                    best = (r - int(sh) + 1, r, sc, c - 1)
                start = sc
            stack.append((start, h))
    return best


def cover_rects(occ, max_rects: int = 4):
    """Cover every True cell of `occ` with at most `max_rects` rectangles.

    Greedy largest-first, which is what makes an L come out as its two arms:
    the biggest all-occupied rectangle is one arm, and what remains is the
    other. The LAST rectangle is the bounding box of whatever is still
    uncovered, so the cover is always complete even when the budget runs out.

    Completeness is not negotiable here and cheapness is: an obstacle map that
    misses an occupied cell drives the robot into furniture, while one that
    covers a little extra free floor only makes it walk around something that
    was not there. When the budget is spent the error is therefore taken on the
    conservative side.
    """
    work = occ.copy()
    out = []
    while work.any() and len(out) < max_rects - 1:
        r = _largest_rect(work)
        if r is None:
            break
        out.append(r)
        work[r[0]:r[1] + 1, r[2]:r[3] + 1] = False
    if work.any():
        # The residue, per connected blob rather than as one bounding box.
        # A single AABB over everything still uncovered is a catastrophe on a
        # ragged shape: measured on the couch, greedy covered 3.73 + 2.07 +
        # 0.63 m2 and left 2.35 m2 scattered around the rim, whose bounding box
        # was 12.11 m2 -- the whole extent, exactly the box the decomposition
        # exists to avoid. Blob by blob the same residue costs a few small
        # rectangles instead.
        import cv2
        ncomp, lab = cv2.connectedComponents(work.astype(np.uint8))
        comps = []
        for j in range(1, ncomp):
            rs, cs = np.nonzero(lab == j)
            comps.append((int(rs.min()), int(rs.max()),
                          int(cs.min()), int(cs.max()), int(rs.size)))
        comps.sort(key=lambda t: -t[4])
        budget = max(1, max_rects - len(out))
        if len(comps) <= budget:
            out += [c[:4] for c in comps]
        else:
            out += [c[:4] for c in comps[:budget - 1]]
            rest = comps[budget - 1:]
            out.append((min(c[0] for c in rest), max(c[1] for c in rest),
                        min(c[2] for c in rest), max(c[3] for c in rest)))
    return out


def mask_footprints_xy(mask, fwd, lat, valid, margin: float, min_px: int = 60,
                       min_valid: int = 40, pct: float = 2.0,
                       max_rects: int = 1, cell: float = 0.10):
    """Ground rectangles from each object's OWN measured depth.

    Takes per-pixel world coordinates that were computed from the DEPTH the
    sensor reported, not from the ground-plane assumption, and reduces each
    connected component of the mask to the extent of its own points.

    Why this exists: projecting a silhouette through the ground plane answers
    "where would this pixel be if it lay on the floor". For the contact line
    that is right; for everything above it the answer runs away from the
    camera, without bound as the pixel rises towards the horizon. A couch
    0.9 m deep standing against a wall measured at 6.2 m came out as a
    footprint 4.4 m deep reaching 6.6 m -- past the wall, which is the tell.

    Percentiles rather than min and max, for the same reason as the projected
    version: a handful of stray depth pixels on a distant surface would
    otherwise stretch the rectangle across the room.

    Returns (x_min, x_max, y_min, y_max) per component, already grown by
    `margin`, plus how many components were skipped for want of valid depth --
    a caller that silently produced nothing would look identical to a scene
    with no obstacles in it.
    """
    import cv2
    if mask is None or not mask.any():
        return [], 0
    n, labels = cv2.connectedComponents(mask.astype(np.uint8))
    out, skipped = [], 0
    for i in range(1, n):
        comp = labels == i
        if int(comp.sum()) < min_px:
            continue
        sel = comp & valid
        if int(sel.sum()) < min_valid:
            skipped += 1
            continue
        f, l = fwd[sel], lat[sel]
        x0, x1 = np.percentile(f, pct), np.percentile(f, 100.0 - pct)
        y0, y1 = np.percentile(l, pct), np.percentile(l, 100.0 - pct)
        if max_rects <= 1 or cell <= 0 or (x1 - x0) < cell or (y1 - y0) < cell:
            out.append((round(float(x0) - margin, 3),
                        round(float(x1) + margin, 3),
                        round(float(y0) - margin, 3),
                        round(float(y1) + margin, 3)))
            continue
        # Occupancy of THIS instance on the ground, then a few rectangles over
        # the cells it actually fills. One bounding box around an L-shaped couch
        # claims the inside of the L, which is exactly where the free floor is.
        pad = int(round(margin / cell)) if cell > 0 else 0
        nx = max(1, int(np.ceil((x1 - x0) / cell))) + 2 * pad
        ny = max(1, int(np.ceil((y1 - y0) / cell))) + 2 * pad
        if nx * ny > 40000:            # a runaway extent: fall back to the box
            out.append((round(float(x0) - margin, 3),
                        round(float(x1) + margin, 3),
                        round(float(y0) - margin, 3),
                        round(float(y1) + margin, 3)))
            continue
        inb = (f >= x0) & (f <= x1) & (l >= y0) & (l <= y1)
        gi = np.clip(((f[inb] - x0) / cell).astype(np.int32) + pad, 0, nx - 1)
        gj = np.clip(((l[inb] - y0) / cell).astype(np.int32) + pad, 0, ny - 1)
        occ = np.zeros((nx, ny), bool)
        occ[gi, gj] = True
        # Close single-cell pinholes so a noisy surface does not fragment into
        # a dozen slivers and burn the whole rectangle budget on speckle.
        occ = cv2.morphologyEx(occ.astype(np.uint8), cv2.MORPH_CLOSE,
                               np.ones((3, 3), np.uint8))
        # The margin is applied ONCE, to the shape, by dilating the occupancy
        # before covering it -- not to each rectangle afterwards. Growing every
        # piece by the margin also grows it around the internal faces where the
        # pieces were cut apart, so splitting one box into four made the total
        # obstacle area LARGER than the single box it replaced: measured at
        # 12.10 m2 against 10.69 m2. Dilating the shape keeps the concavity,
        # because a dilated L is still an L.
        k = int(round(margin / cell)) if cell > 0 else 0
        if k > 0:
            occ = cv2.dilate(occ, np.ones((2 * k + 1, 2 * k + 1), np.uint8))
        occ = occ.astype(bool)
        for (r0, r1, c0, c1) in cover_rects(occ, max_rects):
            out.append((round(x0 + (r0 - k) * cell, 3),
                        round(x0 + (r1 + 1 - k) * cell, 3),
                        round(y0 + (c0 - k) * cell, 3),
                        round(y0 + (c1 + 1 - k) * cell, 3)))
    return out, skipped


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


def clip_footprints(boxes, x_max: float, margin: float = 0.0):
    """Truncate footprints at the far wall, dropping any that start beyond it.

    The projection through the ground plane has no idea where the room ends. A
    mask pixel a few rows above the true contact line lands arbitrarily far
    away, and because a footprint is built from percentiles of those projected
    points, one bad column stretches the rectangle past anything real. Measured
    on this room, whose far wall is at 6.2 m: a published footprint reached
    11.12 m, and the suite's clusterer produced two "objects" at 7.4 and 6.9 m
    that cannot exist.

    Truncating rather than dropping, because the near part of such a rectangle
    is usually a real obstacle whose far edge ran away; dropping it would lose a
    genuine barrier. Only a footprint lying ENTIRELY beyond the wall is removed,
    since nothing about it can be salvaged.

    `margin` is the same obstacle margin already added to the boxes, so a
    footprint that merely touches the wall keeps its margin instead of being
    shaved back to it.
    """
    out = []
    limit = x_max + margin
    for x0, x1, y0, y1 in boxes:
        if x0 >= limit:
            continue
        out.append((x0, min(x1, limit), y0, y1))
    return out
