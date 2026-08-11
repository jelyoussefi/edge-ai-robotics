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
        return [], 0, []
    n, labels = cv2.connectedComponents(mask.astype(np.uint8))
    out, skipped, inst = [], 0, []
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
            inst.append(i)
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
            inst.append(i)
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
            inst.append(i)
    return out, skipped, inst


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


def in_band(fwd, x_min: float, x_max: float):
    """Mask of points whose forward distance is inside the arena, both ends.

    ONE expression, used by every path that has to reject a projected point for
    being somewhere the room is not. It exists because this repository's most
    expensive recurring bug is a guard applied to one of two paths:

      - the footprint was computed twice and the far-wall guard fixed once
      - clip_footprints protected the published RECTANGLES with FOOTPRINT_X_MAX
        while nothing protected the GRID, which is what the navigator steers
        on. Measured: "ground grid 1.2-7.8 m" in a room whose far wall is at
        6.2 m
      - and the mirror of it, still open when this was written: the grid gained
        a near guard (OBSTACLE_X_MIN) and clip_footprints never did, so a
        rectangle could still be published at 1.0 m in front of a camera that
        cannot see the floor closer than 1.4 m

    Two callers sharing this cannot drift apart. A third that writes the
    comparison out by hand can, so do not.
    """
    return (np.asarray(fwd) > x_min) & (np.asarray(fwd) < x_max)


def clip_footprints(boxes, x_max: float, margin: float = 0.0,
                    x_min: float = 0.0):
    """Truncate footprints at the far wall, dropping any that start beyond it.

    `x_min` is the same near guard the grid applies. Below it the camera has no
    floor to project onto and a rectangle there describes nothing; it was
    missing here for as long as the grid has had it, which is the asymmetry
    etape 6 exists to find. Defaults to 0.0 so an existing caller that does not
    pass it keeps its behaviour rather than silently gaining a filter.

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
    near = x_min - margin if x_min > 0 else None
    for x0, x1, y0, y1 in boxes:
        if x0 >= limit:
            continue
        if near is not None and x1 <= near:
            continue
        out.append((max(x0, near) if near is not None else x0,
                    min(x1, limit), y0, y1))
    return out


# ---------------------------------------------------------------------------
# Ground occupancy, as cells rather than rectangles.
#
# Every rectangle-based representation in this file shares one defect: an
# axis-aligned box around a concave object claims the floor the object
# surrounds. Measured on this room's L-shaped couch, the box was 3.35 x 3.19 m
# = 10.7 m2 for a couch occupying about 5.1 m2, and the 5.6 m2 it swallowed
# were the coffee table and both walking corridors. Decomposing the box into
# several boxes reduces that but cannot remove it: covering a depth-ragged
# outline exactly needs so many rectangles that the budget always ends in one
# that crosses the passage.
#
# Cells have no such failure mode, because a cell is occupied or it is not.
# The cost is a fixed grid on the bus, 3.2 kB at 5 cm over 8 x 8 m, which is
# smaller than the silhouette mask already published every frame.
# ---------------------------------------------------------------------------

GRID_BOUNDS = (0.0, 8.0, -4.0, 4.0)   # x_min, x_max, y_min, y_max, world metres
GRID_CELL = 0.05


def grid_shape(cell: float = GRID_CELL, bounds=GRID_BOUNDS):
    """(nx, ny) of the grid these bounds and this cell describe."""
    x0, x1, y0, y1 = bounds
    return max(1, int(round((x1 - x0) / cell))), max(1, int(round((y1 - y0) / cell)))


# --------------------------------------------------------------- clearance
#
# WHERE the clearance is applied, and the two answers this project has given.
#
#   "dilate"  points_to_grid grows every occupied cell by CLEARANCE, and every
#             corridor query then asks for the robot's bare half-width.
#   "query"   the grid stays RAW -- obstacles and nothing else -- and every
#             corridor query asks for half-width + CLEARANCE.
#
# Both demand the same corridor of real floor, and min_corridor() below is the
# single expression that says how much. They are NOT the same test:
#
#   Dilation is a Minkowski sum with the structuring element, and ours is an
#   ELLIPSE, so it rounds corners: the grown obstacle is what a CIRCULAR robot
#   of that radius could not enter. The query test grows the swept rectangle
#   instead, which is what a RECTANGULAR robot could not enter. Diagonally past
#   a corner the ellipse is the more permissive of the two, and the difference
#   is largest exactly where a robot rounds furniture.
#
#   Dilation also DESTROYS INFORMATION. Once the cells are grown there is no
#   way to ask the grid a question at any other width -- so free_lane's own
#   "how wide a lane WOULD have been clear" sweep, and every probe that tries
#   to reproduce the navigator's decision, are answering about a grid that no
#   longer describes the obstacles. That is the mechanism behind lane_probe and
#   the navigator disagreeing, and no amount of keeping two knobs equal fixes
#   it.
#
# A raw obstacle layer with the inflation applied at query time is also what
# Nav2 does, which is why etape 4 gets cheaper if this is where we land.
CLEARANCE_MODES = ("dilate", "query")


def clearance_mode(value: str | None) -> str:
    """Validate a mode name. A typo must not silently pick a behaviour."""
    v = (value or "query").strip().lower()
    if v not in CLEARANCE_MODES:
        raise SystemExit(f"CLEARANCE_MODE={value!r} is not one of "
                         f"{CLEARANCE_MODES}")
    return v


def query_half(robot_half: float, clearance: float, mode: str) -> float:
    """Half-width to hand corridor_blocked, given where the margin lives.

    THE one place that knows. Every corridor query in the navigator and in
    every probe goes through this, so a probe cannot ask a different question
    from the robot by getting the arithmetic subtly right in its own way.
    """
    return robot_half + clearance if clearance_mode(mode) == "query" else robot_half


def query_pad(clearance: float, mode: str) -> float:
    """Longitudinal inflation for a corridor query, given the mode.

    The other half of query_half. Dilation grows an obstacle in EVERY
    direction; widening the query corridor grows it across only. Without this,
    query mode keeps the robot clear of a wall beside it and lets it walk into
    the couch in front. Zero in dilate mode, where the cells already carry it
    on both axes.
    """
    return clearance if clearance_mode(mode) == "query" else 0.0


def min_corridor(robot_half: float, clearance: float) -> float:
    """Narrowest gap of REAL FLOOR the robot will walk through, in metres.

    Deliberately independent of the mode: choosing where to apply the margin
    must not change how much margin there is. If these two ever disagree the
    mode has become a tuning knob, which is the failure this replaces.
    """
    return 2.0 * (robot_half + clearance)


def assert_same_corridor(mine: dict, published: dict | None,
                         who: str = "this probe") -> None:
    """Abort unless we ask the grid EXACTLY what the navigator asks it.

    Checked rather than re-read. Every previous attempt to keep a probe and the
    robot in step relied on a person comparing two files, and it failed every
    time -- LANE_SLACK set in one container and not the other, DETOUR_MAX 1.8
    against 2.4, a margin applied twice on one path and once on the other. A
    number that must match should be compared by the machine, and the machine
    should refuse to print anything if it does not.

    What this does and does not catch. The probe adopts the navigator's knobs
    off the bus first, so this is not a test that two environments agree -- it
    is a test that, GIVEN the same inputs, the two arrive at the same question.
    It therefore catches the thing that actually bites here: the shared helpers
    below disagreeing between processes, because `common/edgebot` is BAKED into
    the sim image and MOUNTED from the tree into a probe container. An edit to
    this file that has not been rebuilt into the sim shows up as a mismatch
    instead of as two plausible reports.

    Compares the numbers that decide, not a summary: half-width and
    longitudinal pad are what reach corridor_blocked, and two different
    (half, pad) pairs can share a min_corridor.
    """
    if not published:
        return
    bad = [k for k, v in mine.items()
           if published.get(k) is not None
           and abs(float(v) - float(published[k])) > 1e-6]
    if bad:
        detail = ", ".join(f"{k}: {who} {mine[k]:.3f} vs navigator "
                           f"{float(published[k]):.3f}" for k in bad)
        raise SystemExit(
            f"{who} would ask the grid a different question from the robot -- "
            f"{detail}. Refusing to report numbers about a question the robot "
            f"is not asking. Most likely common/edgebot is stale in one of the "
            f"two images: rebuild with 'docker compose up -d --build sim'.")


def points_to_grid(fwd, lat, sel, cell: float = GRID_CELL, bounds=GRID_BOUNDS,
                   margin: float = 0.0, passable: float = 0.0):
    """Rasterise selected world points onto the ground, with a real margin.

    `fwd` and `lat` are per-pixel world coordinates and `sel` says which pixels
    count. Grown by `margin` metres AFTER rasterising, so the margin is a
    distance on the floor rather than a pixel count that would mean something
    different at every range -- dilating the image mask instead grows a far
    obstacle far less than a near one, for no reason but perspective.

    `passable` closes slots narrower than the robot. A stool's legs project to
    two patches with real floor between them, and that floor is real, but a
    0.35 m slot is not a passage for a 0.44 m robot. This is the ONLY thing the
    navigator's footprint merging was ever for, done here where it is a local
    morphological fact rather than a global fusion that rebuilt the bounding
    box the decomposition existed to avoid.

    Returns a boolean grid indexed [ix, iy].
    """
    import cv2
    nx, ny = grid_shape(cell, bounds)
    x0, _, y0, _ = bounds
    grid = np.zeros((nx, ny), np.uint8)
    if fwd is None or sel is None or not np.any(sel):
        return grid.astype(bool)
    f = np.asarray(fwd)[sel]
    l = np.asarray(lat)[sel]
    ok = np.isfinite(f) & np.isfinite(l)
    if not ok.any():
        return grid.astype(bool)
    ix = ((f[ok] - x0) / cell).astype(np.int32)
    iy = ((l[ok] - y0) / cell).astype(np.int32)
    inb = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    grid[ix[inb], iy[inb]] = 1
    if margin > 0:
        k = 2 * int(round(margin / cell)) + 1
        grid = cv2.dilate(grid, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                                          (k, k)))
    if passable > 0:
        k = max(3, 2 * int(round(passable / cell / 2)) + 1)
        grid = cv2.morphologyEx(grid, cv2.MORPH_CLOSE,
                                cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                                          (k, k)))
    return grid.astype(bool)


def pack_grid(grid) -> bytes:
    """Grid to bytes for the bus. Same packing as the silhouette mask."""
    return np.packbits(np.asarray(grid, bool).reshape(-1)).tobytes()


def unpack_grid(bits: bytes, nx: int, ny: int):
    """Bytes back to a boolean grid indexed [ix, iy]."""
    flat = np.unpackbits(np.frombuffer(bits, np.uint8))[:nx * ny]
    if flat.size < nx * ny:                       # truncated message
        return None
    return flat.reshape(nx, ny).astype(bool)


def corridor_blocked(occ, x: float, y: float, dx: float, look: float,
                     half_width: float, cell: float = GRID_CELL,
                     bounds=GRID_BOUNDS, behind: float = 0.15,
                     pad: float = 0.0) -> bool:
    """Whether a straight corridor of the robot's width hits an occupied cell.

    `dx` is +1 or -1 along the world x axis, which is the only direction the
    patrol travels. Cells OUTSIDE the grid, and cells the sensor never saw, are
    not occupied and therefore do not block: unknown is not the same as blocked,
    and treating it as blocked is what reduced the walkable floor to a 0.31 m
    band while the height map still showed real floor out to 3.4 m.

    `behind` starts the corridor slightly behind the robot so a footprint it is
    already standing in is not missed by a test that only looks forward.

    `pad` extends the swept box ALONG the direction of travel, and exists so
    that query-mode clearance is the same shape as dilate-mode clearance.
    Widening half_width alone inflates the corridor across but not along, so
    the robot would keep its margin from a wall beside it and none at all from
    the couch in front of it. Measured before this was added: 61 lanes out of
    4305 were clear for the query form and blocked for the dilate form, none
    the other way round, all of them where an obstacle lay ahead rather than
    beside. That is not a corner effect, it is a missing axis.
    """
    nx, ny = grid_shape(cell, bounds)
    gx0, _, gy0, _ = bounds
    a = x - (behind + pad) * dx
    b = x + (look + pad) * dx
    lo, hi = (a, b) if a <= b else (b, a)
    i0 = int(np.floor((lo - gx0) / cell))
    i1 = int(np.ceil((hi - gx0) / cell))
    j0 = int(np.floor((y - half_width - gy0) / cell))
    j1 = int(np.ceil((y + half_width - gy0) / cell))
    # At least one cell in each direction. A zero-length corridor -- which is
    # how "am I standing on something" is asked -- collapsed to an empty slice
    # whenever the robot's coordinate landed on a cell boundary, and an empty
    # slice reports clear. The robot then never escaped anything.
    i1 = max(i1, i0 + 1)
    j1 = max(j1, j0 + 1)
    i0, i1 = max(0, i0), min(nx, i1)
    j0, j1 = max(0, j0), min(ny, j1)
    if i0 >= i1 or j0 >= j1:
        return False
    return bool(occ[i0:i1, j0:j1].any())


def free_lane(occ, x: float, y: float, dx: float, look: float,
              half_width: float, max_shift: float, prefer: float = 0.0,
              step: float = 0.05, cell: float = GRID_CELL,
              bounds=GRID_BOUNDS, pad: float = 0.0):
    """Smallest sideways shift of the lane whose corridor ahead is clear.

    Answers the question the rectangle detour could only approximate: not "how
    far past the edge of a box must I move" but "is there a lane of my own
    width with nothing in it". On a concave obstacle those differ completely --
    the inside of an L has a clear lane and no box edge to measure from.

    `prefer` is the ABSOLUTE lane the robot is already holding, in world y.
    Candidates are ordered by how close they land to it, so the two legs of a
    patrol go round an obstacle on the same side instead of each picking the
    lane nearest its own mirror-image centre line.

    Returns (shift, found). When nothing is clear within `max_shift`, returns
    the shift that gets furthest before being blocked, with found False.
    """
    n = int(max_shift / step)
    cands = [k * step for k in range(-n, n + 1)]
    # Ordered by distance from the lane already being held, in ABSOLUTE world
    # terms, not by the size of the shift. Ordering by shift size makes the two
    # legs of a patrol pick opposite sides of the same obstacle -- each is the
    # nearest lane to ITS own centre line, and those centre lines are mirror
    # images. The robot then has to cross the full width of the room during
    # every about-face, which is where the corner-clipping came from. Wanting
    # the same side twice costs nothing and removes the crossing.
    cands.sort(key=lambda s_: (round(abs(y + s_ - prefer), 6), abs(s_), -s_))
    for s in cands:
        if not corridor_blocked(occ, x, y + s, dx, look, half_width,
                                cell, bounds, pad=pad):
            return float(s), True
    # Nothing clear. Report the shift that reaches furthest, which is what the
    # robot should hold while it says so, rather than snapping back to zero.
    best, best_reach = 0.0, -1.0
    coarse = 4 * step          # this branch runs on a frame that is already
    for s in cands:            # stuck; a centimetre of resolution buys nothing
        reach = 0.0
        while reach < look:
            if corridor_blocked(occ, x, y + s, dx, reach + coarse, half_width,
                                cell, bounds, pad=pad):
                break
            reach += coarse
        if reach > best_reach:
            best, best_reach = float(s), reach
    return best, False


def nearest_free(occ, x: float, y: float, half_width: float,
                 max_reach: float = 3.0, step: float = 0.05,
                 cell: float = GRID_CELL, bounds=GRID_BOUNDS,
                 pad: float = 0.0):
    """Shortest sideways move that puts the robot on unoccupied cells.

    Sideways only, for the reason the rectangle escape gave: leaving through
    the front or the back of an obstacle the leg runs through is futile, the
    walk resumes toward a lane still inside it and the robot re-enters at once.

    Returns (dy, depth) where dy is signed lateral metres and depth is how far
    in the robot currently is, or (None, 0.0) when it is already clear.
    """
    if not corridor_blocked(occ, x, y, 1.0, 0.0, half_width, cell, bounds,
                            behind=0.0, pad=pad):
        return None, 0.0
    n = int(max_reach / step)
    for k in range(1, n + 1):
        for s in (k * step, -k * step):
            if not corridor_blocked(occ, x, y + s, 1.0, 0.0, half_width,
                                    cell, bounds, behind=0.0, pad=pad):
                return float(s), float(k * step)
    return None, 0.0


def grid_extent(grid, cell: float = GRID_CELL, bounds=GRID_BOUNDS):
    """(x_min, x_max, y_min, y_max) of the True cells, or None when empty."""
    if grid is None or not grid.any():
        return None
    x0, _, y0, _ = bounds
    ix, iy = np.nonzero(grid)
    return (float(x0 + ix.min() * cell), float(x0 + (ix.max() + 1) * cell),
            float(y0 + iy.min() * cell), float(y0 + (iy.max() + 1) * cell))


def clear_reach(occ, x: float, y: float, dx: float, look: float,
                half_width: float, step: float = 0.10,
                cell: float = GRID_CELL, bounds=GRID_BOUNDS,
                pad: float = 0.0) -> float:
    """How far ahead the robot's CURRENT line stays clear, in metres.

    Distinct from free_lane, which answers where the robot should be. This
    answers where it actually is, and the difference between the two is the
    lateral error the cross-track law still has to close. A corner is clipped
    whenever that error outlives the distance to the obstacle.
    """
    reach = 0.0
    while reach < look:
        if corridor_blocked(occ, x, y, dx, reach + step, half_width,
                            cell, bounds, behind=0.0, pad=pad):
            return reach
        reach += step
    return look
