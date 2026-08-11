# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Where the robot should go, decided from its pose alone.

Split out of the simulator so the same decisions can drive a real robot. The
interface is already the one a real machine offers: it is handed a pose and a
yaw rate, and answers with a forward and a turn velocity. Nothing here knows
about MuJoCo, about rendering, or about how the pose was obtained.

Swapping the simulated G1 for a real one therefore means writing an embodiment
that reads odometry into a Pose and writes the returned velocities to the robot.
The navigation, the obstacle handling and the patrol pattern stay as they are.
"""
from __future__ import annotations

import logging
import os
import time
from dataclasses import dataclass, field

import numpy as np

from edgebot.floor import (GRID_BOUNDS, GRID_CELL, clear_reach, clearance_mode,
                           min_corridor, query_half, query_pad,
                           corridor_blocked, free_lane, grid_extent,
                           nearest_free, unpack_grid)

log = logging.getLogger("navigator")


@dataclass
class Pose:
    """Everything the navigation needs to know about the robot's state.

    `lead` and `centre` differ on a walking biped: the leading toe is what
    reaches a limit first, while the point between the feet is steady across a
    step. On a wheeled or tracked machine both are simply the base position.
    """

    lead: tuple[float, float]      # ground point furthest along the heading
    centre: tuple[float, float]    # ground point midway between the feet
    yaw: float                     # radians, 0 along +x
    yaw_rate: float = 0.0          # rad/s, measured


class Navigator:
    """Paces the optical axis, going around what perception reports."""

    STOP_AT = float(os.environ.get("STOP_AT", "6.0"))      # far end of the run
    RETURN_TO = float(os.environ.get("RETURN_TO", "1.9"))  # near end
    TURN_DONE = 0.12      # rad, how close to the new heading ends the about-face
    TURN_WZ = float(os.environ.get("TURN_WZ", "0.9"))   # rad/s asked while turning
    TURN_VX = float(os.environ.get("TURN_VX", "0.26"))  # m/s kept while turning
    # The speed that gets a STOPPED robot walking again, which is not the speed
    # that keeps a walking one going. Measured on this policy with
    # scripts/startup_probe.py, ramping vx in 0.02 steps: it starts at 0.42 m/s
    # and does not stop until 0.26, a hysteresis of 0.16 m/s. TURN_VX is the
    # STOP side, so any floor built from it holds a moving robot and cannot
    # restart a halted one -- (0.26, 0) is a stable fixed point, which is
    # exactly how the patrol died at lap 18 while still logging vx 0.26 m/s.
    # 0.45 is the measured 0.42 plus a little, in ABSOLUTE m/s rather than as a
    # fraction of anything, so no other knob can drag it back under.
    START_VX = float(os.environ.get("START_VX", "0.45"))
    # A non-zero command that moves the robot less than this over
    # STALL_WINDOW seconds is a stall, and gets said out loud.
    STALL_MIN_MOVE = float(os.environ.get("STALL_MIN_MOVE", "0.05"))
    STALL_WINDOW = float(os.environ.get("STALL_WINDOW", "2.0"))
    CRUISE_VX = float(os.environ.get("CRUISE_VX", "0.6"))  # m/s along the axis
    SLOW_ABOVE = 0.30     # rad of heading error below which speed is not cut
    CROSS_GAIN = 2.0      # how hard to pull back onto the axis
    LOOKAHEAD = float(os.environ.get("LOOKAHEAD", "2.0"))
    HEAD_GAIN = 1.2       # rad/s of yaw command per rad of heading error
    YAW_DAMP = float(os.environ.get("YAW_DAMP", "0.5"))  # per rad/s measured
    CROSS_MAX = float(os.environ.get("CROSS_MAX", "0.35"))
    LANE = float(os.environ.get("LANE", "0.39"))
    EASE_IN = 0.8         # m before the limit over which speed blends into the turn
    OBSTACLE_LOOK = float(os.environ.get("OBSTACLE_LOOK", "3.5"))   # m ahead
    ROBOT_HALF_WIDTH = float(os.environ.get("ROBOT_HALF_WIDTH", "0.22"))
    # The ONE margin. Metres the robot keeps between its body and anything
    # blocking, applied EXACTLY ONCE, by whoever rasterises the obstacle --
    # points_to_grid grows every occupied cell by this, and the published
    # rectangles carry it too. Everything downstream therefore tests at the
    # robot's own half-width and adds nothing, because the margin is already in
    # the input.
    #
    # It replaces three knobs that stacked without anyone adding them up:
    #   OBSTACLE_MARGIN 0.12  in the grid, both sides            0.24
    #   LANE_SLACK      0.08  on the half-width in free_lane     0.16
    #   robot                                                    0.44
    #                                              corridor      0.84 m
    # and, one file away, a merge threshold that asked a different question
    # again: need = 2 * (ROBOT_HALF_WIDTH + GAP_CLEAR) = 0.64 m measured
    # between rectangles that each already carried 0.12, i.e. 0.88 m of real
    # floor. Two code paths, two answers, neither written down in metres. The
    # corridors beside this room's coffee table measure 0.90 m, so the robot
    # was refusing passages a person walks through with 0.03 m to spare on one
    # path and -0.02 m on the other.
    #
    # Now: corridor = 2 * (ROBOT_HALF_WIDTH + CLEARANCE) = 0.68 m, one
    # expression, logged in metres at startup. Raising CLEARANCE widens the
    # corridor the robot demands, which is the intuitive direction -- GAP_CLEAR
    # ran the other way and that is half of why it was wrong twice.
    CLEARANCE = float(os.environ.get("CLEARANCE", "0.12"))
    # WHERE it is applied: "query" (the grid stays a raw obstacle layer and
    # every query inflates) or "dilate" (the cells were grown when built).
    # Adopted from the grid publisher like CLEARANCE itself -- guessing wrong
    # applies the margin twice or not at all, and both look plausible.
    CLEARANCE_MODE = clearance_mode(os.environ.get("CLEARANCE_MODE"))
    # A footprint must appear in CONFIRM_MIN of the last CONFIRM_OF updates
    # before the robot acts on it. Detections flicker, and reacting to a single
    # frame made the robot escape an imaginary obstacle every 11 seconds.
    # 3 of 3, not 2 of 3: a detection that alternates present, absent, present
    # is in 2 of the last 3 and would still pass, which measured as 31 escapes
    # and 98 frames inside an imaginary obstacle over 90 seconds. Demanding all
    # three removes it entirely and costs nothing on a real obstacle.
    # How long a footprint stays usable after its last update. It MUST exceed
    # the compositor's republication period, or the obstacles go stale between
    # messages: at 2 s against a 5 s period the detour returned zero for three
    # seconds out of five, the lane snapped back inside the obstacle, and the
    # robot ping-ponged in and out of it for a minute at a time.
    STALE = float(os.environ.get("OBSTACLE_STALE", "3.0"))
    CONFIRM_OF = int(os.environ.get("CONFIRM_OF", "3"))
    CONFIRM_MIN = int(os.environ.get("CONFIRM_MIN", "3"))
    # Where obstacles come from: "ours" (perception's footprints, the default
    # and the shipped demo), "suite" (ADBSCAN's clusters only) or "union".
    #
    # The union exists because the two detectors are complementary, measured
    # rather than assumed -- docs/ETAPE-C-RESULTS.md section 8. We start from
    # semantic segmentation and hold the dining table and the kitchen block that
    # their density test melts into one mass; they hold the near right pillar and
    # counter 70-75 % of the time, which no COCO class covers and we therefore
    # never see at all. A false positive costs a detour, a false negative costs
    # a collision, so an obstacle seen by either side is an obstacle.
    #
    # Default "ours" on purpose: the suite bricks are an optional compose
    # profile, and a navigator whose behaviour depended on whether an optional
    # service happened to be running would be a bad default whichever way it
    # went. Nothing changes unless asked.
    SOURCE = os.environ.get("OBSTACLE_SOURCE", "ours").strip().lower()
    # "grid" steers on ground cells, "rects" on the footprint rectangles.
    #
    # Cells, by default, for a reason that is measured rather than aesthetic: a
    # rectangle around this room's L-shaped couch was 3.35 x 3.19 m = 10.7 m2
    # for a couch occupying about 5.1 m2, and the 5.6 m2 it claimed were the
    # coffee table and both walking corridors. Decomposing it into several
    # rectangles improved the cover but never freed the inside of the L: the
    # budget always ended in one box crossing the passage. A cell is occupied
    # or it is not, so the question does not arise.
    #
    # Both representations are published on every message, so "rects" restores
    # the previous behaviour exactly and the two stay comparable.
    REP = os.environ.get("OBSTACLE_REP", "grid").strip().lower()
    # Keep the turn-around inside the floor the sensor actually reports. The
    # committed STOP_AT of 6.0 m dates from a scene whose floor reached
    # further; against the current one it ordered the robot across four metres
    # of floor that were entirely inside footprints, which measured as 100 % of
    # poses scraping and -0.522 m of clearance. That was obedience to an
    # impossible order, not a failure to route. 0 disables the clamp and
    # restores the literal STOP_AT.
    PATROL_CLAMP = float(os.environ.get("PATROL_CLAMP", "0.6"))
    # Floor under the run-up brake. Not zero: a robot that stops entirely in
    # front of an obstacle it is still drifting away from never finishes the
    # drift, because the cross-track law needs forward motion to convert a
    # heading into a sideways displacement. Standing still is a decision taken
    # elsewhere, when there is no lane at all.
    RUNUP_MIN = float(os.environ.get("RUNUP_MIN", "0.25"))
    # LANE_SLACK used to sit here: an extra half-width on every lane test, 0.08
    # by default. It was never a safety margin -- its own comment said so -- it
    # paid for TRACKING LAG. The lane is reached asymptotically, so a lane
    # chosen with exactly the clearance needed is always entered with less, and
    # widening the demand hid that. Its measured table, at the coffee table's
    # near corner over 60 s:
    #
    #   slack   travelled   scraping   worst clearance
    #   0.00     14.81 m      1.6 %        -0.101 m
    #   0.15     16.38 m      0.0 %        -0.022 m
    #
    # It is removed here on a hypothesis that is falsifiable and gets falsified
    # or not by the etape 2 measurement: _runup_cap() now solves the same
    # problem the honest way, by BRAKING when the run-up is too short instead of
    # demanding a wider lane everywhere. That table predates _runup_cap. If it
    # still held, the corridor budget below is 0.16 m too generous and scraping
    # comes back -- which nav_probe reports against the grid, in one number.
    # "patrol" paces the optical axis, the shipped demo. "goal" follows the
    # waypoints of SUITE_PATH, the planner's output, and is etape E2.
    #
    # A mode and not a replacement. Everything below this line -- the heading
    # law, the yaw cap, the TURN_VX floor the policy needs to keep stepping,
    # the smoothing, the escape from a footprint -- is shared, because those
    # are properties of the ROBOT and not of the mission. A goal mode that
    # re-derived them would drift from the patrol that has eight laps of
    # validation behind it.
    MODE = os.environ.get("NAV_MODE", "patrol").strip().lower()
    # Reached, for the purpose of the E2 criterion. 0.45 m is not a comfort
    # margin, it is the measured start snap of the ITS roadmap: it places its
    # start on the nearest node, 0.30-0.45 m away, so a goal is only ever
    # approached to within roughly that. Tightening this would measure the
    # planner's roadmap density, not the robot's tracking.
    GOAL_TOL = float(os.environ.get("GOAL_TOL", "0.45"))
    # How far along the path to aim. Shorter cuts corners and wobbles, longer
    # cuts the corner off the corner. Roughly the turn radius, 0.38 m, doubled.
    PATH_LOOKAHEAD = float(os.environ.get("PATH_LOOKAHEAD", "0.8"))
    # A plan is not a map: it was computed against one occupancy and one pair of
    # endpoints. Past this it is not followed, the robot stands, and the planner
    # is expected to publish a fresh one.
    PATH_STALE = float(os.environ.get("PATH_STALE", "20.0"))
    # Their clusters only, and only for the union. Anything wider than this in
    # either direction is refused before the merge: with GF_Z_LOW in place they
    # still return the right half of the room as one 3.8 x 2.2 m block in 68-82 %
    # of frames, and handing that to the detour walls off half the patrol -- it
    # is not an obstacle, it is a failure to separate several. 3 m is above the
    # largest single piece of furniture here (the 2.64 m dining table) and below
    # that block.
    SUITE_MAX_SPAN = float(os.environ.get("SUITE_MAX_SPAN", "3.0"))
    # And clipped to the arena first, for the same reason suite_compare.py does
    # it: outside these bounds a rectangle from either side is a depth artefact,
    # and theirs in particular runs to the walls.
    SUITE_X_MIN = float(os.environ.get("SUITE_X_MIN", "1.5"))
    SUITE_X_MAX = float(os.environ.get("SUITE_X_MAX", "6.5"))
    SUITE_Y_MIN = float(os.environ.get("SUITE_Y_MIN", "-2.6"))
    SUITE_Y_MAX = float(os.environ.get("SUITE_Y_MAX", "1.5"))
    DETOUR_MAX = float(os.environ.get("DETOUR_MAX", "1.8"))  # m of lane shift
    # How much run-up to take beyond the geometric minimum. 1.0 is the pure
    # geometry and is always slightly short, the cross-track law reaching its
    # target asymptotically.
    DETOUR_RUNUP = float(os.environ.get("DETOUR_RUNUP", "1.6"))
    DETOUR_GAIN = float(os.environ.get("DETOUR_GAIN", "1.25"))
    SMOOTH_TAU = float(os.environ.get("SMOOTH_TAU", "0.5"))
    TURN_RATE = 0.9       # rad/s cap on the yaw command
    VX_SLEW = 1.2         # m/s^2 limit on the forward command
    WZ_SLEW = float(os.environ.get("WZ_SLEW", "4.0"))  # rad/s^2 on the yaw command

    def __init__(self) -> None:
        self._vx = self._wz = 0.0
        # Adopted from the grid publisher when one arrives; see set_grid. The
        # env value is only the opening bid, because the margin that matters is
        # the one actually baked into the cells the robot steers on, and this
        # process is not the one that baked it.
        self.clearance = self.CLEARANCE
        self.mode = self.CLEARANCE_MODE
        self.lane = self.LANE
        self._outbound = True
        self._turning = False
        self._turn_sign = 1.0
        self._turn_y0 = 0.0
        self._turn_started = 0.0
        self._target_yaw = 0.0
        self._laps = 0
        self._yaw_f = 0.0
        self._y_f = 0.0
        self._obstacles: list = []
        self._obstacles_t = 0.0
        # Per source, because the two arrive at different rates -- our ROI once
        # a second, their clusters at about 9 Hz -- so one shared confirmation
        # window would let the fast source fill it and confirm the slow one's
        # footprints by itself. Keyed the same way in all three: history for the
        # confirmation vote, and the time of the last update for staleness.
        self._history: dict = {"ours": [], "suite": []}
        self._src_t: dict = {"ours": 0.0, "suite": 0.0}
        # Which sources fed each rectangle of self._obstacles, same order. Only
        # ever read for logging and telemetry: the geometry is source-blind on
        # purpose, an obstacle being an obstacle whoever saw it.
        self._obstacle_src: list = []
        self._suite_wide = 0    # clusters refused for spanning more than
        self._suite_outside = 0  # SUITE_MAX_SPAN, or falling outside the arena
        self._last_suite_log = 0.0
        # Counted, not just logged once per episode: "no way round" is the
        # failure this whole arrangement risks introducing, so its rate is the
        # number to compare between sources rather than its presence.
        self._no_way_round = 0
        self._path: list = []          # waypoints of the current plan
        self._path_goal = None         # what it was planned to reach
        self._path_t = 0.0
        self._path_i = 0               # how far along we are
        self._arrived = None           # goal already reported reached
        self._goals_done = 0
        self._path_hold = ""           # why we are standing still, logged once
        if self.MODE not in ("patrol", "goal"):
            raise SystemExit(
                f"NAV_MODE={self.MODE!r} is not one of patrol, goal")
        if self.SOURCE not in ("ours", "suite", "union"):
            # Loudly, not silently back to "ours". A typo in OBSTACLE_SOURCE
            # would otherwise look exactly like the union quietly doing nothing.
            raise SystemExit(
                f"OBSTACLE_SOURCE={self.SOURCE!r} is not one of "
                f"ours, suite, union")
        self._roi: list = []
        self._detour_reason = ""
        self._inside_reason = ""
        self._escape_yaw = None
        self._blocked = False
        self._blocked_reason = ""
        # Per-source instance ids for the most recent update only.
        self._inst: dict = {}
        self._last_walk_log = 0.0
        # Ground cells, when the compositor sends them. Kept beside the
        # rectangles rather than instead of them so OBSTACLE_REP can pick.
        self._occ = None
        self._flr = None
        self._occ_t = 0.0
        self._cell = GRID_CELL
        self._bounds = GRID_BOUNDS
        # The lane shift currently held, so a tie between two equally good
        # lanes goes to the one already taken instead of swapping sides every
        # update, which reads as indecision and costs distance.
        self._last_lane = 0.0
        self._runup_logged = 1.0
        # Stall watchdog state. See _watch_stall.
        self._stalled = False
        self._stall_t0 = 0.0
        self._stall_x = 0.0
        self._stall_y = 0.0
        self._stall_said = 0.0
        # Effective far end of the patrol. Starts at the configured STOP_AT and
        # is pulled in to the floor the sensor reports, see PATROL_CLAMP.
        self._stop_at = self.STOP_AT
        # In metres, in words, once, at startup. Three knobs used to add up to
        # this number and no line of code or log ever wrote it down, so the
        # only way to know what the robot demanded was to find all three and do
        # the arithmetic -- which two code paths in this file did differently.
        log.info("clearance budget: the robot is %.2f m wide and keeps %.2f m "
                 "of clearance on each side, so it will only walk through a "
                 "gap of %.2f m or more of REAL FLOOR, measurable with a tape. "
                 "The margin lives in the %s, so the grid is asked for %.2f m "
                 "of half-width. Nothing adds to this later.",
                 2.0 * self.ROBOT_HALF_WIDTH, self.clearance,
                 self.min_corridor,
                 "cells (dilate mode)" if self.mode == "dilate"
                 else "query (raw grid)", self.grid_half)

    @property
    def grid_half(self) -> float:
        """Half-width to ask the GRID for, given where the margin lives.

        One property, read by every corridor query in this file, computed by
        the shared helper the probes use. Not a constant: it is the robot's
        bare half-width when the cells were already grown, and half-width plus
        clearance when they were not.
        """
        return query_half(self.ROBOT_HALF_WIDTH, self.clearance, self.mode)

    @property
    def grid_pad(self) -> float:
        """Longitudinal half of the same inflation. See floor.query_pad."""
        return query_pad(self.clearance, self.mode)

    @property
    def min_corridor(self) -> float:
        """The narrowest real-floor gap this robot will walk through, in metres.

        Deliberately the same in both modes -- choosing where to apply the
        margin must not change how much margin there is -- and deliberately in
        FLOOR metres, not grid metres. In dilate mode a test at half-width h
        passes a grid gap of 2h, which is 2*(h + clearance) of actual floor:
        0.24 m apart at the default. Every log used to quote the grid figure
        while the reader measured the floor with a tape.
        """
        return min_corridor(self.ROBOT_HALF_WIDTH, self.clearance)

    def _floor_width(self, grid_half: float) -> float:
        """A grid half-width back to the metres of floor it corresponds to.

        The inverse of grid_half, for the diagnostics that sweep widths. A
        message that names a corridor the operator cannot measure with a tape
        is worse than no number.
        """
        extra = 0.0 if self.mode == "query" else self.clearance
        return 2.0 * (grid_half + extra)

    def knobs(self) -> dict:
        """The values this navigator is actually running with.

        Published so a probe reports the configuration that IS steering rather
        than the defaults of whatever container it happens to start in.
        `make lane-probe` printed LANE=0.39 DETOUR_MAX=1.80 STOP_AT=6.00 while
        the demo ran 0 / 2.4 / 4.0, and then drew conclusions about a patrol
        that was not running.
        """
        return {"LANE": self.LANE, "DETOUR_MAX": self.DETOUR_MAX,
                "RETURN_TO": self.RETURN_TO, "STOP_AT": self.STOP_AT,
                "ROBOT_HALF_WIDTH": self.ROBOT_HALF_WIDTH,
                # The clearance in force and the corridor it implies, both, so
                # a probe never has to redo the arithmetic and get a different
                # answer from the robot's.
                "CLEARANCE": self.clearance,
                "CLEARANCE_MODE": self.mode,
                "GRID_HALF": self.grid_half,
                "GRID_PAD": self.grid_pad,
                "MIN_CORRIDOR": self.min_corridor,
                "OBSTACLE_REP": self.REP,
                "OBSTACLE_SOURCE": self.SOURCE,
                "NAV_MODE": self.MODE,
                # Carried so every measurement can say which laps it covers.
                # A 60 s window can straddle a stall, and two runs that report
                # the same numbers over different laps are not the same run.
                "lap": self._laps,
                "stalled": self._stalled}

    def set_floor(self, roi: list, blocked: list, inst: list | None = None) -> None:
        """Take the walkable floor and the obstacle footprints from perception.

        Both arrive together and neither is sufficient alone: the polygon is an
        outer boundary and cannot express a hole, which is what an obstacle
        standing away from the walls leaves.
        """
        if roi:
            self._roi = roi
        if blocked is not None:
            self._push("ours", [(float(b[0]), float(b[1]), float(b[2]),
                                 float(b[3])) for b in blocked], inst)

    def set_grid(self, payload: dict) -> None:
        """Take the ground occupancy grids that ride with the footprints.

        Two grids: `occ` is where an object stands, `flr` is where the sensor
        saw floor. Only `occ` stops the robot. Treating "no floor reported" as
        blocked confines it to the observed band, which is how a 0.31 m deep
        walkable floor came out of a room with 3.4 m of real floor in it.
        """
        bits = payload.get("occ")
        if not bits:
            return
        nx, ny = int(payload.get("gnx", 0)), int(payload.get("gny", 0))
        if nx <= 0 or ny <= 0:
            return
        self._cell = float(payload.get("gcell", GRID_CELL))
        gb = payload.get("gbounds")
        self._bounds = tuple(float(v) for v in gb) if gb else GRID_BOUNDS
        # The clearance that is really in these cells, from the process that
        # grew them. Every corridor test below reads the grid, so believing our
        # own environment over the publisher's is how the two silently differ by
        # a knob somebody set on one service and not the other -- the exact
        # failure OBSTACLE_MARGIN carried a hand-written warning about.
        pub = payload.get("clearance")
        pub_mode = payload.get("clearance_mode")
        if ((pub is not None and abs(float(pub) - self.clearance) > 1e-6)
                or (pub_mode and pub_mode != self.mode)):
            was = (self.clearance, self.mode, self.min_corridor)
            if pub is not None:
                self.clearance = float(pub)
            if pub_mode:
                self.mode = clearance_mode(pub_mode)
            log.warning("clearance: adopting %.3f m in %s mode from the grid "
                        "publisher, was %.3f m in %s mode. Corridor %.2f m of "
                        "real floor, was %.2f m; querying the grid at %.3f m.",
                        self.clearance, self.mode, was[0], was[1],
                        self.min_corridor, was[2], self.grid_half)
        occ = unpack_grid(bits, nx, ny)
        if occ is None:
            return
        self._occ = occ
        self._occ_t = time.time()
        fb = payload.get("flr")
        self._flr = unpack_grid(fb, nx, ny) if fb else None
        if self.PATROL_CLAMP > 0:
            # How far the robot can actually WALK, not how far floor is visible.
            # The far edge of the floor grid is the wrong number: a strip of
            # floor behind the couch is floor, is reported, and is unreachable,
            # so clamping to it still sends the robot into the furniture. Ask
            # instead where the last clear lane ends.
            reach = 0.0
            for s in np.arange(0.0, self.DETOUR_MAX + 1e-6, 0.10):
                for lane in (-s, s):
                    _, ok = free_lane(
                        self._occ, self.RETURN_TO, lane, 1.0,
                        self.STOP_AT - self.RETURN_TO,
                        self.grid_half, 0.0,
                        cell=self._cell, bounds=self._bounds,
                        pad=self.grid_pad)
                    if ok:
                        reach = self.STOP_AT - self.RETURN_TO
                        break
                    r2 = 0.0
                    while r2 < self.STOP_AT - self.RETURN_TO:
                        if corridor_blocked(self._occ, self.RETURN_TO,
                                            lane, 1.0, r2 + 0.20,
                                            self.grid_half,
                                            self._cell, self._bounds,
                                            pad=self.grid_pad):
                            break
                        r2 += 0.20
                    reach = max(reach, r2)
                if reach >= self.STOP_AT - self.RETURN_TO:
                    break
            far = self.RETURN_TO + reach - self.PATROL_CLAMP
            lim = max(self.RETURN_TO + 0.3, min(self.STOP_AT, far))
            if abs(lim - self._stop_at) > 0.05:
                log.info("patrol limit %.2f m: the furthest clear lane from "
                         "%.2f m reaches %.2f m, STOP_AT is %.2f m",
                         lim, self.RETURN_TO, self.RETURN_TO + reach,
                         self.STOP_AT)
            self._stop_at = lim

    def _grid_ready(self) -> bool:
        return (self.REP == "grid" and self._occ is not None
                and time.time() - self._occ_t <= self.STALE)

    def _escape_grid(self, pose: Pose, dt: float):
        """Out of an occupied cell, sideways, by the shortest way.

        Same policy as the rectangle escape and for the same measured reason:
        leaving through the front or the back of something the leg runs through
        is futile, because the walk resumes toward a lane still inside it.
        """
        x, y = pose.centre
        dy, depth = nearest_free(self._occ, x, y, self.grid_half,
                                 pad=self.grid_pad,
                                 cell=self._cell, bounds=self._bounds)
        if dy is None:
            self._inside_reason = ""
            self._escape_yaw = None
            return None
        if self._escape_yaw is None:
            # Fixed once. Derived from the current heading it rotates with the
            # robot, which then chases its own target and never leaves.
            self._escape_yaw = float(np.arctan2(1.0 if dy > 0 else -1.0, 0.0))
        want = self._escape_yaw
        err = (want - pose.yaw + np.pi) % (2 * np.pi) - np.pi
        if self._inside_reason != "inside":
            self._inside_reason = "inside"
            log.warning("standing on occupied cells, %.2f m from free floor: "
                        "backing out toward %+.0f deg", depth,
                        np.degrees(want))
        return self._smooth(self.TURN_VX,
                            float(np.clip(self.HEAD_GAIN * err,
                                          -self.TURN_WZ, self.TURN_WZ)), dt)

    def _detour_grid(self, x: float, y: float, d: float, lane: float) -> float:
        """Lateral shift to a lane of the robot's own width that is clear.

        The rectangle detour asked "how far past the edge of this box must I
        move", which has no answer on a concave object: the inside of an L is
        clear floor with no box edge to measure from. This asks the question
        the robot actually has, which is whether a corridor its own width
        exists, and takes the nearest one.
        """
        # Only as far as this leg actually goes. A fixed 3.5 m look from x=2.0
        # reaches 5.5 m, where the couch stands against the far wall, so every
        # lane was blocked and the robot reported "no way round" while the
        # floor it was about to walk on was clear. An obstacle beyond the
        # turn-around point is not in the way.
        target = self._stop_at if self._outbound else self.RETURN_TO
        look = float(np.clip(abs(target - x), 0.5, self.OBSTACLE_LOOK))
        shift, found = free_lane(
            self._occ, x, lane, d, look,
            self.grid_half, self.DETOUR_MAX,
            prefer=self._last_lane, cell=self._cell, bounds=self._bounds,
            pad=self.grid_pad)
        self._last_lane = lane + shift
        if found:
            self._blocked = False
            if self._blocked_reason:
                self._blocked_reason = ""
                log.info("a way round is available again")
            reason = f"{shift:+.2f}"
            if reason != self._detour_reason:
                self._detour_reason = reason
                if abs(shift) > 1e-6:
                    log.info("clear lane %+.2f m from the line, %.1f m ahead, "
                             "corridor %.2f m wide", shift, look,
                             self.min_corridor)
            return shift
        if self._blocked_reason != "no way round":
            self._blocked_reason = "no way round"
            self._no_way_round += 1
            # Say how wide a corridor WOULD have worked, not just that this one
            # did not. "No way round" about a passage a person walks through is
            # nearly always the width being asked for, and a warning that does
            # not name the number cannot distinguish a real wall from an
            # arithmetic mistake -- which is what it was hiding.
            widest = 0.0
            for half in np.arange(self.grid_half, 0.14, -0.02):
                _, ok2 = free_lane(self._occ, x, lane, d, look, float(half),
                                   self.DETOUR_MAX, prefer=self._last_lane,
                                   cell=self._cell, bounds=self._bounds,
                                   pad=self.grid_pad)
                if ok2:
                    widest = self._floor_width(float(half))
                    break
            log.warning("no way round (#%d): no lane %.2f m wide is clear "
                        "within %.2f m of the line over the next %.1f m. "
                        "The widest lane that IS clear is %s. Holding at "
                        "%+.2f m, which reaches furthest.",
                        self._no_way_round,
                        self.min_corridor,
                        self.DETOUR_MAX, look,
                        "%.2f m" % widest if widest else "none at all",
                        shift)
        self._blocked = True
        return shift

    def _runup_cap(self, x: float, y: float, d: float, lane: float) -> float:
        """Speed factor that stops the robot clipping a corner it cannot clear.

        The detour moves the LANE at once, but the robot reaches it
        asymptotically: holding no more than CROSS_MAX off the line, closing
        E metres of lateral error needs E / tan(CROSS_MAX) metres of run-up.
        When the obstacle arrives before that distance does, the robot arrives
        with the error still open and cuts the corner. Measured on a lounge
        whose coffee table stands 0.9 m past the near end of the patrol: the
        lane was correctly placed at -0.94 m, the robot was still at -0.44 m
        on arrival, and 9.9 % of poses crossed the table's real outline with a
        worst clearance of -0.213 m.

        Braking rather than swerving harder, because CROSS_MAX is not a comfort
        setting: a biped asked for more yaw than that stops tracking the line
        at all. Slowing costs seconds and clears the furniture; the alternative
        costs the furniture.

        Returns 1.0 when there is room, down to RUNUP_MIN when there is not.
        """
        err = abs(y - lane)
        if err < 0.05:
            return 1.0
        need = err / max(0.2, float(np.tan(self.CROSS_MAX))) * self.DETOUR_RUNUP
        reach = clear_reach(self._occ, x, y, d, min(need, self.OBSTACLE_LOOK),
                            self.grid_half,
                            cell=self._cell, bounds=self._bounds,
                            pad=self.grid_pad)
        if reach >= need:
            return 1.0
        cap = max(self.RUNUP_MIN, reach / max(1e-6, need))
        if abs(cap - self._runup_logged) > 0.15:
            self._runup_logged = cap
            log.info("easing to %.0f %% of cruise: %.2f m of lateral error "
                     "needs %.2f m of run-up and only %.2f m is clear",
                     100 * cap, err, need, reach)
        return cap

    def set_suite(self, clusters: list) -> None:
        """Take ADBSCAN's clusters, clipped to the arena and de-blobbed.

        Ignored entirely unless OBSTACLE_SOURCE asks for them, so the topic can
        be subscribed unconditionally and the decision stays in one place.

        Clipping before the span test, not after: a cluster that runs from the
        middle of the room into the far wall is 5 m wide uncut and a perfectly
        reasonable 1.5 m once the part outside the patrol is removed. Refusing
        it on its uncut width would throw away the half that matters.
        """
        if self.SOURCE == "ours" or clusters is None:
            return
        keep, wide, outside = [], 0, 0
        for b in clusters:
            x0 = max(float(b[0]), self.SUITE_X_MIN)
            x1 = min(float(b[1]), self.SUITE_X_MAX)
            y0 = max(float(b[2]), self.SUITE_Y_MIN)
            y1 = min(float(b[3]), self.SUITE_Y_MAX)
            if x1 - x0 <= 1e-6 or y1 - y0 <= 1e-6:
                outside += 1
                continue
            if x1 - x0 > self.SUITE_MAX_SPAN or y1 - y0 > self.SUITE_MAX_SPAN:
                wide += 1
                continue
            keep.append((x0, x1, y0, y1))
        self._suite_wide += wide
        self._suite_outside += outside
        now = time.time()
        if (wide or outside) and now - self._last_suite_log > 10.0:
            self._last_suite_log = now
            log.info("suite clusters: %d kept, %d refused as wider than "
                     "%.1f m, %d outside the arena (cumulative %d / %d)",
                     len(keep), wide, self.SUITE_MAX_SPAN, outside,
                     self._suite_wide, self._suite_outside)
        self._push("suite", keep)

    def set_path(self, payload) -> None:
        """Take a plan from SUITE_PATH. Ignored unless NAV_MODE asks for it.

        A NEW GOAL RESETS the progress index; a replan toward the SAME goal does
        not. Without that distinction the chair test fails in a way that looks
        like success: the planner republishes a detour around the chair, the
        index survives from the old path, and the robot skips straight to a
        waypoint past the obstacle -- arriving at the goal having walked through
        the thing it was supposed to avoid.
        """
        if self.MODE != "goal" or not payload:
            return
        pts = [(float(a), float(b)) for a, b in (payload.get("path") or [])]
        if len(pts) < 2:
            return
        goal = tuple(float(v) for v in (payload.get("goal") or ()))
        if goal != self._path_goal:
            self._path_i = 0
            self._arrived = None
            log.info("new goal (%.2f, %.2f), %d waypoint(s), %.2f m",
                     goal[0], goal[1], len(pts),
                     float(payload.get("length_m", 0.0)))
        else:
            # Same goal, fresh plan: keep going forward, but never backwards.
            # The new path has its own indexing, so the old index is only a hint
            # and is clamped rather than trusted.
            self._path_i = min(self._path_i, max(0, len(pts) - 1))
        self._path = pts
        self._path_goal = goal
        self._path_t = time.time()

    def _follow(self, pose: Pose, dt: float) -> np.ndarray:
        """Walk the current plan. The goal-mode counterpart of the patrol.

        Pure pursuit on the waypoints, then the SAME heading law, yaw cap,
        TURN_VX floor and smoothing the patrol uses. The RL policy's
        constraints are properties of the robot: it stops stepping below about
        0.26 m/s whatever yaw is asked, and it tracks 0.9 rad/s at about 75 %.
        Neither changes because the mission changed.
        """
        x, y = pose.centre
        if not self._path or time.time() - self._path_t > self.PATH_STALE:
            # BOOTSTRAP. The robot spawns at the world origin, under the camera,
            # and the mapped floor only begins at about 1.5 m: the planner is
            # asked to plan from a cell it has never seen, calls it an obstacle,
            # and returns nothing -- so the robot stands still for ever waiting
            # for a plan that cannot exist while it stands there. Measured on
            # the first E2 run: "plugin failed to plan from (-0.02, 0.00)",
            # every five seconds, indefinitely.
            #
            # The patrol has always had this bootstrap implicitly: it walks
            # straight out from under the camera before it knows anything. Goal
            # mode needs it explicitly. Straight ahead, no steering, until the
            # robot is on mapped floor and a plan arrives.
            if x < self.RETURN_TO:
                self._hold("walking out from under the camera onto the mapped "
                           "floor, where a plan can start")
                err = (0.0 - pose.yaw + np.pi) % (2 * np.pi) - np.pi
                return self._smooth(self.CRUISE_VX, float(np.clip(
                    err * self.HEAD_GAIN - self.YAW_DAMP * pose.yaw_rate,
                    -self.TURN_RATE, self.TURN_RATE)), dt)
            self._hold("no current plan (waiting for the planner)")
            return self._smooth(0.0, 0.0, dt)

        if self._path_goal is not None:
            d_goal = float(np.hypot(self._path_goal[0] - x,
                                    self._path_goal[1] - y))
            if d_goal <= self.GOAL_TOL:
                if self._arrived != self._path_goal:
                    self._arrived = self._path_goal
                    self._goals_done += 1
                    log.info("GOAL %d REACHED: (%.2f, %.2f), final distance "
                             "%.3f m (tolerance %.2f)", self._goals_done,
                             self._path_goal[0], self._path_goal[1], d_goal,
                             self.GOAL_TOL)
                return self._smooth(0.0, 0.0, dt)

        # Out of a footprint first, exactly as the patrol does. This is the
        # union acting as the reactive layer: whatever the plan says, standing
        # inside an obstacle is resolved before anything else.
        escape = self._escape(pose, dt)
        if escape is not None:
            return escape

        # Advance the index to the waypoint nearest the robot, never backwards,
        # then aim PATH_LOOKAHEAD further along.
        best_i, best_d = self._path_i, float("inf")
        for i in range(self._path_i, len(self._path)):
            d = float(np.hypot(self._path[i][0] - x, self._path[i][1] - y))
            if d < best_d:
                best_i, best_d = i, d
        self._path_i = best_i
        target, ahead = self._path[-1], 0.0
        for i in range(best_i, len(self._path) - 1):
            ahead += float(np.hypot(self._path[i + 1][0] - self._path[i][0],
                                    self._path[i + 1][1] - self._path[i][1]))
            if ahead >= self.PATH_LOOKAHEAD:
                target = self._path[i + 1]
                break

        # The reactive layer, second half. The patrol answers an obstacle by
        # shifting its lane sideways, which only means something on an axis; off
        # it, the honest answer is to stop and let the planner produce another
        # route. Standing still in front of an obstacle reads as deliberate.
        if self._obstacle_on(target):
            self._hold("an obstacle sits on the next waypoint, holding for a "
                       "replan")
            return self._smooth(0.0, 0.0, dt)
        self._hold("")

        want = float(np.arctan2(target[1] - y, target[0] - x))
        err = (want - pose.yaw + np.pi) % (2 * np.pi) - np.pi
        damp = self.YAW_DAMP * pose.yaw_rate
        wz = float(np.clip(err * self.HEAD_GAIN - damp,
                           -self.TURN_RATE, self.TURN_RATE))
        # Slow down when the heading is badly wrong, but never below TURN_VX:
        # the policy plants both feet under that and stops turning at all, so a
        # speed floor is what lets it rotate out of a bad heading rather than
        # freezing in it. The same figure and the same reason as the patrol.
        vx = self.CRUISE_VX
        if abs(err) > self.CROSS_MAX:
            vx = max(self.TURN_VX,
                     self.CRUISE_VX * (1.0 - min(1.0, abs(err) / np.pi)))
        return self._smooth(vx, wz, dt)

    def _obstacle_on(self, pt) -> bool:
        """Whether a confirmed obstacle covers a point, with the robot's width."""
        if not self._obstacles or time.time() - self._obstacles_t > self.STALE:
            return False
        r = self.ROBOT_HALF_WIDTH
        return any(x0 - r <= pt[0] <= x1 + r and y0 - r <= pt[1] <= y1 + r
                   for x0, x1, y0, y1 in self._obstacles)

    def _hold(self, reason: str) -> None:
        if reason != self._path_hold:
            self._path_hold = reason
            if reason:
                log.info("%s", reason)

    def _push(self, source: str, boxes: list, inst: list | None = None) -> None:
        """Record one update from one source and rebuild the obstacle set.

        `inst` says which detected object each box belongs to. It is kept for
        the LATEST update only and deliberately out of the confirmation
        history: the ids are per-frame indices and would differ between frames
        for the same object, so comparing them across updates would stop
        anything ever confirming.
        """
        hist = self._history[source]
        hist.append(boxes)
        self._inst[source] = list(inst) if inst else []
        del hist[:-self.CONFIRM_OF]
        self._src_t[source] = time.time()
        self._recompute()

    def _recompute(self) -> None:
        """Confirm each source separately, then merge the union of them.

        Confirmation per source and merging after, not the other way round. A
        rectangle only one detector ever sees -- the near right pillar is
        exactly that -- must be allowed to confirm against its own history;
        pooling first would make it compete with the other source's updates and
        it would never reach CONFIRM_MIN.

        A source past STALE contributes nothing, so an optional service that
        stops publishing fades out instead of freezing its last obstacles into
        the patrol. Because of that the union can legitimately shrink to one
        source's view while the other is down, which is the behaviour wanted.
        """
        now = time.time()
        boxes, tags = [], []
        for source in self._wanted():
            if not self._history[source]:
                continue
            if now - self._src_t[source] > self.STALE:
                continue
            ids = self._inst.get(source) or []
            for k, box in self._confirmed(source):
                boxes.append(box)
                tag = {source}
                if k < len(ids) and ids[k] is not None:
                    # Pieces of one detected object carry the same mark and are
                    # never merged with each other, so a decomposed L-shaped
                    # couch cannot be glued back into its bounding box.
                    tag.add(f"{source}#{ids[k]}")
                tags.append(tag)
        self._obstacles, self._obstacle_src = self._merge(boxes, tags)
        # The freshest contributing source. Taking the oldest would expire the
        # union as soon as the slower of the two lagged, and taking a fixed
        # source would ignore whichever one is actually still talking.
        live = [self._src_t[s] for s in self._wanted()
                if self._history[s] and now - self._src_t[s] <= self.STALE]
        self._obstacles_t = max(live) if live else 0.0

    def _src_of(self, idx: int) -> str:
        """The sources behind one merged rectangle, for the log line."""
        if idx >= len(self._obstacle_src):
            return "?"
        return "+".join(sorted(self._obstacle_src[idx]))

    def _wanted(self) -> tuple:
        if self.SOURCE == "suite":
            return ("suite",)
        if self.SOURCE == "union":
            return ("ours", "suite")
        return ("ours",)

    def _confirmed(self, source: str = "ours") -> list:
        """(index, box) for the footprints seen in several consecutive updates.

        The index is into the LATEST update, so the caller can look the box's
        instance id up in _inst -- which is how the pieces of one concave
        object stay recognisable as one object all the way to the merge.

        Detections flicker: measured at 55 % of consecutive updates changing the
        obstacle count, and one escape every 11 seconds, three quarters of them
        for less than 0.35 m of penetration. A footprint that appears around the
        robot cannot be a real object, since perception never sees the composited
        robot, only the raw camera frame, so it is a detection artefact.

        Acting on a single frame therefore makes the robot thrash. A footprint
        must be present in CONFIRM_MIN of the last CONFIRM_OF updates before it
        counts, which costs a fraction of a second of reaction time and removes
        the flicker entirely.
        """
        hist = self._history[source]
        if len(hist) < self.CONFIRM_OF:
            return list(enumerate(hist[-1])) if hist else []
        latest = hist[-1]
        out = []
        for k, box in enumerate(latest):
            seen = sum(1 for past in hist
                       if any(self._overlaps(box, q) for q in past))
            if seen >= self.CONFIRM_MIN:
                out.append((k, box))
        if len(out) < len(latest):
            log.info("ignoring %d unconfirmed %s footprint(s) of %d: seen in "
                     "fewer than %d of the last %d updates",
                     len(latest) - len(out), source, len(latest),
                     self.CONFIRM_MIN, self.CONFIRM_OF)
        return out

    @staticmethod
    def _overlaps(a, b) -> bool:
        """Whether two footprints describe the same thing, roughly.

        Compared by overlap rather than by identity: an object's footprint moves
        a few centimetres between frames as the mask breathes, so an exact match
        would confirm nothing.
        """
        return not (a[1] < b[0] or b[1] < a[0] or a[3] < b[2] or b[3] < a[2])

    def reset(self) -> None:
        """Back to the start of the patrol, without forgetting the floor."""
        self._vx = self._wz = 0.0
        self._outbound = True
        self._turning = False
        self._laps = 0
        self._yaw_f = self._y_f = 0.0
        self._escape_yaw = None
        self._blocked = False

    def _merge(self, obs: list, tags: list | None = None):
        """Fuse rectangles the robot cannot fit between.

        Returns (rectangles, sources), the second being the set of sources that
        contributed to each rectangle, in the same order. Merging across sources
        is the point rather than a side effect: when both detectors see the same
        table, one barrier comes out carrying both tags, and the robot goes
        round it once.

        Two objects standing close together, a side table and a stool say, are
        two boxes to a detector but one barrier to a robot. Treated separately
        they produce opposite detours and the controller alternates between them
        every frame, which walks it straight through the gap.

        Rectangles, because circles cascade: merging two circles grows the
        radius, which makes the next merge easier, and a few objects across a
        room collapse into one disc covering everything. A bounding rectangle
        grows only in the direction of the merge.
        """
        need = 2.0 * self.ROBOT_HALF_WIDTH
        items = [list(o) for o in obs]
        marks = ([set(t) for t in tags] if tags is not None
                 else [{"ours"} for _ in obs])
        changed = True
        while changed and len(items) > 1:
            changed = False
            for i in range(len(items)):
                for j in range(i + 1, len(items)):
                    a_, b_ = items[i], items[j]
                    gx = max(a_[0] - b_[1], b_[0] - a_[1])   # gap in x, <0 if overlapping
                    gy = max(a_[2] - b_[3], b_[2] - a_[3])
                    # Only a gap if there is clearance in BOTH directions;
                    # otherwise the robot cannot slip between them.
                    if max(gx, gy) >= need:
                        continue
                    # Touching or overlapping rectangles are NOT merged. This
                    # exists to close a gap the robot cannot pass through, and
                    # where there is no gap there is nothing to close. It is
                    # also what keeps the pieces of one concave object apart:
                    # the compositor now decomposes an L-shaped couch into two
                    # or three abutting rectangles, and merging them on a
                    # negative gap would rebuild the bounding box the
                    # decomposition exists to avoid -- the inside of the L
                    # would go back to being an obstacle.
                    if max(gx, gy) <= 0.0:
                        continue
                    # Same detected object: never merge. This is the guard the
                    # decomposition depends on. Without it the two arms of an
                    # L-shaped couch, whose crook is narrower than `need`, are
                    # fused straight back into the bounding box the
                    # decomposition exists to avoid -- and the touching test
                    # above does not catch it, because the crook is a genuine
                    # POSITIVE gap.
                    if any(t.count("#") and t in marks[j] for t in marks[i]):
                        continue
                    fused = [min(a_[0], b_[0]), max(a_[1], b_[1]),
                             min(a_[2], b_[2]), max(a_[3], b_[3])]
                    # The span guard has to survive the merge, or it does not
                    # exist. Refusing their 3.8 x 2.2 m block one cluster at a
                    # time is pointless if three of their sub-3 m clusters then
                    # chain with one of ours and rebuild it: measured as a
                    # 5.3 x 3.8 m barrier tagged ours+suite, 43 escapes and 5
                    # "no way round" over three minutes, against 1 and 0 for
                    # our footprints alone on the same scene.
                    #
                    # Only when a suite box is involved, so "ours" stays
                    # bit-identical to the shipped demo -- our own footprints
                    # merge into a 5.3 x 3.6 m barrier on this scene too, and
                    # that one costs nothing because it lies along the far edge
                    # rather than across the lane. This is not a claim that big
                    # merges are wrong in general; it is that a source which
                    # cannot separate objects must not be allowed to grow one.
                    if ("suite" in marks[i] or "suite" in marks[j]) and (
                            fused[1] - fused[0] > self.SUITE_MAX_SPAN
                            or fused[3] - fused[2] > self.SUITE_MAX_SPAN):
                        continue
                    items[i] = fused
                    marks[i] |= marks[j]
                    items.pop(j)
                    marks.pop(j)
                    changed = True
                    break
                if changed:
                    break
        if len(items) < len(obs):
            log.info("merged %d footprints into %d: gaps narrower than %.2f m "
                     "are not gaps", len(obs), len(items), need)
        if obs and items:
            _b4 = max(max(r[1] - r[0], r[3] - r[2]) for r in obs)
            _af = max(max(r[1] - r[0], r[3] - r[2]) for r in items)
            log.info("merge: largest span %.2f m before -> %.2f m after "
                     "(%d rects -> %d)", _b4, _af, len(obs), len(items))
        return [tuple(o) for o in items], marks

    def _escape(self, pose: Pose, dt: float):
        """Command that gets the robot out of a footprint it is already inside.

        The detour only looks AHEAD, so it says nothing about an obstacle the
        robot is standing in. That happens whenever an object is detected late,
        moved, or first seen while close: the shift is applied, but the robot
        keeps walking forward through the footprint while it slowly steers out.
        Here it walks straight out by the shortest way instead, which is the one
        case where going backwards is right.

        Returns None when the robot is in the clear.
        """
        if not self._obstacles or time.time() - self._obstacles_t > self.STALE:
            self._inside_reason = ""
            self._escape_yaw = None
            return None
        x, y = pose.centre
        here = np.array([x, y])
        worst, depth = None, 0.0
        # Grown while escaping, so the robot leaves with room to spare. Exiting
        # at the exact boundary hands control straight back to the walk, which
        # steers to a lane that may sit inside the same obstacle, and the two
        # fight: measured as an escape every five seconds for two minutes.
        pad = self.ROBOT_HALF_WIDTH if self._escape_yaw is not None else 0.0
        for x0, x1, y0, y1 in self._obstacles:
            if not (x0 - pad <= x <= x1 + pad and y0 - pad <= y <= y1 + pad):
                continue
            # Sideways only. Leaving through the front or the back of a
            # footprint the leg runs through is futile: the robot steps out,
            # the walk resumes toward a lane still inside it, and it re-enters
            # at once. Measured as 6 of 33 escapes taking a 0.06 m exit along
            # the travel axis and looping. A lateral exit is the one the detour
            # is trying to reach anyway, so the two now pull the same way.
            outs = ((y - y0 + pad, np.array([0.0, -1.0])),
                    (y1 - y + pad, np.array([0.0, 1.0])))
            d_out, direction = min(outs, key=lambda t: t[0])
            if d_out > depth:
                depth, worst = d_out, (direction, d_out)
        if worst is None:
            self._inside_reason = ""
            self._escape_yaw = None
            return None
        direction, d_out = worst
        yaw = pose.yaw
        if self._escape_yaw is None:
            # Fix the way out once. Derived from the current heading it would
            # rotate with the robot, which then chases its own target and never
            # leaves: measured as never escaping from the exact centre.
            self._escape_yaw = float(np.arctan2(direction[1], direction[0]))
        want = self._escape_yaw
        err = (want - yaw + np.pi) % (2 * np.pi) - np.pi
        if self._inside_reason != "inside":
            self._inside_reason = "inside"
            log.warning("inside an obstacle footprint by %.2f m: backing out "
                        "toward %+.0f deg", depth, np.degrees(want))
        # Always keep walking, even while still turning toward the exit. This
        # policy only turns BY STEPPING: below about 0.25 m/s the feet stop
        # leaving the ground and the delivered yaw collapses from 0.66 to
        # 0.04 rad/s. Commanding vx=0 to "turn on the spot" therefore asks for a
        # rotation the robot cannot perform, and it sits there: 23 samples at
        # vx 0.00 in a five-minute run, with the heading oscillating instead of
        # converging. Walking through a short arc is slower in principle and far
        # faster in practice.
        vx = self.TURN_VX
        return self._smooth(vx, float(np.clip(err * 1.5, -self.TURN_RATE,
                                              self.TURN_RATE)), dt)

    def _detour(self, x: float, y: float, along, lane: float) -> float:
        """Lateral shift of the lane that clears the obstacles ahead.

        Rather than stopping or replanning a path, the target lane is pushed
        sideways: the robot is already holding a line by cross-track control, so
        moving the line is all that is needed and the motion stays smooth. The
        push falls off with distance so the robot drifts around an obstacle well
        before reaching it instead of swerving at the last moment.

        Only obstacles AHEAD and near the line count. Something beside the path
        or behind is not in the way, and reacting to it would make the robot
        wander for no reason.
        """
        if not self._obstacles or time.time() - self._obstacles_t > self.STALE:
            self._detour_reason = ""
            return 0.0
        # How far ahead to start reacting is not a free choice: moving S metres
        # sideways while heading no more than CROSS_MAX off the line needs at
        # least S / tan(CROSS_MAX) metres of run-up. A fixed look-ahead therefore
        # silently fails on wide obstacles, which is exactly what happened: a
        # 1.31 m detour needs 3.7 m and only 3.5 m was allowed. Scale it.
        widest = max((max(o[1] - o[0], o[3] - o[2]) / 2.0
                      for o in self._obstacles), default=0.3)
        need = (widest + self.ROBOT_HALF_WIDTH) * self.DETOUR_GAIN
        # Distance the robot needs to move `need` metres sideways while heading
        # no more than CROSS_MAX off the line. The MARGIN factor is what turns a
        # geometric minimum into something the cross-track law can actually
        # deliver, since it approaches its target asymptotically.
        runup = need / max(0.2, np.tan(self.CROSS_MAX)) * self.DETOUR_RUNUP
        look = max(self.OBSTACLE_LOOK, runup)
        dx = 1.0 if along[0] >= 0 else -1.0
        # The obstacle's offset is measured from the LANE, not from the robot.
        # Measuring from the robot is self-defeating: once the detour has moved
        # it aside, the obstacle leaves the corridor, the push drops to zero, the
        # robot returns to the lane and the cycle repeats. It oscillates past the
        # obstacle instead of going round it. The lane does not move, so an
        # obstacle on it stays on it until the robot is past.
        # All of this in WORLD coordinates. The legs run along +-x, so "ahead"
        # is a signed distance along x and "sideways" is plain y. Expressing it
        # in a travel frame instead mixed the two: the offset was measured along
        # a normal that flips with direction while the lane it was compared and
        # added to stayed in world y, so the shift pushed the robot INTO
        # obstacles on one leg out of two.
        push, blocking = 0.0, None
        for idx, (x0, x1, y0, y1) in enumerate(self._obstacles):
            ox, oy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
            ahead = (ox - x) * dx
            # From the LANE, not from the robot. Measured from the robot, the
            # obstacle leaves the corridor as soon as the shift has moved it
            # aside, the push drops to zero, the robot returns, and it
            # oscillates past instead of going round.
            side = oy - lane
            # Also count an obstacle the robot is currently ALONGSIDE, not only
            # one ahead. A 2.2 m deep footprint stops being "ahead" as soon as
            # its centre is passed, the shift drops, and the lane snaps back
            # into the obstacle the robot is still beside: measured as the robot
            # stuck at x=3.5 for two minutes, leaving at 0.2 cm per second.
            behind = -(x1 - x0) / 2.0 - self.ROBOT_HALF_WIDTH
            if not (behind < ahead < look):
                continue
            # Half-extent ACROSS the lane, not a radius. A circle round a 2.4 m
            # dining table has a 1.28 m radius and claims two and a half times
            # the floor the table occupies, which is where the 2.36 m obstacles
            # and the impossible 2.78 m detours came from. Seen end-on, that
            # same table blocks only 0.45 m of the line.
            half = (y1 - y0) / 2.0
            # Ask for a little more than is strictly needed. The cross-track
            # law reaches its target asymptotically, so a shift of exactly the
            # required clearance is always slightly under-delivered: measured at
            # 0.71 m for a 0.75 m request. Over-asking by this factor closes that
            # gap without changing the control law.
            # The footprints already carry OBSTACLE_MARGIN, applied by whoever
            # produced them, so only the robot's own half-width is added here.
            # A second full clearance on top would push it a metre wide of a
            # chair. This used to switch on where the obstacles came from, back
            # when two sources wrote the same list.
            clear = (half + self.ROBOT_HALF_WIDTH) * self.DETOUR_GAIN
            if abs(side) > clear:
                continue
            # Push to whichever side needs the smaller move, by exactly what
            # clears the obstacle. The shift must reach its FULL value before
            # the robot arrives, not at the moment of contact: fading it all the
            # way in leaves the robot brushing past. So it ramps to full over the
            # outer part of the look-ahead and holds from there in.
            want = clear - abs(side)
            direction = 1.0 if side <= 0.0 else -1.0
            # Complete the shift `runup` metres out, not at a fixed fraction of
            # the look-ahead. Tied to the look-ahead it gave 1.58 m of run-up to
            # every small obstacle, whatever it needed: measured as 1.61 m
            # required for a stool and 2.81 m for a chair, so the robot was
            # always late and clipped anything sitting on its lane.
            ramp = (look - ahead) / max(1e-6, min(runup, look))
            contribution = direction * want * float(np.clip(ramp, 0.0, 1.0))
            if abs(contribution) > abs(push):
                push, blocking = contribution, (ox, oy, x1 - x0, ahead, side,
                                                y1 - y0, self._src_of(idx))
        if blocking is None:
            self._detour_reason = ""
            return 0.0
        reason = f"{blocking[0]:.1f},{blocking[1]:.1f}"
        if reason != self._detour_reason:
            self._detour_reason = reason
            # The source is named because it is the whole question the union is
            # there to answer: a detour tagged "suite" is one our own perception
            # would never have taken.
            log.info("obstacle %.1f x %.1f m at (%.1f, %.1f) [%s], %.1f m "
                     "ahead, %+.2f m to the side: shifting the line %+.2f m",
                     blocking[2], blocking[5], blocking[0], blocking[1],
                     blocking[6], blocking[3], blocking[4], push)
        # Back into world coordinates before returning. The shift above is
        # measured along the travel normal, which points +y outbound and -y on
        # the way back; the caller adds it to a lane expressed as a plain y.
        # Returning it unconverted pushed the robot INTO obstacles on every
        # return leg, which is why they were only ever hit in one direction.
        capped = float(np.clip(push, -self.DETOUR_MAX, self.DETOUR_MAX))
        if abs(capped) < abs(push) - 1e-3:
            # No detour of this size exists: the obstacle spans more of the room
            # than the robot can go round. Repeating a warning every frame while
            # walking into it is the worst of both, so stop and say so once. A
            # barrier five metres wide is a scene to be changed, not a control
            # problem to be solved.
            if self._blocked_reason != "no way round":
                self._blocked_reason = "no way round"
                self._no_way_round += 1
                log.warning("no way round (#%d): that obstacle [%s] needs a "
                            "%.2f m detour and only %.2f m is available. "
                            "Stopping. Move the furniture, raise DETOUR_MAX, "
                            "or shorten the run with STOP_AT.",
                            self._no_way_round, blocking[6], abs(push),
                            self.DETOUR_MAX)
            self._blocked = True
            return capped
        self._blocked = False
        if self._blocked_reason:
            self._blocked_reason = ""
            log.info("a way round is available again")
        return capped

    def step(self, pose: Pose, dt: float) -> np.ndarray:
        # Before anything decides where to go, notice whether the last decision
        # actually moved the robot.
        # centre, not lead: the toe swings with every step and would read
        # as movement while the robot goes nowhere.
        self._watch_stall(float(pose.centre[0]), float(pose.centre[1]))
        # Goal mode short-circuits the patrol entirely: there is no axis, no
        # lane and no about-face, only the plan. Everything it uses below --
        # _escape, _smooth, the yaw cap, the TURN_VX floor -- is shared.
        if self.MODE == "goal":
            return self._follow(pose, dt)
        # The polygon is only followed in perimeter mode. In axis mode it is
        # still received and used to keep the run inside the real floor.
        """Pace the optical axis: out to STOP_AT, about-face, back to RETURN_TO.

        The world origin is the floor under the camera and +x is the optical
        axis, so staying on the axis means holding y = 0. Heading alone will not
        do that: a small yaw bias integrates into a drift that never comes back.
        The lateral error is nulled over LOOKAHEAD metres instead, in whichever
        direction the robot is currently travelling.

        At either end it stops and turns on the spot until it faces the other
        way, then walks. Turning while still moving would swing it off the line
        and need correcting afterwards, which is what makes an about-face look
        hesitant.

        Distances are measured from the front of the feet, the point that
        actually reaches a limit first, while the lateral error uses the midpoint
        between them, which is steady across the gait.
        """
        x = pose.lead[0]
        x_c, y_raw = pose.centre
        yaw_raw = pose.yaw

        # Filter what the controller reacts to. A biped yaws and sways with every
        # step, several degrees at around two steps per second. Feeding that into
        # the heading term makes the controller fight the gait: it commands yaw at
        # step frequency, which deforms the step, which increases the sway.
        # Averaging over half a second ignores the stride and still catches a
        # real drift within a metre. The RAW values drive the limits and the logs,
        # so nothing downstream is delayed.
        a = dt / max(dt, self.SMOOTH_TAU)
        dyaw = (yaw_raw - self._yaw_f + np.pi) % (2 * np.pi) - np.pi
        self._yaw_f = (self._yaw_f + a * dyaw + np.pi) % (2 * np.pi) - np.pi
        self._y_f += a * (y_raw - self._y_f)
        yaw, y = self._yaw_f, self._y_f

        # Turning until the new heading is reached. Not strictly on the spot:
        # the policy is trained to walk, and a yaw command with no forward speed
        # is the regime it handles worst, because a biped pivots by stepping. A
        # little forward speed lets it take those steps, so the about-face is a
        # tight arc rather than a long shuffle. TURN_VX=0 restores a pure pivot.
        if self._turning:
            done = self._turn_step(x, y_raw, yaw_raw, dt)
            if done is not None:
                return done

        limit = self._stop_at if self._outbound else self.RETURN_TO
        reached = x >= limit if self._outbound else x <= limit
        if reached:
            self._start_turn(x, y_raw)
            return self._smooth(self.TURN_VX, self._turn_sign * self.TURN_WZ, dt)

        # Two lanes, not one line. A turn of radius R always displaces the robot
        # 2R sideways, so a robot walking exactly on the axis ends every turn
        # 2R off it and has to walk back diagonally, which is the wobble that
        # shows up worst in front of the camera. Holding the outbound leg at -R
        # and the inbound leg at +R means each turn lands the robot exactly on
        # the other lane: no recovery at all, and the largest deviation from the
        # axis is halved, from 2R to R. LANE=0 restores a single centred line.
        # Out of a footprint first, if standing in one.
        _grid = self._grid_ready()
        escape = self._escape_grid(pose, dt) if _grid else self._escape(pose, dt)
        if escape is not None:
            return escape

        d = 1.0 if self._outbound else -1.0
        lane = -self.lane * d
        # Shift the lane sideways to clear whatever YOLO sees on the way. The
        # cross-track law below then steers to the shifted line, so avoiding is
        # the same motion as lane keeping and needs no separate mode.
        lane += (self._detour_grid(x_c, lane, d, lane) if _grid
                 else self._detour(x_c, y_raw, (d, 0.0), lane))
        if self._blocked:
            if _grid and x > self.RETURN_TO + 0.3:
                # The far end of the floor the robot can actually reach, found
                # by walking to it rather than by a number set months ago. A
                # patrol that turns round here covers the room it has; one that
                # stands in front of the couch waiting for a lane that does not
                # exist covers nothing. Standing still was the right answer
                # when the barrier was an artefact of bounding boxes -- with
                # cells it is a real wall, and a real wall is where you turn.
                if self._outbound and self._stop_at > x:
                    log.info("turning at %.2f m: nothing is clear beyond it "
                             "(STOP_AT is %.2f m)", x, self.STOP_AT)
                    self._stop_at = x
                self._start_turn(x, y_raw)
                return self._smooth(self.TURN_VX,
                                    self._turn_sign * self.TURN_WZ, dt)
            # Hold position rather than press on into something there is no way
            # round. Standing still in front of an obstacle reads as deliberate;
            # walking into it does not.
            return self._smooth(0.0, 0.0, dt)
        err_y = (y - lane) * d
        want = (0.0 if self._outbound else np.pi) + float(np.clip(
            np.arctan2(-self.CROSS_GAIN * err_y, self.LOOKAHEAD),
            -self.CROSS_MAX, self.CROSS_MAX))
        err = (want - yaw + np.pi) % (2 * np.pi) - np.pi
        # Damp with the measured yaw rate. The policy answers a yaw command with
        # a lag of a few tenths of a second, so a purely proportional term keeps
        # pushing while the turn it already asked for is still arriving, and the
        # heading overshoots and comes back: a slow oscillation, about eight
        # seconds a cycle, which is what showed as wandering on the straights.
        damp = self.YAW_DAMP * pose.yaw_rate
        # Full speed unless the heading is genuinely wrong. Scaling speed by the
        # heading error at any size meant the small corrections that KEEP the
        # robot on the axis also slowed it: five centimetres off the line cost
        # 8% of cruise, and it never ran at the speed it was asked for. Only a
        # real misalignment, beyond SLOW_ABOVE, is worth braking for.
        over = max(0.0, abs(err) - self.SLOW_ABOVE)
        vx = self.CRUISE_VX * max(0.3, 1.0 - over / 0.9)
        # And no faster than the detour can actually be delivered. The lane
        # moves instantly, the robot does not; without this the corner is
        # clipped every lap however correct the lane was.
        if _grid:
            vx *= self._runup_cap(x_c, y, d, lane)
        # Never below the speed at which this policy still steps. The G1 walker
        # has start/stop hysteresis around TURN_VX: asked for less it stops
        # lifting its feet and stands, which is exactly what a run-up brake
        # commanding 0.15 m/s produced -- a robot correctly deciding to slow
        # down and then not walking at all. Braking is a control choice; not
        # stepping is a failure, and it looks identical to being blocked.
        #
        # Applied to BOTH reductions above, not just the run-up: the heading
        # term's own floor of 0.3 x CRUISE_VX is 0.18 m/s, which is under the
        # threshold too and would bite as soon as a correction was large.
        vx = max(self.TURN_VX, vx)
        # Ease down to turning speed over the last stretch rather than dropping
        # to it the instant the limit is crossed. The slew limiter would spread
        # that step over a quarter of a second anyway, but as a visible lurch;
        # blending it over the approach makes the robot flow into the turn.
        to_go = abs(limit - x)
        if to_go < self.EASE_IN:
            blend = to_go / self.EASE_IN
            vx = self.TURN_VX + (vx - self.TURN_VX) * blend
        if time.time() - self._last_walk_log > 2.0:
            self._last_walk_log = time.time()
            log.info("walking %s: toes at %.2f m (limit %.2f), %+.3f m off the "
                     "axis (lane %+.2f), heading %+.1f deg, vx %.2f m/s, lap %d",
                     "out" if self._outbound else "back", x, limit, y_raw, lane,
                     np.degrees(yaw_raw), self._vx, self._laps)
        return self._smooth(vx, float(np.clip(err * self.HEAD_GAIN - damp,
                                              -self.TURN_RATE,
                                              self.TURN_RATE)), dt)

    def _start_turn(self, x: float, y: float) -> None:
        """Begin the about-face at the end of a leg."""
        self._turning = True
        self._turn_started = time.time()
        self._target_yaw = np.pi if self._outbound else 0.0
        self._turn_sign = self._turn_side(x, y)
        self._turn_y0 = y
        self._laps += 0 if self._outbound else 1
        log.info("reached %.2f m, turning to walk %s", x,
                 "back" if self._outbound else "away again")

    def _turn_side(self, x: float, y: float) -> float:
        """Which way to sweep the about-face, +1 or -1.

        A turn displaces the robot about 2R sideways, and R is what `lane` has
        been measuring all along. Always turning the same way was right while
        obstacles were bounding boxes, because the sweep happened in open floor
        at the end of a run that stopped well short of anything. With cells the
        patrol now turns as late as the floor allows, which puts the sweep
        beside the furniture: measured on this lounge as the robot clipping the
        coffee table's near corner on the about-face, 189 poses of 3000 with a
        worst clearance of -0.205 m, while the straights themselves were clean.

        Derived from free space and not from the heading error, which is what
        made the old choice flip between laps: two degrees of a near-180-degree
        error decided left or right. Free space on one side does not flicker.
        Ties keep the previous sign, so a symmetric room still turns the same
        way every lap.
        """
        if not self._grid_ready():
            return 1.0
        sweep = max(0.3, 2.0 * self.lane)
        room = self.grid_half
        # The sweep is sideways AND slightly along the leg, so the corridor is
        # checked over the whole arc rather than at its end point only.
        ok = {}
        for sgn in (1.0, -1.0):
            ok[sgn] = not any(
                corridor_blocked(self._occ, x, y + sgn * f * sweep, 1.0, 0.0,
                                 room, self._cell, self._bounds, behind=0.35,
                                 pad=self.grid_pad)
                for f in (0.33, 0.66, 1.0))
        if ok[self._turn_sign]:
            return self._turn_sign
        other = -self._turn_sign
        if ok[other]:
            log.info("turning the other way at %.2f m: a %.2f m sweep to "
                     "%+.0f is clear and the usual side is not",
                     x, sweep, 90 * other)
            return other
        return self._turn_sign

    def _turn_step(self, x: float, y: float, yaw: float, dt: float):
        """Drive the about-face. Returns a command, or None when it is over.

        Always the same way round. Deriving the direction from the sign of a
        near-180-degree error meant two degrees of heading decided left or
        right, and it flipped between laps; with the two lanes below, a left
        turn from one lands exactly on the other, so there is nothing to decide.

        Speed is held constant. Cutting it near the end of the turn was tried
        and is precisely this policy's failure mode: at 0.10 m/s the feet stop
        leaving the ground and the yaw collapses from 0.68 to 0.04 rad/s,
        turning a 4.6 s about-face into a 9.7 s one.
        """
        err = (self._target_yaw - yaw + np.pi) % (2 * np.pi) - np.pi
        if abs(err) >= self.TURN_DONE:
            return self._smooth(self.TURN_VX, self._turn_sign * self.TURN_WZ, dt)

        self._turning = False
        self._outbound = not self._outbound
        log.info("about-face complete in %.1f s, now walking %s at %.2f m",
                 time.time() - self._turn_started,
                 "away from the camera" if self._outbound else "back", x)
        # Learn the lane offset from the turn just performed. A turn displaces
        # the robot 2R sideways, and the lane that makes the NEXT turn land on
        # the opposite lane is half of that. Measuring beats a constant: the
        # radius depends on what the policy actually delivers, which drifts a
        # few percent between laps, and a lane 5 cm wrong is 5 cm the
        # cross-track term fights on every leg.
        swept = abs(y - self._turn_y0)
        if 0.2 < swept < 2.0:
            self.lane = 0.8 * self.lane + 0.2 * (swept / 2.0)
            log.info("  swept %.2f m sideways, lane now %.2f m", swept, self.lane)
        return None

    def _smooth(self, vx_des: float, wz_des: float, dt: float) -> np.ndarray:
        """Rate limit the command, lift it over the start threshold if the robot
        is stalled, and shout if it stays stalled anyway.

        Every path out of this navigator funnels through here, which is why the
        floor and the watchdog live here rather than at the three places that
        ask for TURN_VX. Putting them at the call sites is how the last one got
        missed.
        """
        if vx_des > 0.0 and self._stalled:
            # Stopped, and being asked to move: ask for enough to actually
            # start. See START_VX -- the floor the rest of the code uses is the
            # speed that keeps a walking robot walking, 0.16 m/s below the one
            # that gets a stopped one going.
            vx_des = max(vx_des, self.START_VX)
        self._vx += float(np.clip(vx_des - self._vx, -self.VX_SLEW * dt,
                                  self.VX_SLEW * dt))
        self._wz += float(np.clip(wz_des - self._wz, -self.WZ_SLEW * dt,
                                  self.WZ_SLEW * dt))
        return np.array([self._vx, 0.0, self._wz])

    def _watch_stall(self, x: float, y: float) -> None:
        """Notice a robot that is being commanded to move and is not moving.

        The failure this exists for looked like a healthy demo: the navigator
        logged "walking back ... vx 0.26 m/s, lap 18" once a second, forever,
        with the robot stationary. Nothing was wrong with any single line. What
        was missing was anyone comparing the command against the displacement.
        """
        now = time.time()
        if self._stall_t0 == 0.0:
            self._stall_t0, self._stall_x, self._stall_y = now, x, y
            return
        if now - self._stall_t0 < self.STALL_WINDOW:
            return
        moved = float(np.hypot(x - self._stall_x, y - self._stall_y))
        commanded = self._vx
        self._stalled = commanded > 0.01 and moved < self.STALL_MIN_MOVE
        if self._stalled and now - self._stall_said > 5.0:
            self._stall_said = now
            log.warning("STALLED: commanded vx %.2f m/s for %.1f s and moved "
                        "%.3f m (under %.2f m). The policy starts at %.2f m/s "
                        "and stops at %.2f, so a command between the two holds "
                        "a walking robot and cannot restart a stopped one; "
                        "lifting to START_VX %.2f.",
                        commanded, now - self._stall_t0, moved,
                        self.STALL_MIN_MOVE, 0.42, self.TURN_VX, self.START_VX)
        self._stall_t0, self._stall_x, self._stall_y = now, x, y
