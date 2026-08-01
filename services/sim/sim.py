# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Physics service.

Runs the MuJoCo model at a fixed rate, applies whatever controller was selected,
and publishes the resulting configuration. It does no rendering, so it can be
pinned to isolated cores without fighting the GPU for anything.
"""

from __future__ import annotations

import logging
import os
import random
import signal
import time

import mujoco
import numpy as np
from edgebot import topics
from edgebot.bus import Publisher, Subscriber

from behaviours import AvoidBehaviour
from controllers import make_controller

log = logging.getLogger("sim")

ROBOT = os.environ.get("ROBOT", "g1")
PHYSICS_HZ = float(os.environ.get("PHYSICS_HZ", "1000"))
CONTROL_HZ = float(os.environ.get("CONTROL_HZ", "50"))
PUBLISH_HZ = float(os.environ.get("PUBLISH_HZ", "60"))

SCENES = {
    "g1": "/models/mujoco_menagerie/unitree_g1/scene.xml",
    "h1": "/models/mujoco_menagerie/unitree_h1/scene.xml",
    "t1": "/models/mujoco_menagerie/booster_t1/scene.xml",
    # 29-DoF G1 matching the RL walker policy, fetched by make policy.
    "g1_walker": "/models/g1_walker/scene.xml",
}

# Below this base height the robot has clearly gone over. Used only to report
# state and offer a reset, never to stop the physics.
FALLEN_HEIGHT = 0.4


def _point_in_poly(x: float, y: float, poly: list) -> bool:
    """Ray-casting point-in-polygon test for keep-out zones (world ground)."""
    inside = False
    n = len(poly)
    if n < 3:
        return False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi):
            inside = not inside
        j = i
    return inside


class Sim:
    def __init__(self) -> None:
        scene = SCENES.get(ROBOT)
        if scene is None:
            raise SystemExit(f"unknown robot {ROBOT!r}, expected one of {sorted(SCENES)}")
        if not os.path.exists(scene):
            raise SystemExit(
                f"{scene} is missing. Fetch the model first: "
                "run 'make build'")

        log.info("loading %s", scene)
        self.model = mujoco.MjModel.from_xml_path(scene)
        self.data = mujoco.MjData(self.model)

        self.controller = make_controller(self.model, self.data)

        # A controller may require a specific physics rate (the RL policy's
        # gains and armature are tuned for one) and its own standing pose. Honor
        # those before anything steps, falling back to the configured defaults.
        physics_hz = getattr(self.controller, "physics_hz", None) or PHYSICS_HZ
        self.model.opt.timestep = 1.0 / physics_hz
        self.physics_hz = physics_hz

        if hasattr(self.controller, "stand"):
            self.controller.stand(self.data)
        elif self.model.nkey:
            # Menagerie scenes ship a "home" keyframe with the robot standing.
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
            mujoco.mj_forward(self.model, self.data)
        else:
            mujoco.mj_forward(self.model, self.data)

        self.behaviour = AvoidBehaviour()
        self.pub = Publisher()
        # Sim only listens to perception; the walk is decided here
        # autonomously and avoids obstacles. No manual/teleop mode.
        self.sub = Subscriber([topics.PERCEPTION_OBSTACLES, topics.CMD_RESET, topics.CMD_TURN,
                               topics.KEEPOUT_ZONES])
        self.cmd = np.zeros(3)
        self._manual_turn = 0.0  # pending operator yaw (radians), applied per frame
        self._keepout: list[list[tuple[float, float]]] = []  # world-ground polygons
        self._avoid_reason = ""  # current avoidance cause, for change-logging
        self._vx = 0.0           # rate-limited commands, for smooth heading changes
        self._wz = 0.0
        self._outbound = True     # True walking away from the camera
        self._turning = False     # True while the about-face is in progress
        self._turn_sign = 1.0     # direction of rotation, fixed when a turn starts
        self._turn_y0 = 0.0       # lateral position when the turn began
        self.lane = self.LANE     # learned from the turns actually performed
        self._yaw_f = 0.0         # gait-filtered heading, for control only
        self._y_f = 0.0           # gait-filtered lateral position
        self._target_yaw = 0.0
        self._turn_started = 0.0
        self._turn_track: list = []   # (commanded, actual) yaw during a turn
        self._turn_lift: list = []    # highest foot during a turn, m
        self._laps = 0
        self._last_walk_log = 0.0
        self._steer_reason = ""
        # Every position test below uses the front of the feet on the ground, not
        # the free joint. qpos[0:2] is the pelvis, 0.79 m up and roughly 0.2 m
        # behind the toes in mid-stride, so a boundary checked against it is a
        # boundary the feet have already crossed. The G1's four contact spheres
        # per foot sit at x = -0.05 (heel) and x = +0.12 (toe) in the ankle frame,
        # and MuJoCo gives their world position every step.
        self._foot_geoms = [
            g for g in range(self.model.ngeom)
            if "ankle_roll" in (mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_BODY,
                int(self.model.geom_bodyid[g])) or "")]
        log.info("ground reference: %d foot contact geoms", len(self._foot_geoms))
        # True once the robot has reached the walkable floor for the first time.
        # Before that it simply walks straight out from under the camera; after
        # that it is steered back in if it ever leaves.
        self.running = True

        self._steps_per_control = max(1, int(physics_hz / CONTROL_HZ))
        self._steps_per_publish = max(1, int(physics_hz / PUBLISH_HZ))
        log.info("physics %.0f Hz, control %.0f Hz", physics_hz, CONTROL_HZ)

        signal.signal(signal.SIGTERM, self._stop)
        signal.signal(signal.SIGINT, self._stop)

    def _stop(self, *_: object) -> None:
        self.running = False

    def _poll_bus(self) -> None:
        """Consume perception messages, then compute the walk command."""
        while (msg := self.sub.recv(0)) is not None:
            topic, payload = msg
            if topic == topics.PERCEPTION_OBSTACLES:
                payload = self._obstacles_to_robot_frame(payload)
                self.behaviour.observe(payload)
            elif topic == topics.CMD_RESET:
                # Compositor asked (via 'z') to send the robot back to its start
                # pose. Reset physics and the walk together.
                self._reset()
                self.behaviour = AvoidBehaviour()
            elif topic == topics.CMD_TURN:
                # Operator turn nudge (l/r keys). Accumulate a yaw offset in radians
                # that is applied on top of the walk command over a few frames.
                self._manual_turn += float(payload.get("wz", 0.0))
            elif topic == topics.KEEPOUT_ZONES:
                # Operator-drawn no-go polygons (world ground coords). The robot
                # steers away from these, covering obstacles the detector misses.
                self._keepout = [[tuple(p) for p in poly] for poly in payload.get("zones", [])]

        # Step 1 of the movement work: walk straight along the optical axis and
        # stop at STOP_AT. Everything else (patrol regions, wandering, about-face
        # rules) has been removed; it will be rebuilt one step at a time.
        raw = self._walk_the_axis()

        st = self.behaviour.status()
        if st.get("avoiding") and st.get("closest_m") is not None:
            if self._avoid_reason != "obstacle":
                self._avoid_reason = "obstacle"
                log.info("avoiding: obstacle at %.2fm ahead", st["closest_m"])
        elif self._avoid_reason == "obstacle":
            self._avoid_reason = ""

        raw = self._avoid_keepout(raw)

        self.cmd = np.array(
            [
                np.clip(raw[0], -topics.MAX_VX, topics.MAX_VX),
                np.clip(raw[1], -topics.MAX_VY, topics.MAX_VY),
                np.clip(raw[2], -topics.MAX_WZ, topics.MAX_WZ),
            ]
        )

    def _apply_manual_turn(self, dt: float) -> None:
        """Rotate the robot's heading by any pending operator turn, smoothly.

        Applied a slice per physics step so a 5 deg nudge sweeps over a fraction
        of a second instead of snapping, which keeps the walker balanced.
        """
        if abs(self._manual_turn) < 1e-4:
            return
        # Take a capped slice of the pending turn this step.
        rate = np.radians(120.0)  # deg/s sweep speed
        step = float(np.clip(self._manual_turn, -rate * dt, rate * dt))
        self._manual_turn -= step
        # Rotate the base quaternion about z by 'step'.
        qw, qx, qy, qz = (float(self.data.qpos[3]), float(self.data.qpos[4]),
                          float(self.data.qpos[5]), float(self.data.qpos[6]))
        h = step / 2.0
        cw, sw = np.cos(h), np.sin(h)
        # Quaternion multiply: q_turn (about z) * q_current.
        self.data.qpos[3] = cw * qw - sw * qz
        self.data.qpos[4] = cw * qx - sw * qy
        self.data.qpos[5] = cw * qy + sw * qx
        self.data.qpos[6] = cw * qz + sw * qw

    def _steer(self, reason: str, detail: str = "") -> None:
        """Log why the heading is being changed, once per change of reason."""
        if reason != self._steer_reason:
            self._steer_reason = reason
            log.info("heading change: %s%s", reason, f" ({detail})" if detail else "")

    def _yaw(self) -> float:
        qw, qz = float(self.data.qpos[3]), float(self.data.qpos[6])
        return 2.0 * float(np.arctan2(qz, qw))

    def _ground_ref(self) -> tuple[float, float]:
        """Ground position of the front of the feet: how far the robot has got.

        Among the foot contact geoms, the one furthest along the heading is the
        leading toe. It is the point that reaches a limit first, so distances are
        measured from it.

        It is NOT usable for lateral control: the leading toe alternates between
        the left and the right foot at every step, which injects a square wave of
        about +-0.15 m into any cross-track error built on it. Use _ground_centre
        for that.
        """
        if not self._foot_geoms:
            return float(self.data.qpos[0]), float(self.data.qpos[1])
        pts = self.data.geom_xpos[self._foot_geoms][:, :2]
        yaw = self._yaw()
        fwd = np.array([np.cos(yaw), np.sin(yaw)])
        lead = pts[int(np.argmax(pts @ fwd))]
        return float(lead[0]), float(lead[1])

    def _ground_centre(self) -> tuple[float, float]:
        """Ground position midway between the feet: steady across the gait."""
        if not self._foot_geoms:
            return float(self.data.qpos[0]), float(self.data.qpos[1])
        c = self.data.geom_xpos[self._foot_geoms][:, :2].mean(axis=0)
        return float(c[0]), float(c[1])

    def _obstacles_to_robot_frame(self, payload: dict) -> dict:
        """Re-express camera-measured obstacles relative to the robot.

        Perception reports range and bearing from the CAMERA, which sits at the
        world origin looking along +x. The avoidance behaviour reads them as if
        they were relative to the robot. With the robot several metres down the
        room, a person standing right in front of the lens reads as an obstacle
        a few centimetres ahead of it, and the robot brakes and turns away from
        something that is in fact well behind it.

        The camera sees an obstacle at world (r*cos b, -r*sin b), b positive to
        the right. Subtract the robot's position, rotate into its heading, and
        drop whatever ends up behind it.
        """
        obstacles = payload.get("obstacles") or payload.get("people") or []
        if not obstacles:
            return payload
        rx, ry = self._ground_ref()
        yaw = self._yaw()
        out = []
        for obs in obstacles:
            rng = obs.get("range_m")
            if rng is None or not np.isfinite(rng):
                out.append(obs)
                continue
            b = np.radians(float(obs.get("bearing_deg", 0.0)))
            ox, oy = rng * np.cos(b), -rng * np.sin(b)
            dx, dy = ox - rx, oy - ry
            b_rel = (np.arctan2(dy, dx) - yaw + np.pi) % (2 * np.pi) - np.pi
            if abs(np.degrees(b_rel)) > 100.0:
                continue           # behind the robot: not its problem
            o = dict(obs)
            o["range_m"] = float(np.hypot(dx, dy))
            o["bearing_deg"] = float(-np.degrees(b_rel))
            out.append(o)
        p2 = dict(payload)
        p2["obstacles"] = out
        p2.pop("people", None)
        return p2

    STOP_AT = float(os.environ.get("STOP_AT", "6.0"))      # far end of the run
    RETURN_TO = float(os.environ.get("RETURN_TO", "1.5"))  # near end
    TURN_DONE = 0.12      # rad, how close to the new heading ends the about-face
    # About-face. Measured on this policy: at 0.89 rad/s it delivers 0.83, but
    # at 1.20 the actual yaw collapses to 0.04 and even reverses. That is not
    # saturation, it is falling off the end of the training distribution, and
    # asking for more made the turn three times SLOWER (10.5 s against 3.5 s).
    # So the command is capped at what the policy can actually track. Forward
    # speed helps because a biped turns by stepping, but it must stay small or
    # the about-face sweeps the robot metres off the line.
    # TURN_VX matters more than TURN_WZ. Measured on this policy, yaw collapses
    # from 0.74 to 0.03 rad/s at a CONSTANT 0.9 command as the robot slows: it
    # turns by stepping, and with almost no forward speed it plants both feet and
    # stops rotating. Walking a proper arc keeps it in the regime it was trained
    # for. The cost is that the about-face sweeps some ground sideways.
    TURN_WZ = float(os.environ.get("TURN_WZ", "0.9"))   # rad/s asked while turning
    # The arc radius is TURN_VX divided by the yaw actually delivered, so the
    # only safe way to tighten it is less speed OR more yaw. Less speed stalls
    # the policy, so TURN_VX stays at the value where the feet keep stepping and
    # the yaw command is what gets raised. 0.9 is the most this policy tracks:
    # beyond that it collapses, which the yaw-tracking line will report.
    TURN_VX = float(os.environ.get("TURN_VX", "0.26"))  # m/s kept while turning
    CRUISE_VX = float(os.environ.get("CRUISE_VX", "0.6"))  # m/s along the axis
    SLOW_ABOVE = 0.30     # rad of heading error below which speed is not cut
    CROSS_GAIN = 2.0      # how hard to pull back onto the axis
    # Distance over which the lateral error is nulled. This is the OUTER loop,
    # and it has to be slow compared with the inner heading loop or the two
    # fight each other. At 1.0 m and 0.6 m/s the outer loop was under two
    # seconds, barely slower than the policy's own response, and the pair
    # limit-cycled: 3.6 deg of heading swing holding a lane, 7 deg recovering
    # from a turn. At 2.0 m those become 0.8 and 2.5 deg.
    LOOKAHEAD = float(os.environ.get("LOOKAHEAD", "2.0"))
    HEAD_GAIN = 1.2       # rad/s of yaw command per rad of heading error
    YAW_DAMP = float(os.environ.get("YAW_DAMP", "0.5"))  # per rad/s measured
    # Cap on the heading used to rejoin the axis. At 0.6 rad the robot walks
    # back at 34 degrees, which looks like it has lost its way rather than like
    # a correction. Lower, it takes longer to converge but always looks like it
    # is walking down the line.
    CROSS_MAX = float(os.environ.get("CROSS_MAX", "0.35"))
    # Half the turn diameter: the lane offset that makes each about-face land on
    # the opposite lane. TURN_VX / delivered yaw, and this policy delivers about
    # 0.66 rad/s of the 0.9 asked.
    LANE = float(os.environ.get("LANE", "0.39"))
    EASE_IN = 0.8         # m before the limit over which speed blends into the turn
    # Time constant of the filter on the heading and lateral position used for
    # control. Must exceed one stride or the controller chases the gait's own
    # sway; too long and it is slow to notice a real drift.
    SMOOTH_TAU = float(os.environ.get("SMOOTH_TAU", "0.5"))
    TURN_RATE = 0.9       # rad/s cap on the yaw command
    VX_SLEW = 1.2         # m/s^2 limit on the forward command
    WZ_SLEW = float(os.environ.get("WZ_SLEW", "4.0"))  # rad/s^2 on the yaw command

    def _walk_the_axis(self) -> np.ndarray:
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
        x, _ = self._ground_ref()
        _, y_raw = self._ground_centre()
        yaw_raw = self._yaw()
        dt = 1.0 / 50.0

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
            # The turn compares against the UNFILTERED heading: it is a large,
            # fast rotation, and the filter would still be reporting it as in
            # progress well after it finished.
            err = (self._target_yaw - yaw_raw + np.pi) % (2 * np.pi) - np.pi
            if abs(err) < self.TURN_DONE:
                self._turning = False
                self._outbound = not self._outbound
                took = time.time() - self._turn_started
                log.info("about-face complete in %.1f s, now walking %s at %.2f m",
                         took, "away from the camera" if self._outbound else "back", x)
                # How much of the commanded yaw the policy actually delivered.
                # Below about half, the command is outside what it was trained
                # for and LOWERING it will speed the turn up, not slow it down.
                if self._turn_track:
                    cmd_avg = sum(c for c, _ in self._turn_track) / len(self._turn_track)
                    act_avg = sum(a for _, a in self._turn_track) / len(self._turn_track)
                    ratio = act_avg / max(cmd_avg, 1e-6)
                    log.info("  yaw tracking %.0f%% (commanded %.2f, delivered "
                             "%.2f rad/s)", 100.0 * ratio, cmd_avg, act_avg)
                    if ratio < 0.5:
                        lift = max(self._turn_lift) if self._turn_lift else 0.0
                        if lift < 0.03:
                            log.warning("  the feet barely left the ground (highest "
                                        "%.3f m): this policy turns by STEPPING, so "
                                        "below about 0.25 m/s it plants both feet "
                                        "and stops rotating. Raise TURN_VX above "
                                        "%.2f m/s.", lift, self.TURN_VX)
                        else:
                            log.warning("  the policy is not following that yaw "
                                        "rate: try a different TURN_WZ than %.2f",
                                        self.TURN_WZ)
                # Learn the lane offset from the turn just performed. A turn
                # displaces the robot 2R sideways, and the lane that makes the
                # next turn land exactly on the opposite lane is half of that.
                # Measuring beats the constant it replaces: the radius depends on
                # what the policy actually delivers, which drifts a few percent
                # between laps, and a lane that is even 5 cm wrong is 5 cm the
                # cross-track term has to correct on every single leg.
                swept = abs(y_raw - self._turn_y0)
                if 0.2 < swept < 2.0:
                    self.lane = 0.8 * self.lane + 0.2 * (swept / 2.0)
                    log.info("  swept %.2f m sideways, lane now %.2f m",
                             swept, self.lane)
                self._turn_track = []
                self._turn_lift = []
            else:
                actual = float(self.data.qvel[5])
                self._turn_track.append((abs(self._wz), abs(actual)))
                # Foot clearance says whether the policy is still stepping. Yaw
                # collapsing at a CONSTANT command is not a command-range issue:
                # a biped turns by stepping, and once both feet stay planted the
                # yaw goes to zero whatever is asked. Reported so the difference
                # between "asked too much" and "stopped walking" is visible.
                if self._foot_geoms:
                    _fz = self.data.geom_xpos[self._foot_geoms][:, 2]
                    self._turn_lift.append(float(_fz.max()))
                if time.time() - self._last_walk_log > 2.0:
                    self._last_walk_log = time.time()
                    lift = self._turn_lift[-1] if self._turn_lift else -1.0
                    log.info("turning: %+.0f deg to go, commanding %.2f rad/s, "
                             "actual %.2f rad/s, vx %.2f, highest foot %.3f m%s",
                             np.degrees(err), self._wz, actual, self._vx, lift,
                             "  <- stalled, feet not stepping"
                             if abs(actual) < 0.15 and self._vx > 0.05 else "")
                # Constant speed. Cutting it near the end of the turn was tried
                # and is exactly the failure this policy has: at 0.10 m/s the
                # feet stop leaving the ground (0.021 m against 0.077 m) and the
                # yaw collapses from 0.68 to 0.04 rad/s, turning a 4.6 s
                # about-face into a 9.7 s one. The radius has to be reduced by
                # asking for MORE yaw at the same speed, not by slowing down.
                return self._smooth(self.TURN_VX,
                                    self._turn_sign * self.TURN_WZ, dt)

        limit = self.STOP_AT if self._outbound else self.RETURN_TO
        reached = x >= limit if self._outbound else x <= limit
        if reached:
            self._turning = True
            self._target_yaw = np.pi if self._outbound else 0.0
            self._laps += 0 if self._outbound else 1
            self._turn_started = time.time()
            # Fix the direction of rotation once, here, and hold it for the whole
            # turn. Deriving it from the sign of a near-180 degree error, as the
            # loop below did, means two degrees of heading decide whether the
            # robot goes left or right: it flipped between laps, and this policy
            # turns visibly worse one way than the other.
            #
            # The arc always displaces the robot 2R sideways, toward whichever
            # side it turns. Turning toward the axis therefore ENDS the turn
            # closer to the line instead of a metre off it. Left of the heading
            # is +y outbound and -y inbound, hence the direction factor.
            # Always the same way round. With the two lanes below, a left turn
            # from one lane lands exactly on the other, so the direction never
            # needs to depend on where the robot happens to be.
            self._turn_sign = 1.0
            self._turn_y0 = y_raw
            log.info("reached %.2f m, turning %s to walk %s (%.2f m off the axis)",
                     x, "left" if self._turn_sign > 0 else "right",
                     "back" if self._outbound else "away again", y_raw)
            return self._smooth(self.TURN_VX, self._turn_sign * self.TURN_WZ, dt)

        # Two lanes, not one line. A turn of radius R always displaces the robot
        # 2R sideways, so a robot walking exactly on the axis ends every turn
        # 2R off it and has to walk back diagonally, which is the wobble that
        # shows up worst in front of the camera. Holding the outbound leg at -R
        # and the inbound leg at +R means each turn lands the robot exactly on
        # the other lane: no recovery at all, and the largest deviation from the
        # axis is halved, from 2R to R. LANE=0 restores a single centred line.
        d = 1.0 if self._outbound else -1.0
        lane = -self.lane * d
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
        damp = self.YAW_DAMP * float(self.data.qvel[5])
        # Full speed unless the heading is genuinely wrong. Scaling speed by the
        # heading error at any size meant the small corrections that KEEP the
        # robot on the axis also slowed it: five centimetres off the line cost
        # 8% of cruise, and it never ran at the speed it was asked for. Only a
        # real misalignment, beyond SLOW_ABOVE, is worth braking for.
        over = max(0.0, abs(err) - self.SLOW_ABOVE)
        vx = self.CRUISE_VX * max(0.3, 1.0 - over / 0.9)
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

    def _smooth(self, vx_des: float, wz_des: float, dt: float) -> np.ndarray:
        """Rate limit the command, so no heading or speed change is a step."""
        self._vx += float(np.clip(vx_des - self._vx, -self.VX_SLEW * dt,
                                  self.VX_SLEW * dt))
        self._wz += float(np.clip(wz_des - self._wz, -self.WZ_SLEW * dt,
                                  self.WZ_SLEW * dt))
        return np.array([self._vx, 0.0, self._wz])

    TURN_AWAY_WZ = 1.6  # rad/s turn when hitting a keep-out zone

    def _avoid_keepout(self, raw: np.ndarray) -> np.ndarray:
        """Stop and turn if the robot is in or approaching a keep-out zone."""
        if not self._keepout:
            return raw
        x, y = self._ground_ref()
        yaw = self._yaw()
        look = 0.6   # m ahead, enough to react before entering a keep-out zone
        ax = x + np.cos(yaw) * look
        ay = y + np.sin(yaw) * look
        for i, poly in enumerate(self._keepout):
            if _point_in_poly(x, y, poly) or _point_in_poly(ax, ay, poly):
                # Inside or about to enter: kill forward speed and turn away hard.
                if self._avoid_reason != f"zone{i}":
                    self._avoid_reason = f"zone{i}"
                    log.info("avoiding: keep-out zone %d at robot (%.1f, %.1f)", i, x, y)
                return np.array([-0.1, 0.0, self.TURN_AWAY_WZ])
        if self._avoid_reason and self._avoid_reason.startswith("zone"):
            self._avoid_reason = ""
        return raw

    def _reset(self) -> None:
        """Return the robot to its start pose without restarting the service."""
        if self.model.nkey:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        else:
            mujoco.mj_resetData(self.model, self.data)
        # Back to the world origin, which is the floor directly under the camera,
        # facing away from it. The robot then walks out into the scene again.
        self._outbound = True
        self._turning = False
        self._laps = 0
        self._yaw_f = self._y_f = 0.0
        self._vx = self._wz = 0.0
        mujoco.mj_forward(self.model, self.data)
        # Reset the controller's internal state in place instead of recreating it,
        # which would recompile the policy (slow, and can stall the reset).
        if hasattr(self.controller, "reset"):
            self.controller.reset(self.data)
        if hasattr(self.controller, "stand"):
            self.controller.stand(self.data)
        log.info("reset to start pose")

    def _fallen(self) -> bool:
        return bool(self.data.qpos[2] < FALLEN_HEIGHT)

    def run(self) -> None:
        dt = self.model.opt.timestep
        step = 0
        next_deadline = time.perf_counter()
        jitter = np.zeros(int(PHYSICS_HZ), dtype=np.float64)
        policy_ms = 0.0
        last_report = time.perf_counter()
        steps_at_report = 0

        log.info("physics %.0f Hz, control %.0f Hz, publish %.0f Hz", PHYSICS_HZ, CONTROL_HZ, PUBLISH_HZ)

        while self.running:
            if step % self._steps_per_control == 0:
                self._poll_bus()
                t0 = time.perf_counter()
                self.controller.update(self.cmd, self.data)
                policy_ms = (time.perf_counter() - t0) * 1e3

            self.controller.apply(self.data)
            self._apply_manual_turn(dt)
            mujoco.mj_step(self.model, self.data)

            if step % self._steps_per_publish == 0:
                self.pub.send(
                    topics.ROBOT_STATE,
                    {
                        "t": float(self.data.time),
                        "qpos": self.data.qpos.tolist(),
                        "fallen": self._fallen(),
                    },
                )

            # Pace the loop against a wall-clock deadline rather than sleeping a
            # fixed amount, so a slow step is absorbed instead of accumulating.
            next_deadline += dt
            slack = next_deadline - time.perf_counter()
            jitter[step % len(jitter)] = -slack * 1e6
            if slack > 0:
                time.sleep(slack)
            else:
                next_deadline = time.perf_counter()

            step += 1

            now = time.perf_counter()
            if now - last_report >= 1.0:
                elapsed = now - last_report
                achieved_hz = (step - steps_at_report) / elapsed
                self.pub.send(
                    topics.SIM_TELEMETRY,
                    {
                        "physics_hz": float(achieved_hz),
                        # Real-time factor: 1.0 means simulated time keeps up
                        # with wall-clock time. Below 1.0 the demo is slow motion.
                        "rtf": float(achieved_hz * dt),
                        "jitter_p99_us": float(np.percentile(jitter[:min(step, len(jitter))], 99)),
                        "policy_ms": policy_ms,
                        **self.behaviour.status(),
                    },
                )
                last_report = now
                steps_at_report = step

        log.info("stopping")
        self.pub.close()
        self.sub.close()


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    Sim().run()


if __name__ == "__main__":
    main()
