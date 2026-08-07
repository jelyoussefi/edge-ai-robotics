#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Bridge between this project's ZeroMQ bus and Intel's ADBSCAN clusterer.

Feeds the same depth path the groundfloor node consumes -- one depth image plus
a camera_info plus a static TF -- and returns ADBSCAN's clusters to the bus as
footprints in the same rectangle format this project already uses.

Why a second bridge rather than one shared with services/groundfloor. The depth
half is genuinely the same and is duplicated here on purpose: groundfloor is the
measured etape B baseline and refactoring it while etape C is being measured
would put both results in doubt at once. Merging the two is worth doing, after.

Why the comparison target is right this time. ADBSCAN produces obstacle
clusters and so do we; both are "something is here, roughly this big". Etape B's
first mistake was comparing our footprints against a floor segmenter's derived
output, which measured definitions rather than perception. Here the two sides
answer the same question.
"""
from __future__ import annotations

import json
import math
import os
import sys
import time

import numpy as np

import rclpy
from geometry_msgs.msg import TransformStamped
from nav2_dynamic_msgs.msg import ObstacleArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster

sys.path.insert(0, "/opt/edgebot")
from edgebot import topics                                    # noqa: E402
from edgebot.bus import Publisher, Subscriber                 # noqa: E402

CALIB = os.environ.get("CAMERA_CALIBRATION", "/config/camera_calibration.json")
SENSOR = os.environ.get("GF_SENSOR_NAME", "camera")
# Identical to the compositor's and the groundfloor bridge's, or the three
# footprint sets are not comparable.
MARGIN = float(os.environ.get("OBSTACLE_MARGIN", "0.20"))
# Same impossible-return guards as the groundfloor bridge, applied to cluster
# CENTRES here rather than to points.
Z_MIN = float(os.environ.get("GF_Z_MIN", "-0.3"))
X_MAX = float(os.environ.get("GF_X_MAX", "8.0"))


class AdbscanBridge(Node):
    """Bus in, ROS out, ROS in, bus out."""

    def __init__(self) -> None:
        super().__init__("edgebot_adbscan_bridge")

        try:
            with open(CALIB) as fh:
                calib = json.load(fh)
        except Exception as exc:                              # noqa: BLE001
            self.get_logger().error(
                f"no calibration at {CALIB} ({exc}). The cluster positions come "
                f"back in a camera-centred frame and cannot be placed in the "
                f"world without one. Run 'make calibrate' first.")
            raise SystemExit(1)

        self.height = float(calib.get("camera_height_m", 1.56))
        self.pitch = math.radians(abs(float(calib.get("pitch_deg", 0.0))))
        self.fx = float(calib.get("fx", 386.4))
        self.fy = float(calib.get("fy", 386.5))
        self.ppx = float(calib.get("ppx", 325.6))
        self.ppy = float(calib.get("ppy", 239.6))

        # BEST_EFFORT to match what a camera driver does. Their node subscribes
        # with rclcpp::SensorDataQoS(), which is best-effort too, so unlike the
        # groundfloor node there is no reliability mismatch to work around and
        # no launch flag to pass.
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.depth_pub = self.create_publisher(
            Image, f"/{SENSOR}/depth/image_rect_raw", qos)
        self.info_pub = self.create_publisher(
            CameraInfo, f"/{SENSOR}/depth/camera_info", qos)
        # Their publisher is a plain rclcpp::QoS(1), i.e. RELIABLE, so this
        # subscription has to be reliable as well.
        self.create_subscription(
            ObstacleArray, "/obstacle_array", self._on_obstacles,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))

        self._publish_static_tf()

        self.bus_pub = Publisher()
        self.bus_sub = Subscriber([topics.CAMERA_DEPTH])
        self.create_timer(0.01, self._pump_bus)

        self._sent = 0
        self._got = 0
        self._clusters = 0
        self._dropped = 0
        self._stamps: dict = {}
        self._last_report = time.time()
        self.get_logger().info(
            f"adbscan bridge up: camera at {self.height:.2f} m, pitch "
            f"{math.degrees(self.pitch):.1f} deg, feeding /{SENSOR}/depth")

    def _publish_static_tf(self) -> None:
        """base_link -> camera optical frame, from the calibration.

        ADBSCAN itself never looks this up -- it rotates axes internally and
        publishes in a camera-centred frame with no frame_id at all. The TF is
        published anyway so the depth image and its cloud are placed correctly
        for anything else that joins the graph, and so this container's view of
        the mounting is identical to the groundfloor container's.
        """
        self._tf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = f"{SENSOR}_depth_optical_frame"
        t.transform.translation.z = self.height
        qx, qy, qz, qw = self._optical_quaternion(self.pitch)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._tf.sendTransform(t)

    @staticmethod
    def _optical_quaternion(pitch: float):
        """Body-to-optical rotation, camera pitched nose down.

        Same derivation as the groundfloor bridge, including the sign: a
        positive rotation here put the forward axis at (+0.97, 0, +0.24),
        pointing at the ceiling.
        """
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        bx, by, bz, bw = -0.5, 0.5, -0.5, 0.5
        px, py, pz, pw = -sp, 0.0, 0.0, cp
        return (bw * px + bx * pw + by * pz - bz * py,
                bw * py - bx * pz + by * pw + bz * px,
                bw * pz + bx * py - by * px + bz * pw,
                bw * pw - bx * px - by * py - bz * pz)

    def _pump_bus(self) -> None:
        """Forward every depth frame from the bus into ROS."""
        for _ in range(4):
            msg = self.bus_sub.recv(0)
            if msg is None:
                return
            topic, payload = msg
            if topic != topics.CAMERA_DEPTH:
                continue
            w, h = int(payload["w"]), int(payload["h"])
            scale = float(payload.get("scale", 0.001))
            now = self.get_clock().now().to_msg()

            img = Image()
            img.header.stamp = now
            img.header.frame_id = f"{SENSOR}_depth_optical_frame"
            img.height, img.width = h, w
            depth = np.frombuffer(payload["depth"], np.uint16).reshape(h, w)
            if abs(scale - 0.001) > 1e-6:
                depth = (depth.astype(np.float32) * scale * 1000.0).astype(np.uint16)
            img.encoding = "16UC1"
            img.is_bigendian = 0
            img.step = w * 2
            img.data = depth.tobytes()
            self.depth_pub.publish(img)

            info = CameraInfo()
            info.header = img.header
            info.height, info.width = h, w
            sx, sy = w / 640.0, h / 480.0
            info.k = [self.fx * sx, 0.0, self.ppx * sx,
                      0.0, self.fy * sy, self.ppy * sy,
                      0.0, 0.0, 1.0]
            info.p = [self.fx * sx, 0.0, self.ppx * sx, 0.0,
                      0.0, self.fy * sy, self.ppy * sy, 0.0,
                      0.0, 0.0, 1.0, 0.0]
            info.distortion_model = "plumb_bob"
            info.d = [0.0] * 5
            self.info_pub.publish(info)

            self._sent += 1
            self._stamps[(now.sec, now.nanosec)] = time.perf_counter()
            if len(self._stamps) > 64:
                self._stamps.pop(next(iter(self._stamps)))

    def _to_world(self, cx, cy, cz, sx, sy, sz):
        """One ADBSCAN cluster -> one world footprint (x0, x1, y0, y1).

        ADBSCAN publishes no frame_id, and its positions are NOT in the optical
        frame it was fed. For Lidar_type RS it rotates every input point by
        (x, y, z) <- (z, -x, -y) before clustering, which is the optical-to-body
        axis swap: x forward along the optical axis, y left, z up. That frame is
        camera-CENTRED and still carries the camera's pitch, because the swap is
        axes only and no tilt is undone anywhere in their pipeline.

        So two things are needed to reach this project's world frame, whose
        origin is the ground point under the camera: undo the pitch about y, and
        lift by the camera height.

        The extent is rotated the same way. Pitch mixes x and z, so the
        axis-aligned depth of the footprint is sx*cos(p) + sz*sin(p) -- at 14
        degrees and a 1 m tall cluster that is a 24 cm difference, too much to
        ignore. y is untouched by a rotation about y.
        """
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        wx = cx * cp + cz * sp
        wy = cy
        wz = -cx * sp + cz * cp + self.height
        hx = abs(sx * cp) + abs(sz * sp)
        hy = abs(sy)
        return wx, wy, wz, hx / 2.0, hy / 2.0

    def _on_obstacles(self, msg: ObstacleArray) -> None:
        """Turn their clusters into footprints and put them on the bus."""
        boxes = []
        dropped = 0
        for ob in msg.obstacles:
            wx, wy, wz, hx, hy = self._to_world(
                ob.position.x, ob.position.y, ob.position.z,
                ob.size.x, ob.size.y, ob.size.z)
            # Same guards as the groundfloor bridge, on centres rather than
            # points: a cluster whose centre is below the floor or past the far
            # wall is a depth artefact, and it would stretch a rectangle across
            # the room exactly as the raw returns did.
            if wz < Z_MIN or wx > X_MAX or not math.isfinite(wx + wy + wz):
                dropped += 1
                continue
            boxes.append((round(wx - hx - MARGIN, 3), round(wx + hx + MARGIN, 3),
                          round(wy - hy - MARGIN, 3), round(wy + hy + MARGIN, 3)))

        self._got += 1
        self._clusters = len(boxes)
        self._dropped += dropped
        self.bus_pub.send(topics.SUITE_CLUSTERS, {
            "blocked": boxes, "clusters": len(msg.obstacles),
            "stamp": time.time()})

        sent_at = self._stamps.pop(
            (msg.header.stamp.sec, msg.header.stamp.nanosec), None)
        if sent_at is not None:
            self._latency = (time.perf_counter() - sent_at) * 1000.0
        if time.time() - self._last_report > 5.0:
            self._last_report = time.time()
            lat = getattr(self, "_latency", float("nan"))
            self.get_logger().info(
                f"{self._sent} depth frames in, {self._got} cluster sets out, "
                f"{len(msg.obstacles)} cluster(s), {self._clusters} footprint(s) "
                f"after filtering, {self._dropped} dropped as impossible, "
                f"round trip {lat:.0f} ms")


def main() -> None:
    rclpy.init()
    node = AdbscanBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.bus_pub.close()
        node.bus_sub.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
