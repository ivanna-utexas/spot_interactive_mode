#!/usr/bin/env python3
"""Bridge a ``geometry_msgs/PoseArray`` of people (in map frame) to ``/nearby_people``.

Designed to consume ``/zed_people`` produced by ``zed_people_relay`` running in the
``zed_ws`` container. Positions arrive **already in the ``map`` frame**, so this
node performs no TF lookups -- it only needs ``/robot_pose`` (PoseStamped in the
``map`` frame) to express each person as ego-frame ``(x, y, cos_theta, sin_theta)``.

Each ``Pose.orientation`` is interpreted as a yaw-only rotation that encodes the
person's velocity heading (as written by ``zed_people_relay``). When the
quaternion is identity, the person is treated as near-stationary and the heading
falls back to the radial direction from the robot.

Run directly with ``python3``; no entry-point registration is required.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from bva_msgs.msg import NearbyObstacle, NearbyObstacles
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.node import Node


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _world_to_ego(world_xy: np.ndarray, robot_pose: PoseStamped) -> np.ndarray:
    rx = robot_pose.pose.position.x
    ry = robot_pose.pose.position.y
    yaw = _yaw_from_quaternion(robot_pose.pose.orientation)
    dx = float(world_xy[0]) - rx
    dy = float(world_xy[1]) - ry
    c = math.cos(-yaw)
    s = math.sin(-yaw)
    return np.array([c * dx - s * dy, s * dx + c * dy], dtype=np.float64)


def _is_identity_quat(q, eps: float = 1e-6) -> bool:
    return (
        abs(q.x) < eps
        and abs(q.y) < eps
        and abs(q.z) < eps
        and abs(q.w - 1.0) < eps
    )


class ZedPoseArrayToNearbyNode(Node):
    MAX_OBSTACLES = 16

    def __init__(self) -> None:
        super().__init__("zed_posearray_to_nearby")

        self.declare_parameter("zed_people_topic", "/zed_people")
        self.declare_parameter("robot_pose_topic", "/robot_pose")
        self.declare_parameter("output_topic", "/nearby_people")
        self.declare_parameter("publish_rate_hz", 20.0)

        self._zed_topic = str(self.get_parameter("zed_people_topic").value)
        self._pose_topic = str(self.get_parameter("robot_pose_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))

        self._robot_pose: Optional[PoseStamped] = None
        self._latest_people: Optional[PoseArray] = None

        self._pub = self.create_publisher(NearbyObstacles, self._output_topic, 10)

        self.create_subscription(PoseArray, self._zed_topic, self._on_people, 10)
        self.create_subscription(
            PoseStamped, self._pose_topic, self._on_robot_pose, 10
        )

        self.create_timer(1.0 / rate, self._tick_publish)
        self.get_logger().info(
            f"ZED PoseArray bridge: {self._zed_topic} + {self._pose_topic} "
            f"-> {self._output_topic} ({rate:.1f} Hz)"
        )

    def _on_robot_pose(self, msg: PoseStamped) -> None:
        self._robot_pose = msg

    def _on_people(self, msg: PoseArray) -> None:
        self._latest_people = msg

    def _tick_publish(self) -> None:
        robot = self._robot_pose
        if robot is None:
            self.get_logger().warn(
                "Waiting for /robot_pose before publishing /nearby_people",
                throttle_duration_sec=10.0,
            )
            return

        nearby = NearbyObstacles()
        nearby.header = robot.header

        people = self._latest_people
        if people is None or not people.poses:
            self._pub.publish(nearby)
            return

        # NearbyObstacles is ego-frame so frame mismatches between the PoseArray
        # ("map") and the robot pose ("map") only matter logically. We assume the
        # relay delivered map-frame positions, matching the robot pose frame.
        scored: List[Tuple[float, float, float, float, float]] = []
        for pose in people.poses:
            mx = float(pose.position.x)
            my = float(pose.position.y)
            ego = _world_to_ego(np.array([mx, my], dtype=np.float64), robot)

            if _is_identity_quat(pose.orientation):
                if math.hypot(float(ego[0]), float(ego[1])) > 1e-3:
                    radial_yaw = math.atan2(float(ego[1]), float(ego[0]))
                    cos_t, sin_t = math.cos(radial_yaw), math.sin(radial_yaw)
                else:
                    cos_t, sin_t = 1.0, 0.0
            else:
                map_yaw = _yaw_from_quaternion(pose.orientation)
                robot_yaw = _yaw_from_quaternion(robot.pose.orientation)
                ego_yaw = map_yaw - robot_yaw
                cos_t = math.cos(ego_yaw)
                sin_t = math.sin(ego_yaw)

            scored.append(
                (
                    float(ego[0]),
                    float(ego[1]),
                    cos_t,
                    sin_t,
                    float(ego[0]) ** 2 + float(ego[1]) ** 2,
                )
            )

        scored.sort(key=lambda r: r[4])
        scored = scored[: self.MAX_OBSTACLES]

        for ex, ey, c, s, _ in scored:
            o = NearbyObstacle()
            o.x = ex
            o.y = ey
            o.cos_theta = c
            o.sin_theta = s
            nearby.obstacles.append(o)

        self._pub.publish(nearby)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZedPoseArrayToNearbyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
