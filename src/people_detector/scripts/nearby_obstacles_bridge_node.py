#!/usr/bin/env python3
"""Bridge people poses (map frame) to ``bva_msgs/NearbyObstacles`` on ``/nearby_people``.

Subscribes to ``people_detector/MapPersonArray`` (default ``/people/map_tracks``) and
``geometry_msgs/PoseStamped`` (default ``/robot_pose``). Optionally fuses
``people_detector/PeopleArray`` when map tracks are empty, transforming each
detection into ``map`` via ``tf2``.
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from bva_msgs.msg import NearbyObstacle, NearbyObstacles
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3
from people_detector.msg import MapPersonArray, PeopleArray
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from tf2_geometry_msgs import do_transform_point
except ImportError:
    do_transform_point = None  # type: ignore[misc, assignment]


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


def _velocity_map_to_ego(v_map, robot_pose: PoseStamped) -> tuple[float, float]:
    yaw = _yaw_from_quaternion(robot_pose.pose.orientation)
    c = math.cos(-yaw)
    s = math.sin(-yaw)
    vx = float(v_map.x)
    vy = float(v_map.y)
    return (c * vx - s * vy, s * vx + c * vy)


def _heading_from_velocity(vex: float, vey: float, eps: float = 0.08) -> tuple[float, float]:
    speed = math.hypot(vex, vey)
    if speed < eps:
        return (1.0, 0.0)
    theta = math.atan2(vey, vex)
    return (math.cos(theta), math.sin(theta))


class NearbyObstaclesBridgeNode(Node):
    MAX_OBSTACLES = 16

    def __init__(self) -> None:
        super().__init__("nearby_obstacles_bridge")

        self.declare_parameter("map_tracks_topic", "/people/map_tracks")
        self.declare_parameter("people_detections_topic", "/people_detections")
        self.declare_parameter("robot_pose_topic", "/robot_pose")
        self.declare_parameter("output_topic", "/nearby_people")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("use_people_detections_fallback", True)
        self.declare_parameter("transform_timeout_sec", 0.1)
        self.declare_parameter("velocity_heading_eps_m_s", 0.08)

        self._map_frame = str(self.get_parameter("map_frame").value).strip().lstrip("/") or "map"
        self._tf_timeout = float(self.get_parameter("transform_timeout_sec").value)
        self._vel_eps = float(self.get_parameter("velocity_heading_eps_m_s").value)
        self._use_fallback = bool(self.get_parameter("use_people_detections_fallback").value)

        self._robot_pose: Optional[PoseStamped] = None
        self._last_map_tracks: Optional[MapPersonArray] = None
        self._last_people: Optional[PeopleArray] = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        out_topic = str(self.get_parameter("output_topic").value)
        self._pub = self.create_publisher(NearbyObstacles, out_topic, 10)

        self.create_subscription(
            MapPersonArray,
            str(self.get_parameter("map_tracks_topic").value),
            self._on_map_tracks,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("robot_pose_topic").value),
            self._on_robot_pose,
            10,
        )
        if self._use_fallback:
            self.create_subscription(
                PeopleArray,
                str(self.get_parameter("people_detections_topic").value),
                self._on_people,
                10,
            )

        self.create_timer(0.05, self._tick_publish)
        self.get_logger().info(
            f"NearbyObstacles bridge: publishing {out_topic} "
            f"(map_tracks={self.get_parameter('map_tracks_topic').value}, "
            f"robot_pose={self.get_parameter('robot_pose_topic').value})"
        )

    def _on_robot_pose(self, msg: PoseStamped) -> None:
        self._robot_pose = msg

    def _on_map_tracks(self, msg: MapPersonArray) -> None:
        self._last_map_tracks = msg

    def _on_people(self, msg: PeopleArray) -> None:
        self._last_people = msg

    def _gather_map_rows(
        self,
    ) -> list[tuple[float, float, float, float]]:
        """List of (map_x, map_y, vx_map, vy_map) in ``map`` frame."""
        robot = self._robot_pose
        if robot is None:
            return []

        rows: list[tuple[float, float, float, float]] = []
        msg = self._last_map_tracks
        if msg is not None:
            for person in msg.people:
                px = float(person.position.x)
                py = float(person.position.y)
                vx = float(person.velocity.x)
                vy = float(person.velocity.y)
                rows.append((px, py, vx, vy))

        if rows or not self._use_fallback:
            return rows

        if do_transform_point is None:
            self.get_logger().error(
                "tf2_geometry_msgs missing; cannot use people_detections fallback",
                throttle_duration_sec=30.0,
            )
            return []

        pmsg = self._last_people
        if pmsg is None or not pmsg.people:
            return []

        src = pmsg.header.frame_id.strip().lstrip("/") or self._map_frame
        t = Time.from_msg(pmsg.header.stamp)
        try:
            tf_m = self._tf_buffer.lookup_transform(
                self._map_frame,
                src,
                t,
                timeout=Duration(seconds=self._tf_timeout),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"TF {src} -> {self._map_frame}: {exc}",
                throttle_duration_sec=5.0,
            )
            return []
        for person in pmsg.people:
            if not person.is_human:
                continue
            ps = PointStamped()
            ps.header = pmsg.header
            ps.point = person.position
            p_map = do_transform_point(ps, tf_m)
            vx = float(person.velocity.x)
            vy = float(person.velocity.y)
            rows.append((float(p_map.point.x), float(p_map.point.y), vx, vy))
        return rows

    def _tick_publish(self) -> None:
        robot = self._robot_pose
        if robot is None:
            return

        map_rows = self._gather_map_rows()
        nearby = NearbyObstacles()
        nearby.header = robot.header

        if not map_rows:
            self._pub.publish(nearby)
            return

        scored: list[tuple[float, float, float, float, float, float]] = []
        for px, py, vx_m, vy_m in map_rows:
            ego = _world_to_ego(np.array([px, py], dtype=np.float64), robot)
            vmap = Vector3(x=vx_m, y=vy_m, z=0.0)
            vex, vey = _velocity_map_to_ego(vmap, robot)
            c, s = _heading_from_velocity(vex, vey, self._vel_eps)
            if math.hypot(vex, vey) < self._vel_eps and math.hypot(ego[0], ego[1]) > 1e-3:
                c, s = _heading_from_velocity(float(ego[0]), float(ego[1]), 1e-3)
            scored.append((float(ego[0]), float(ego[1]), c, s, ego[0] ** 2 + ego[1] ** 2))

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
    node = NearbyObstaclesBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
