#!/usr/bin/env python3

from collections import deque
import math
from typing import Deque, Dict, Iterable, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from people_detector.msg import People, PeopleArray
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Int32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

class PeopleFormatAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("people_format_adapter")

        self.declare_parameter("output_topic", "/people_detections")
        self.declare_parameter("enable_markers", True)
        self.declare_parameter("marker_topic", "/people_detections_markers")
        self.declare_parameter("marker_scale", 0.35)
        self.declare_parameter("marker_lifetime_sec", 0.5)

        self.declare_parameter("enable_ptv3", True)
        self.declare_parameter("ptv3_labels_topic", "/pointcept/labels")
        self.declare_parameter("ptv3_points_topic", "/velodyne_points")
        self.declare_parameter("ptv3_person_label", 1)
        self.declare_parameter("ptv3_cluster_tolerance", 0.6)
        self.declare_parameter("ptv3_min_cluster_points", 20)
        self.declare_parameter("ptv3_min_z", -1.0)
        self.declare_parameter("ptv3_max_z", 2.5)
        self.declare_parameter("ptv3_point_buffer_size", 30)
        self.declare_parameter("ptv3_match_count_tolerance", 32)

        output_topic = self.get_parameter("output_topic").value
        self._pub = self.create_publisher(PeopleArray, output_topic, 10)
        self._markers_enabled = bool(self.get_parameter("enable_markers").value)
        marker_topic = self.get_parameter("marker_topic").value
        self._marker_scale = float(self.get_parameter("marker_scale").value)
        self._marker_lifetime_sec = float(self.get_parameter("marker_lifetime_sec").value)
        self._marker_pub = self.create_publisher(MarkerArray, marker_topic, 10) if self._markers_enabled else None

        self._latest_points_xyz: Optional[np.ndarray] = None
        self._latest_points_header = None
        self._ptv3_point_buffer: Deque[Tuple[np.ndarray, object]] = deque(
            maxlen=int(self.get_parameter("ptv3_point_buffer_size").value)
        )
        self._ptv3_id_counter = 0

        if self.get_parameter("enable_ptv3").value:
            labels_topic = self.get_parameter("ptv3_labels_topic").value
            points_topic = self.get_parameter("ptv3_points_topic").value
            self.create_subscription(PointCloud2, points_topic, self._on_points, 10)
            self.create_subscription(Int32MultiArray, labels_topic, self._on_ptv3_labels, 10)
            self.get_logger().info(f"PTv3 input enabled: labels={labels_topic}, points={points_topic}")

        self.get_logger().info(f"Publishing standardized people detections on: {output_topic}")
        if self._markers_enabled:
            self.get_logger().info(f"Publishing RViz markers on: {marker_topic}")

    def _on_points(self, msg: PointCloud2) -> None:
        try:
            xyz = point_cloud2.read_points_numpy(
                msg,
                field_names=["x", "y", "z"],
                skip_nans=True,
            )
        except AssertionError:
            # Fallback for mixed PointCloud2 field datatypes (common in Velodyne clouds).
            points = point_cloud2.read_points(
                msg,
                field_names=["x", "y", "z"],
                skip_nans=True,
            )
            raw = np.asarray(points)
            if raw.dtype.names is not None:
                xyz = np.stack([raw["x"], raw["y"], raw["z"]], axis=-1).astype(np.float32, copy=False)
            else:
                xyz = np.asarray(list(points), dtype=np.float32).reshape(-1, 3)
        self._latest_points_xyz = xyz
        self._latest_points_header = msg.header
        self._ptv3_point_buffer.append((xyz, msg.header))

    def _on_ptv3_labels(self, msg: Int32MultiArray) -> None:
        if self._latest_points_xyz is None or self._latest_points_header is None:
            self.get_logger().warn("No point cloud received yet for PTv3 labels; skipping.", throttle_duration_sec=5.0)
            return

        labels = np.asarray(msg.data, dtype=np.int32)
        match = self._select_ptv3_points_for_labels(labels.shape[0])
        if match is None:
            latest_count = self._latest_points_xyz.shape[0]
            self.get_logger().warn(
                f"PTv3 labels/points length mismatch: labels={labels.shape[0]}, points={latest_count}; skipping.",
                throttle_duration_sec=5.0,
            )
            return
        xyz, header = match

        person_label = int(self.get_parameter("ptv3_person_label").value)
        min_z = float(self.get_parameter("ptv3_min_z").value)
        max_z = float(self.get_parameter("ptv3_max_z").value)
        cluster_tol = float(self.get_parameter("ptv3_cluster_tolerance").value)
        min_pts = int(self.get_parameter("ptv3_min_cluster_points").value)

        mask = labels == person_label
        mask &= xyz[:, 2] >= min_z
        mask &= xyz[:, 2] <= max_z

        person_points = xyz[mask]
        clusters = self._cluster_points(person_points, cluster_tol)

        out = PeopleArray()
        out.header = header

        for cluster in clusters:
            if cluster.shape[0] < min_pts:
                continue

            self._ptv3_id_counter += 1
            centroid = np.mean(cluster, axis=0)
            mins = np.min(cluster, axis=0)
            maxs = np.max(cluster, axis=0)
            size = maxs - mins

            person = People()
            person.id = self._ptv3_id_counter
            person.source = "ptv3_labels"
            person.label = "person"
            person.confidence = 1.0
            person.is_human = True
            person.position.x = float(centroid[0])
            person.position.y = float(centroid[1])
            person.position.z = float(centroid[2])
            person.velocity = Vector3()
            person.size.x = float(size[0])
            person.size.y = float(size[1])
            person.size.z = float(size[2])
            out.people.append(person)

        self._publish_outputs(out)

    def _select_ptv3_points_for_labels(self, label_count: int) -> Optional[Tuple[np.ndarray, object]]:
        if not self._ptv3_point_buffer:
            return None

        # Prefer exact count match from most-recent buffered cloud.
        for xyz, header in reversed(self._ptv3_point_buffer):
            if xyz.shape[0] == label_count:
                return (xyz, header)

        tolerance = int(self.get_parameter("ptv3_match_count_tolerance").value)
        if tolerance < 0:
            tolerance = 0

        best: Optional[Tuple[np.ndarray, object]] = None
        best_diff: Optional[int] = None
        for xyz, header in reversed(self._ptv3_point_buffer):
            if xyz.shape[0] < label_count:
                continue
            diff = abs(int(xyz.shape[0]) - int(label_count))
            if best_diff is None or diff < best_diff:
                best_diff = diff
                best = (xyz, header)

        if best is None or best_diff is None or best_diff > tolerance:
            return None

        xyz, header = best
        if xyz.shape[0] > label_count:
            xyz = xyz[:label_count, :]

        self.get_logger().warn(
            f"PTv3 using near-match cloud count (labels={label_count}, points={label_count + best_diff}).",
            throttle_duration_sec=5.0,
        )
        return (xyz, header)

    def _publish_outputs(self, people_msg: PeopleArray) -> None:
        self._pub.publish(people_msg)
        if not self._markers_enabled or self._marker_pub is None:
            return
        self._marker_pub.publish(self._build_markers(people_msg))

    def _build_markers(self, people_msg: PeopleArray) -> MarkerArray:
        out = MarkerArray()

        clear = Marker()
        clear.header = people_msg.header
        clear.action = Marker.DELETEALL
        out.markers.append(clear)

        lifetime = Duration(seconds=self._marker_lifetime_sec).to_msg()

        for idx, person in enumerate(people_msg.people):
            color = self._source_color(person.source)
            marker_id_base = int(person.id) if person.id >= 0 else idx
            marker_id_base = marker_id_base * 10

            body = Marker()
            body.header = people_msg.header
            body.ns = "people_body"
            body.id = marker_id_base
            body.type = Marker.SPHERE
            body.action = Marker.ADD
            body.pose.position = person.position
            body.pose.orientation.w = 1.0
            body.scale.x = max(float(person.size.x), self._marker_scale)
            body.scale.y = max(float(person.size.y), self._marker_scale)
            body.scale.z = max(float(person.size.z), self._marker_scale)
            body.color.r = color[0]
            body.color.g = color[1]
            body.color.b = color[2]
            body.color.a = 0.65
            body.lifetime = lifetime
            out.markers.append(body)

            label = Marker()
            label.header = people_msg.header
            label.ns = "people_label"
            label.id = marker_id_base + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = person.position.x
            label.pose.position.y = person.position.y
            label.pose.position.z = person.position.z + (0.5 * max(body.scale.z, self._marker_scale)) + 0.2
            label.pose.orientation.w = 1.0
            label.scale.z = max(0.25, self._marker_scale * 0.8)
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.lifetime = lifetime
            label.text = f"id={person.id} src={person.source}"
            out.markers.append(label)

        return out

    @staticmethod
    def _source_color(source: str) -> Tuple[float, float, float]:
        if source == "ptv3_labels":
            return (0.2, 0.5, 1.0)
        return (0.8, 0.8, 0.8)

    def _cluster_points(self, points: np.ndarray, tol: float) -> List[np.ndarray]:
        if points.shape[0] == 0:
            return []

        grid: Dict[Tuple[int, int, int], List[int]] = {}
        inv = 1.0 / tol

        for idx, p in enumerate(points):
            key = (int(math.floor(p[0] * inv)), int(math.floor(p[1] * inv)), int(math.floor(p[2] * inv)))
            grid.setdefault(key, []).append(idx)

        visited = np.zeros(points.shape[0], dtype=bool)
        clusters: List[np.ndarray] = []

        for start in range(points.shape[0]):
            if visited[start]:
                continue

            queue: Deque[int] = deque([start])
            visited[start] = True
            component: List[int] = []

            while queue:
                current = queue.popleft()
                component.append(current)
                px, py, pz = points[current]
                ck = (int(math.floor(px * inv)), int(math.floor(py * inv)), int(math.floor(pz * inv)))

                for neighbor_key in self._neighbor_keys(ck):
                    for neighbor_idx in grid.get(neighbor_key, []):
                        if visited[neighbor_idx]:
                            continue
                        if np.linalg.norm(points[neighbor_idx] - points[current]) <= tol:
                            visited[neighbor_idx] = True
                            queue.append(neighbor_idx)

            clusters.append(points[np.array(component, dtype=np.int32)])

        return clusters

    @staticmethod
    def _neighbor_keys(key: Tuple[int, int, int]) -> Iterable[Tuple[int, int, int]]:
        kx, ky, kz = key
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    yield (kx + dx, ky + dy, kz + dz)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PeopleFormatAdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
