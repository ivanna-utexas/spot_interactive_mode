"""Launch the canonical CUDA-CenterPoint pedestrian tracking pipeline."""

import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _argument(name: str, default: str, description: str):
    return DeclareLaunchArgument(name, default_value=default, description=description)


def generate_launch_description():
    package_share = get_package_share_directory("people_detector")
    workspace = os.path.dirname(os.path.dirname(get_package_prefix("people_detector")))
    default_params = os.path.join(package_share, "config", "centerpoint_people.yaml")
    default_plan = os.path.join(
        workspace, ".navws_runtime", "centerpoint", "rpn_centerhead_sim.plan"
    )
    default_scn = os.path.join(
        workspace,
        "src",
        "Lidar_AI_Solution",
        "CUDA-CenterPoint",
        "model",
        "centerpoint.scn.onnx",
    )

    arguments = [
        _argument("params_file", default_params, "CenterPoint/tracker parameter YAML."),
        _argument("points_topic", "/velodyne_points", "Input PointCloud2 topic."),
        _argument("people_topic", "/people_detections", "Tracked PeopleArray output."),
        _argument("map_topic", "/people/map_tracks", "MapPersonArray output."),
        _argument("nearby_topic", "/nearby_people", "Base-frame NearbyObstacles output."),
        _argument(
            "marker_topic",
            "/people_detections_markers",
            "Stable-ID visualization markers.",
        ),
        _argument(
            "diagnostics_topic",
            "/centerpoint_people/diagnostics",
            "DiagnosticArray output.",
        ),
        _argument("tracking_frame", "odom", "Frame used for tracking and PeopleArray."),
        _argument("map_frame", "map", "Frame used for MapPersonArray."),
        _argument("base_frame", "base_link", "Frame used for NearbyObstacles."),
        _argument("model_plan_path", default_plan, "Versioned TensorRT plan symlink."),
        _argument("scn_onnx_path", default_scn, "Sparse-convolution ONNX model."),
        _argument("sweep_count", "10", "Number of motion-compensated LiDAR sweeps."),
        _argument("score_threshold", "0.5", "New-track score threshold."),
        _argument(
            "maintain_score_threshold", "0.3", "Existing-track score threshold."
        ),
        _argument("association_gate_m", "1.5", "Maximum Euclidean association range."),
        _argument("mahalanobis_gate", "9.21", "Covariance-aware association gate."),
        _argument(
            "association_velocity_weight",
            "0.5",
            "Velocity-consistency weight used during association.",
        ),
        _argument("confirm_hits", "3", "Hits required before publication."),
        _argument("max_coast_frames", "6", "Missed frames retained before expiration."),
        _argument("max_obstacles", "16", "Nearest people retained in /nearby_people."),
        _argument("publish_markers", "true", "Publish box, velocity, and ID markers."),
        _argument("verbose", "false", "Enable verbose CenterPoint timing output."),
    ]
    overrides = {
        name: LaunchConfiguration(name)
        for name in (
            "points_topic",
            "people_topic",
            "map_topic",
            "nearby_topic",
            "marker_topic",
            "diagnostics_topic",
            "tracking_frame",
            "map_frame",
            "base_frame",
            "model_plan_path",
            "scn_onnx_path",
            "sweep_count",
            "score_threshold",
            "maintain_score_threshold",
            "association_gate_m",
            "mahalanobis_gate",
            "association_velocity_weight",
            "confirm_hits",
            "max_coast_frames",
            "max_obstacles",
            "publish_markers",
            "verbose",
        )
    }
    node = Node(
        package="people_detector",
        executable="centerpoint_people_node",
        name="centerpoint_people_node",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("params_file"), overrides],
    )
    return LaunchDescription([*arguments, node])
