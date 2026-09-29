from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("output_topic", default_value="/people_detections"),
            DeclareLaunchArgument("enable_markers", default_value="true"),
            DeclareLaunchArgument("marker_topic", default_value="/people_detections_markers"),
            DeclareLaunchArgument("marker_scale", default_value="0.35"),
            DeclareLaunchArgument("marker_lifetime_sec", default_value="0.5"),
            DeclareLaunchArgument("enable_ptv3", default_value="true"),
            DeclareLaunchArgument("ptv3_labels_topic", default_value="/pointcept/labels"),
            DeclareLaunchArgument("ptv3_points_topic", default_value="/velodyne_points"),
            DeclareLaunchArgument("ptv3_person_label", default_value="1"),
            DeclareLaunchArgument("ptv3_cluster_tolerance", default_value="0.6"),
            DeclareLaunchArgument("ptv3_min_cluster_points", default_value="20"),
            DeclareLaunchArgument("ptv3_min_z", default_value="-1.0"),
            DeclareLaunchArgument("ptv3_max_z", default_value="2.5"),
            Node(
                package="people_detector",
                executable="people_format_adapter_node.py",
                name="people_format_adapter",
                output="screen",
                parameters=[
                    {
                        "output_topic": LaunchConfiguration("output_topic"),
                        "enable_markers": LaunchConfiguration("enable_markers"),
                        "marker_topic": LaunchConfiguration("marker_topic"),
                        "marker_scale": LaunchConfiguration("marker_scale"),
                        "marker_lifetime_sec": LaunchConfiguration("marker_lifetime_sec"),
                        "enable_ptv3": LaunchConfiguration("enable_ptv3"),
                        "ptv3_labels_topic": LaunchConfiguration("ptv3_labels_topic"),
                        "ptv3_points_topic": LaunchConfiguration("ptv3_points_topic"),
                        "ptv3_person_label": LaunchConfiguration("ptv3_person_label"),
                        "ptv3_cluster_tolerance": LaunchConfiguration("ptv3_cluster_tolerance"),
                        "ptv3_min_cluster_points": LaunchConfiguration("ptv3_min_cluster_points"),
                        "ptv3_min_z": LaunchConfiguration("ptv3_min_z"),
                        "ptv3_max_z": LaunchConfiguration("ptv3_max_z"),
                    }
                ],
            ),
        ]
    )
