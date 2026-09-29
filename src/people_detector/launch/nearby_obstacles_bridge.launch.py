from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_tracks_topic", default_value="/people/map_tracks"),
            DeclareLaunchArgument("people_detections_topic", default_value="/people_detections"),
            DeclareLaunchArgument("robot_pose_topic", default_value="/robot_pose"),
            DeclareLaunchArgument("output_topic", default_value="/nearby_people"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("use_people_detections_fallback", default_value="true"),
            Node(
                package="people_detector",
                executable="nearby_obstacles_bridge_node.py",
                name="nearby_obstacles_bridge",
                output="screen",
                parameters=[
                    {
                        "map_tracks_topic": LaunchConfiguration("map_tracks_topic"),
                        "people_detections_topic": LaunchConfiguration(
                            "people_detections_topic"
                        ),
                        "robot_pose_topic": LaunchConfiguration("robot_pose_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "map_frame": LaunchConfiguration("map_frame"),
                        "use_people_detections_fallback": LaunchConfiguration(
                            "use_people_detections_fallback"
                        ),
                    }
                ],
            ),
        ]
    )
