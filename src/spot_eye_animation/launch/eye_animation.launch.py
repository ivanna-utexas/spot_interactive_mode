import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('spot_eye_animation')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    display_arg = DeclareLaunchArgument('display', default_value=':0')

    return LaunchDescription([
        display_arg,
        SetEnvironmentVariable('DISPLAY', LaunchConfiguration('display')),

        Node(
            package='spot_eye_animation',
            executable='face_gaze_node',
            name='face_gaze',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='spot_eye_animation',
            executable='eye_display_node',
            name='eye_display',
            output='screen',
            parameters=[
                params_file,
                {'display': LaunchConfiguration('display')},
            ],
        ),
    ])
