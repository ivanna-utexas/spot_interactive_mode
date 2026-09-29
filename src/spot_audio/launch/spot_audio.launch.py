import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('spot_audio'),
        'config',
        'audio.yaml',
    )

    return LaunchDescription([
        Node(
            package='spot_audio',
            executable='audio_capture_node',
            name='audio_capture_node',
            output='screen',
            parameters=[config],
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package='spot_audio',
            executable='stt_node',
            name='stt_node',
            output='screen',
            parameters=[config],
        ),
    ])
