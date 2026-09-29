"""Interactive mode: dog-mode gaze controller + the interactive_mode node.

Expects already running (as in the auto_dog_mode tmux session): the Spot
driver, Velodyne, teleop (spot_joy + joy_linux), and optionally CenterPoint.

  ros2 launch spot_interactive_mode interactive_mode.launch.py \
      audio_file:=/home/ros/spot_interactive_mode/media/DancingQueenDemoClip.wav
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = get_package_share_directory('spot_interactive_mode')
    dog_pkg = get_package_share_directory('spot_dog_mode')

    args = [
        DeclareLaunchArgument('include_tracking', default_value='false',
                              description='Also launch CUDA-CenterPoint people tracking'),
        DeclareLaunchArgument('timeline', default_value=os.path.join(pkg, 'config', 'timeline.json')),
        DeclareLaunchArgument('audio_file', default_value=''),
        DeclareLaunchArgument('audio_lead_sec', default_value='0.0'),
        DeclareLaunchArgument('calibration_mode', default_value='false'),
        DeclareLaunchArgument('start_mode', default_value='idle'),
    ]

    # Resolved lazily, so the package is only needed when the include runs.
    tracking = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('people_detector'),
            'launch', 'pedestrian_tracking.launch.py'])),
        condition=IfCondition(LaunchConfiguration('include_tracking')),
    )

    dog_mode = Node(
        package='spot_dog_mode',
        executable='gaze_controller_node',
        name='dog_mode_gaze_controller',
        output='screen',
        parameters=[
            os.path.join(dog_pkg, 'config', 'params.yaml'),
            os.path.join(pkg, 'config', 'dog_mode_overrides.yaml'),
        ],
    )

    interactive = Node(
        package='spot_interactive_mode',
        executable='interactive_mode_node',
        name='interactive_mode',
        output='screen',
        emulate_tty=True,
        parameters=[
            os.path.join(pkg, 'config', 'interactive_mode.yaml'),
            {
                'timeline': LaunchConfiguration('timeline'),
                'audio_file': LaunchConfiguration('audio_file'),
                'audio_lead_sec': ParameterValue(
                    LaunchConfiguration('audio_lead_sec'), value_type=float),
                'calibration_mode': ParameterValue(
                    LaunchConfiguration('calibration_mode'), value_type=bool),
                'start_mode': LaunchConfiguration('start_mode'),
            },
        ],
    )

    return LaunchDescription(args + [tracking, dog_mode, interactive])
