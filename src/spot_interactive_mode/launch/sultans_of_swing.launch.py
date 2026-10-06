"""Interactive mode with the Sultans of Swing dance instead of Dancing Queen.

Same node, same modes and controls (dance = Circle / `d`, R1 held); only
the choreography and the music change. Every other interactive_mode.launch.py
argument (audio_lead_sec, calibration_mode, start_mode, include_tracking)
still works here.

  ros2 launch spot_interactive_mode sultans_of_swing.launch.py

The clip (media/SultansOfSwingDemoClip.wav) is copyrighted and gitignored;
see the README for how to recreate it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('spot_interactive_mode')
    return LaunchDescription([
        DeclareLaunchArgument(
            'audio_file',
            default_value=os.path.expanduser(
                '~/spot_interactive_mode/media/SultansOfSwingDemoClip.wav')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'interactive_mode.launch.py')),
            launch_arguments={
                'timeline': os.path.join(pkg, 'config', 'sultans_of_swing.json'),
                'audio_file': LaunchConfiguration('audio_file'),
            }.items(),
        ),
    ])
