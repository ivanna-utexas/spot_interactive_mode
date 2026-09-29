"""Compatibility alias for pedestrian_tracking.launch.py."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    primary = os.path.join(
        get_package_share_directory("people_detector"),
        "launch",
        "pedestrian_tracking.launch.py",
    )
    return LaunchDescription(
        [IncludeLaunchDescription(PythonLaunchDescriptionSource(primary))]
    )
