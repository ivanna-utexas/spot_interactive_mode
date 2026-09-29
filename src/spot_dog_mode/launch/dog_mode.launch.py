import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('spot_dog_mode')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    include_tracking_arg = DeclareLaunchArgument(
        'include_tracking',
        default_value='false',
        description='Also launch the canonical CUDA-CenterPoint tracker',
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Gaze-controller parameter YAML file',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock for rosbag or simulation validation',
    )

    tracking_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('people_detector'),
                'launch',
                'pedestrian_tracking.launch.py',
            )
        ),
        condition=IfCondition(LaunchConfiguration('include_tracking')),
    )

    gaze_controller = Node(
        package='spot_dog_mode',
        executable='gaze_controller_node',
        name='dog_mode_gaze_controller',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool
                )
            },
        ],
    )

    return LaunchDescription([
        include_tracking_arg,
        params_file_arg,
        use_sim_time_arg,
        tracking_launch,
        gaze_controller,
    ])
