"""PS4 teleop for interactive mode: joy_linux + spot_joy's teleop_node.

Use this instead of `ros2 launch spot_joy teleop.launch.py`. The upstream
spot_joy launch starts the SDL `joy` node, which numbers the PS4 buttons
differently (R1 = 10, L1 = 9, ...) from the joy_linux layout that
teleop_node and interactive_mode are written against (L1 = 4, R1 = 5). Under
SDL every teleop button is wrong and the R1 autonomy deadman is never seen.
SDL also picks whichever joystick enumerates first, which can be the Spektrum
receiver. joy_linux reads the udev symlink for the PS4 pad directly.

Matches spot_companion_mode's teleop launch.

  ros2 launch spot_interactive_mode teleop.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/ps4_primary',
                              description='udev symlink for the PS4 pad (USB only)'),
        DeclareLaunchArgument('verbose', default_value='false'),
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': LaunchConfiguration('joy_dev'),
                'deadzone': 0.1,
                # interactive_mode treats /joy older than joy_timeout (0.5 s)
                # as a released deadman, so a held R1 must keep publishing.
                'autorepeat_rate': 50.0,
            }],
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package='spot_joy',
            executable='teleop_node',
            name='spot_joy_teleop',
            output='screen',
            parameters=[{'verbose': ParameterValue(
                LaunchConfiguration('verbose'), value_type=bool)}],
        ),
    ])
