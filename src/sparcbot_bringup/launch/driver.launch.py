"""Start only the hardware driver (motors + encoders), for open-loop testing.

    ros2 launch sparcbot_bringup driver.launch.py

Then drive it from a second terminal:

    ros2 run sparcbot_teleop pwm_teleop

Parameters come from config/sparcbot.yaml.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = str(
        Path(get_package_share_directory('sparcbot_bringup')) / 'config' / 'sparcbot.yaml')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='SPARCBot parameter file'),

        Node(
            package='sparcbot_nodes',
            executable='sparcbot_driver_node',
            name='sparcbot_driver',
            parameters=[params_file],
            output='screen',
            emulate_tty=True),
    ])
