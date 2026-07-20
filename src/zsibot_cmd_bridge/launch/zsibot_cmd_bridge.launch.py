"""Launch the ROS 2 cmd_vel to ZsiBot HighLevel SDK bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("zsibot_cmd_bridge")
    default_config = os.path.join(share, "config", "zsibot_cmd_bridge.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/scan_planner/cmd_vel"),
            Node(
                package="zsibot_cmd_bridge",
                executable="zsibot_cmd_bridge",
                name="zsibot_cmd_bridge",
                output="screen",
                parameters=[LaunchConfiguration("config_file")],
                remappings=[("cmd_vel", LaunchConfiguration("cmd_vel_topic"))],
            ),
        ]
    )
