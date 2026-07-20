from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def _setup(context):
    config_file = LaunchConfiguration("config_file").perform(context)
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic").perform(context)
    if not config_file:
        config_file = os.path.join(
            get_package_share_directory("zsibot_cmd_bridge"),
            "config",
            "zsibot_cmd_udp_client.yaml",
        )
    return [
        Node(
            package="zsibot_cmd_bridge",
            executable="zsibot_cmd_udp_client",
            name="zsibot_cmd_udp_client",
            output="screen",
            parameters=[config_file],
            remappings=[("cmd_vel", cmd_vel_topic)],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=""),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/scan_planner/cmd_vel"),
            OpaqueFunction(function=_setup),
        ]
    )
