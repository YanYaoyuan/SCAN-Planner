from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def _setup(context):
    enabled = LaunchConfiguration("enable_deprecated_zsibot_transport").perform(context)
    if enabled.lower() not in ("1", "true", "yes", "on"):
        raise RuntimeError(
            "zsibot_cmd_udp_client is a deprecated compatibility transport and is "
            "disabled by default. Use the unified robot bridge, or explicitly set "
            "enable_deprecated_zsibot_transport:=true for a temporary migration test."
        )
    config_file = LaunchConfiguration("config_file").perform(context)
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic").perform(context)
    if not config_file:
        config_file = os.path.join(
            get_package_share_directory("zsibot_cmd_bridge"),
            "config",
            "zsibot_cmd_udp_client.yaml",
        )
    return [
        LogInfo(
            msg=(
                "[DEPRECATED] Starting zsibot_cmd_udp_client. Ensure the unified "
                "robot bridge is not forwarding the same command stream."
            )
        ),
        Node(
            package="zsibot_cmd_bridge",
            executable="zsibot_cmd_udp_client",
            name="zsibot_cmd_udp_client",
            output="screen",
            parameters=[config_file, {"enable_deprecated_transport": True}],
            remappings=[("cmd_vel", cmd_vel_topic)],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_deprecated_zsibot_transport", default_value="false"
            ),
            DeclareLaunchArgument("config_file", default_value=""),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/scan_planner/cmd_vel"),
            OpaqueFunction(function=_setup),
        ]
    )
