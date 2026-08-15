"""Explicit compatibility launch for the deprecated vendor SDK bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return value.lower() in ("1", "true", "yes", "on")


def _setup(context):
    if not _as_bool(
        LaunchConfiguration("enable_deprecated_zsibot_transport").perform(context)
    ):
        raise RuntimeError(
            "zsibot_cmd_bridge is deprecated and disabled by default. Use the "
            "unified robot bridge, or explicitly set "
            "enable_deprecated_zsibot_transport:=true for a temporary compatibility test."
        )

    share = get_package_share_directory("zsibot_cmd_bridge")
    default_config = os.path.join(share, "config", "zsibot_cmd_bridge.yaml")
    config_file = LaunchConfiguration("config_file").perform(context) or default_config
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic").perform(context)
    return [
        LogInfo(
            msg=(
                "[DEPRECATED] Starting zsibot_cmd_bridge. Ensure the unified robot "
                "bridge and every other vendor SDK client are stopped."
            )
        ),
        Node(
            package="zsibot_cmd_bridge",
            executable="zsibot_cmd_bridge",
            name="zsibot_cmd_bridge",
            output="screen",
            parameters=[config_file, {"enable_deprecated_transport": True}],
            remappings=[("cmd_vel", cmd_vel_topic)],
        ),
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
