"""dog3 mode-3 planner-only entry point with YAML-defined interfaces."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parameters(config_file):
    with open(config_file, encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    parameters = document.get("scan_planner_node", {}).get("ros__parameters", {})
    if not isinstance(parameters, dict):
        raise RuntimeError(
            f"{config_file}: scan_planner_node.ros__parameters must be a mapping"
        )
    return parameters


def _read_bool(parameters, name, default):
    value = parameters.get(name, default)
    if not isinstance(value, bool):
        raise RuntimeError(f"dog3.yaml: parameter '{name}' must be true or false")
    return value


def _read_string(parameters, name, default):
    value = parameters.get(name, default)
    if not isinstance(value, str) or not value.strip():
        raise RuntimeError(f"dog3.yaml: parameter '{name}' must be a non-empty string")
    return value


def _as_bool(value, name):
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise RuntimeError(f"launch argument '{name}' must be true or false")


def _setup(context):
    package_share = get_package_share_directory("scan_planner")
    planner_config = os.path.join(package_share, "config", "planner.yaml")
    dog3_config = os.path.join(package_share, "config", "dog3.yaml")
    rviz_config = os.path.join(package_share, "rviz", "dog3_mode3.rviz")
    parameters = _parameters(dog3_config)

    requested_rviz = _as_bool(
        LaunchConfiguration("use_rviz").perform(context), "use_rviz"
    )
    requested_use_sim_time = _as_bool(
        LaunchConfiguration("use_sim_time").perform(context), "use_sim_time"
    )
    republish_point_cloud_for_rviz = _as_bool(
        LaunchConfiguration("republish_point_cloud_for_rviz").perform(context),
        "republish_point_cloud_for_rviz",
    )
    running_on_dog = (
        os.environ.get("WHEELDOG_PLATFORM") == "dog"
        or os.environ.get("SCAN_PLANNER_IN_DOCKER") == "1"
    )
    runtime_use_sim_time = False if running_on_dog else requested_use_sim_time

    topic_defaults = {
        "topics.body_odom": "/relocalizing/map_frame/odometry",
        "topics.sensor_odom": "/relocalizing/map_frame/odometry",
        "topics.point_cloud": "/lidar_points",
        "topics.depth_image": "/camera/aligned_depth_to_color/image_raw",
        "topics.reference_path": "/map_frame/global_path",
        "topics.manual_goal": "/move_base_simple/goal",
        "topics.execution_frozen": "/planning/go2_execution_frozen",
        "topics.bspline": "/planning/bspline",
        "topics.planner_heartbeat": "/planning/planner_heartbeat",
        "topics.local_path": "/map_frame/local_path",
        "topics.data_display": "/planning/data_display",
        "topics.self_inflation": "/planning/self_inflation",
        "topics.occupancy": "/grid_map/occupancy",
        "topics.occupancy_inflate": "/grid_map/occupancy_3d_ex",
        "topics.sliding_map_bbox": "/grid_map/sliding_map_bbox",
        "topics.unknown": "/grid_map/unknown",
        "topics.depth_cloud": "/grid_map/depth_cloud",
        "topics.sensor_pose_extrinsic": "/grid_map/sensor_pose_extrinsic",
    }
    topics = {
        name: _read_string(parameters, name, default)
        for name, default in topic_defaults.items()
    }

    actions = [
        Node(
            package="scan_planner",
            executable="scan_planner_node",
            name="scan_planner_node",
            output="screen",
            parameters=[
                planner_config,
                dog3_config,
                {"use_sim_time": runtime_use_sim_time},
            ],
            remappings=[
                ("body_pose", topics["topics.body_odom"]),
                ("sensor_pose", topics["topics.sensor_odom"]),
                ("cloud", topics["topics.point_cloud"]),
                ("depth", topics["topics.depth_image"]),
                ("initial_path", topics["topics.reference_path"]),
                ("move_base_simple/goal", topics["topics.manual_goal"]),
                (
                    "planning/go2_execution_frozen",
                    topics["topics.execution_frozen"],
                ),
                ("planning/bspline", topics["topics.bspline"]),
                (
                    "planning/planner_heartbeat",
                    topics["topics.planner_heartbeat"],
                ),
                ("planning/local_path", topics["topics.local_path"]),
                ("planning/data_display", topics["topics.data_display"]),
                ("self_inflation", topics["topics.self_inflation"]),
                ("grid_map/occupancy", topics["topics.occupancy"]),
                (
                    "grid_map/occupancy_inflate",
                    topics["topics.occupancy_inflate"],
                ),
                (
                    "grid_map/sliding_map_bbox",
                    topics["topics.sliding_map_bbox"],
                ),
                ("grid_map/unknown", topics["topics.unknown"]),
                ("grid_map/depth_cloud", topics["topics.depth_cloud"]),
                (
                    "grid_map/sensor_pose_extrinsic",
                    topics["topics.sensor_pose_extrinsic"],
                ),
            ],
        )
    ]

    if (requested_rviz or requested_use_sim_time) and running_on_dog:
        actions.append(
            LogInfo(
                msg=(
                    "RViz and simulated time are disabled on dog3 to keep the "
                    "planner-only deployment headless and on the live clock."
                )
            )
        )
    elif requested_rviz:
        if not os.path.isfile(rviz_config):
            raise RuntimeError(f"RViz config not found: {rviz_config}")
        if republish_point_cloud_for_rviz:
            rviz_cloud_topic = _read_string(
                parameters,
                "visualization.rviz_point_cloud_topic",
                "/lidar_points_rviz",
            )
            rviz_cloud_frame = _read_string(
                parameters,
                "visualization.rviz_point_cloud_frame",
                "lidar_frame",
            )
            actions.append(
                Node(
                    package="scan_planner",
                    executable="pointcloud_frame_alias.py",
                    name="rviz_pointcloud_frame_alias",
                    output="screen",
                    parameters=[
                        {
                            "input_topic": topics["topics.point_cloud"],
                            "output_topic": rviz_cloud_topic,
                            "output_frame": rviz_cloud_frame,
                        }
                    ],
                )
            )
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": runtime_use_sim_time}],
            )
        )
    else:
        actions.append(LogInfo(msg="dog3 planner-only mode: RViz disabled."))

    return actions


def generate_launch_description():
    package_share = get_package_share_directory("scan_planner")
    dog3_config = os.path.join(package_share, "config", "dog3.yaml")
    parameters = _parameters(dog3_config)
    default_rviz = _read_bool(parameters, "visualization.use_rviz", False)
    default_use_sim_time = _read_bool(
        parameters,
        "visualization.use_sim_time",
        _read_bool(parameters, "use_sim_time", False),
    )
    default_alias = _read_bool(
        parameters, "visualization.republish_point_cloud_for_rviz", False
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value=str(default_rviz).lower(),
                description="Start dog3 RViz on a PC (forced false on dog3).",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=str(default_use_sim_time).lower(),
                description="Use /clock for PC rosbag playback (forced false on dog3).",
            ),
            DeclareLaunchArgument(
                "republish_point_cloud_for_rviz",
                default_value=str(default_alias).lower(),
                description="Publish the PC-only lidar frame alias for RViz.",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
