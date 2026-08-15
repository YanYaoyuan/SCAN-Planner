"""Main ROS 2 launch entry point for simulation and real-robot remapping."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return value.lower() in ("1", "true", "yes", "on")


def _set_optional_float(context, overrides, launch_name, param_name):
    value = LaunchConfiguration(launch_name).perform(context)
    if value:
        overrides[param_name] = float(value)


def _setup(context):
    scan_share = get_package_share_directory("scan_planner")
    planner_yaml = os.path.join(scan_share, "config", "planner.yaml")
    controllers_yaml = os.path.join(scan_share, "config", "controllers.yaml")
    is_real = _as_bool(LaunchConfiguration("is_real_world").perform(context))
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    publish_robot_description = _as_bool(
        LaunchConfiguration("publish_robot_description").perform(context)
    )
    sensor_type = LaunchConfiguration("sensor_type").perform(context)
    controller_mode = LaunchConfiguration("controller_mode").perform(context)
    controller_tracking_mode = LaunchConfiguration("controller_tracking_mode").perform(context)
    controller_drive_mode = LaunchConfiguration("controller_drive_mode").perform(context)
    keypoints_file = LaunchConfiguration("keypoints_file").perform(context)
    enable_deprecated_zsibot_transport = _as_bool(
        LaunchConfiguration("enable_deprecated_zsibot_transport").perform(context)
    )
    use_zsibot_bridge = _as_bool(LaunchConfiguration("use_zsibot_bridge").perform(context))
    use_zsibot_udp_client = _as_bool(
        LaunchConfiguration("use_zsibot_udp_client").perform(context)
    )
    zsibot_config_file = LaunchConfiguration("zsibot_config_file").perform(context)
    zsibot_udp_client_config_file = LaunchConfiguration(
        "zsibot_udp_client_config_file"
    ).perform(context)
    use_lidar_to_body_odom = _as_bool(
        LaunchConfiguration("use_lidar_to_body_odom").perform(context)
    )
    lidar_odom_topic = LaunchConfiguration("lidar_odom_topic").perform(context)
    body_odom_topic = LaunchConfiguration("body_odom_topic").perform(context)
    body_odom_frame_id = LaunchConfiguration("body_odom_frame_id").perform(context)
    body_odom_sensor_frame_id = LaunchConfiguration("body_odom_sensor_frame_id").perform(context)
    body_odom_world_frame_id = LaunchConfiguration("body_odom_world_frame_id").perform(context)
    body_odom_publish_tf = _as_bool(LaunchConfiguration("body_odom_publish_tf").perform(context))
    body_odom_transform_twist = _as_bool(
        LaunchConfiguration("body_odom_transform_twist").perform(context)
    )
    body_to_sensor = {
        "body_to_sensor.x": float(LaunchConfiguration("body_to_sensor_x").perform(context)),
        "body_to_sensor.y": float(LaunchConfiguration("body_to_sensor_y").perform(context)),
        "body_to_sensor.z": float(LaunchConfiguration("body_to_sensor_z").perform(context)),
        "body_to_sensor.roll": float(LaunchConfiguration("body_to_sensor_roll").perform(context)),
        "body_to_sensor.pitch": float(LaunchConfiguration("body_to_sensor_pitch").perform(context)),
        "body_to_sensor.yaw": float(LaunchConfiguration("body_to_sensor_yaw").perform(context)),
    }
    goal_frame_id = LaunchConfiguration("goal_frame_id").perform(context)
    goal_transform_timeout = float(LaunchConfiguration("goal_transform_timeout").perform(context))
    use_global_path_publisher = _as_bool(
        LaunchConfiguration("use_global_path_publisher").perform(context)
    )
    global_path_topic = LaunchConfiguration("global_path_topic").perform(context)
    global_path_spacing = float(LaunchConfiguration("global_path_spacing").perform(context))
    navi_mode = int(LaunchConfiguration("navi_mode").perform(context))
    if sensor_type not in ("lidar", "depth"):
        raise RuntimeError("sensor_type must be 'lidar' or 'depth'")
    if controller_mode not in ("open_loop", "closed_loop"):
        raise RuntimeError("controller_mode must be 'open_loop' or 'closed_loop'")
    if navi_mode not in (1, 2, 3):
        raise RuntimeError("navi_mode must be 1, 2, or 3")
    if navi_mode == 2 and (not keypoints_file or not os.path.isfile(keypoints_file)):
        raise RuntimeError(
            "navi_mode=2 requires keypoints_file to reference a ROS 2 parameter YAML"
        )
    if use_zsibot_bridge and use_zsibot_udp_client:
        raise RuntimeError("use_zsibot_bridge and use_zsibot_udp_client cannot both be true")
    if (use_zsibot_bridge or use_zsibot_udp_client) and not enable_deprecated_zsibot_transport:
        raise RuntimeError(
            "zsibot_cmd_bridge is deprecated and is no longer part of the default "
            "SCAN-Planner control path. Use the unified robot bridge. For temporary "
            "compatibility only, also set enable_deprecated_zsibot_transport:=true."
        )

    if is_real:
        body_pose = LaunchConfiguration("real_body_pose_topic").perform(context)
        sensor_pose = LaunchConfiguration("real_sensor_pose_topic").perform(context)
        cloud = LaunchConfiguration("real_cloud_topic").perform(context)
        depth = LaunchConfiguration("real_depth_topic").perform(context)
        cmd_vel = LaunchConfiguration("real_cmd_vel_topic").perform(context)
        goal = LaunchConfiguration("goal_topic").perform(context)
        initial_path = LaunchConfiguration("initial_path_topic").perform(context)
        grid_frame_id = LaunchConfiguration("real_grid_frame_id").perform(context)
        cloud_is_world = _as_bool(LaunchConfiguration("real_cloud_is_world").perform(context))
        need_extrinsic = _as_bool(LaunchConfiguration("real_need_extrinsic").perform(context))
        grid_sensor_frame_id = body_odom_sensor_frame_id
        intrinsics = {
            "grid_map.cx": 317.19183349609375,
            "grid_map.cy": 256.4806823730469,
            "grid_map.fx": 609.5884399414062,
            "grid_map.fy": 609.22021484375,
        }
        if use_lidar_to_body_odom:
            body_pose = body_odom_topic
    else:
        body_pose = "/quad_0/body_pose"
        sensor_pose = "/quad_0/camera_pose" if sensor_type == "depth" else "/quad_0/lidar_pose"
        cloud = "/quad_0/cloud"
        depth = "/quad_0/depth"
        cmd_vel = "/quad_0/cmd_vel"
        goal = "/move_base_simple/goal"
        initial_path = "/initial_path"
        grid_frame_id = "world"
        cloud_is_world = True
        need_extrinsic = False
        grid_sensor_frame_id = ""
        intrinsics = {}

    if not goal_frame_id:
        goal_frame_id = grid_frame_id

    common = {"use_sim_time": use_sim_time}
    closed_loop_overrides = dict(common)
    closed_loop_overrides["expected_frame_id"] = grid_frame_id
    if controller_tracking_mode:
        closed_loop_overrides["tracking_mode"] = controller_tracking_mode
    if controller_drive_mode:
        closed_loop_overrides["drive_mode"] = controller_drive_mode
    for launch_name, param_name in (
        ("controller_max_vx", "max_vx"),
        ("controller_max_vy", "max_vy"),
        ("controller_max_vyaw", "max_vyaw"),
        ("controller_lookahead_dist", "lookahead_dist"),
        ("controller_yaw_lookahead_dist", "yaw_lookahead_dist"),
        ("controller_pure_pursuit_speed", "pure_pursuit_speed"),
        ("controller_heading_error_threshold", "heading_error_threshold"),
        ("controller_heading_error_exit_threshold", "heading_error_exit_threshold"),
        ("controller_heading_slowdown_start", "heading_slowdown_start"),
        ("controller_align_heading_gain", "align_heading_gain"),
        ("controller_pp_heading_gain", "pp_heading_gain"),
        ("controller_lateral_error_deadband", "lateral_error_deadband"),
        ("controller_heading_error_deadband", "heading_error_deadband"),
        ("controller_curvature_deadband", "curvature_deadband"),
        ("controller_curvature_speed_gain", "curvature_speed_gain"),
        ("controller_yaw_rate_reserve", "yaw_rate_reserve"),
        ("controller_max_lateral_acceleration", "max_lateral_acceleration"),
        ("controller_max_linear_acceleration", "max_linear_acceleration"),
        ("controller_max_linear_deceleration", "max_linear_deceleration"),
        ("controller_yaw_rate_deadband", "yaw_rate_deadband"),
        ("controller_yaw_filter_time_constant", "yaw_filter_time_constant"),
        ("controller_max_yaw_acceleration", "max_yaw_acceleration"),
        ("controller_yaw_reversal_threshold", "yaw_reversal_threshold"),
        ("controller_yaw_start_threshold", "yaw_start_threshold"),
        ("controller_yaw_stop_threshold", "yaw_stop_threshold"),
        ("controller_min_nonzero_yaw_rate", "min_nonzero_yaw_rate"),
        ("controller_min_nonzero_linear_speed", "min_nonzero_linear_speed"),
        ("controller_yaw_sync_threshold", "yaw_sync_threshold"),
        ("controller_launch_yaw_readiness_ratio", "launch_yaw_readiness_ratio"),
        ("controller_planner_heartbeat_timeout", "planner_heartbeat_timeout"),
        ("controller_control_rate", "control_rate"),
    ):
        _set_optional_float(context, closed_loop_overrides, launch_name, param_name)
    planner_overrides = {
        **common,
        **intrinsics,
        "fsm.navi_mode": navi_mode,
        "fsm.expected_odom_frame": grid_frame_id,
        "grid_map.frame_id": grid_frame_id,
        "grid_map.sensor_type": sensor_type,
        "grid_map.sensor_frame_id": grid_sensor_frame_id,
        "grid_map.cloud_is_world": cloud_is_world,
        "grid_map.need_extrinsic": need_extrinsic,
        "grid_map.vis_height": float(
            LaunchConfiguration("grid_map_vis_height").perform(context)
        ),
        "fsm.goal_frame_id": goal_frame_id,
        "fsm.goal_transform_timeout": goal_transform_timeout,
    }
    # FSM 的恢复碰撞包络必须与 closed-loop controller 使用同一组几何和
    # 底盘死区。用户通过 controller_* 调真机参数时同步覆盖 FSM，避免只改
    # 控制输出而安全预测仍沿用 YAML 旧值；YAML 仅作为无 launch override
    # 时的默认兜底。
    for launch_name, param_name in (
        ("controller_lookahead_dist", "fsm.local_recovery_lookahead"),
        ("controller_yaw_lookahead_dist", "fsm.local_recovery_yaw_lookahead"),
        ("controller_heading_error_threshold", "fsm.local_recovery_align_enter_error"),
        ("controller_heading_error_exit_threshold", "fsm.local_recovery_align_exit_error"),
        ("controller_align_heading_gain", "fsm.local_recovery_align_yaw_gain"),
        ("controller_pp_heading_gain", "fsm.local_recovery_heading_feedback_gain"),
        ("controller_lateral_error_deadband", "fsm.local_recovery_lateral_error_deadband"),
        ("controller_heading_error_deadband", "fsm.local_recovery_heading_error_deadband"),
        ("controller_curvature_deadband", "fsm.local_recovery_curvature_deadband"),
        ("controller_yaw_rate_deadband", "fsm.local_recovery_yaw_deadband"),
        ("controller_yaw_start_threshold", "fsm.local_recovery_yaw_start_threshold"),
        ("controller_min_nonzero_yaw_rate", "fsm.local_recovery_min_yaw_rate"),
        ("controller_min_nonzero_linear_speed", "fsm.local_recovery_min_forward_speed"),
        ("controller_max_vyaw", "fsm.local_recovery_max_yaw_rate"),
    ):
        _set_optional_float(context, planner_overrides, launch_name, param_name)
    for launch_name, param_name in (
        ("fsm_emergency_time", "fsm.emergency_time"),
        ("fsm_local_progress_collision_backtrack", "fsm.local_progress_collision_backtrack"),
        ("fsm_local_progress_max_cross_track", "fsm.local_progress_max_cross_track"),
        ("fsm_local_recovery_check_min_cross_track", "fsm.local_recovery_check_min_cross_track"),
        ("fsm_local_recovery_lookahead", "fsm.local_recovery_lookahead"),
        ("fsm_local_recovery_sample_distance", "fsm.local_recovery_sample_distance"),
        ("fsm_local_finish_speed", "fsm.local_finish_speed"),
        ("manager_max_vel", "manager.max_vel"),
        ("manager_max_acc", "manager.max_acc"),
    ):
        _set_optional_float(context, planner_overrides, launch_name, param_name)
    actions = []

    if use_zsibot_bridge or use_zsibot_udp_client:
        actions.append(
            LogInfo(
                msg=(
                    "[DEPRECATED] Starting a legacy zsibot transport from SCAN-Planner. "
                    "Only one process may own or forward commands to the vendor SDK."
                )
            )
        )

    if is_real and use_global_path_publisher:
        if navi_mode != 3:
            raise RuntimeError(
                "use_global_path_publisher=true requires navi_mode=3"
            )
        actions.append(
            Node(
                package="scan_planner",
                executable="global_path_publisher",
                name="global_path_publisher",
                output="screen",
                parameters=[
                    common,
                    {
                        "frame_id": grid_frame_id,
                        "body_frame_id": body_odom_frame_id,
                        "path_spacing": global_path_spacing,
                        "goal_transform_timeout": goal_transform_timeout,
                    },
                ],
                remappings=[
                    ("body_pose", body_pose),
                    ("goal", goal),
                    ("global_path", global_path_topic),
                ],
            )
        )
        initial_path = global_path_topic

    if is_real and use_lidar_to_body_odom:
        actions.append(
            Node(
                package="scan_planner",
                executable="lidar_to_body_odom",
                name="lidar_to_body_odom",
                output="screen",
                parameters=[
                    common,
                    body_to_sensor,
                    {
                        "body_frame_id": body_odom_frame_id,
                        "sensor_frame_id": body_odom_sensor_frame_id,
                        "world_frame_id": body_odom_world_frame_id,
                        "publish_tf": body_odom_publish_tf,
                        "transform_twist": body_odom_transform_twist,
                    },
                ],
                remappings=[
                    ("sensor_odom", lidar_odom_topic),
                    ("body_odom", body_odom_topic),
                ],
            )
        )

    actions.append(
        Node(
            package="scan_planner",
            executable="scan_planner_node",
            name="scan_planner_node",
            output="screen",
            parameters=[planner_yaml] + ([keypoints_file] if keypoints_file else []) + [planner_overrides],
            remappings=[
                ("body_pose", body_pose),
                ("sensor_pose", sensor_pose),
                ("cloud", cloud),
                ("depth", depth),
                ("move_base_simple/goal", goal),
                ("initial_path", initial_path),
            ],
        )
    )
    if publish_robot_description:
        go2_share = get_package_share_directory("go2_description")
        actions.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="go2_robot_state_publisher",
                output="screen",
                parameters=[
                    common,
                    {
                        "robot_description": Command(
                            ["xacro ", os.path.join(go2_share, "xacro", "robot.xacro"),
                             " use_gazebo:=false"]
                        )
                    },
                ],
            )
        )

    if controller_mode == "open_loop":
        actions.append(
            Node(
                package="scan_planner",
                executable="open_loop_controller",
                name="open_loop_controller",
                output="screen",
                parameters=[controllers_yaml, common],
                remappings=[
                    ("planning/bspline", "/planning/bspline"),
                    ("body_pose", body_pose),
                ],
            )
        )
    else:
        actions.append(
            Node(
                package="scan_planner",
                executable="closed_loop_controller",
                name="closed_loop_controller",
                output="screen",
                parameters=[controllers_yaml, closed_loop_overrides],
                remappings=[
                    ("body_pose", body_pose),
                    ("cmd_vel", cmd_vel),
                ],
            )
        )
        if is_real and use_zsibot_bridge:
            if not zsibot_config_file:
                zsibot_share = get_package_share_directory("zsibot_cmd_bridge")
                zsibot_config_file = os.path.join(
                    zsibot_share, "config", "zsibot_cmd_bridge.yaml"
                )
            actions.append(
                Node(
                    package="zsibot_cmd_bridge",
                    executable="zsibot_cmd_bridge",
                    name="zsibot_cmd_bridge",
                    output="screen",
                    parameters=[
                        zsibot_config_file,
                        common,
                        {"enable_deprecated_transport": True},
                    ],
                    remappings=[("cmd_vel", cmd_vel)],
                )
            )
        if is_real and use_zsibot_udp_client:
            if not zsibot_udp_client_config_file:
                zsibot_share = get_package_share_directory("zsibot_cmd_bridge")
                zsibot_udp_client_config_file = os.path.join(
                    zsibot_share, "config", "zsibot_cmd_udp_client.yaml"
                )
            actions.append(
                Node(
                    package="zsibot_cmd_bridge",
                    executable="zsibot_cmd_udp_client",
                    name="zsibot_cmd_udp_client",
                    output="screen",
                    parameters=[
                        zsibot_udp_client_config_file,
                        common,
                        {"enable_deprecated_transport": True},
                    ],
                    remappings=[("cmd_vel", cmd_vel)],
                )
            )
        if not is_real:
            actions.append(
                Node(
                    package="scan_planner",
                    executable="go2_kinematic_sim",
                    name="go2_kinematic_sim",
                    output="screen",
                    parameters=[
                        controllers_yaml,
                        common,
                        {
                            "init_x": float(LaunchConfiguration("init_x").perform(context)),
                            "init_y": float(LaunchConfiguration("init_y").perform(context)),
                            "init_z": float(LaunchConfiguration("init_z").perform(context)),
                            "publish_tf": False,
                        },
                    ],
                    remappings=[
                        ("body_pose", "/quad_0/body_pose"),
                        ("cmd_vel", "/quad_0/cmd_vel"),
                    ],
                )
            )

    if not is_real:
        actions.extend(
            [
                Node(
                    package="scan_planner",
                    executable="go2_gait_publisher",
                    name="go2_gait_publisher",
                    output="screen",
                    parameters=[controllers_yaml, common],
                    remappings=[("body_pose", body_pose)],
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(scan_share, "launch", "simulator.launch.py")
                    ),
                    launch_arguments={
                        name: LaunchConfiguration(name)
                        for name in (
                            "is_real_world",
                            "sensor_type",
                            "use_gpu",
                            "use_pcd_map",
                            "pcd_map_file",
                            "map_size_x",
                            "map_size_y",
                            "map_size_z",
                            "use_sim_time",
                        )
                    }.items(),
                ),
            ]
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("is_real_world", default_value="false"),
            DeclareLaunchArgument("navi_mode", default_value="1"),
            DeclareLaunchArgument("sensor_type", default_value="lidar"),
            DeclareLaunchArgument("controller_mode", default_value="closed_loop"),
            DeclareLaunchArgument("controller_tracking_mode", default_value=""),
            DeclareLaunchArgument("controller_drive_mode", default_value=""),
            DeclareLaunchArgument("controller_max_vx", default_value=""),
            DeclareLaunchArgument("controller_max_vy", default_value=""),
            DeclareLaunchArgument("controller_max_vyaw", default_value=""),
            DeclareLaunchArgument("controller_lookahead_dist", default_value=""),
            DeclareLaunchArgument("controller_yaw_lookahead_dist", default_value=""),
            DeclareLaunchArgument("controller_pure_pursuit_speed", default_value=""),
            DeclareLaunchArgument("controller_heading_error_threshold", default_value=""),
            DeclareLaunchArgument("controller_heading_error_exit_threshold", default_value=""),
            DeclareLaunchArgument("controller_heading_slowdown_start", default_value=""),
            DeclareLaunchArgument("controller_align_heading_gain", default_value=""),
            DeclareLaunchArgument("controller_pp_heading_gain", default_value=""),
            DeclareLaunchArgument("controller_lateral_error_deadband", default_value=""),
            DeclareLaunchArgument("controller_heading_error_deadband", default_value=""),
            DeclareLaunchArgument("controller_curvature_deadband", default_value=""),
            DeclareLaunchArgument("controller_curvature_speed_gain", default_value=""),
            DeclareLaunchArgument("controller_yaw_rate_reserve", default_value=""),
            DeclareLaunchArgument("controller_max_lateral_acceleration", default_value=""),
            DeclareLaunchArgument("controller_max_linear_acceleration", default_value=""),
            DeclareLaunchArgument("controller_max_linear_deceleration", default_value=""),
            DeclareLaunchArgument("controller_yaw_rate_deadband", default_value=""),
            DeclareLaunchArgument("controller_yaw_filter_time_constant", default_value=""),
            DeclareLaunchArgument("controller_max_yaw_acceleration", default_value=""),
            DeclareLaunchArgument("controller_yaw_reversal_threshold", default_value=""),
            DeclareLaunchArgument("controller_yaw_start_threshold", default_value=""),
            DeclareLaunchArgument("controller_yaw_stop_threshold", default_value=""),
            DeclareLaunchArgument("controller_min_nonzero_yaw_rate", default_value=""),
            DeclareLaunchArgument("controller_min_nonzero_linear_speed", default_value=""),
            DeclareLaunchArgument("controller_yaw_sync_threshold", default_value=""),
            DeclareLaunchArgument("controller_launch_yaw_readiness_ratio", default_value=""),
            DeclareLaunchArgument("controller_planner_heartbeat_timeout", default_value=""),
            DeclareLaunchArgument("controller_control_rate", default_value=""),
            DeclareLaunchArgument("fsm_emergency_time", default_value=""),
            DeclareLaunchArgument("fsm_local_progress_collision_backtrack", default_value=""),
            DeclareLaunchArgument("fsm_local_progress_max_cross_track", default_value=""),
            DeclareLaunchArgument("fsm_local_recovery_check_min_cross_track", default_value=""),
            DeclareLaunchArgument("fsm_local_recovery_lookahead", default_value=""),
            DeclareLaunchArgument("fsm_local_recovery_sample_distance", default_value=""),
            DeclareLaunchArgument("fsm_local_finish_speed", default_value=""),
            DeclareLaunchArgument("manager_max_vel", default_value=""),
            DeclareLaunchArgument("manager_max_acc", default_value=""),
            DeclareLaunchArgument("keypoints_file", default_value=""),
            DeclareLaunchArgument("use_gpu", default_value="false"),
            DeclareLaunchArgument("use_pcd_map", default_value="false"),
            DeclareLaunchArgument("pcd_map_file", default_value=""),
            DeclareLaunchArgument("publish_robot_description", default_value="true"),
            DeclareLaunchArgument(
                "enable_deprecated_zsibot_transport", default_value="false"
            ),
            DeclareLaunchArgument("use_zsibot_bridge", default_value="false"),
            DeclareLaunchArgument("use_zsibot_udp_client", default_value="false"),
            DeclareLaunchArgument("zsibot_config_file", default_value=""),
            DeclareLaunchArgument("zsibot_udp_client_config_file", default_value=""),
            DeclareLaunchArgument("real_body_pose_topic", default_value="/state_estimation_global"),
            DeclareLaunchArgument("real_sensor_pose_topic", default_value="/state_estimation_global"),
            DeclareLaunchArgument("real_cloud_topic", default_value="/cloud_registered_global"),
            DeclareLaunchArgument("real_depth_topic", default_value="/camera/aligned_depth_to_color/image_raw"),
            DeclareLaunchArgument("real_cmd_vel_topic", default_value="/scan_planner/cmd_vel"),
            DeclareLaunchArgument("real_grid_frame_id", default_value="lio_map"),
            DeclareLaunchArgument("real_cloud_is_world", default_value="true"),
            DeclareLaunchArgument("real_need_extrinsic", default_value="false"),
            DeclareLaunchArgument("use_lidar_to_body_odom", default_value="false"),
            DeclareLaunchArgument("lidar_odom_topic", default_value="/state_estimation_global"),
            DeclareLaunchArgument("body_odom_topic", default_value="/body_state_estimation_global"),
            DeclareLaunchArgument("body_odom_frame_id", default_value="scan_base_link"),
            DeclareLaunchArgument("body_odom_sensor_frame_id", default_value="livox_frame"),
            DeclareLaunchArgument("body_odom_world_frame_id", default_value="lio_map"),
            DeclareLaunchArgument("body_odom_publish_tf", default_value="false"),
            DeclareLaunchArgument("body_odom_transform_twist", default_value="true"),
            DeclareLaunchArgument("body_to_sensor_x", default_value="0.0"),
            DeclareLaunchArgument("body_to_sensor_y", default_value="0.0"),
            DeclareLaunchArgument("body_to_sensor_z", default_value="0.0"),
            DeclareLaunchArgument("body_to_sensor_roll", default_value="0.0"),
            DeclareLaunchArgument("body_to_sensor_pitch", default_value="0.0"),
            DeclareLaunchArgument("body_to_sensor_yaw", default_value="0.0"),
            DeclareLaunchArgument("goal_frame_id", default_value=""),
            DeclareLaunchArgument("goal_transform_timeout", default_value="0.2"),
            DeclareLaunchArgument("goal_topic", default_value="/move_base_simple/goal"),
            DeclareLaunchArgument("initial_path_topic", default_value="/initial_path"),
            DeclareLaunchArgument("use_global_path_publisher", default_value="false"),
            DeclareLaunchArgument("global_path_topic", default_value="/planning/global_path"),
            DeclareLaunchArgument("global_path_spacing", default_value="0.25"),
            DeclareLaunchArgument("map_size_x", default_value="40.0"),
            DeclareLaunchArgument("map_size_y", default_value="40.0"),
            DeclareLaunchArgument("map_size_z", default_value="5.0"),
            DeclareLaunchArgument("init_x", default_value="-19.0"),
            DeclareLaunchArgument("init_y", default_value="1.0"),
            DeclareLaunchArgument("init_z", default_value="0.3"),
            DeclareLaunchArgument("grid_map_vis_height", default_value="0.3"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_setup),
        ]
    )
