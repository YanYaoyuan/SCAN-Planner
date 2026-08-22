"""Regression checks for Omni node naming and TF ownership boundaries."""

from pathlib import Path
import re


REPO_ROOT = Path(__file__).resolve().parents[4]


def read(relative_path: str) -> str:
    return (REPO_ROOT / relative_path).read_text(encoding="utf-8")


def top_level_yaml_keys(relative_path: str):
    keys = []
    for line in read(relative_path).splitlines():
        if line and not line[0].isspace() and line.endswith(":"):
            keys.append(line[:-1])
    return keys


def test_launch_node_names_use_omni_prefix():
    launch_files = (
        "src/planner/plan_manage/launch/run.launch.py",
        "src/planner/plan_manage/launch/simulator.launch.py",
        "src/planner/plan_manage/launch/rviz.launch.py",
        "src/zsibot_cmd_bridge/launch/zsibot_cmd_bridge.launch.py",
        "src/zsibot_cmd_bridge/launch/zsibot_cmd_udp_client.launch.py",
    )
    for launch_file in launch_files:
        names = re.findall(r"name\s*=\s*['\"]([^'\"]+)['\"]", read(launch_file))
        assert names, f"no explicit node names found in {launch_file}"
        assert all(name.startswith("omni_") for name in names), (
            launch_file,
            names,
        )


def test_cpp_default_node_names_use_omni_prefix():
    source_files = (
        "src/planner/plan_manage/src/scan_planner_node.cpp",
        "src/planner/plan_manage/src/global_path_publisher.cpp",
        "src/planner/plan_manage/src/open_loop_controller.cpp",
        "src/planner/plan_manage/src/closed_loop_controller.cpp",
        "src/planner/plan_manage/src/go2_kinematic_sim.cpp",
        "src/planner/plan_manage/src/go2_gait_publisher.cpp",
        "src/zsibot_cmd_bridge/src/zsibot_cmd_bridge.cpp",
        "src/zsibot_cmd_bridge/src/zsibot_cmd_udp_client.cpp",
    )
    pattern = r'(?:Node\(|make_shared<rclcpp::Node>\()\s*"([^"]+)"'
    for source_file in source_files:
        names = re.findall(pattern, read(source_file))
        assert names, f"no default ROS node name found in {source_file}"
        assert all(name.startswith("omni_") for name in names), (
            source_file,
            names,
        )


def test_parameter_files_match_omni_node_names():
    config_files = (
        "src/planner/plan_manage/config/planner.yaml",
        "src/planner/plan_manage/config/controllers.yaml",
        "src/planner/plan_manage/config/simulator.yaml",
        "src/planner/plan_manage/config/keypoints.example.yaml",
        "src/zsibot_cmd_bridge/config/zsibot_cmd_bridge.yaml",
        "src/zsibot_cmd_bridge/config/zsibot_cmd_udp_client.yaml",
        "src/zsibot_cmd_bridge/config/matrix_sim_bridge.yaml",
    )
    for config_file in config_files:
        keys = top_level_yaml_keys(config_file)
        assert keys, f"no top-level parameter keys found in {config_file}"
        assert all(key.startswith("omni_") for key in keys), (config_file, keys)


def test_legacy_lidar_tf_adapter_is_not_built_or_launched():
    assert not (
        REPO_ROOT / "src/planner/plan_manage/src/lidar_to_body_odom.cpp"
    ).exists()
    assert "lidar_to_body_odom" not in read(
        "src/planner/plan_manage/CMakeLists.txt"
    )
    assert "lidar_to_body_odom" not in read(
        "src/planner/plan_manage/launch/run.launch.py"
    )
