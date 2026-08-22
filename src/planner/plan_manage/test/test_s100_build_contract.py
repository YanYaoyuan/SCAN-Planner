#!/usr/bin/env python3
"""Static regression checks for the production-only RDK S100 build."""

from pathlib import Path
import unittest


REPO_ROOT = Path(__file__).resolve().parents[4]
CORE_PACKAGES = (
    "omni_robot_interfaces",
    "scan_planner_msgs",
    "plan_env",
    "path_searching",
    "bspline_opt",
    "traj_utils",
    "scan_planner",
)
EXCLUDED_ARTIFACTS = (
    "open_loop_controller",
    "go2_kinematic_sim",
    "go2_gait_publisher",
    "go2_description",
    "local_sensing_node",
    "map_generator",
    "mockamap",
    "zsibot_cmd_bridge",
)


def read(relative_path: str) -> str:
    return (REPO_ROOT / relative_path).read_text(encoding="utf-8")


class S100BuildContractTest(unittest.TestCase):
    def test_cross_build_uses_an_explicit_core_allowlist(self):
        script = read("scripts/build_s100_cross.sh")
        package_selection = "--packages-select " + " ".join(CORE_PACKAGES)

        self.assertIn(package_selection, script)
        self.assertNotIn("--packages-up-to", script)
        self.assertIn("SCAN_PLANNER_BUILD_OPEN_LOOP_CONTROLLER=OFF", script)
        self.assertIn("SCAN_PLANNER_BUILD_SIMULATION_NODES=OFF", script)

        for artifact in EXCLUDED_ARTIFACTS:
            self.assertIn(artifact, script)

        package_xml = read("src/planner/plan_manage/package.xml")
        self.assertNotIn("<exec_depend>go2_description</exec_depend>", package_xml)

        launch = read("src/planner/plan_manage/launch/run.launch.py")
        self.assertIn(
            'DeclareLaunchArgument("publish_robot_description", default_value="false")',
            launch,
        )

    def test_cmake_keeps_x86_compatibility_but_supports_core_only_builds(self):
        cmake = read("src/planner/plan_manage/CMakeLists.txt")

        self.assertIn("SCAN_PLANNER_BUILD_OPEN_LOOP_CONTROLLER", cmake)
        self.assertIn("SCAN_PLANNER_BUILD_SIMULATION_NODES", cmake)
        self.assertIn("legacy open-loop controller executable\" ON", cmake)
        self.assertIn("Go2-only kinematic simulation executables\" ON", cmake)
        self.assertIn("scan_planner_runtime_targets", cmake)

    def test_plan_env_exports_only_required_pcl_components(self):
        cmake = read("src/planner/plan_env/CMakeLists.txt")
        extras = read(
            "src/planner/plan_env/cmake/plan_env_pcl_components.cmake"
        )

        self.assertIn("CONFIG_EXTRAS cmake/plan_env_pcl_components.cmake", cmake)
        self.assertIn(
            "find_package(PCL REQUIRED COMPONENTS common io filters)", extras
        )
        self.assertNotIn("COMPONENTS visualization", extras)
        self.assertIn("pcl_common pcl_io pcl_filters", cmake)

    def test_closed_loop_controller_uses_humble_tf2_header(self):
        controller = read(
            "src/planner/plan_manage/src/closed_loop_controller.cpp"
        )

        self.assertIn("#include <tf2/utils.h>", controller)
        self.assertNotIn("#include <tf2/utils.hpp>", controller)

    def test_local_entrypoint_calls_the_shared_cross_build(self):
        local_script = read("scripts/test_s100_local.sh")

        self.assertIn('"${ROOT_DIR}/scripts/build_s100_cross.sh"', local_script)
        self.assertIn("OMNI_ROBOT_INTERFACES_SOURCE", local_script)
        self.assertIn("/data/scan-planner-s100-local", local_script)

    def test_workflow_packages_only_the_core_overlay(self):
        workflow = read(".github/workflows/build-s100.yml")

        self.assertIn("actions/checkout@v7.0.1", workflow)
        self.assertIn("actions/upload-artifact@v7.0.1", workflow)
        self.assertIn("OMNI_ROBOT_INTERFACES_DEPLOY_KEY", workflow)
        self.assertIn("./scripts/build_s100_cross.sh", workflow)
        for artifact in EXCLUDED_ARTIFACTS:
            self.assertIn(artifact, workflow)

        x86_workflow = read(".github/workflows/ros2-humble-ci.yml")
        self.assertIn("SCAN_PLANNER_BUILD_OPEN_LOOP_CONTROLLER=OFF", x86_workflow)
        self.assertIn("SCAN_PLANNER_BUILD_SIMULATION_NODES=OFF", x86_workflow)


if __name__ == "__main__":
    unittest.main()
