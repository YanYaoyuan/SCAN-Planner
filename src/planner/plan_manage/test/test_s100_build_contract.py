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
        package_xml = read("src/planner/plan_env/package.xml")
        extras = read(
            "src/planner/plan_env/cmake/plan_env_pcl_components.cmake"
        )

        self.assertIn("CONFIG_EXTRAS cmake/plan_env_pcl_components.cmake", cmake)
        self.assertIn(
            "find_package(PCL REQUIRED COMPONENTS common io filters)", extras
        )
        self.assertNotIn("COMPONENTS visualization", extras)
        self.assertIn("pcl_common pcl_io pcl_filters", cmake)
        self.assertIn("<depend>libopencv-dev</depend>", package_xml)
        self.assertIn("<depend>libpcl-all-dev</depend>", package_xml)

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
        self.assertIn("/data/omni-s100-cache/images", local_script)
        self.assertIn('exec 8> "${DOCKER_BASE}/.build.lock"', local_script)
        self.assertIn("trap cleanup EXIT", local_script)
        self.assertIn("stop_docker_daemon", local_script)
        self.assertIn('2>&1 8>&- 9>&- &', local_script)

    def test_workflow_packages_only_the_core_overlay(self):
        workflow = read(".github/workflows/build-s100.yml")

        self.assertIn("actions/checkout@v7.0.1", workflow)
        self.assertIn("actions/upload-artifact@v7.0.1", workflow)
        self.assertIn("OMNI_ROBOT_INTERFACES_DEPLOY_KEY", workflow)
        self.assertIn("./scripts/build_s100_cross.sh", workflow)
        for artifact in EXCLUDED_ARTIFACTS:
            self.assertIn(artifact, workflow)
        self.assertIn('runtime_root="${S100_CROSS_ROOT}/runtime"', workflow)
        self.assertIn("lib/DEPENDENCIES.txt", workflow)
        self.assertIn("run_product_planner.sh", workflow)
        self.assertNotIn("dist/*.sha256", workflow)

        x86_workflow = read(".github/workflows/ros2-humble-ci.yml")
        self.assertIn("openssh-client", x86_workflow)
        self.assertIn("SCAN_PLANNER_BUILD_OPEN_LOOP_CONTROLLER=OFF", x86_workflow)
        self.assertIn("SCAN_PLANNER_BUILD_SIMULATION_NODES=OFF", x86_workflow)
        self.assertIn(
            "--packages-select path_searching bspline_opt scan_planner",
            x86_workflow,
        )

    def test_s100_runtime_is_release_and_self_contained(self):
        build_script = read("scripts/build_s100_cross.sh")
        packager = read("scripts/package_s100_runtime.sh")

        self.assertIn("-DCMAKE_BUILD_TYPE=Release", build_script)
        self.assertIn("-DBUILD_TESTING=OFF", build_script)
        self.assertIn("-DCMAKE_SKIP_RPATH=ON", build_script)
        self.assertIn("-DCMAKE_SKIP_INSTALL_RPATH=ON", build_script)
        self.assertIn("package_s100_runtime.sh", build_script)
        self.assertIn("runtime/COLCON_IGNORE", build_script)
        self.assertIn("DT_NEEDED", packager)
        self.assertIn("DEPENDENCIES.txt", packager)
        self.assertIn("local/lib/python3.10/dist-packages", packager)
        self.assertIn("workspace_root", packager)
        self.assertIn("cross-build path leaked into runtime metadata", packager)
        self.assertIn("resource_index/parent_prefix_path", packager)
        self.assertIn("runtime output cannot be inside the sysroot", packager)
        self.assertIn("executable is not in the S100 runtime allowlist", packager)
        self.assertIn("is_real_world:=true", packager)
        self.assertIn("require_tf_ready:=true", packager)
        self.assertIn("use_sim_time:=false", packager)
        self.assertLess(packager.index('  "$@"'), packager.index("  is_real_world:=true"))
        self.assertIn("find \"${output_root}/lib\" -maxdepth 1 -type f -name '*.a' -delete", packager)


if __name__ == "__main__":
    unittest.main()
