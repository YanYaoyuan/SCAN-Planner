#!/usr/bin/env python3
"""Static regression checks for the deprecated ZsiBot transport boundary."""

from pathlib import Path
import os
import re
import subprocess
import tempfile
import unittest


REPO_ROOT = Path(__file__).resolve().parents[4]


def read(relative_path: str) -> str:
    return (REPO_ROOT / relative_path).read_text(encoding="utf-8")


class LegacyZsiBotOptInTest(unittest.TestCase):
    def test_main_launch_defaults_to_no_legacy_transport(self):
        launch = read("src/planner/plan_manage/launch/run.launch.py")
        for argument in (
            "enable_deprecated_zsibot_transport",
            "use_zsibot_bridge",
            "use_zsibot_udp_client",
        ):
            pattern = rf'DeclareLaunchArgument\(\s*"{argument}", default_value="false"'
            self.assertRegex(launch, pattern)
        self.assertIn(
            "(use_zsibot_bridge or use_zsibot_udp_client) "
            "and not enable_deprecated_zsibot_transport",
            launch,
        )

    def test_planner_has_no_default_runtime_dependency_on_legacy_bridge(self):
        package_xml = read("src/planner/plan_manage/package.xml")
        self.assertNotIn("<exec_depend>zsibot_cmd_bridge</exec_depend>", package_xml)

        legacy_cmake = read("src/zsibot_cmd_bridge/CMakeLists.txt")
        self.assertRegex(
            legacy_cmake,
            r"option\(\s*ZSIBOT_ENABLE_DEPRECATED_SDK_TARGETS\s+"
            r'"[^"]+"\s+OFF\)',
        )
        self.assertIn("if(ZSIBOT_ENABLE_DEPRECATED_SDK_TARGETS)", legacy_cmake)

    def test_legacy_launch_files_are_disabled_by_default(self):
        for relative_path in (
            "src/zsibot_cmd_bridge/launch/zsibot_cmd_bridge.launch.py",
            "src/zsibot_cmd_bridge/launch/zsibot_cmd_udp_client.launch.py",
        ):
            launch = read(relative_path)
            self.assertRegex(
                launch,
                r'DeclareLaunchArgument\(\s*'
                r'"enable_deprecated_zsibot_transport", default_value="false"',
            )
            self.assertIn("disabled by default", launch)
            self.assertIn('{"enable_deprecated_transport": True}', launch)

    def test_legacy_binaries_have_their_own_opt_in_guard(self):
        for relative_path in (
            "src/zsibot_cmd_bridge/src/zsibot_cmd_bridge.cpp",
            "src/zsibot_cmd_bridge/src/zsibot_cmd_udp_client.cpp",
        ):
            source = read(relative_path)
            self.assertIn(
                'declare_parameter<bool>("enable_deprecated_transport", false)',
                source,
            )

        proxy = read("src/zsibot_cmd_bridge/src/zsibot_sdk_proxy.cpp")
        self.assertIn("bool enable_deprecated_transport{false}", proxy)
        self.assertIn('arg == "--enable-deprecated-transport"', proxy)
        self.assertIn("if (!options.enable_deprecated_transport)", proxy)

        direct = read("src/zsibot_cmd_bridge/src/zsibot_cmd_bridge.cpp")
        for source in (direct, proxy):
            self.assertIn("SdkOwnerLock", source)
        owner_lock = read(
            "src/zsibot_cmd_bridge/include/zsibot_cmd_bridge/sdk_owner_lock.hpp"
        )
        self.assertIn("LOCK_EX | LOCK_NB", owner_lock)
        self.assertIn("OMNI_ZSIBOT_SDK_OWNER_LOCK", owner_lock)
        self.assertIn("/run/lock/omni/zsibot_sdk_owner.lock", owner_lock)
        self.assertIn("path.front() != '/'", owner_lock)
        self.assertIn("O_NOFOLLOW", owner_lock)
        self.assertIn("::fstat", owner_lock)
        self.assertIn("status.st_nlink != 1", owner_lock)

    def test_default_scripts_do_not_enable_legacy_bridge(self):
        matrix = read("run_matrix_planner.sh")
        self.assertIn('CONTROL_MODE="${CONTROL_MODE:-none}"', matrix)

        real = read("tools/orin_runtime/run_real_planner.sh")
        self.assertIn(
            'ENABLE_DEPRECATED_ZSIBOT_TRANSPORT="${ENABLE_DEPRECATED_ZSIBOT_TRANSPORT:-0}"',
            real,
        )
        self.assertNotIn("use_zsibot_bridge:=true", real)

        cross_build = read("tools/cross/build_orin_nx.sh")
        self.assertIn(
            'BUILD_LEGACY_ZSIBOT_UDP_CLIENT="${BUILD_LEGACY_ZSIBOT_UDP_CLIENT:-0}"',
            cross_build,
        )
        self.assertIn(
            'BUILD_DEPRECATED_ZSIBOT_SDK_TARGETS="${BUILD_DEPRECATED_ZSIBOT_SDK_TARGETS:-0}"',
            cross_build,
        )
        self.assertIn("PACKAGES_UP_TO=(scan_planner)", cross_build)
        self.assertIn(
            '"-DZSIBOT_ENABLE_DEPRECATED_SDK_TARGETS=${BUILD_DEPRECATED_ZSIBOT_SDK_TARGETS}"',
            cross_build,
        )
        self.assertIn("assert_no_legacy_zsibot_owner.sh", cross_build)
        self.assertIn("--artifacts-only", cross_build)

        real = read("tools/orin_runtime/run_real_planner.sh")
        self.assertIn('"${SCRIPT_DIR}/assert_no_legacy_zsibot_owner.sh"', real)

    def test_stale_installed_sdk_owner_fails_closed(self):
        guard = REPO_ROOT / "tools/orin_runtime/assert_no_legacy_zsibot_owner.sh"
        with tempfile.TemporaryDirectory() as temp_dir:
            prefix = Path(temp_dir)
            fake_bin = prefix / "test-bin"
            fake_bin.mkdir()
            fake_pgrep = fake_bin / "pgrep"
            fake_pgrep.write_text("#!/usr/bin/env bash\nexit 1\n", encoding="utf-8")
            fake_pgrep.chmod(0o755)
            env = {**os.environ, "PATH": f"{fake_bin}:{os.environ.get('PATH', '')}"}
            clean = subprocess.run(
                ["bash", str(guard), str(prefix)],
                check=False,
                capture_output=True,
                text=True,
                env=env,
            )
            self.assertEqual(clean.returncode, 0, clean.stderr)

            stale = prefix / "lib/zsibot_cmd_bridge/zsibot_cmd_bridge"
            stale.parent.mkdir(parents=True)
            stale.touch()
            blocked = subprocess.run(
                ["bash", str(guard), str(prefix)],
                check=False,
                capture_output=True,
                text=True,
                env=env,
            )
            self.assertEqual(blocked.returncode, 3)
            self.assertIn("stale deprecated SDK-owner binary", blocked.stderr)

            fake_pgrep.write_text(
                "#!/usr/bin/env bash\necho process-list-denied >&2\nexit 2\n",
                encoding="utf-8",
            )
            artifact_prefix = prefix / "clean-artifact-prefix"
            artifact_prefix.mkdir()
            artifacts_only = subprocess.run(
                ["bash", str(guard), "--artifacts-only", str(artifact_prefix)],
                check=False,
                capture_output=True,
                text=True,
                env={**env, "AMENT_PREFIX_PATH": str(prefix)},
            )
            self.assertEqual(artifacts_only.returncode, 0, artifacts_only.stderr)

            enumeration_failed = subprocess.run(
                ["bash", str(guard), str(prefix)],
                check=False,
                capture_output=True,
                text=True,
                env=env,
            )
            self.assertEqual(enumeration_failed.returncode, 5)
            self.assertIn("process enumeration failed", enumeration_failed.stderr)

    def test_explicit_legacy_runtime_wrappers_require_confirmation(self):
        wrappers = (
            "tools/orin_runtime/run_bridge_only.sh",
            "tools/orin_runtime/run_cmd_udp_client_only.sh",
            "tools/orin_runtime/run_real_planner_udp.sh",
            "tools/rk_runtime/run_zsibot_sdk_proxy.sh",
        )
        for relative_path in wrappers:
            script = read(relative_path)
            self.assertIn("ENABLE_DEPRECATED_ZSIBOT_TRANSPORT", script)
            self.assertRegex(
                script,
                re.compile(
                    r'ENABLE_DEPRECATED_ZSIBOT_TRANSPORT:-0[^\n]*!= "1"'
                ),
            )


if __name__ == "__main__":
    unittest.main()
