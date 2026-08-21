"""Smoke-test that the migrated planner accepts its ROS 2 parameter file."""

import os
import time
import unittest

import launch
from launch.events.process import ProcessExited
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest


@pytest.mark.launch_test
def generate_test_description():
    config = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "config", "planner.yaml")
    )
    planner = launch_ros.actions.Node(
        package="scan_planner",
        executable="scan_planner_node",
        name="omni_scan_planner",
        parameters=[config],
        output="screen",
    )
    shutdown = launch.actions.TimerAction(
        period=2.0,
        actions=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
    )
    return (
        launch.LaunchDescription(
            [planner, launch_testing.actions.ReadyToTest(), shutdown]
        ),
        {"planner": planner},
    )


class TestPlannerStartup(unittest.TestCase):
    def test_process_starts(self, proc_info, planner):
        proc_info.assertWaitForStartup(process=planner, timeout=15)
        time.sleep(0.5)
        self.assertNotIsInstance(proc_info[planner], ProcessExited)
