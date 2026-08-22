"""Verify that the flat ROS 2 waypoint array is accepted at startup."""

import os
import time
import unittest

import launch
from launch.events.process import ProcessExited
import launch_ros.actions
import launch_testing.actions
import pytest


@pytest.mark.launch_test
def generate_test_description():
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    planner = launch_ros.actions.Node(
        package="scan_planner",
        executable="scan_planner_node",
        name="omni_scan_planner",
        parameters=[
            os.path.join(root, "config", "planner.yaml"),
            os.path.join(root, "config", "keypoints.example.yaml"),
        ],
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


class TestWaypointParameters(unittest.TestCase):
    def test_process_starts(self, proc_info, planner):
        proc_info.assertWaitForStartup(process=planner, timeout=15)
        time.sleep(0.5)
        self.assertNotIsInstance(proc_info[planner], ProcessExited)
