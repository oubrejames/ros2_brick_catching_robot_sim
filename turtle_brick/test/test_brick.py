"""Test that launch file goes at 100 hz."""

# Code based on link below and Matt's in_out launch test
# https://github.com/ros2/launch_ros/blob/humble/launch_testing_ros/test/examples/talker_listener_launch_test.py
# Accessed 10/19/2022

from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import time
import unittest
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import Twist


@pytest.mark.rostest
def generate_test_description():
    turtle_robot_action = Node(package="turtle_brick", executable="turtle_robot")
    return (
        LaunchDescription([
            turtle_robot_action,
            launch_testing.actions.ReadyToTest()
            ]),
        # These are extra parameters that get passed to the test functions
        {
            'turtle_robot': turtle_robot_action
        }
    )


class TestME495Tf(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_node")

    def tearDown(self):
        self.node.destroy_node()

    def test_hz(self):
        msgs_rx = []

        sub = self.node.create_subscription(
            Twist,
            'turtle1/cmd_vel',
            lambda msg: msgs_rx.append(msg),
            10
        )
        try:
            # Wait until the talker transmits two messages over the ROS topic
            end_time = time.time() + 5  # Creates time and adds 10 seconds
            # For 10 seconds it spins through that node
            # it listens and calls the local function
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)

        finally:
            self.node.destroy_subscription(sub)

        self.assertAlmostEqual(5/len(msgs_rx), 0.01, 2)  # use to check if it was running a 100 hz
