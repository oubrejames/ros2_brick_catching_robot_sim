"""Test angular velocity calculate."""

from turtle_brick.turtle_robot import calculate_angular_vel


def test_identity():
    assert calculate_angular_vel(10, 5) == 2
    
    
import unittest
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import pytest
import rclpy

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


@pytest.mark.rostest
def generate_test_description():
    in_out_action = Node(package="turtle_brick",
                         executable="turtle_robot",
                         )
    return (
        LaunchDescription([
            in_out_action,
            launch_testing.actions.ReadyToTest()
            ]),
        # These are extra parameters that get passed to the test functions
        {
            'in_out': in_out_action
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
        
    def test_talker_transmits(self, launch_service, talker, proc_output):     # dont think these args matter   
        msgs_rx = []

        sub = self.node.create_subscription(
            std_msgs.msg.String, # Change msg to twist
            'talker_chatter', # CHange to cmd_vel
            lambda msg: msgs_rx.append(msg), # keep
            10
        )
        try:
            # Wait until the talker transmits two messages over the ROS topic
            end_time = time.time() + 10 # Creates tmie and adds 10 seconds
            # For 10 seconds it spins through that node
            # it listens and calls the local function
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if len(msgs_rx) > 2: #After adding two things the loop breaks
                    break # Dont need
        # ONce loop breaks we have shit in list and the amount of stuff in list corresponds to the time that it ran
        # Do math -> get hz and check below
        self.assertAlmostEqual # use to check if it was running a 100 hz