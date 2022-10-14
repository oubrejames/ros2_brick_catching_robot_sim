import math
import sys
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster

# Modified code from ROS2 static broadcater tutorial source code 
# Accessed 10/9/2022
# https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.html

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class TurtleRobot(Node):
    """
    Broadcast transforms that never change.

    This example publishes transforms from `world` to a static turtle frame.
    The transforms are only published once at startup, and are constant for all
    time.
    """

    def __init__(self):
        """TODO - update comments so not just from ros website

        Args:
            transformation (_type_): _description_
        """
        # Initialize node with name turtle_robot.
        super().__init__('turtle_robot')

        # Publish a static tf upon start up
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.make_transforms()
        
        ### Below from in_out.py ###
        self.dx = 10  # used to control frame movement
        # create the broadcaster
        self.broadcaster = TransformBroadcaster(self)
        # Create a timer to do the rest of the transforms
        self.tmr = self.create_timer(0.004, self.timer_callback)

    def make_transforms(self):
        # "TransformStamped object, which will be the message we will send over once populated" - from tutorial 
        t = TransformStamped()

        # Give the transform being published a timestamp
        t.header.stamp = self.get_clock().now().to_msg()
        # Name parent frame
        t.header.frame_id = 'world'
        # Set child frame
        t.child_frame_id = "odom"

        # TODO update this to initial position of the turtle
        t.transform.translation.x = 1.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        quat = quaternion_from_euler(
            float(0), float(0), float(0))
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        # Broadcast TF
        self.tf_static_broadcaster.sendTransform(t)
        
    def timer_callback(self):
        # Define base_link frame 
        base_link = TransformStamped()
        base_link.header.frame_id = "odom"
        base_link.child_frame_id = "base_link"
        # TODO update this to initial position of the turtle
        base_link.transform.translation.x = 1.0
        base_link.transform.translation.y = 0.0
        base_link.transform.translation.z = 0.65
        quat_base = quaternion_from_euler(float(0), float(0), float(0)) # Roll pitch yaw
        base_link.transform.rotation.x = quat_base[0]
        base_link.transform.rotation.y = quat_base[1]
        base_link.transform.rotation.z = quat_base[2]
        base_link.transform.rotation.w = quat_base[3]      
       
                
        # don't forget to put a timestamp
        time = self.get_clock().now().to_msg()
        base_link.header.stamp = time
        
        # wheel.header.stamp = time
        # stem.header.stamp = time
        # platform.header.stamp = time
        # self.broadcaster.sendTransform(platform)
        # self.broadcaster.sendTransform(wheel)
        # self.broadcaster.sendTransform(stem)
        
        self.broadcaster.sendTransform(base_link)
        

        # update the movement
        self.dx -= 1
        if self.dx == 0:
            self.dx = 10


def main():
    logger = rclpy.logging.get_logger('logger')

    # # obtain parameters from command line arguments
    # if len(sys.argv) != 8:
    #     logger.info('Invalid number of parameters. Usage: \n'
    #                 '$ ros2 run learning_tf2_py static_turtle_tf2_broadcaster'
    #                 'child_frame_name x y z roll pitch yaw')
    #     sys.exit(1)

    # if sys.argv[1] == 'world':
    #     logger.info('Your static turtle name cannot be "world"')
    #     sys.exit(2)

    # pass parameters and initialize node
    rclpy.init()
    node = TurtleRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()