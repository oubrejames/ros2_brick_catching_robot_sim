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
        self.tmr = self.create_timer(1, self.timer_callback)

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
        base_link.transform.translation.z = 0.0
        quat_base = quaternion_from_euler(float(0), float(0), float(0)) # Roll pitch yaw
        base_link.transform.rotation.x = quat_base[0]
        base_link.transform.rotation.y = quat_base[1]
        base_link.transform.rotation.z = quat_base[2]
        base_link.transform.rotation.w = quat_base[3]


        # # # Define brick frame 
        brick = TransformStamped()
        brick.header.frame_id = "world"
        brick.child_frame_id = "brick"
        # # # TODO update this to iwhatever the brick needs to be, starting it off as just 6 m above world frame
        brick.transform.translation.x = 0.0
        brick.transform.translation.y = 0.0
        brick.transform.translation.z = 2.0
        quat_brick = quaternion_from_euler(float(0), float(0), float(0.0))
        brick.transform.rotation.x = quat_brick[0]
        brick.transform.rotation.y = quat_brick[1]
        brick.transform.rotation.z = quat_brick[2]
        brick.transform.rotation.w = quat_brick[3]
        
        ##### Make frames for robot parts #####
        
        # Define wheel frame 
        wheel = TransformStamped()
        wheel.header.frame_id = "base_link"
        wheel.child_frame_id = "wheel"
        # TODO update this to initial position of the turtle
        wheel.transform.translation.x = 0.0
        wheel.transform.translation.y = 0.0
        wheel.transform.translation.z = 0.2
        # Right now the wheel tf is flipped 90, will be easier to make urdf flip
        quat_wheel = quaternion_from_euler(float(np.pi/2), float(0), float(0)) # Roll pitch yaw
        wheel.transform.rotation.x = quat_wheel[0]
        wheel.transform.rotation.y = quat_wheel[1]
        wheel.transform.rotation.z = quat_wheel[2]
        wheel.transform.rotation.w = quat_wheel[3]
        
        # Define stem frame 
        stem = TransformStamped()
        stem.header.frame_id = "wheel"
        stem.child_frame_id = "stem"
        # TODO update this to initial position of the turtle
        stem.transform.translation.x = 0.0
        stem.transform.translation.y = 0.2
        stem.transform.translation.z = 0.0
        quat_stem = quaternion_from_euler(float(-np.pi/2), float(0), float(0)) # Roll pitch yaw
        stem.transform.rotation.x = quat_stem[0]
        stem.transform.rotation.y = quat_stem[1]
        stem.transform.rotation.z = quat_stem[2]
        stem.transform.rotation.w = quat_stem[3]
        
        # Define stem frame 
        platform = TransformStamped()
        platform.header.frame_id = "stem"
        platform.child_frame_id = "platform"
        # TODO update this to initial position of the turtle
        platform.transform.translation.x = 0.0
        platform.transform.translation.y = 0.0
        platform.transform.translation.z = 0.25
        quat_platform = quaternion_from_euler(float(0), float(0), float(0)) # Roll pitch yaw
        platform.transform.rotation.x = quat_platform[0]
        platform.transform.rotation.y = quat_platform[1]
        platform.transform.rotation.z = quat_platform[2]
        platform.transform.rotation.w = quat_platform[3]
                
        # don't forget to put a timestamp
        time = self.get_clock().now().to_msg()
        base_link.header.stamp = time
        brick.header.stamp = time
        wheel.header.stamp = time
        stem.header.stamp = time
        platform.header.stamp = time
        self.broadcaster.sendTransform(platform)
        self.broadcaster.sendTransform(wheel)
        self.broadcaster.sendTransform(stem)
        self.broadcaster.sendTransform(brick)
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