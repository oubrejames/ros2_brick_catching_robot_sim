import math
import sys
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

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


class Arena(Node):
    """
    Publishes markers to denote the boundaries of the environment in rviz. 
    Publishes the brick frame at the location of the brick and a marker to show its location in rviz
    offers a service called place (of custom type) that moves the brick to  a pecified location
    offers a service called drop that causes the brick to fall in gravity according to specified rule (on website)
    """

    def __init__(self):
        """TODO -

        Args:
            transformation (_type_): _description_
        """
        # Initialize node with name turtle_robot.
        super().__init__('arena')
        self.pub_boundary = self.create_publisher(Marker, "visualization_marker", 10) # Marker publisher for boundary of the arena
        # self.cube = Marker()
        # self.cube.header._stamp = Arena.get_clock(self).now()
        # self.cube.id = 1
        # self.cube.type = 1
        # #self.cube.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
        # self.pub_boundary.publish(self.cube)

        self.marker = Marker()
        self.marker.header.frame_id = "/world"
        self.marker.type = self.marker.CUBE
        self.marker.action = self.marker.ADD
        self.marker.scale.x = 10.0
        self.marker.scale.y = 0.2
        self.marker.scale.z = 1.0
        self.marker.color.a = 1.0
        trans = quaternion_from_euler(1.57, 0, 0)
        self.marker.pose.orientation.w = trans[0]
        self.marker.pose.orientation.x = trans[1]
        self.marker.pose.orientation.y = trans[2]
        self.marker.pose.orientation.z = trans[3]
        self.marker.pose.position.x = 5.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0
        
        self.tmr = self.create_timer(1, self.timer_callback) 
        self.pub_boundary.publish(self.marker)
        
    def timer_callback(self):
        """
        """
        #self.pub_boundary.publish(self.marker)

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
    node = Arena()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()