import math
import sys
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from rcl_interfaces.msg import ParameterDescriptor


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


class Catcher(Node):
    """
    Publishes markers to denote the boundaries of the environment in rviz. 
    Publishes the brick frame at the location of the brick and a marker to show its location in rviz
    offers a service called place (of custom type) that moves the brick to  a pecified location
    offers a service called drop that causes the brick to fall in gravity according to specified rule (on website)
    """

    def __init__(self):
        """TODO TODO TODO TODO TODO this is all nonsense - not actually catcher

        Args:
            transformation (_type_): _description_
        """
        # Initialize node with name turtle_robot.
        super().__init__('catcher')
        self.pub_boundary = self.create_publisher(MarkerArray, "visualization_marker_array", 10) # Marker publisher for boundary of the arena
        self.pub_brick = self.create_publisher(Marker, "visualization_marker", 10) # Marker publisher for brick
        
        self.declare_parameter("platform_height", 0.9,
                               ParameterDescriptor(description="The height of the turtle robot's platform in meters"))
        self.platform_h  = self.get_parameter("max_velocity").get_parameter_value().double_value        
        
        self.declare_parameter("max_velocity", 0.22,
                               ParameterDescriptor(description="The maximum velocity of the turtle robot in meters/sec"))
        self.max_velocity  = self.get_parameter("max_velocity").get_parameter_value().double_value
        
        self.make_marker_array()
        
        self.broadcaster = TransformBroadcaster(self)
        self.tmr = self.create_timer(0.004, self.timer_callback) 
        
        
    def timer_callback(self):
        """
        """

        
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
    node = Catcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()