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
        self.pub_boundary = self.create_publisher(MarkerArray, "visualization_marker_array", 10) # Marker publisher for boundary of the arena
        self.pub_brick = self.create_publisher(Marker, "visualization_marker", 10) # Marker publisher for brick
        # self.cube = Marker()
        # self.cube.header._stamp = Arena.get_clock(self).now()
        # self.cube.id = 1
        # self.cube.type = 1
        # #self.cube.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
        # self.pub_boundary.publish(self.cube)
        
        self.make_marker_array()
        
        self.broadcaster = TransformBroadcaster(self)
        self.tmr = self.create_timer(0.004, self.timer_callback) 
        
    def make_brick(self):
        self.brick = Marker()
        self.brick.header.frame_id = "/brick"
        self.brick.header.stamp = self.get_clock().now().to_msg()
        self.brick.type = self.brick.CUBE
        self.brick.id =5
        #self.marker1.action = self.marker1.ADD
        self.brick.scale.x = 0.4
        self.brick.scale.y = 0.2
        self.brick.scale.z = 0.2
        self.brick.color.a = 1.0
        self.brick.color.b = 0.3
        self.brick.color.g = 0.3
        self.brick.color.r = 0.7
        transb = quaternion_from_euler(0, 0, 0)
        self.brick.pose.orientation.w = transb[0]
        self.brick.pose.orientation.x = transb[1]
        self.brick.pose.orientation.y = transb[2]
        self.brick.pose.orientation.z = transb[3]
        self.brick.pose.position.x = 0.0
        self.brick.pose.position.y = 0.0
        self.brick.pose.position.z = 0.0
        
    def make_markers(self):
        self.marker1 = Marker()
        self.marker1.header.frame_id = "/world"
        self.marker1.header.stamp = self.get_clock().now().to_msg()
        self.marker1.type = self.marker1.CUBE
        self.marker1.id =1
        #self.marker1.action = self.marker1.ADD
        self.marker1.scale.x = 10.0
        self.marker1.scale.y = 0.2
        self.marker1.scale.z = 1.0
        self.marker1.color.a = 1.0
        trans = quaternion_from_euler(0, 0, 0)
        self.marker1.pose.orientation.w = trans[0]
        self.marker1.pose.orientation.x = trans[1]
        self.marker1.pose.orientation.y = trans[2]
        self.marker1.pose.orientation.z = trans[3]
        self.marker1.pose.position.x = 0.0
        self.marker1.pose.position.y = 5.0
        self.marker1.pose.position.z = 0.5
        
        self.marker2 = Marker()
        self.marker2.header.frame_id = "/world"
        self.marker2.header.stamp = self.get_clock().now().to_msg()
        self.marker2.type = self.marker2.CUBE
        self.marker2.id =2
        #self.marker2.action = self.marker2.ADD
        self.marker2.scale.x = 10.0
        self.marker2.scale.y = 0.2
        self.marker2.scale.z = 1.0
        self.marker2.color.a = 1.0
        trans = quaternion_from_euler(0,0, 0)
        self.marker2.pose.orientation.w = trans[0]
        self.marker2.pose.orientation.x = trans[1]
        self.marker2.pose.orientation.y = trans[2]
        self.marker2.pose.orientation.z = trans[3]
        self.marker2.pose.position.x = 0.0
        self.marker2.pose.position.y = -5.0
        self.marker2.pose.position.z = 0.5
        
        self.marker3 = Marker()
        self.marker3.header.frame_id = "/world"
        self.marker3.header.stamp = self.get_clock().now().to_msg()
        self.marker3.type = self.marker3.CUBE
        self.marker3.id =3
        #self.marker3.action = self.marker3.ADD
        self.marker3.scale.x = 10.0
        self.marker3.scale.y = 0.2
        self.marker3.scale.z = 1.0
        self.marker3.color.a = 1.0
        trans = quaternion_from_euler(1.57, 0, 0)
        self.marker3.pose.orientation.w = trans[0]
        self.marker3.pose.orientation.x = trans[1]
        self.marker3.pose.orientation.y = trans[2]
        self.marker3.pose.orientation.z = trans[3]
        self.marker3.pose.position.x = -5.0
        self.marker3.pose.position.y = 0.0
        self.marker3.pose.position.z = 0.5
        
        self.marker4 = Marker()
        self.marker4.header.frame_id = "/world"
        self.marker4.header.stamp = self.get_clock().now().to_msg()
        self.marker4.type = self.marker4.CUBE
        self.marker4.id =4
        #self.marker4.action = self.marker4.ADD
        self.marker4.scale.x = 10.0
        self.marker4.scale.y = 0.2
        self.marker4.scale.z = 1.0
        self.marker4.color.a = 1.0
        trans = quaternion_from_euler(1.57, 0, 0)
        self.marker4.pose.orientation.w = trans[0]
        self.marker4.pose.orientation.x = trans[1]
        self.marker4.pose.orientation.y = trans[2]
        self.marker4.pose.orientation.z = trans[3]
        self.marker4.pose.position.x = 5.0
        self.marker4.pose.position.y = 0.0
        self.marker4.pose.position.z = 0.5
        
    def make_marker_array(self):
        self.make_markers()
        self.marker_array = MarkerArray()
        self.marker_array.markers.append(self.marker1)
        self.marker_array.markers.append(self.marker2)
        self.marker_array.markers.append(self.marker3)
        self.marker_array.markers.append(self.marker4)
        
    def timer_callback(self):
        """
        """
        self.pub_boundary.publish(self.marker_array)
        
        # # # Define brick frame 
        brick = TransformStamped()
        brick.header.frame_id = "world"
        brick.child_frame_id = "brick"
        # # # TODO update this to iwhatever the brick needs to be, starting it off as just 6 m above world frame
        brick.transform.translation.x = 0.0
        brick.transform.translation.y = 0.0
        brick.transform.translation.z = 8.0
        quat_brick = quaternion_from_euler(float(0), float(0), float(0.0))
        brick.transform.rotation.x = quat_brick[0]
        brick.transform.rotation.y = quat_brick[1]
        brick.transform.rotation.z = quat_brick[2]
        brick.transform.rotation.w = quat_brick[3]
        time = self.get_clock().now().to_msg()
        brick.header.stamp = time
        self.broadcaster.sendTransform(brick)
        self.make_brick()
        self.pub_brick.publish(self.brick)
        
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