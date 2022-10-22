import math
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from rcl_interfaces.msg import ParameterDescriptor
from turtle_brick_interfaces.srv import Place
from std_srvs.srv import Empty
from enum import Enum, auto
from turtlesim.msg import Pose
from std_msgs.msg import Bool
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException


# Modified code from ROS2 static broadcater tutorial source code
# Accessed 10/9/2022
# https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.html

def quaternion_from_euler(ai, aj, ak):
    """
    Take in Euler angles and converts them to quaternions. Function taken from link above.

    Args:
    ----
        ai (float): Roll angle.
        aj (float): Pitch angle.
        ak (float): Yaw angle.

    Return:
    ------
        float array: Array of quaternion angles.

    """
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = np.empty((4, ))
    q[0] = cj * sc - sj * cs
    q[1] = cj * ss + sj * cc
    q[2] = cj * cs - sj * sc
    q[3] = cj * cc + sj * ss

    return q


class State(Enum):
    """States to keep track of where the system is."""

    DROP = auto(),
    INIT = auto(),
    NEW_BRICK = auto(),
    ON_PLATFORM = auto(),
    ABOVE_PLATFORM = auto(),
    CAUGHT = auto(),
    TILT = auto(),
    SLIDE = auto(),
    RESET = auto()


class Arena(Node):
    """
    Node responsible for simulating environment.

    Publishes markers to denote the boundaries of the environment in rviz over
    the visualization_marker_array topic.
    Publishes a marker over the visualization_marker topic to represent the brick.
    Publishes a boolean to the brick_status topic to let other nodes know if the
    brick has spawned.
    Publishes a boolean to the reset_sim topic to indicate when the other nodes
    must reset.
    Subscribes to the turtle1/pose topic to keep up with the pose of the robot.
    Subscribes to the brick_caught topic to know whether or not the brick has been
    caught by the platform.
    Subscribes to the tilt_in_arena topic to know when the platform tilts to create
    associated pyhics for the brick.
    Braodcasts the brick frame at the location of the brick relative to the world
    frame.
    Offers a service called place (of custom type) that moves the brick to a
    specified location.
    Offers a service called drop that causes the brick to fall in gravity.
    """

    def __init__(self):
        """Initialize variables, states, publishers and subscribers."""
        # Initialize node with name arena.
        super().__init__('arena')
        self.state = State.INIT
        self.state_brick = State.INIT
        self.sub = self.create_subscription(
            Pose, "turtle1/pose", self.listener_callback, 10)
        self.pub_boundary = self.create_publisher(
            MarkerArray,
            "visualization_marker_array",
            10)  # Marker publisher for boundary of the arena
        self.pub_brick = self.create_publisher(
            Marker, "visualization_marker", 10)  # Marker publisher for brick
        self.pub_reset = self.create_publisher(Bool, "reset_sim", 10)
        self.time = 0.0
        self.brick_x = 12.0  # Made 12 so that brick frame always spawns outside
        self.brick_y = 12.0  # of arena originally
        self.brick_z0 = 8.0
        self.brick_z_current = self.brick_z0
        self.pub_brick_status = self.create_publisher(Bool, "brick_status", 10)

        self.reset_bool = Bool()
        self.reset_bool.data = False

        self.brick_init = False  # Flag to not spawn brick until called
        self.sub = self.create_subscription(
            Pose, "turtle1/pose", self.listener_callback, 10)
        self.turtle_pose = Pose()
        self.declare_parameter("platform_height", 1.3, ParameterDescriptor(
            description="The height of the turtle robot's platform in meters"))
        self.platform_h = self.get_parameter(
            "platform_height").get_parameter_value().double_value

        self.declare_parameter("gravity", 9.8, ParameterDescriptor(
            description="The brick's acceleration due to gravity"))
        self.gravity = self.get_parameter(
            "gravity").get_parameter_value().double_value

        self.declare_parameter("max_velocity", 0.4, ParameterDescriptor(
            description="The maximum velocity of the turtle robot in meters/sec"))
        self.max_velocity = self.get_parameter(
            "max_velocity").get_parameter_value().double_value
        self.catch_once = True
        self.place = self.create_service(Place, "place", self.place_callback)
        self.drop = self.create_service(Empty, "drop", self.drop_callback)
        self.is_brick_caught = self.create_subscription(
            Bool, "brick_caught", self.brick_caught_callback, 10)
        self.sub_tilt = self.create_subscription(
            Bool, "tilt_in_arena", self.tilt_callback, 10)

        self.make_marker_array()
        self.brick = TransformStamped()
        self.brick.header.frame_id = "world"
        self.brick.child_frame_id = "brick"
        self.broadcaster = TransformBroadcaster(self)

        self.target_frame = self.declare_parameter(
            'target_frame', 'brick').get_parameter_value().string_value
        self.tf_buffer_platform = Buffer()
        self.tf_listener_platform = TransformListener(
            self.tf_buffer_platform, self)
        self.tmr = self.create_timer(0.004, self.timer_callback)

    def tilt_callback(self, msg):
        """Read the tilt message."""
        if msg.data:
            self.state_brick = State.TILT
            self.state = State.TILT

    def brick_caught_callback(self, data):
        """
        Read brick_caught topic to see if the brick has been caught.

        Args:
        ----
            data (Bool): Boolean that describes if the brick has been caught

        """
        if data.data and self.catch_once:
            self.state_brick = State.CAUGHT
            self.state = State.CAUGHT
            self.catch_once = False

    def listener_callback(self, msg):
        """Get turtle pose."""
        self.turtle_pose = msg

    def listen_to_platforrm(self):
        """
        Listen for brick to platform tf.

        Return:
        ------
            Buffer: relationship between brick and platform frames

        """
        from_frame_rel = self.target_frame
        to_frame_rel = 'platform'

        try:
            t = self.tf_buffer_platform.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        return t

    def place_callback(self, request, response):
        """
        Put brick at a specified location.

        Args:
        ----
            request (Place): Coordinates of brick
            response (Place): Null

        Return:
        ------
            Place: Null

        """
        self.state = State.NEW_BRICK
        self.brick_x = request.x
        self.brick_y = request.y
        self.brick_z0 = request.z
        self.brick_z_current = self.brick_z0
        self.brick_init = True
        self.brick_tf_and_pub()
        return response

    def make_brick(self):
        """Create the marker to publish as the brick."""
        self.brick_marker = Marker()
        self.brick_marker.header.frame_id = "/brick"
        self.brick_marker.header.stamp = self.get_clock().now().to_msg()
        self.brick_marker.type = self.brick_marker.CUBE
        self.brick_marker.id = 5
        self.brick_marker.scale.x = 0.4
        self.brick_marker.scale.y = 0.2
        self.brick_marker.scale.z = 0.1
        self.brick_marker.color.a = 1.0
        self.brick_marker.color.b = 0.3
        self.brick_marker.color.g = 0.3
        self.brick_marker.color.r = 0.7
        transb = quaternion_from_euler(0, 0, 0)
        self.brick_marker.pose.orientation.w = transb[0]
        self.brick_marker.pose.orientation.x = transb[1]
        self.brick_marker.pose.orientation.y = transb[2]
        self.brick_marker.pose.orientation.z = transb[3]
        self.brick_marker.pose.position.x = 0.0
        self.brick_marker.pose.position.y = 0.0
        self.brick_marker.pose.position.z = 0.0

    def make_markers(self):
        """Create the 4 markers used as walls of the arena."""
        self.marker1 = Marker()
        self.marker1.header.frame_id = "/world"
        self.marker1.header.stamp = self.get_clock().now().to_msg()
        self.marker1.type = self.marker1.CUBE
        self.marker1.id = 1
        self.marker1.scale.x = 11.0
        self.marker1.scale.y = 0.2
        self.marker1.scale.z = 1.0
        self.marker1.color.a = 1.0
        trans = quaternion_from_euler(0, 0, 0)
        self.marker1.pose.orientation.w = trans[0]
        self.marker1.pose.orientation.x = trans[1]
        self.marker1.pose.orientation.y = trans[2]
        self.marker1.pose.orientation.z = trans[3]
        self.marker1.pose.position.x = 5.5
        self.marker1.pose.position.y = 11.0
        self.marker1.pose.position.z = 0.5

        self.marker2 = Marker()
        self.marker2.header.frame_id = "/world"
        self.marker2.header.stamp = self.get_clock().now().to_msg()
        self.marker2.type = self.marker2.CUBE
        self.marker2.id = 2
        self.marker2.scale.x = 11.0
        self.marker2.scale.y = 0.2
        self.marker2.scale.z = 1.0
        self.marker2.color.a = 1.0
        trans = quaternion_from_euler(0, 0, 0)
        self.marker2.pose.orientation.w = trans[0]
        self.marker2.pose.orientation.x = trans[1]
        self.marker2.pose.orientation.y = trans[2]
        self.marker2.pose.orientation.z = trans[3]
        self.marker2.pose.position.x = 5.5
        self.marker2.pose.position.y = 0.0
        self.marker2.pose.position.z = 0.5

        self.marker3 = Marker()
        self.marker3.header.frame_id = "/world"
        self.marker3.header.stamp = self.get_clock().now().to_msg()
        self.marker3.type = self.marker3.CUBE
        self.marker3.id = 3
        self.marker3.scale.x = 11.0
        self.marker3.scale.y = 0.2
        self.marker3.scale.z = 1.0
        self.marker3.color.a = 1.0
        trans = quaternion_from_euler(1.57, 0, 0)
        self.marker3.pose.orientation.w = trans[0]
        self.marker3.pose.orientation.x = trans[1]
        self.marker3.pose.orientation.y = trans[2]
        self.marker3.pose.orientation.z = trans[3]
        self.marker3.pose.position.x = 0.0
        self.marker3.pose.position.y = 5.5
        self.marker3.pose.position.z = 0.5

        self.marker4 = Marker()
        self.marker4.header.frame_id = "/world"
        self.marker4.header.stamp = self.get_clock().now().to_msg()
        self.marker4.type = self.marker4.CUBE
        self.marker4.id = 4
        self.marker4.scale.x = 11.0
        self.marker4.scale.y = 0.2
        self.marker4.scale.z = 1.0
        self.marker4.color.a = 1.0
        trans = quaternion_from_euler(1.57, 0, 0)
        self.marker4.pose.orientation.w = trans[0]
        self.marker4.pose.orientation.x = trans[1]
        self.marker4.pose.orientation.y = trans[2]
        self.marker4.pose.orientation.z = trans[3]
        self.marker4.pose.position.x = 11.0
        self.marker4.pose.position.y = 5.5
        self.marker4.pose.position.z = 0.5

    def make_marker_array(self):
        """Put all markers in an array to publish."""
        self.make_markers()
        self.marker_array = MarkerArray()
        self.marker_array.markers.append(self.marker1)
        self.marker_array.markers.append(self.marker2)
        self.marker_array.markers.append(self.marker3)
        self.marker_array.markers.append(self.marker4)

    def drop_callback(self, request, response):
        """
        Service callback to change to drop state.

        Args:
        ----
            request (Empty): triggers callback
            response (Empty): Nulll

        Return:
        ------
            Empty: Null

        """
        self.state = State.DROP
        return response

    def drop_brick(self):
        """Handle the pyhsics of dropping the brick."""
        if self.state == State.DROP:
            self.brick_init = True
            if self.state_brick == State.INIT:
                if self.brick_z_current > 0.1:
                    # Check if on platform -> if yes stop brick at that height
                    # -> change state to on platform

                    self.time += 0.004
                    self.brick_z_current = self.brick_z0 - 0.5 * 9.8 * self.time**2
                else:
                    self.state = State.RESET

            # If you are dropping and above the platform stop at platform height
            # Else stop at ground
            if self.state_brick == State.ABOVE_PLATFORM:
                if self.brick_z_current > self.platform_h + 0.1:
                    self.brick_z_current = self.brick_z0 - 0.5 * 9.8 * self.time**2
                    self.time += 0.004
                else:
                    self.brick_z_current = self.platform_h + 0.1

        if abs(
            self.turtle_pose.x -
            self.brick_x) < 0.05 and abs(
            self.turtle_pose.y -
                self.brick_y) < 0.05:  # If you are above the platform
            self.state_brick = State.ABOVE_PLATFORM

    def brick_tf_and_pub(self):
        """Calculate the brick tf based off the state and publish it."""
        if self.state == State.NEW_BRICK:
            self.brick.transform.translation.x = self.brick_x
            self.brick.transform.translation.y = self.brick_y
            self.brick.transform.translation.z = self.brick_z0
        elif self.state == State.DROP:
            self.brick.transform.translation.x = self.brick_x
            self.brick.transform.translation.y = self.brick_y
            self.brick.transform.translation.z = self.brick_z_current

        if self.state == State.CAUGHT:
            self.brick.transform.translation.x = self.turtle_pose.x
            self.brick.transform.translation.y = self.turtle_pose.y
            self.brick.transform.translation.z = self.platform_h + 0.05

        if self.state_brick == State.TILT:
            tilt_quant = quaternion_from_euler(0, 0.7, 0)
            self.brick.transform.rotation.x = tilt_quant[0]
            self.brick.transform.rotation.y = tilt_quant[1]
            self.brick.transform.rotation.z = tilt_quant[2]
            self.brick.transform.rotation.w = tilt_quant[3]
            self.state = State.SLIDE
            self.state_brick = State.SLIDE
            self.time = 0

        if self.state == State.SLIDE:
            if self.brick.transform.translation.z > (
                    0.3 + 0.2):  # platform radius + brick length/2
                self.time += 0.04
                diff = 0.5 * 9.8 * self.time**2
                self.brick.transform.translation.z -= diff
                self.brick.transform.translation.x += (diff / math.tan(0.7))
            else:
                self.state = State.RESET

        time = self.get_clock().now().to_msg()
        self.brick.header.stamp = time
        self.broadcaster.sendTransform(self.brick)
        self.make_brick()

        if self.brick_init:  # Dont publish brick until initialized
            self.pub_brick.publish(self.brick_marker)

    def reset_the_brick(self):
        """Reset the brick and states so simulation can run again."""
        self.brick.transform.translation.x = self.brick_x
        self.brick.transform.translation.y = self.brick_y
        self.brick.transform.translation.z = self.brick_z0
        tilt_quant = quaternion_from_euler(0, 0, 0)
        self.brick.transform.rotation.x = tilt_quant[0]
        self.brick.transform.rotation.y = tilt_quant[1]
        self.brick.transform.rotation.z = tilt_quant[2]
        self.brick.transform.rotation.w = tilt_quant[3]

        self.brick_tf_and_pub()
        self.state = State.NEW_BRICK
        self.state_brick = State.INIT
        self.catch_once = True
        self.reset_bool.data = True
        self.brick_init = False
        self.pub_reset.publish(self.reset_bool)
        self.reset_bool.data = False

    def timer_callback(self):
        """Iterate every time the timer is called."""
        # Publish arena walls
        self.pub_boundary.publish(self.marker_array)

        self.drop_brick()

        self.brick_tf_and_pub()
        if self.state_brick == State.TILT:
            self.listen_to_platforrm()

        brick_bool = Bool()
        brick_bool.data = self.brick_init
        self.pub_brick_status.publish(brick_bool)

        if self.state == State.RESET:
            self.reset_the_brick()


def main():
    rclpy.init()
    node = Arena()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
