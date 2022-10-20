from cmath import atan
import math
from re import S
import sys
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from turtlesim.msg import Pose
from rcl_interfaces.msg import ParameterDescriptor
import yaml
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import Point, PoseStamped
from enum import Enum, auto
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState


# Modified code from ROS2 static broadcater tutorial source code 
# Accessed 10/9/2022
# https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.html

# TODO bug when dropping brick from center
# TODO publish cmd_vel always
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


class State(Enum):
    """ States to keep track of where the system is.
    """
    FALLING = auto(),
    INIT = auto(),
    DETECTING = auto(),
    BACKHOME = auto(),
    MOVING = auto(),
    STOPPED = auto(),
    RESET = auto()
    
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
        self.turtle_pose = Pose()
        self.state = State.INIT
        # self.turtle_pose.x = 5.5
        # self.turtle_pose.y = 5.5
        self.sub_turtle_pose = self.create_subscription(Pose, "turtle1/pose", self.listener_callback_turtle_pose, 10)
        self.sub_goal_pose = self.create_subscription(PoseStamped, "goal_pose", self.listener_callback_goal_pose, 10)
        self.declare_parameter("wheel_radius", 0.2,
                               ParameterDescriptor(description="The radius of the turtle robot wheel in meters"))
        self.wheel_radius  = self.get_parameter("wheel_radius").get_parameter_value().double_value
        
        self.declare_parameter("platform_height", 1.3,
                               ParameterDescriptor(description="The height of the turtle robot's platform in meters"))
        self.platform_h  = self.get_parameter("platform_height").get_parameter_value().double_value  
        
        self.declare_parameter("max_velocity", 0.4,
                               ParameterDescriptor(description="The maximum velocity of the turtle robot in meters/sec"))
        self.max_velocity  = self.get_parameter("max_velocity").get_parameter_value().double_value
        self.pub_vel = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.pub_joints = self.create_publisher(JointState, "joint_states", 10)
        self.sub_reset = self.create_subscription(Bool, "reset_sim", self.reset_callback, 10)
        self.is_brick_caught = self.create_subscription(Bool, "brick_caught", self.brick_caught_callback, 10)
        self.caught_flag = False
        #######
        self.pub_tilt_to_arena = self.create_publisher(Bool, "tilt_in_arena", 10)
        self.time = 0
        
        self.joints = JointState()
        self.platform_tilt_rads = 0.0
        self.stem_turn_rads = 0.0
        self.wheel_turn_rads = 0.0
        self.platform_tilt_vel = 0.0 
        self.stem_turn_vel = 0.0 
        self.wheel_turn_vel = 0.0
        ###########
        self.sub_go_robot = self.create_subscription(Bool, "send_turtle_robot", self.go_robot_callback, 10)
        self.go_robot_flag = False
        self.turtle_init_flag = True
        self.goal_pose = PoseStamped()
        self.base_offset = self.platform_h - 0.475
        self.odom_x = self.turtle_pose.x
        self.odom_y = self.turtle_pose.y
        # Publish a static tf upon start up
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.make_transforms()
        
        ### Below from in_out.py ###
        # create the broadcaster
        self.broadcaster = TransformBroadcaster(self)
        # Create a timer to do the rest of the transforms

        self.tmr = self.create_timer(0.004, self.timer_callback)
        
    def brick_caught_callback(self, data):
        self.caught_flag = data.data
        
    def reset_callback(self, data):
        """_summary_

        Args:
            data (_type_): _description_
        """
        self.platform_tilt_rads = 0
        self.state = State.INIT
        self.caught_flag = False
        self.platform_tilt_rads = 0.0
        self.stem_turn_rads = 0.0
        self.wheel_turn_rads = 0.0
        self.platform_tilt_vel = 0.0 
        self.stem_turn_vel = 0.0 
        self.wheel_turn_vel = 0.0
        self.go_robot_flag = False
        #self.__init__()
        #self.go_robot_flag = False
        
    def go_robot_callback(self, data):
        self.go_robot_flag = data.data
        
    def listener_callback_goal_pose(self, msg):
        """TODO"""
        self.goal_pose = msg  
        
    def listener_callback_turtle_pose(self, msg):
        """Get turtle pose.
        """
        if self.turtle_init_flag:
            self.turtle_init = msg
            self.turtle_init_flag = False
            
        self.turtle_pose = msg
        
    def cmd_vel_to_goal(self):
        """"""
        x = self.goal_pose.pose.position.x - self.turtle_pose.x
        y = self.goal_pose.pose.position.y - self.turtle_pose.y
        theta = math.atan2(y,x)
        x_vel = self.max_velocity*math.cos(theta)
        y_vel = self.max_velocity*math.sin(theta)
        cmd_2_goal = Twist(linear = Vector3(x = x_vel, y = y_vel ,z =0.0), 
                        angular = Vector3(x = 0.0, y = 0.0, z = 0.0))
        
        if abs(x)<0.01 and abs(y)<0.01: # At goal, Stop the jiggle
            cmd_2_goal = Twist(linear = Vector3(x = 0.0, y = 0.0 ,z =0.0), 
                angular = Vector3(x = 0.0, y = 0.0, z = 0.0))
            self.state = State.STOPPED
            self.pub_vel.publish(cmd_2_goal)
            # Are we at the center
            if (self.goal_pose.pose.position.x - self.turtle_init.x) < 0.01 and (self.goal_pose.pose.position.y - self.turtle_init.y) < 0.01:
                self.state = State.BACKHOME
                tilt_arena_bool = Bool()
                tilt_arena_bool.data = True
                if self.caught_flag:
                    self.pub_tilt_to_arena.publish(tilt_arena_bool)
        else:
            self.state = State.MOVING
            self.pub_vel.publish(cmd_2_goal)
        
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
        t.transform.translation.x = self.odom_x
        t.transform.translation.y = self.odom_y
        t.transform.translation.z = 0.0
        quat = quaternion_from_euler(
            float(0), float(0), float(0))
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        # Broadcast TF
        self.tf_static_broadcaster.sendTransform(t)
            
    def get_stem_angle(self):
        """"""
        x = self.goal_pose.pose.position.x - self.turtle_pose.x
        y = self.goal_pose.pose.position.y - self.turtle_pose.y
        heading = math.atan2(y,x)
        self.stem_turn_rads = heading

    def wheel_spin(self):
        """"""
        if self.state == State.MOVING:
            self.wheel_turn_vel = self.max_velocity / self.wheel_radius
            self.wheel_turn_rads = self.max_velocity * self.time

    def timer_callback(self):
        # Define base_link frame 
        self.time += 0.01
        self.joints.header.stamp = self.get_clock().now().to_msg()
        self.joints.name = ["turn_wheel", "spin_wheel", "base_to_tube", "tilt_platform"]
        self.joints.position = [float(self.stem_turn_rads), float(self.wheel_turn_rads), float(0.0), float(self.platform_tilt_rads)]
        self.joints.velocity = [float(self.stem_turn_vel), float(self.wheel_turn_vel), float(0.0), float(self.platform_tilt_vel)]
        self.pub_joints.publish(self.joints)          
        
        base_link = TransformStamped()
        base_link.header.frame_id = "odom"
        base_link.child_frame_id = "base_link"
        # TODO update this to initial position of the turtle
        base_link.transform.translation.x = self.turtle_pose.x - self.odom_x
        base_link.transform.translation.y = self.turtle_pose.y - self.odom_y
        base_link.transform.translation.z = self.base_offset
        quat_base = quaternion_from_euler(float(0), float(0), float(self.turtle_pose.theta)) # Roll pitch yaw
        base_link.transform.rotation.x = quat_base[0]
        base_link.transform.rotation.y = quat_base[1]
        base_link.transform.rotation.z = quat_base[2]
        base_link.transform.rotation.w = quat_base[3]      
               
        # don't forget to put a timestamp
        time = self.get_clock().now().to_msg()
        base_link.header.stamp = time
        
        self.broadcaster.sendTransform(base_link)
        if self.go_robot_flag:
            self.get_stem_angle()
            self.wheel_spin()
            self.cmd_vel_to_goal()
        
        if self.state == State.BACKHOME:
            # Tilt
            self.platform_tilt_rads = 0.7
            self.platform_tilt_vel = 0.3
            print("lol")
        self.get_logger().info(f'State: {self.state}')
            
        

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