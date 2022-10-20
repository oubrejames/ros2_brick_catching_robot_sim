import math
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
from geometry_msgs.msg import PoseStamped
from enum import Enum, auto
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from turtle_brick_interfaces.msg import Tilt
from nav_msgs.msg import Odometry

# Modified code from ROS2 static broadcater tutorial source code 
# Accessed 10/9/2022
# https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.html


def quaternion_from_euler(ai, aj, ak):
    """Takes in Euler angles and converts them to quaternions.

    Args:
        ai (float): Roll angle.
        aj (float): Pitch angle.
        ak (float): Yaw angle.

    Returns:
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
    """Node for controlling the turtle's actions. Broadcasts the location of the base_link 
    frame relative to the odom frame. Publishes to cmd_vel to actually move the robot. Publishes 
    joint states to the joint_states topic. Publishes a boolean to the tilt topic to indicate
    to other nodes when the platform is tilting. Publishes the odometry of the robot to the 
    odom topic. Subscribes to the turtle1/pose topic to keep up with the pose of the robot.
    Subscribes to the goal_pose topic to know where the goal point is. Subscribes to the tilt 
    topic to get the tilting angle for the platform of the robot. Subscribes to the reset_sim 
    and brick_caught boolean topics to know when to reset its states and when the brick is 
    caught on the platform.
    """

    def __init__(self):
        """Initialize intitial variables, states, publishers and subscribers.
        """
        # Initialize node with name turtle_robot.
        super().__init__('turtle_robot')
        
        # Initial states and variables
        self.turtle_pose = Pose()
        self.state = State.INIT
        self.tilt_degrees = 0.7 # Default tilt degree in rads
        self.caught_flag = False # Flag for if the brick is caught
        self.time = 0 # Time counter
        
        # Initial joint state values
        self.joints = JointState()
        self.platform_tilt_rads = 0.0
        self.stem_turn_rads = 0.0
        self.wheel_turn_rads = 0.0
        self.platform_tilt_vel = 0.0 
        self.stem_turn_vel = 0.0 
        self.wheel_turn_vel = 0.0
        
        # Parameters
        self.declare_parameter("wheel_radius", 0.2,
                               ParameterDescriptor(description="The radius of the turtle robot wheel in meters"))
        self.wheel_radius  = self.get_parameter("wheel_radius").get_parameter_value().double_value    
        self.declare_parameter("platform_height", 1.3,
                               ParameterDescriptor(description="The height of the turtle robot's platform in meters"))
        self.platform_h  = self.get_parameter("platform_height").get_parameter_value().double_value  
        
        self.declare_parameter("max_velocity", 0.4,
                               ParameterDescriptor(description="The maximum velocity of the turtle robot in meters/sec"))
        self.max_velocity  = self.get_parameter("max_velocity").get_parameter_value().double_value

        # Publishers
        self.pub_vel = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.pub_joints = self.create_publisher(JointState, "joint_states", 10)
        self.pub_tilt_to_arena = self.create_publisher(Bool, "tilt_in_arena", 10)
        self.odom_publisher = self.create_publisher(Odometry, "odom", 10)
        
        # Subscribers
        self.sub_turtle_pose = self.create_subscription(Pose, "turtle1/pose", self.listener_callback_turtle_pose, 10)
        self.sub_goal_pose = self.create_subscription(PoseStamped, "goal_pose", self.listener_callback_goal_pose, 10)
        self.tilt_sub = self.create_subscription(Tilt, "tilt", self.tilt_callback, 10)
        self.sub_reset = self.create_subscription(Bool, "reset_sim", self.reset_callback, 10)
        self.is_brick_caught = self.create_subscription(Bool, "brick_caught", self.brick_caught_callback, 10) 
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
        
        # Create the broadcaster
        self.broadcaster = TransformBroadcaster(self)
        
        # Create a timer to do the rest of the transforms
        self.tmr = self.create_timer(0.004, self.timer_callback)
        
    def brick_caught_callback(self, data):
        """Subscriber to read when the brick is caught from the brick_caught
        topic.

        Args:
            data (Bool): boolean that indicates if the brick is caught on the platform
        """
        self.caught_flag = data.data
        
    def tilt_callback(self, msg):
        """Subscriber to read the tilt topic to get a user specfied tilt angle for the platform.

        Args:
            msg (Tilt): Custom message type indication radian value of tilt angle. 
        """
        self.tilt_degrees = msg.theta
        
    def reset_callback(self, data):
        """Subscriber to the reset_sim topic to reset the simulation when the
        brick respawns.

        Args:
            data (Bool): Boolean that indicates if resetting must happen
        """
        if data.data:
            # Reset to initial states
            self.caught_flag = False
            self.platform_tilt_rads = 0.0
            self.stem_turn_rads = 0.0
            self.wheel_turn_rads = 0.0
            self.platform_tilt_vel = 0.0 
            self.stem_turn_vel = 0.0 
            self.wheel_turn_vel = 0.0
            self.pub_joints.publish(self.joints)   
            self.go_robot_flag = False
            self.state = State.INIT
      
    def go_robot_callback(self, data):
        """Subscriber to the send_turtle_robot topic 

        Args:
            data (Bool): Boolean that indicates if the robot should start moving
        """
        self.go_robot_flag = data.data
        
    def listener_callback_goal_pose(self, msg):
        """Subscriber to the goal_pose topic.

        Args:
            msg (PoseStamped): Pose indicating the point for the turtle to move to.
        """
        self.goal_pose = msg  
        
    def listener_callback_turtle_pose(self, msg):
        """Subscriber to the turtle1/pose topic to get the current
        position of the robot.

        Args:
            msg (Pose): Position of the robot.
        """
        # Get just initial position
        if self.turtle_init_flag:
            self.turtle_init = msg
            self.turtle_init_flag = False
            
        self.turtle_pose = msg
        
    def pub_odom(self):
        """Publishes an odometry msg to the odom topic corresponding to the position of the turtle.
        """
        odom = Odometry()
        odom.twist.twist = Twist(linear = Vector3(x = self.turtle_pose.x, y = self.turtle_pose.y ,z = 0.0))
        self.odom_publisher.publish(odom)
        
    def cmd_vel_to_goal(self):
        """Publish necesssary velocity commands to the robot.
        """
        # Get the difference between current position and goal
        x = self.goal_pose.pose.position.x - self.turtle_pose.x
        y = self.goal_pose.pose.position.y - self.turtle_pose.y
        
        # Calculate x and y velocities based off max velocity
        theta = math.atan2(y,x)
        x_vel = self.max_velocity*math.cos(theta)
        y_vel = self.max_velocity*math.sin(theta)
        cmd_2_goal = Twist(linear = Vector3(x = x_vel, y = y_vel ,z =0.0), 
                        angular = Vector3(x = 0.0, y = 0.0, z = 0.0))
        
        # If close enough to goal stop moving
        if abs(x)<0.01 and abs(y)<0.01: 
            cmd_2_goal = Twist(linear = Vector3(x = 0.0, y = 0.0 ,z =0.0), 
                angular = Vector3(x = 0.0, y = 0.0, z = 0.0))
            self.state = State.STOPPED
            self.pub_vel.publish(cmd_2_goal)
            
            # Is the turtle back at the home position
            if (self.goal_pose.pose.position.x - self.turtle_init.x) < 0.01 and (self.goal_pose.pose.position.y - self.turtle_init.y) < 0.01:
                self.state = State.BACKHOME
                tilt_arena_bool = Bool()
                tilt_arena_bool.data = True
                if self.caught_flag: # Tilt the platform when back at home position with brick
                    self.pub_tilt_to_arena.publish(tilt_arena_bool)
        else:
            self.state = State.MOVING
            self.pub_vel.publish(cmd_2_goal)
  
    def make_transforms(self):
        """Compute transforms for world to odom frame.
        Modified from link at top of page.
        """
        t = TransformStamped()

        # Give the transform being published a timestamp
        t.header.stamp = self.get_clock().now().to_msg()
        # Name parent frame
        t.header.frame_id = 'world'
        # Set child frame
        t.child_frame_id = "odom"
        
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
        """Compute the heading angle of the robot.
        """
        x = self.goal_pose.pose.position.x - self.turtle_pose.x
        y = self.goal_pose.pose.position.y - self.turtle_pose.y
        heading = math.atan2(y,x)
        self.stem_turn_rads = heading

    def wheel_spin(self):
        """Function to spin the wheel at the proper speed if moving.
        """
        if self.state == State.MOVING:
            self.wheel_turn_vel = self.max_velocity / self.wheel_radius
            self.wheel_turn_rads = self.max_velocity * self.time

    def timer_callback(self):
        """Commands to run every time the timer itterates.
        """
        self.time += 0.01
        
        # Publish joints to joint state publisher
        self.joints.header.stamp = self.get_clock().now().to_msg()
        self.joints.name = ["turn_wheel", "spin_wheel", "base_to_tube", "tilt_platform"]
        self.joints.position = [float(self.stem_turn_rads), float(self.wheel_turn_rads), float(0.0), float(self.platform_tilt_rads)]
        self.joints.velocity = [float(self.stem_turn_vel), float(self.wheel_turn_vel), float(0.0), float(self.platform_tilt_vel)]
        self.pub_joints.publish(self.joints)          
        
        # Define base link and broadcast it
        base_link = TransformStamped()
        base_link.header.frame_id = "odom"
        base_link.child_frame_id = "base_link"
        base_link.transform.translation.x = self.turtle_pose.x - self.odom_x
        base_link.transform.translation.y = self.turtle_pose.y - self.odom_y
        base_link.transform.translation.z = self.base_offset
        quat_base = quaternion_from_euler(float(0), float(0), float(self.turtle_pose.theta)) # Roll pitch yaw
        base_link.transform.rotation.x = quat_base[0]
        base_link.transform.rotation.y = quat_base[1]
        base_link.transform.rotation.z = quat_base[2]
        base_link.transform.rotation.w = quat_base[3]      
        time = self.get_clock().now().to_msg()
        base_link.header.stamp = time
        self.broadcaster.sendTransform(base_link)
        
        # Move turtle towards goal
        if self.go_robot_flag:
            self.get_stem_angle()
            self.wheel_spin()
            self.cmd_vel_to_goal()
        
        # Tilt platform if robot have brick at home position
        if self.state == State.BACKHOME:
            # Tilt
            self.platform_tilt_rads = self.tilt_degrees
            self.platform_tilt_vel = 0.3
        
        # Publish odometry to odom topic        
        self.pub_odom()
            
def main():
    rclpy.init()
    node = TurtleRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()