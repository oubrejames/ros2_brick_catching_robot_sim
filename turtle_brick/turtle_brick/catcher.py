import math
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from rcl_interfaces.msg import ParameterDescriptor
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Point, PoseStamped
from turtlesim.msg import Pose
from enum import Enum, auto
from std_msgs.msg import Bool

# Modified code from ROS2 static broadcater tutorial source code 
# Accessed 10/9/2022
# https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.html

def quaternion_from_euler(ai, aj, ak):
    """Takes in Euler angles and converts them to quaternions. 
    Function taken from link above.

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
    LISTENING = auto(),
    UNREACHABLE = auto(),
    REACHABLE = auto(),
    CAUGHT = auto(),
    RESET = auto()
    
class Catcher(Node):
    """
    Node to see if it is possible to catch the brick and take appropriate actions if it can or cannot.
    """

    def __init__(self):
        """Initialize intitial variables, states, publishers and subscribers.
        """
        # Initialize node with name turtle_robot.        
        super().__init__('catcher')
        self.state = State.INIT
        self.pub_boundary = self.create_publisher(MarkerArray, "visualization_marker_array", 10) # Marker publisher for boundary of the arena
        self.pub_brick = self.create_publisher(Marker, "visualization_marker", 10) # Marker publisher for brick
        
        self.declare_parameter("platform_height", 1.3,
                               ParameterDescriptor(description="The height of the turtle robot's platform in meters"))
        self.platform_h  = self.get_parameter("platform_height").get_parameter_value().double_value     
        
        self.declare_parameter("gravity", 9.8,
                               ParameterDescriptor(description="The brick's acceleration due to gravity"))
        self.gravity  = self.get_parameter("gravity").get_parameter_value().double_value    
               
        self.declare_parameter("max_velocity", 0.4,
                               ParameterDescriptor(description="The maximum velocity of the turtle robot in meters/sec"))
        self.max_velocity  = self.get_parameter("max_velocity").get_parameter_value().double_value
        
        self.sub = self.create_subscription(Pose, "turtle1/pose", self.listener_callback, 10)
        self.sub_brick_status = self.create_subscription(Bool, "brick_status", self.brick_status_callback, 10)
        self.sub_reset = self.create_subscription(Bool, "reset_sim", self.reset_callback, 10)
        self.pub_send_turtle_robot = self.create_publisher(Bool, "send_turtle_robot", 10)
        self.pub_is_brick_caught = self.create_publisher(Bool, "brick_caught", 10)
        self.turtle_init = PoseStamped()
        self.turtle_init_flag = True
        self.brick_status = False
        self.counter = 0
        self.pub_text = self.create_publisher(Marker, "text_marker", 10) # Marker publisher for text
        self.text_counter = 0
        self.flag = False
        self.broadcaster = TransformBroadcaster(self)
        self.brick_z0 = 20.0 
        self.brick_pose = Point() 
        self.prev_brick_pose = Point()
        self.pub_goal_pose = self.create_publisher(PoseStamped, "/goal_pose", 10)

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
          'target_frame', 'brick').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.tmr = self.create_timer(0.01, self.timer_callback) 

    def reset_callback(self, msg):
        """Callback that listens to the reset_sim topic to see if the simulation must be reset

        Args:
            msg (Bool): Describes if system must be reset
        """
        if msg.data:
            self.state = State.INIT
            self.flag = False
            self.__init__()

    def show_text_rviz(self):
        """Publishes text marker on RVIZ
        """
        if self.text_counter == 0:
            self.text_marker = Marker()
            self.text_marker.header.frame_id = "/world"
            self.text_marker.header.stamp = self.get_clock().now().to_msg()
            self.text_marker.type = self.text_marker.TEXT_VIEW_FACING
            self.text_marker.text = "Unreachable"
            self.text_marker.id =1
            self.text_marker.action = self.text_marker.ADD
            self.text_marker.scale.x = 1.0
            self.text_marker.scale.y = 3.0
            self.text_marker.scale.z = 1.0
            self.text_marker.color.a = 1.0
            self.text_marker.lifetime.sec = 3
            trans = quaternion_from_euler(0, 0, 0)
            self.text_marker.pose.orientation.w = trans[0]
            self.text_marker.pose.orientation.x = trans[1]
            self.text_marker.pose.orientation.y = trans[2]
            self.text_marker.pose.orientation.z = trans[3]
            self.text_marker.pose.position.x = 5.5
            self.text_marker.pose.position.y = 5.5
            self.text_marker.pose.position.z = 0.5
            self.pub_text.publish(self.text_marker)
            self.text_counter += 1
        
    def brick_status_callback(self, data):
        """Subcribes to the brick_status topic to see if the brick is present or not.

        Args:
            data (Bool): Indicates presence of a brick
        """
        self.brick_status = data.data
        
    def listener_callback(self, msg):
        """Get turtle pose.
        """
        if self.turtle_init_flag:
            self.turtle_init = msg
            self.turtle_init_flag = False
            
        self.turtle_pose = msg
        
    def detect_falling(self):
        """Detects if the brick is falling and changes states.
        """
        # Is brick falling?
        
        if self.is_brick_falling():
            self.state = State.FALLING
            
        # Find time until brick is at platform height
        time_to_platform = np.sqrt(self.brick_pose.z/self.gravity)
        distance_to_brick = np.sqrt((self.brick_pose.x-self.turtle_pose.x)**2+(self.brick_pose.y-self.turtle_pose.y)**2)
        # Can robot make it to goal in time?        
        if (self.max_velocity*time_to_platform) < distance_to_brick and self.state == State.FALLING:
            self.state = State.UNREACHABLE
        elif self.state == State.FALLING:
            self.state = State.REACHABLE
    
    def is_brick_falling(self):
        """Math to detect falling.

        Returns:
            bool: Sasys if the brick is falling
        """
        if self.prev_brick_pose.z > self.brick_pose.z:
            self.flag = True
        return self.flag
    
    def publish_goal(self):
        """Publishes the robots end goal.
        """
        goal_pose = PoseStamped()
        goal_pose.pose.position.x = self.brick_pose.x
        goal_pose.pose.position.y = self.brick_pose.y
        self.pub_goal_pose.publish(goal_pose) 
    
    def did_brick_reset(self):
        """Checks to see if the brick has reset in its original position.
        """
        if self.prev_brick_pose.z < self.brick_pose.z:
            self.state = State.INIT
            self.flag = False
    
    def is_brick_caught(self):
        """Checks if the brick has been caught and changes states if so.
        """
        zdiff = abs(self.brick_pose.z - self.platform_h)

        if  zdiff < 0.15: # Is brick caught?
            self.state = State.CAUGHT
   
    def listen_to_the_brick(self):
        """Finds the relationship between the world frame and brick frame and updates the bricks pose.
        """
        from_frame_rel = self.target_frame
        to_frame_rel = 'world'        
                
        # Look up for the transformation between target_frame and turtle2 frames
        # and send velocity commands for turtle2 to reach target_frame
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            if self.counter == 0:
                self.prev_brick_pose =  t.transform.translation
            self.counter += 1
            
            if self.counter > 100:
                self.counter = 0
                
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
  
        self.brick_pose.x = t.transform.translation.x
        self.brick_pose.y = t.transform.translation.y
        self.brick_pose.z = t.transform.translation.z
    
    def timer_callback(self):
        """Function to interate every time the timer is called.
        """
        
        if self.brick_status:
            self.listen_to_the_brick()
            
            if self.state == State.INIT:
                self.detect_falling()
                
            if self.state == State.UNREACHABLE:
                self.show_text_rviz()
                
            if self.state == State.REACHABLE:
                self.publish_goal()
                
                # Tell the robot node to move
                go_robot = Bool()
                go_robot.data = True
                self.pub_send_turtle_robot.publish(go_robot)
                self.is_brick_caught()
                
            if self.state == State.CAUGHT:
                home = PoseStamped()
                home.pose.position.x = self.turtle_init.x
                home.pose.position.y = self.turtle_init.y
                self.pub_goal_pose.publish(home) 
                caught_flag = Bool()
                caught_flag.data = True
                self.pub_is_brick_caught.publish(caught_flag)
        
def main():
    rclpy.init()
    node = Catcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()