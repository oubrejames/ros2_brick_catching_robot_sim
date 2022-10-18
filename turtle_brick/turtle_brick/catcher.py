import math
from re import T
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
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point, PoseStamped
from turtlesim.msg import Pose
from enum import Enum, auto
from std_msgs.msg import Bool

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

class State(Enum):
    """ States to keep track of where the system is.
    """
    FALLING = auto(),
    INIT = auto(),
    UNDETECTED = auto(),
    UNREACHABLE = auto(),
    REACHABLE = auto()
    
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
        ####TODO TODO TODO Create a publisher in arena that says if brick is there and detect in here if the brick is there for doing stuff
        
        super().__init__('catcher')
        self.state = State.INIT
        self.pub_boundary = self.create_publisher(MarkerArray, "visualization_marker_array", 10) # Marker publisher for boundary of the arena
        self.pub_brick = self.create_publisher(Marker, "visualization_marker", 10) # Marker publisher for brick
        
        self.declare_parameter("platform_height", 0.3,
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
        self.brick_status = False
        self.counter = 0
        self.pub_text = self.create_publisher(Marker, "text_marker", 10) # Marker publisher for text
        self.text_counter = 0
        self.flag = False
        self.broadcaster = TransformBroadcaster(self)
        self.brick_z0 = 20.0 # TODO this is wrong
        self.brick_pose = Point() 
        self.prev_brick_pose = Point()
        #self.brick_init = True
        self.pub_goal_pose = self.create_publisher(PoseStamped, "/goal_pose", 10)

        #TODO Listener 
        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
          'target_frame', 'brick').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.tmr = self.create_timer(0.01, self.timer_callback) 

    def show_text_rviz(self):
        if self.text_counter == 0:
            self.text_marker = Marker()
            self.text_marker.header.frame_id = "/world"
            self.text_marker.header.stamp = self.get_clock().now().to_msg()
            self.text_marker.type = self.text_marker.TEXT_VIEW_FACING
            self.text_marker.text = "Unreachable :("
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
        self.brick_status = data.data
        # print("Brick status",self.brick_status)
        
    def listener_callback(self, msg):
        """Get turtle pose.
        """
        self.turtle_pose = msg
        
    def detect_falling(self):
        """TODO"""
        # Is brick falling?
        
        if self.is_brick_falling():
            self.state = State.FALLING
            
        # Find time until brick is at platform height
        time_to_platform = np.sqrt(self.brick_pose.z/self.gravity)
        distance_to_brick = np.sqrt((self.brick_pose.x-self.turtle_pose.x)**2+(self.brick_pose.y-self.turtle_pose.y)**2)
        # Can robot make it to goal in time?
        #print("Its falling")
        print("Max veloctiy", self.max_velocity)
        print("Time to platform", time_to_platform)
        print("distance_to_brick", distance_to_brick)
        print("vel*time", self.max_velocity*time_to_platform)
        
        if (self.max_velocity*time_to_platform) < distance_to_brick and self.state == State.FALLING:
            # I cant reach!
            self.state = State.UNREACHABLE
            print("I cant reach")  
        elif self.state == State.FALLING:
            self.state = State.REACHABLE
            print("me thinks i can reach") 

    
    def is_brick_falling(self):
        # Should only enter falling state once
        #print("Last z = ", self.last_brick_pose.z, "Current z = ", self.brick_pose)
        if self.prev_brick_pose.z > self.brick_pose.z:
            print("brick is falling")
            self.flag = True
        return self.flag
    
    def publish_goal(self):
        goal_pose = PoseStamped()
        goal_pose.pose.position.x = self.brick_pose.x
        goal_pose.pose.position.y = self.brick_pose.y
        self.pub_goal_pose.publish(goal_pose) 
    
    def did_brick_reset(self):
        # Should only enter falling state once
        #print("Last z = ", self.last_brick_pose.z, "Current z = ", self.brick_pose)
        if self.prev_brick_pose.z < self.brick_pose.z:
            print("brick is reset")
            self.state = State.INIT
            self.flag = False
    
    def listen_to_the_brick(self):
        from_frame_rel = self.target_frame
        to_frame_rel = 'world'        
        

        # try:
        #     self.last_brick_pose = self.brick_pose
        # except:
        #     print("AGHHHHHHHHHHHHHHH")
        #     return
        
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
                
            # self.get_logger().info(
            #     f'Z value {t.transform.translation.z}')
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        # msg = Twist()
        # scale_rotation_rate = 1.0
        # msg.angular.z = scale_rotation_rate * math.atan2(
        #     t.transform.translation.y,
        #     t.transform.translation.x)

        self.brick_pose.x = t.transform.translation.x
        self.brick_pose.y = t.transform.translation.y
        self.brick_pose.z = t.transform.translation.z
    
    def timer_callback(self):
        """
        """
        if self.brick_status:
            self.listen_to_the_brick()
            self.did_brick_reset()  
            
            if self.state == State.INIT:
                self.detect_falling()
                
            if self.state == State.UNREACHABLE:
                self.show_text_rviz()
                
            if self.state == State.REACHABLE:
                print("Publishing goal")
                self.publish_goal()
        
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