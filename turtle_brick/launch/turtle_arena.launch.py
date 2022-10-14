from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
# Modified code from ROS2 urdf tutorial source code 
# Accessed 10/13/2022
# https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html
# https://index.ros.org/p/urdf_tutorial/

def generate_launch_description():
    #Uses show_turtle.launch.py as appropriate to draw the turtle robot in rviz

    # Start turtle sim node
    arena_node = Node(
            package='turtle_brick',
            executable='arena'
        )

    

    return LaunchDescription([
        arena_node
    ])
