from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
# Modified code from ROS2 urdf tutorial source code
# Accessed 10/13/2022
# https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html
# https://index.ros.org/p/urdf_tutorial/


def generate_launch_description():

    turtle_brick_path = get_package_share_path('turtle_brick')
    turtle_param_path = turtle_brick_path / 'turtle.yaml'

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node'
    )

    run_turtle_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtle_brick')),
            '/show_turtle.launch.py'])
    )

    # Start turtle robot node
    turtle_robot_node = Node(
        package='turtle_brick',
        executable='turtle_robot',
        parameters=[turtle_param_path]
    )

    return LaunchDescription([
        turtle_robot_node,
        run_turtle_node,
        turtlesim_node
    ])
