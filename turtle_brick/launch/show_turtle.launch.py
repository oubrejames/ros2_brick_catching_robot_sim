from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals

# Modified code from ROS2 urdf tutorial source code
# Accessed 10/9/2022
# https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html
# https://index.ros.org/p/urdf_tutorial/


def generate_launch_description():
    turtle_brick_path = get_package_share_path('turtle_brick')
    default_model_path = turtle_brick_path / 'turtle.urdf.xacro'
    default_rviz_config_path = turtle_brick_path / 'urdf.rviz'

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        choices=[
            'true',
            'false'],
        description='Flag to enable joint_state_publisher_gui')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(
        default_model_path), description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file')

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

    DeclareLaunchArgument(
        'joint_states',
        default_value='gui',
        choices=[
            'gui',
            'jsp',
            'none'],
        description="Option to choose a joint state publisher")

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=(LaunchConfigurationEquals('joint_states', 'jsp'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=(LaunchConfigurationEquals('joint_states', 'gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
