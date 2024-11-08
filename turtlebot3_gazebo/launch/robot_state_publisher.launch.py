import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the robot model argument
    robot_state_publisher_model = DeclareLaunchArgument(
        'robot_state_publisher_model',
        default_value='turtlebot3_burger.urdf',
        description='Robot model name in robot_state_publisher'
    )

    # Use LaunchConfiguration for the model file name
    model_file_name = LaunchConfiguration('robot_state_publisher_model')

    # URDF path as a command substitution for robot_description
    urdf_path = PathJoinSubstitution([
        get_package_share_directory('turtlebot3_gazebo'),
        'urdf',
        model_file_name,
    ])

    # Define use_sim_time launch argument
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Define the robot_state_publisher node with the robot_description from the URDF path
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['cat ', urdf_path])  # Use Command substitution to read file content
        }],
    )

    # Create the launch description and add actions
    ld = LaunchDescription()

    # Add declared arguments and nodes
    ld.add_action(robot_state_publisher_model)
    ld.add_action(robot_state_publisher_node)

    return ld
