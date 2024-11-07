import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
def generate_launch_description():
    # Declare launch arguments
    robot_model = DeclareLaunchArgument(
        'model',
        default_value='burger',
        description='Specify robot model'
    )
    robot_model_folder = DeclareLaunchArgument(
        'model_path',
        default_value='turtlebot3_burger',
        description='Robot Model Folder'
    )
    # Get the model name from the launch configuration
    model_name = LaunchConfiguration('model')
    model_path = LaunchConfiguration('model_path')
    # Create the path using PathJoinSubstitution
    urdf_path = PathJoinSubstitution([
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_path,
        'model.sdf'
    ])
    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Specify x position of the robot'
    )
    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Specify y position of the robot'
    )
    # Create the spawn entity node
    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', model_name,
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen',
    )
    # Create and return launch description
    ld = LaunchDescription()
    # Add all launch arguments
    ld.add_action(robot_model)
    ld.add_action(robot_model_folder)
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    # Add the spawn command
    ld.add_action(start_gazebo_ros_spawner_cmd)
    return ld