import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package path and construct URDF file path manually
    package_share_directory = get_package_share_directory('turtlebot3_gazebo')
    urdf_file_path = os.path.join(package_share_directory, 'urdf', 'turtlebot3_burger.urdf')

    # Open and read the URDF file
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    model_name = LaunchConfiguration('model')
    model_path = LaunchConfiguration('model_path')
    robot_state_publisher_model = LaunchConfiguration('robot_state_publisher_model')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'model', 
            default_value='burger',
            description='Specify robot model'),
        DeclareLaunchArgument(
            'model_path',
            default_value='turtlebot3_burger',
            description='Robot Model Folder'), 
        DeclareLaunchArgument(
            'robot_state_publisher_model', default_value='turtlebot3_burger.urdf',
            description='robot model name in robot_state_publisher'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }],
        ),
    ])
