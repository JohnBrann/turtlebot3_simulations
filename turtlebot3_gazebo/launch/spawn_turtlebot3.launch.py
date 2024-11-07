import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare the launch argument for the model
    turtlebot3_model = DeclareLaunchArgument(
        'model', 
        default_value='burger',
        description='Turtlebot3 Robot Model: burger, waffle, waffle_pi'
    )

    # Declare the launch arguments for x and y positions
    x_pose = DeclareLaunchArgument('x_pose', default_value='0.0')
    y_pose = DeclareLaunchArgument('y_pose', default_value='0.0')

    # Concatenate "turtlebot3_" with the model name using PythonExpression
    model_folder = PythonExpression(["'turtlebot3_' + '", LaunchConfiguration('model'), "'"])

    # Construct the path dynamically with PathJoinSubstitution
    urdf_path = PathJoinSubstitution([
        FindPackageShare('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    ])

    # Use LaunchConfiguration to reference the 'model', 'x_pose', and 'y_pose' arguments
    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', LaunchConfiguration('model'),  # Reference the model value
            '-file', urdf_path,
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', '0.01'
        ],
        output='screen',
    )

    # Create the LaunchDescription and add all actions
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(turtlebot3_model)
    ld.add_action(x_pose)
    ld.add_action(y_pose)

    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd)

    return ld
