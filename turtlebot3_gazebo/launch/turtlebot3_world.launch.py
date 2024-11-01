import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_robot_common_sim = get_package_share_directory('robot-common-sim')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # Declare the launch argument for the world name
    world_arg = DeclareLaunchArgument(
        'world', default_value='4x4m_empty.world',
        description='Name of the Gazebo world to load'
    )

    # Path to the worlds directory
    world_dir = os.path.join(pkg_robot_common_sim, 'worlds')

    # Declare launch argument for simulation time
    sim_time = DeclareLaunchArgument(
       'use_sim_time', default_value='true',
       description='Use simulation time'
    )

    # Declare launch arguments for spawn position
    x_position = DeclareLaunchArgument(
        'x_pose', default_value='-1.8',
        description='X position of the robot'
    )
    y_position = DeclareLaunchArgument(
        'y_pose', default_value='-1.8',
        description='Y position of the robot'
    )

    # Include Gazebo server launch with world substitution
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': [world_dir + '/', LaunchConfiguration('world')]}.items()
    )

    # Include Gazebo client launch
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        launch_arguments={'world': [world_dir + '/', LaunchConfiguration('world')]}.items()
    )

    # Include the robot state publisher
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # Spawn TurtleBot3 with x and y positions
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose')
        }.items()
    )

    # Define the launch description
    ld = LaunchDescription()

    # Add arguments and actions to the launch description
    ld.add_action(world_arg)
    ld.add_action(sim_time)
    ld.add_action(x_position)
    ld.add_action(y_position)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    return ld
