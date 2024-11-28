from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the share directory of the package
    package_dir = get_package_share_directory('my_robot_package')
    world_path = os.path.join(package_dir, 'worlds', 'sample_world.world')
    urdf_path = os.path.join(package_dir, 'urdf', 'robot_with_camera.urdf')
    )


    # Create the launch description
    return LaunchDescription([
        
        # Gazebo launch
        Node(
            package='gazebo_ros',
            executable='gzserver',
            arguments=[world_path],
            output='screen'
        ),
        
        Node(
            package='gazebo_ros',
            executable='gzclient',
            arguments=[],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen"
        )

        # Spawn the robot into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'differential_bot_with_camera', '-file', 'urdf/obot_with_camera.urdf'],
            output='screen'
        ),
    ])





