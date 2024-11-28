from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('my_robot_package')
    world_path = os.path.join(package_dir, 'worlds', 'sample_world.world')
    urdf_path = os.path.join(package_dir, 'urdf', 'robot_with_camera.urdf')
    

    return LaunchDescription([
        # Launch Gazebo with ROS
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(

                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_path}.items()
        ),

        # Spawn the robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'differential_bot_with_camera', '-file','urdf/obot_with_camera.urdf'],
            output='screen'
            
        ),
    ])

