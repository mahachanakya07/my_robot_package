from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='d435_camera',
            parameters=[{
                'depth_module.profile': '640x480x30',
                'rgb_camera.profile': '640x480x30',
                'enable_pointcloud': True
            }],
            output='screen'
        ),
    ])
