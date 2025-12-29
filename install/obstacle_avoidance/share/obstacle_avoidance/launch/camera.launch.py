from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            parameters=[{
                'pointcloud.enable': True,
                'pointcloud.ordered_pc': False,
                'depth_module.profile': '640x480x30',
                'rgb_camera.profile': '640x480x30',
                'enable_sync': True,
                'align_depth.enable': True,
            }],
            output='screen'
        )
    ])
