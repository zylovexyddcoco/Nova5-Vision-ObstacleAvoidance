from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 计算后的正确变换: base_link -> camera_link
    
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_base_tf',
            arguments=[
                '--x', '0.900009',
                '--y', '0.568978', 
                '--z', '0.961892',
                '--qx', '-0.216035',
                '--qy', '-0.093703',
                '--qz', '0.870570',
                '--qw', '-0.432038',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link'
            ],
            output='screen'
        )
    ])
