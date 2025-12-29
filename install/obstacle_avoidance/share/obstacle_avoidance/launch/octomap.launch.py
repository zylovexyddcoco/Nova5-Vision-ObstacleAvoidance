from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 调优版自身过滤节点
        Node(
            package='obstacle_avoidance',
            executable='self_filter.py',
            name='self_filter',
            output='screen'
        ),
        # OctoMap Server
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            parameters=[{
                'resolution': 0.03,            # 3cm方块，平衡精度和性能
                'frame_id': 'base_link',
                'sensor_model.max_range': 1.5,
                'sensor_model.hit': 0.6,       # 适中
                'sensor_model.miss': 0.45,     # 适中
                'sensor_model.min': 0.12,
                'sensor_model.max': 0.97,
                'latch': False,
                'filter_ground': False,
                'filter_speckles': True,       # 开启OctoMap自带的噪点过滤
                'compress_map': False,
                'occupancy_min_z': -0.5,
                'occupancy_max_z': 1.0,
            }],
            remappings=[
                ('cloud_in', '/filtered_points')
            ],
            output='screen'
        )
    ])
