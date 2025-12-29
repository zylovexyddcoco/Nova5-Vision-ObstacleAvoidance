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
                # 点云设置
                'pointcloud.enable': True,
                'pointcloud.ordered_pc': False,
                
                # 深度模块设置
                'depth_module.profile': '640x480x30',
                'depth_module.enable_auto_exposure': True,
                
                # 深度质量设置
                'depth_module.visual_preset': 4,  # 4 = High Density
                
                # RGB设置
                'rgb_camera.profile': '640x480x30',
                'rgb_camera.enable_auto_exposure': True,
                
                # 对齐和同步
                'align_depth.enable': True,
                'enable_sync': True,
                
                # 滤波器（相机内置）
                'spatial_filter.enable': True,
                'spatial_filter.smooth_alpha': 0.5,
                'spatial_filter.smooth_delta': 20,
                
                'temporal_filter.enable': True,
                'temporal_filter.smooth_alpha': 0.4,
                'temporal_filter.smooth_delta': 20,
                
                'hole_filling_filter.enable': False,  # 关闭，避免填充错误
                
                'decimation_filter.enable': False,  # 我们自己做降采样
            }],
            output='screen'
        )
    ])
