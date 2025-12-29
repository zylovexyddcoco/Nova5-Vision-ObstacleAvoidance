import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    obstacle_avoidance_dir = get_package_share_directory('obstacle_avoidance')
    nova5_moveit_dir = get_package_share_directory('nova5_moveit')
    
    # 1. TF变换
    camera_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(obstacle_avoidance_dir, 'launch', 'camera_tf.launch.py')
        )
    )
    
    # 2. Robot State Publisher 和 静态TF
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nova5_moveit_dir, 'launch', 'rsp.launch.py')
        )
    )
    
    # 3. Move Group（带传感器配置）
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(obstacle_avoidance_dir, 'launch', 'move_group_sensors.launch.py')
        )
    )
    
    # 4. RViz
    rviz_config = os.path.join(nova5_moveit_dir, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        camera_tf,
        robot_state_publisher,
        move_group,
        rviz_node,
    ])
