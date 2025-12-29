import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 加载MoveIt配置
    moveit_config = MoveItConfigsBuilder(
        "nova5_robot", 
        package_name="nova5_moveit"
    ).to_moveit_configs()
    
    # 获取配置字典
    move_group_config = moveit_config.to_dict()
    
    # 添加octomap配置
    move_group_config.update({
        "octomap_frame": "base_link",
        "octomap_resolution": 0.05,
        "max_range": 2.5,
    })
    
    # 加载传感器配置
    sensors_yaml_path = os.path.join(
        get_package_share_directory("nova5_moveit"),
        "config",
        "sensors_3d.yaml"
    )
    
    with open(sensors_yaml_path, 'r') as f:
        sensors_config = yaml.safe_load(f)
    
    # move_group节点
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            move_group_config,
            sensors_config,
            {
                "publish_monitored_planning_scene": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
            }
        ],
    )
    
    return LaunchDescription([move_group_node])
