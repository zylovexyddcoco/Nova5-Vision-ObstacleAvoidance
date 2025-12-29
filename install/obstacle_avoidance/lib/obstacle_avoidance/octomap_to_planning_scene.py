#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, PlanningSceneWorld
from octomap_msgs.msg import Octomap

class OctomapToPlanningScene(Node):
    def __init__(self):
        super().__init__('octomap_to_planning_scene')
        
        # 订阅OctoMap
        self.octomap_sub = self.create_subscription(
            Octomap,
            '/octomap_binary',
            self.octomap_callback,
            10
        )
        
        # 发布到Planning Scene
        self.planning_scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )
        
        self.get_logger().info('OctoMap to Planning Scene node started')
    
    def octomap_callback(self, msg):
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.world.octomap.header = msg.header
        planning_scene.world.octomap.octomap = msg
        
        self.planning_scene_pub.publish(planning_scene)
        self.get_logger().info('Published OctoMap to Planning Scene', throttle_duration_sec=5.0)

def main(args=None):
    rclpy.init(args=args)
    node = OctomapToPlanningScene()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
