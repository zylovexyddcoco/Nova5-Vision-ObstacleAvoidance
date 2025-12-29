#!/usr/bin/env python3
"""
动态自身过滤节点 - 调优版
降低滤波强度，保留更多有效点云
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import numpy as np
from tf2_ros import Buffer, TransformListener

class SelfFilter(Node):
    def __init__(self):
        super().__init__('self_filter')
        
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Nova5的连杆名称
        self.link_names = ['Link1', 'Link2', 'Link3', 'Link4', 'Link5', 'Link6']
        
        # 每个连杆的过滤半径（米）
        self.link_radius = {
            'Link1': 0.20, 'Link2': 0.28, 'Link3': 0.36,
            'Link4': 0.30, 'Link5': 0.30, 'Link6': 0.30,
        }
        self.default_radius = 0.18
        self.base_filter_radius = 0.25
        self.base_filter_height = 0.35
        
        # 降采样参数
        self.subsample_rate = 2  # 保留1/2的点
        
        # 距离过滤参数
        self.min_distance = 0.15  # 最小距离
        self.max_distance = 1.5   # 最大距离
        
        # 聚类滤波参数（只保留这一个滤波，去除孤立噪点）
        self.enable_cluster_filter = True
        self.cluster_min_points = 5    # 降低阈值，从15改为5
        self.cluster_radius = 0.02     # 减小半径，从0.03改为0.02
        
        # 订阅原始点云
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.pointcloud_callback,
            10
        )
        
        # 发布过滤后的点云
        self.pc_pub = self.create_publisher(
            PointCloud2,
            '/filtered_points',
            10
        )
        
        # 存储连杆位置
        self.link_positions = {}
        
        # 缓存TF变换
        self.cached_rot_matrix = None
        self.cached_translation = None
        
        # 定时更新
        self.create_timer(0.03, self.update_link_positions)
        
        self.get_logger().info('Self Filter Node Started (Tuned Version)')

    def update_link_positions(self):
        """更新连杆位置和TF缓存"""
        for link_name in self.link_names:
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_link', link_name,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.03)
                )
                self.link_positions[link_name] = np.array([
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                ])
            except Exception:
                pass
        
        # 更新相机TF缓存
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'camera_depth_optical_frame',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.03)
            )
            t = transform.transform.translation
            r = transform.transform.rotation
            qx, qy, qz, qw = r.x, r.y, r.z, r.w
            
            self.cached_rot_matrix = np.array([
                [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
                [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
                [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
            ], dtype=np.float32)
            self.cached_translation = np.array([t.x, t.y, t.z], dtype=np.float32)
        except Exception:
            pass

    def cluster_filter(self, points, min_points=5, radius=0.02):
        """轻量级聚类滤波：只去除非常孤立的点"""
        if len(points) < min_points * 2:
            return points
        
        # 使用体素网格加速
        voxel_size = radius
        voxel_indices = np.floor(points / voxel_size).astype(np.int32)
        
        # 统计每个体素的点数
        voxel_counts = {}
        for idx in voxel_indices:
            key = tuple(idx)
            voxel_counts[key] = voxel_counts.get(key, 0) + 1
        
        # 检查每个体素及其邻居的总点数
        mask = np.ones(len(points), dtype=bool)
        for i, idx in enumerate(voxel_indices):
            total_neighbors = 0
            # 检查3x3x3邻域
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    for dz in [-1, 0, 1]:
                        neighbor_key = (idx[0]+dx, idx[1]+dy, idx[2]+dz)
                        total_neighbors += voxel_counts.get(neighbor_key, 0)
            
            if total_neighbors < min_points:
                mask[i] = False
        
        return points[mask]

    def pointcloud_callback(self, msg):
        try:
            if len(self.link_positions) == 0 or self.cached_rot_matrix is None:
                return
            
            # 读取点云并降采样
            points_list = []
            for i, point in enumerate(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
                if i % self.subsample_rate == 0:
                    x, y, z = point[0], point[1], point[2]
                    # 距离过滤
                    dist = np.sqrt(x*x + y*y + z*z)
                    if self.min_distance < dist < self.max_distance:
                        points_list.append([x, y, z])
            
            if len(points_list) < 50:
                return
            
            points = np.array(points_list, dtype=np.float32)
            original_count = len(points)
            
            # 轻量级聚类滤波（只去除孤立点）
            if self.enable_cluster_filter and len(points) > 100:
                points = self.cluster_filter(
                    points,
                    min_points=self.cluster_min_points,
                    radius=self.cluster_radius
                )
            
            after_cluster = len(points)
            
            if len(points) < 10:
                return
            
            # 坐标变换到base_link
            points_base = (self.cached_rot_matrix @ points.T).T + self.cached_translation
            
            # 机械臂自身过滤
            mask = np.ones(len(points), dtype=bool)
            
            # 基座过滤
            dist_xy = np.sqrt(points_base[:, 0]**2 + points_base[:, 1]**2)
            base_mask = (dist_xy < self.base_filter_radius) & (points_base[:, 2] < self.base_filter_height)
            mask[base_mask] = False
            
            # 连杆过滤
            for link_name, link_pos in self.link_positions.items():
                radius = self.link_radius.get(link_name, self.default_radius)
                diff = points_base - link_pos
                distances_sq = np.sum(diff**2, axis=1)
                mask[distances_sq < radius**2] = False
            
            filtered_points = points[mask]
            
            # 日志
            self.get_logger().info(
                f'Points: {original_count} -> cluster:{after_cluster} -> self:{len(filtered_points)}',
                throttle_duration_sec=2.0
            )
            
            if len(filtered_points) < 10:
                return
            
            # 发布
            header = Header()
            header.stamp = msg.header.stamp
            header.frame_id = msg.header.frame_id
            filtered_msg = pc2.create_cloud_xyz32(header, filtered_points.tolist())
            self.pc_pub.publish(filtered_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error: {e}', throttle_duration_sec=5.0)

def main(args=None):
    rclpy.init(args=args)
    node = SelfFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
