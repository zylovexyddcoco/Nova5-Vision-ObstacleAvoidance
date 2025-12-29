#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
轨迹执行服务器 - 保持角度连续性版本
确保关节角度连续变化，避免不必要的大幅旋转
'''
import time
import math
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from dobot_msgs_v3.srv import JointMovJ, Sync, GetAngle
import os


class FollowJointTrajectoryServer(Node):
    def __init__(self):
        super().__init__('dobot_group_controller')
        name = os.getenv("DOBOT_TYPE", "nova5")
        
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            f'/{name}_group_controller/follow_joint_trajectory',
            self.execute_callback
        )
        self.get_logger().info("FollowJointTrajectory Action Server (Continuous Angle) is ready...")
        
        # 服务客户端
        self.JointMovJ_client = self.create_client(JointMovJ, '/dobot_bringup_v3/srv/JointMovJ')
        self.Sync_client = self.create_client(Sync, '/dobot_bringup_v3/srv/Sync')
        self.GetAngle_client = self.create_client(GetAngle, '/dobot_bringup_v3/srv/GetAngle')
        
        # ========== 配置参数 ==========
        # 轨迹点采样：每隔N个点取一个
        self.point_sample_rate = 3
        
        # 最小角度变化阈值（度）
        self.min_angle_change = 2.0
        
        # 是否沿路径执行
        self.follow_path = True
        # ==============================
        
        while not self.JointMovJ_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for JointMovJ service...')
        
        self.get_logger().info(f'Config: follow_path={self.follow_path}, '
                               f'sample_rate={self.point_sample_rate}')

    async def execute_callback(self, goal_handle):
        self.get_logger().info("=" * 50)
        self.get_logger().info("Received new trajectory goal!")
        
        trajectory = goal_handle.request.trajectory
        success = self.execute_trajectory(trajectory)
        
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = 0 if success else -1
        return result

    def rad_to_deg(self, rad):
        """弧度转角度"""
        return 180.0 * rad / math.pi

    def get_current_angles(self):
        """获取机械臂当前角度"""
        try:
            if self.GetAngle_client.wait_for_service(timeout_sec=1.0):
                future = self.GetAngle_client.call_async(GetAngle.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                if future.result() is not None and future.result().res == 0:
                    # 解析角度字符串 '{j1,j2,j3,j4,j5,j6}'
                    angle_str = future.result().angle
                    angles = angle_str.strip('{}').split(',')
                    return [float(a) for a in angles]
        except Exception as e:
            self.get_logger().warn(f'GetAngle error: {e}')
        return None

    def find_nearest_angle(self, target, reference):
        """
        找到与参考角度最近的等效目标角度
        target: 目标角度（可能需要调整）
        reference: 参考角度（当前位置）
        返回: 调整后的目标角度，使运动最短
        """
        # 计算差值
        diff = target - reference
        
        # 将差值归一化到 [-180, 180]
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        
        # 返回调整后的目标角度
        return reference + diff

    def make_continuous_trajectory(self, all_points, start_angles):
        """
        调整轨迹点使角度连续变化
        all_points: 所有轨迹点（角度，已从弧度转换）
        start_angles: 起始角度（机械臂当前位置）
        返回: 调整后的连续轨迹
        """
        if len(all_points) == 0:
            return []
        
        continuous = []
        
        # 使用起始角度作为第一个参考
        if start_angles:
            reference = start_angles.copy()
        else:
            reference = all_points[0].copy()
        
        for point in all_points:
            adjusted = []
            for i in range(6):
                # 找到最近的等效角度
                adj_angle = self.find_nearest_angle(point[i], reference[i])
                adjusted.append(adj_angle)
            
            continuous.append(adjusted)
            reference = adjusted.copy()  # 更新参考为当前点
        
        return continuous

    def max_angle_diff(self, pos1, pos2):
        """计算两组关节角度的最大差值"""
        max_diff = 0.0
        for a, b in zip(pos1, pos2):
            diff = abs(a - b)
            if diff > max_diff:
                max_diff = diff
        return max_diff

    def select_key_points(self, all_points):
        """从所有轨迹点中选择关键点"""
        if len(all_points) <= 2:
            return all_points
        
        key_points = [all_points[0]]  # 起点
        last_key = all_points[0]
        
        for i in range(1, len(all_points) - 1):
            # 采样率过滤
            if i % self.point_sample_rate != 0:
                continue
            
            # 角度变化过滤
            if self.max_angle_diff(last_key, all_points[i]) >= self.min_angle_change:
                key_points.append(all_points[i])
                last_key = all_points[i]
        
        # 终点
        key_points.append(all_points[-1])
        
        return key_points

    def wait_motion_complete(self):
        """等待当前运动完成"""
        try:
            if self.Sync_client.wait_for_service(timeout_sec=0.5):
                future = self.Sync_client.call_async(Sync.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
                return True
        except Exception as e:
            self.get_logger().warn(f'Sync error: {e}')
        return False

    def send_joint_movj(self, angles):
        """发送JointMovJ并等待完成"""
        req = JointMovJ.Request()
        req.j1, req.j2, req.j3 = float(angles[0]), float(angles[1]), float(angles[2])
        req.j4, req.j5, req.j6 = float(angles[3]), float(angles[4]), float(angles[5])
        
        # 发送命令
        future = self.JointMovJ_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            res = future.result().res
            if res == 0:
                return True
            else:
                self.get_logger().error(f'JointMovJ failed: error={res}')
                return False
        else:
            self.get_logger().error('JointMovJ timeout')
            return False

    def execute_trajectory(self, trajectory: JointTrajectory):
        """执行轨迹"""
        total_points = len(trajectory.points)
        self.get_logger().info(f"Trajectory has {total_points} points")
        
        if total_points == 0:
            self.get_logger().warn("Empty trajectory!")
            return False
        
        # 获取当前机械臂角度
        current_angles = self.get_current_angles()
        if current_angles:
            self.get_logger().info(f"Current: [{', '.join(f'{a:.1f}' for a in current_angles)}]")
        
        # 转换所有点：弧度 → 角度
        all_positions = []
        for point in trajectory.points:
            angles = [self.rad_to_deg(r) for r in point.positions]
            all_positions.append(angles)
        
        # 打印原始起点和终点
        self.get_logger().info(f"Raw Start: [{', '.join(f'{a:.1f}' for a in all_positions[0])}]")
        self.get_logger().info(f"Raw Goal:  [{', '.join(f'{a:.1f}' for a in all_positions[-1])}]")
        
        # 使轨迹角度连续化（关键步骤！）
        continuous_positions = self.make_continuous_trajectory(all_positions, current_angles)
        
        # 打印调整后的起点和终点
        self.get_logger().info(f"Adj Start: [{', '.join(f'{a:.1f}' for a in continuous_positions[0])}]")
        self.get_logger().info(f"Adj Goal:  [{', '.join(f'{a:.1f}' for a in continuous_positions[-1])}]")
        
        if self.follow_path:
            # ========== 沿路径执行 ==========
            key_points = self.select_key_points(continuous_positions)
            self.get_logger().info(f"Selected {len(key_points)} key points")
            
            for i, pos in enumerate(key_points):
                self.get_logger().info(f"[{i+1}/{len(key_points)}] → "
                                       f"[{', '.join(f'{a:.1f}' for a in pos)}]")
                
                if not self.send_joint_movj(pos):
                    self.get_logger().error(f"Failed at point {i+1}")
                    return False
                
                self.wait_motion_complete()
        else:
            # ========== 只到终点 ==========
            final = continuous_positions[-1]
            self.get_logger().info(f"Direct to: [{', '.join(f'{a:.1f}' for a in final)}]")
            
            if not self.send_joint_movj(final):
                return False
            
            self.wait_motion_complete()
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("Trajectory completed!")
        self.get_logger().info("=" * 50)
        return True


def main(args=None):
    rclpy.init(args=args)
    server = FollowJointTrajectoryServer()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
