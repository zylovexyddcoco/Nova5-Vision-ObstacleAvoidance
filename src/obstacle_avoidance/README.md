# Nova5机械臂避障系统 - 环境配置与TF问题总结

## 系统概述

本系统实现了基于MoveIt2的Nova5机械臂避障功能，使用RealSense D435深度相机采集环境点云，通过OctoMap转换为八叉树地图，接入MoveIt进行碰撞检测和路径规划。

## 技术栈

- Ubuntu 22.04
- ROS2 Humble
- MoveIt2
- 越疆Nova5机械臂 (DOBOT_6Axis_ROS2_V3)
- Intel RealSense D435深度相机
- OctoMap

## 系统架构
```
RealSense D435 → 点云 → OctoMap → Planning Scene → RRT* + FCL碰撞检测 → Nova5执行
```

---

## TF坐标系问题总结

### 问题1：OctoMap提示"octree is empty"

**现象：**
```
[WARN] [octomap_server]: Nothing to publish, octree is empty
```

**原因：**
OctoMap Server订阅的点云话题名称与实际发布的不一致。

**解决方案：**
检查实际点云话题名称：
```bash
ros2 topic list | grep points
```

修改`octomap.launch.py`中的`cloud_in`重映射：
```python
remappings=[
    ('cloud_in', '/camera/camera/depth/color/points')  # 实际话题名
]
```

---

### 问题2：OctoMap丢弃消息 - frame不匹配

**现象：**
```
[INFO] Message Filter dropping message: frame 'camera_depth_optical_frame' at time xxx for reason 'discarding message because the queue is full'
```

**原因：**
OctoMap配置的`frame_id`与点云数据的坐标系之间没有TF变换。

**解决方案：**
1. 确保`frame_id`设置正确（与机械臂基座坐标系一致）
2. 发布相机到机械臂基座的TF变换

---

### 问题3：TF树断开 - 两棵独立的树

**现象：**
```bash
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
# 报错: they are not part of the same tree
```

**原因：**
机械臂和相机的TF树没有连接：
- 机械臂树: `odom → base_footprint → base_link → ...`
- 相机树: `camera_link → camera_depth_frame → ...`

`camera_link`没有父节点，与机械臂树断开。

**解决方案：**
发布静态TF，将`camera_link`连接到`base_link`：
```python
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=[
        '--frame-id', 'base_link',
        '--child-frame-id', 'camera_link',
        # ... 变换参数
    ]
)
```

---

### 问题4：手眼标定结果应用后位置不对

**现象：**
直接使用手眼标定结果发布TF后，OctoMap方块与机械臂位置不对应。

**原因：**
手眼标定得到的是 `base → camera_color_optical_frame` 的变换，但我们需要发布的是 `base_link → camera_link` 的变换。

相机内部存在变换链：
```
camera_link → camera_color_frame → camera_color_optical_frame
```

**解决方案：**
计算正确的变换：
```
T_base_to_camera_link = T_base_to_color_optical × inv(T_camera_link_to_color_optical)
```

步骤：

1. 获取相机内部变换：
```bash
ros2 run tf2_ros tf2_echo camera_link camera_color_optical_frame
```

2. 使用Python计算：
```python
import numpy as np
from scipy.spatial.transform import Rotation as R

# 标定结果: base -> camera_color_optical_frame
t_base_to_color = np.array([x, y, z])  # 标定平移
q_base_to_color = np.array([qx, qy, qz, qw])  # 标定四元数

# 相机内部: camera_link -> camera_color_optical_frame
t_link_to_color = np.array([x, y, z])  # tf2_echo结果
q_link_to_color = np.array([qx, qy, qz, qw])

# 构建变换矩阵
def quat_to_matrix(q, t):
    r = R.from_quat(q)
    T = np.eye(4)
    T[:3, :3] = r.as_matrix()
    T[:3, 3] = t
    return T

T_base_to_color = quat_to_matrix(q_base_to_color, t_base_to_color)
T_link_to_color = quat_to_matrix(q_link_to_color, t_link_to_color)

# 计算 base_link -> camera_link
T_base_to_link = T_base_to_color @ np.linalg.inv(T_link_to_color)
```

3. 提取结果并更新launch文件。

---

## 关键坐标系说明

| 坐标系名称 | 说明 |
|-----------|------|
| `world` | MoveIt全局坐标系 |
| `base_link` | Nova5机械臂基座坐标系 |
| `camera_link` | RealSense相机本体坐标系 |
| `camera_color_frame` | 彩色相机坐标系 |
| `camera_color_optical_frame` | 彩色相机光学坐标系（Z朝前）|
| `camera_depth_frame` | 深度相机坐标系 |
| `camera_depth_optical_frame` | 深度相机光学坐标系（点云所在坐标系）|

**注意：** 
- `camera_link`和`camera_*_optical_frame`相差约90度旋转
- 手眼标定通常得到的是到`optical_frame`的变换
- RealSense驱动会自动发布相机内部各坐标系之间的TF

---

## 常用调试命令

### 查看TF树
```bash
ros2 run tf2_tools view_frames
# 生成frames.pdf
```

### 查看两个坐标系之间的变换
```bash
ros2 run tf2_ros tf2_echo <target_frame> <source_frame>
```

### 查看话题列表
```bash
ros2 topic list | grep <keyword>
```

### 查看话题频率
```bash
ros2 topic hz <topic_name>
```

### 查看已安装的ROS包
```bash
ros2 pkg list | grep <keyword>
```

---

## 启动顺序
```bash
# 终端1：启动相机
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true

# 终端2：启动MoveIt
ros2 launch nova5_moveit demo.launch.py

# 终端3：启动TF变换
ros2 launch obstacle_avoidance camera_tf.launch.py

# 终端4：启动OctoMap
ros2 launch obstacle_avoidance octomap.launch.py

# 终端5：启动Planning Scene桥接
ros2 run obstacle_avoidance octomap_to_planning_scene.py
```

---

## 文件结构
```
obstacle_avoidance/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── octomap_config.yaml
├── launch/
│   ├── camera_tf.launch.py      # 相机到机械臂的TF发布
│   └── octomap.launch.py        # OctoMap Server启动
└── scripts/
    └── octomap_to_planning_scene.py  # OctoMap到Planning Scene桥接
```

---

## 常见问题检查清单

- [ ] 相机是否正常发布点云？(`ros2 topic hz /camera/camera/depth/color/points`)
- [ ] OctoMap是否正常生成？(`ros2 topic hz /octomap_binary`)
- [ ] TF树是否连通？(`ros2 run tf2_tools view_frames`)
- [ ] RViz的Fixed Frame是否设置正确？(应为`base_link`或`world`)
- [ ] 手眼标定结果是否正确转换？(考虑相机内部变换链)
