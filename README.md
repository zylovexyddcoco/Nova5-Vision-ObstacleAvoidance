# Nova5-Vision-ObstacleAvoidance

基于ROS2和MoveIt2的Dobot Nova5机械臂3D视觉避障系统

![Weixin Image_20251229200601_1_46](https://github.com/user-attachments/assets/dcf5bc66-d60d-4898-a03d-0f594112b425)
![demo](https://github.com/user-attachments/assets/351b3b5e-bdfa-4cd5-9e1d-e1d277f8e65d)


## 项目简介

本项目实现了Dobot Nova5六轴协作机械臂的实时视觉避障功能。系统采用Intel RealSense D435深度相机采集环境点云，通过OctoMap构建3D占据栅格地图，并与MoveIt2运动规划框架集成，实现机械臂在动态环境中的自主避障路径规划与轨迹执行。

## 系统架构

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│  RealSense  │───▶│  自身过滤   │───▶│   OctoMap   │───▶│   MoveIt2   │
│    D435     │    │ Self-Filter │    │   Server    │    │  Planning   │
└─────────────┘    └─────────────┘    └─────────────┘    └──────┬──────┘
                                                                │
┌─────────────┐    ┌─────────────┐    ┌─────────────┐           │
│   Nova5     │◀───│   Dobot     │◀───│  Trajectory │◀──────────┘
│   Robot     │    │   Driver    │    │  Execution  │
└─────────────┘    └─────────────┘    └─────────────┘
```

**工作流程**：RealSense D435深度相机以30Hz采集环境点云 → 自身过滤节点剔除机械臂本体点云 → OctoMap基于八叉树构建概率占据地图 → MoveIt2接收障碍物信息进行碰撞检测 → OMPL规划器生成无碰撞轨迹 → Action Server逐点执行关节运动指令

## 功能特性

- ✅ **实时环境感知**：RealSense D435深度点云采集与预处理
- ✅ **动态自身过滤**：根据关节状态实时过滤机械臂本体点云
- ✅ **3D障碍物建图**：OctoMap概率栅格地图，支持动态更新
- ✅ **避障路径规划**：MoveIt2 + OMPL实现配置空间无碰撞规划
- ✅ **轨迹路径执行**：沿规划路径逐点运动，保持角度连续性
- ✅ **拖拽模式控制**：支持手动拖拽机械臂复位

## 环境要求

| 依赖项 | 版本要求 |
|--------|----------|
| Ubuntu | 22.04 LTS |
| ROS2 | Humble |
| MoveIt2 | 2.5+ |
| Python | 3.10+ |
| OpenCV | 4.5+ |

### 硬件要求

- Dobot Nova5 六轴协作机械臂
- Intel RealSense D435/D435i 深度相机
- 以太网连接（机械臂IP：192.168.5.1）

## 安装步骤

### 1. 安装ROS2

```
bash
#1. 设置编码
$ sudo apt update && sudo apt install locales
$ sudo locale-gen en_US en_US.UTF-8
$ sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 
$ export LANG=en_US.UTF-8
#2. 添加源
$ sudo apt update && sudo apt install curl gnupg lsb-release 
$ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg 
$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
如遇报错“Failed to connect to raw.githubusercontent.com”，可参考https://www.guyuehome.com/37844

#3. 安装ROS2
$ sudo apt update
$ sudo apt upgrade
$ sudo apt install ros-humble-desktop
#4. 设置环境变量
$ source /opt/ros/humble/setup.bash
$ echo " source /opt/ros/humble/setup.bash" >> ~/.bashrc 
至此，ROS2就已经在系统中安装好了。

ROS2示例测试
为了验证ROS2安装成功，我们可以通过以下示例进行测试。

#示例一：命令行示例
启动第一个终端，通过以下命令启动一个数据的发布者节点：

$ ros2 run demo_nodes_cpp talker

#启动第二个终端，通过以下命令启动一个数据的订阅者节点：

$ ros2 run demo_nodes_py listener

如果“Hello World”字符串在两个终端中正常传输，说明通信系统没有问题。
```

### 2. 安装Python依赖

```bash
pip install open3d numpy transforms3d
```

### 3. 克隆并编译

```bash
# 克隆本仓库
cd ~
git clone https://github.com/zylovexyddcoco/Nova5-Vision-ObstacleAvoidance.git
cd Nova5-Vision-ObstacleAvoidance

# 编译
colcon build

# 设置环境变量
source install/setup.bash
echo "source ~/Nova5-Vision-ObstacleAvoidance/install/setup.bash" >> ~/.bashrc下

```

## 使用方法

### 快速启动

```bash
# 终端1：启动机械臂驱动
ros2 launch dobot_bringup_v3 dobot_bringup_ros2.launch.py

# 终端2：使能机械臂
ros2 service call /dobot_bringup_v3/srv/EnableRobot dobot_msgs_v3/srv/EnableRobot
ros2 service call /dobot_bringup_v3/srv/SpeedFactor dobot_msgs_v3/srv/SpeedFactor "{ratio: 30}"

# 终端3：启动MoveIt
ros2 launch dobot_moveit dobot_moveit.launch.py

# 终端4：启动相机
ros2 launch realsense2_camera rs_launch.py depth_module.profile:=640x480x30 pointcloud.enable:=true

# 终端5：启动相机TF
ros2 launch obstacle_avoidance camera_tf.launch.py

# 终端6：启动OctoMap + 自身过滤
ros2 launch obstacle_avoidance octomap.launch.py

# 终端7：启动Planning Scene桥接
ros2 run obstacle_avoidance octomap_to_planning_scene.py
```

### RViz操作

1. 在RViz中添加显示项：`MotionPlanning`、`OccupancyGrid (OctoMap)`
2. 拖动交互球设置目标位姿
3. 点击 **Plan** 规划避障路径
4. 点击 **Execute** 执行轨迹

### 拖拽模式

执行完成后，如需手动复位机械臂：

```bash
python3 ~/nova5_ws/src/obstacle_avoidance/scripts/drag_controller.py
# 按 'd' 进入拖拽模式
# 手动拖动机械臂
# 按 's' 退出拖拽
# 按 'e' 重新使能
```


## 配置参数

### OctoMap参数 (`config/octomap_config.yaml`)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `resolution` | 0.03 | 体素分辨率（米） |
| `sensor_model.hit` | 0.7 | 占据概率增量 |
| `sensor_model.miss` | 0.35 | 空闲概率增量 |
| `occupancy_min_z` | 0.0 | 最小高度过滤 |
| `occupancy_max_z` | 2.0 | 最大高度过滤 |

### 自身过滤参数 (`scripts/self_filter.py`)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `filter_radius` | 0.08 | 连杆过滤半径（米） |
| `base_filter_radius` | 0.15 | 基座过滤半径（米） |
| `subsample_rate` | 3 | 点云降采样率 |

## 手眼标定

可参考(https://github.com/zylovexyddcoco/Dobot-Nova5-Controll-and-Calibration)

## 常见问题

### Q: JointMovJ返回-1错误
**A**: 检查机械臂是否处于使能状态（mode=5），如果是拖拽模式（mode=6），需先退出拖拽。

### Q: OctoMap中出现悬浮噪点
**A**: 调整OctoMap的`hit/miss`概率参数，或增大自身过滤的`filter_radius`。

### Q: 规划的路径与实际执行不符
**A**: 检查`action_move_server.py`是否执行了所有轨迹点，确保角度连续性处理正确。

### Q: TF树断开
**A**: 使用`ros2 run tf2_tools view_frames`检查，确保`base_link → camera_link`链路完整。

## 致谢

- [Dobot](https://www.dobot.cc/) - Nova5机械臂及ROS2驱动
- [MoveIt2](https://moveit.ros.org/) - 运动规划框架
- [OctoMap](https://octomap.github.io/) - 3D占据栅格地图
- [Intel RealSense](https://www.intelrealsense.com/) - 深度相机SDK

## 许可证

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件

## 联系方式

如有问题或建议，欢迎提交 Issue 或 Pull Request。
