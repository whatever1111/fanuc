# Fanuc机器人运动测试指南

本文档介绍如何使用提供的测试脚本来控制Fanuc机器人运动。

## 前置条件

1. 确保fanuc_driver包已编译安装
2. 机器人控制器已启动并运行SimpleMessage服务器
3. 网络连接正常

## 启动机器人接口

首先启动机器人控制接口节点：

```bash
# Source环境
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# 启动完整接口（状态读取 + 运动控制）
ros2 launch fanuc_driver robot_interface_streaming.launch.py \
  robot_ip:=192.168.1.100 \
  J23_factor:=0 \
  use_bswap:=false
```

参数说明：
- `robot_ip`: 机器人控制器IP地址
- `J23_factor`: J2-J3联动系数（-1/0/1）
- `use_bswap`: 是否启用字节交换（用于big-endian控制器）

## 测试工具

### 1. 简单运动测试 (simple_motion_test.py)

命令行工具，用于快速测试基本运动：

```bash
# 移动到预定义位置
ros2 run fanuc_driver simple_motion_test.py home    # 回零位
ros2 run fanuc_driver simple_motion_test.py ready   # 准备位置
ros2 run fanuc_driver simple_motion_test.py test1   # 测试位置1
ros2 run fanuc_driver simple_motion_test.py test2   # 测试位置2

# 运行演示序列
ros2 run fanuc_driver simple_motion_test.py demo

# 单关节运动（角度单位：度）
ros2 run fanuc_driver simple_motion_test.py j1 30   # 关节1移动30度
ros2 run fanuc_driver simple_motion_test.py j2 -45  # 关节2移动-45度
ros2 run fanuc_driver simple_motion_test.py j3 60   # 关节3移动60度
```

预定义位置（弧度）：
- `home`: [0, 0, 0, 0, 0, 0] - 所有关节归零
- `ready`: [0, -45°, 90°, 0, 45°, 0] - 准备姿态
- `test1`: [30°, -30°, 45°, 0, 30°, 0] - 测试位置1
- `test2`: [-30°, -45°, 60°, 0, 45°, 0] - 测试位置2

### 2. 交互式运动测试 (test_robot_motion.py)

提供交互式菜单，支持多种测试模式：

```bash
ros2 run fanuc_driver test_robot_motion.py
```

功能菜单：
1. **直接轨迹发布测试** - 通过`/joint_path_command`话题发送轨迹
2. **FollowJointTrajectory Action测试** - 使用标准Action接口
3. **正弦波运动测试** - 多关节协调正弦运动
4. **运行所有测试** - 依次执行所有测试

### 3. 手动轨迹发布

也可以直接通过命令行发布轨迹：

```bash
# 发布简单的关节轨迹
ros2 topic pub /joint_path_command trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'], \
   points: [{positions: [0,0,0,0,0,0], time_from_start: {sec: 0}}, \
           {positions: [0.5,-0.5,0.5,0,0,0], time_from_start: {sec: 3}}]}"
```

## 监控机器人状态

在另一个终端监控机器人状态：

```bash
# 查看当前关节位置
ros2 topic echo /joint_states

# 查看机器人状态
ros2 topic echo /robot_status

# 查看关节反馈
ros2 topic echo /joint_feedback
```

## 安全注意事项

⚠️ **警告**：
1. 在运行任何运动命令前，确保机器人周围没有障碍物
2. 首次测试时使用较慢的速度和较小的运动范围
3. 准备好紧急停止按钮
4. 确认机器人处于适当的操作模式（自动/手动）

## 故障排除

### 问题：节点无法连接到机器人
- 检查IP地址和端口配置
- 确认网络连接：`ping <robot_ip>`
- 检查机器人控制器上的SimpleMessage服务器是否运行

### 问题：运动命令没有响应
- 检查机器人是否处于自动模式
- 查看`/robot_status`确认`motion_possible`为true
- 检查是否有错误：`ros2 topic echo /robot_status | grep error`

### 问题：关节位置不正确
- 检查J23_factor参数设置
- 验证字节序设置(use_bswap)
- 确认关节名称顺序正确

## 示例工作流

完整的测试流程示例：

```bash
# 终端1：启动机器人接口
source ~/ros2_ws/install/setup.bash
ros2 launch fanuc_driver robot_interface_streaming.launch.py robot_ip:=192.168.1.100

# 终端2：监控状态
source ~/ros2_ws/install/setup.bash
ros2 topic echo /joint_states

# 终端3：运行测试
source ~/ros2_ws/install/setup.bash
# 先回零
ros2 run fanuc_driver simple_motion_test.py home
# 运行演示
ros2 run fanuc_driver simple_motion_test.py demo
# 交互式测试
ros2 run fanuc_driver test_robot_motion.py
```

## 高级用法

### 自定义轨迹
可以修改`test_robot_motion.py`中的轨迹参数来创建自定义运动模式。

### 速度控制
调整轨迹点之间的`time_from_start`来控制运动速度。

### 连续轨迹
使用`streaming_rate`参数调整轨迹流的频率（默认125Hz）。

---

更多信息请参考主README文档。