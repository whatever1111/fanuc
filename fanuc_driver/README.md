# fanuc_driver (ROS 2)

[![ROS2 Version](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![License](https://img.shields.io/badge/License-BSD-green)](LICENSE)

完整的Fanuc机器人ROS 2驱动包，提供机器人运动控制、状态监控和焊接控制功能。

## 📋 目录

- [功能特性](#功能特性)
- [系统要求](#系统要求)
- [安装与编译](#安装与编译)
- [快速开始](#快速开始)
- [机器人控制接口](#机器人控制接口)
- [焊接控制接口](#焊接控制接口)
- [配置说明](#配置说明)
- [测试工具](#测试工具)
- [故障排除](#故障排除)
- [技术细节](#技术细节)

## 🎯 功能特性

### 核心功能
- ✅ **完整的SimpleMessage协议实现** - 兼容ROS-Industrial标准
- ✅ **机器人运动控制** - 支持轨迹流和Action接口
- ✅ **实时状态监控** - 关节位置、机器人状态反馈
- ✅ **焊接控制接口** - 专用的焊接命令和状态管理
- ✅ **J2-J3联动补偿** - Fanuc特有的关节耦合处理
- ✅ **字节序自适应** - 支持大小端控制器

### 节点列表

| 节点 | 功能 | 默认端口 |
|------|------|----------|
| `robot_state_node` | 接收关节状态和机器人状态 | 11002 |
| `joint_streamer_node` | 发送轨迹命令到机器人 | 11000 |
| `weld_command_node` | 发送焊接控制命令 | 11002 |
| `weld_state_node_tcp` | 接收焊接状态（TCP协议） | 11002 |
| `weld_state_node_simple` | 接收焊接状态（SimpleMessage） | 11002 |

## 🔧 系统要求

- **ROS 2**: Jazzy Jalisco (Ubuntu 24.04)
- **构建工具**: colcon, ament_cmake
- **依赖包**: 
  - `control_msgs` - 轨迹控制Action
  - `sensor_msgs` - 关节状态消息
  - `trajectory_msgs` - 轨迹消息
  - `yaml-cpp` - 配置文件解析

## 📦 安装与编译

### 1. 安装依赖

```bash
sudo apt update
sudo apt install ros-jazzy-control-msgs ros-jazzy-sensor-msgs ros-jazzy-trajectory-msgs
sudo apt install libyaml-cpp-dev
```

### 2. 克隆仓库

```bash
cd ~/ros2_ws/src
git clone <repository_url> fanuc_driver
```

### 3. 编译

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select fanuc_driver
```

### 4. 设置环境

```bash
source install/setup.bash
```

## 🚀 快速开始

### 最简启动（机器人控制）

```bash
# 启动完整的机器人接口
ros2 launch fanuc_driver robot_interface_streaming.launch.py \
  robot_ip:=192.168.1.100
```

### 最简启动（焊接控制）

```bash
# 启动焊接控制接口
ros2 launch fanuc_driver weld_control.launch.py
```

### 测试连接

```bash
# 查看机器人状态
ros2 topic echo /joint_states

# 发送简单运动命令
ros2 run fanuc_driver simple_motion_test.py home
```

## 🤖 机器人控制接口

### 启动选项

#### 完整接口（推荐）
```bash
ros2 launch fanuc_driver robot_interface_streaming.launch.py \
  robot_ip:=192.168.1.100 \
  J23_factor:=0 \
  use_bswap:=false
```

#### 分离启动
```bash
# 仅状态读取
ros2 launch fanuc_driver robot_state.launch.py \
  robot_ip:=192.168.1.100 \
  robot_port:=11002

# 仅运动控制
ros2 launch fanuc_driver motion_streaming_interface.launch.py \
  robot_ip:=192.168.1.100 \
  robot_port:=11000
```

### 话题接口

#### 发布的话题
| 话题 | 类型 | 描述 |
|------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | 当前关节位置 |
| `/robot_status` | `fanuc_driver/RobotStatus` | 机器人状态信息 |
| `/joint_feedback` | `fanuc_driver/JointPosition` | 关节位置反馈 |

#### 订阅的话题
| 话题 | 类型 | 描述 |
|------|------|------|
| `/joint_path_command` | `trajectory_msgs/JointTrajectory` | 轨迹命令 |

### Action接口

- **服务名**: `/follow_joint_trajectory`
- **类型**: `control_msgs/FollowJointTrajectory`
- **用途**: 标准的轨迹执行接口，支持反馈和结果返回

### 使用示例

#### 发送轨迹命令
```python
import rclpy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# 创建轨迹
trajectory = JointTrajectory()
trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 
                          'joint_4', 'joint_5', 'joint_6']

# 添加轨迹点
point = JointTrajectoryPoint()
point.positions = [0.0, -0.785, 1.571, 0.0, 0.785, 0.0]
point.time_from_start.sec = 3
trajectory.points.append(point)

# 发布
publisher.publish(trajectory)
```

## ⚡ 焊接控制接口

### 启动

```bash
ros2 launch fanuc_driver weld_control.launch.py
```

### 焊接命令（发送）

**话题**: `/weld_command` (类型: `fanuc_driver/msg/WeldCommand`)

消息字段：
```
int32 target_wire_spd    # 送丝速度
int32 correction_val     # 修正值
int32 dyn_setting        # 动态设置
int32 operation_mode     # 操作模式
int32 std_pulse_val      # 脉冲值
int32 program_number     # 程序号
int32 arc_start_cmd      # 起弧命令
int32 gas_control        # 气体控制
int32 jog_feed_cmd       # 送丝点动
int32 jog_retract_cmd    # 回抽点动
```

发送示例：
```bash
ros2 topic pub -r 2 /weld_command fanuc_driver/msg/WeldCommand \
"{target_wire_spd: 100, correction_val: 0, dyn_setting: 50, 
  operation_mode: 0, std_pulse_val: 0, program_number: 1, 
  arc_start_cmd: 1, gas_control: 1, jog_feed_cmd: 0, jog_retract_cmd: 0}"
```

### 焊接状态（接收）

**话题**: `/weld_state` (类型: `fanuc_driver/msg/WeldState`)

监控示例：
```bash
ros2 topic echo /weld_state
```

## ⚙️ 配置说明

### 机器人参数配置

配置文件：`config/robot_state.yaml`
```yaml
/**:
  ros__parameters:
    robot_ip: "192.168.1.100"    # 机器人IP
    robot_port: 11002             # 状态端口
    J23_factor: 0                 # J2-J3耦合系数 (-1/0/1)
    use_bswap: false              # 字节序交换
    joint_names: ["joint_1", "joint_2", "joint_3", 
                  "joint_4", "joint_5", "joint_6"]
```

### 焊接参数配置

配置文件：`config/weld_command.yaml`
```yaml
/**:
  ros__parameters:
    robot_ip: "192.168.1.100"
    robot_port: 11002
    byte_order: "little"         # 字节序: "little" 或 "big"
    enable_heartbeat: false      # 3秒心跳包
```

### J2-J3联动补偿

Fanuc机器人的J2和J3关节可能存在机械耦合，通过`J23_factor`参数配置：

| 值 | 含义 | 公式 |
|----|------|------|
| -1 | 负向联动 | J3' = J3 - J2 |
| 0 | 无联动 | J3' = J3 |
| 1 | 正向联动 | J3' = J3 + J2 |

## 🧪 测试工具

### 1. 简单运动测试

```bash
# 预定义位置
ros2 run fanuc_driver simple_motion_test.py home    # 回零
ros2 run fanuc_driver simple_motion_test.py ready   # 准备位置
ros2 run fanuc_driver simple_motion_test.py demo    # 演示序列

# 单关节运动
ros2 run fanuc_driver simple_motion_test.py j1 30   # 关节1转30度
ros2 run fanuc_driver simple_motion_test.py j2 -45  # 关节2转-45度
```

### 2. 交互式测试

```bash
ros2 run fanuc_driver test_robot_motion.py
```

菜单选项：
1. 直接轨迹发布测试
2. FollowJointTrajectory Action测试
3. 正弦波运动测试
4. 运行所有测试

### 3. 焊接测试

```bash
# 频率测试
ros2 run fanuc_driver weld_control_freq_test.py

# 命令测试
ros2 run fanuc_driver weld_command_test.py
```

### 4. 运行单元测试

```bash
colcon test --packages-select fanuc_driver
colcon test-result --all
```

## 🔍 故障排除

### 常见问题

#### 无法连接到机器人
```bash
# 检查网络
ping <robot_ip>

# 检查端口
nc -zv <robot_ip> 11000
nc -zv <robot_ip> 11002

# 查看节点日志
ros2 run fanuc_driver robot_state_node --ros-args --log-level debug
```

#### 关节数据异常
- 检查`J23_factor`设置是否正确
- 验证`use_bswap`参数（大端/小端）
- 确认`joint_names`顺序

#### 运动命令无响应
```bash
# 检查机器人状态
ros2 topic echo /robot_status | grep motion_possible

# 确认机器人模式
ros2 topic echo /robot_status | grep mode
# mode应该为2 (AUTO)
```

#### 焊接命令失败
- 确认`byte_order`设置正确
- 检查端口号（焊接通常使用11002）
- 验证消息字段值范围

### 日志级别调整

```bash
# 调试模式启动
ros2 run fanuc_driver robot_state_node \
  --ros-args --log-level debug
```

## 🔧 技术细节

### SimpleMessage协议

本实现支持以下消息类型：
- `JOINT_POSITION (10)` - 关节位置反馈
- `JOINT_TRAJ_PT (11)` - 轨迹点
- `STATUS (13)` - 机器人状态
- `JOINT_FEEDBACK (15)` - 关节反馈

特殊序列值：
- `START_TRAJECTORY_STREAMING (-2)` - 开始轨迹流
- `END_TRAJECTORY (-3)` - 结束轨迹
- `STOP_TRAJECTORY (-4)` - 停止轨迹

### 消息格式

#### SimpleMessage标准格式
```
[Prefix: 4 bytes][Header: 12 bytes][Data: variable]
```

- **Prefix**: 包长度（不含自身）
- **Header**: 消息类型、通信类型、回复码
- **Data**: 消息数据

#### 焊接消息格式
支持两种格式：
1. **TCP格式**: 固定头部 + 可变数据
2. **SimpleMessage格式**: 标准SimpleMessage包装

### 架构设计

```
ROS 2 Application
       ↓
[fanuc_driver nodes]
       ↓
TCP/IP Socket (Port 11000/11002)
       ↓
Fanuc Controller (KAREL)
```

## 📄 许可证

BSD License - 详见 [LICENSE](LICENSE) 文件

## 👥 维护者

- robopath <robopath@example.com>

## 🔗 相关链接

- [ROS 2 Documentation](https://docs.ros.org/en/jazzy/)
- [ROS-Industrial](http://wiki.ros.org/Industrial)
- [Fanuc ROS Wiki](http://wiki.ros.org/fanuc)

---

*本项目从ROS 1迁移至ROS 2，保持了原有功能的同时充分利用了ROS 2的新特性。*