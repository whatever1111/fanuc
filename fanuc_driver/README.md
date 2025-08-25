# fanuc_driver (ROS 2) — 焊接命令与状态节点

本包为 Fanuc 焊接控制的 ROS 2 迁移版，提供一个订阅 ROS 2 话题并通过 TCP 向机器人控制器发送焊接控制数据的节点。

- 适配 ROS 2：Jazzy（Ubuntu 24.04）
- 构建系统：ament_cmake（colcon）
- 可执行文件：
  - `weld_command_node`：订阅 `/weld_command` 并以 TCP 发送给控制器
  - `weld_state_node_tcp`：通过固定头格式（非 SimpleMessage）接收焊接状态
  - `weld_state_node_simple`：通过 SimpleMessage 兼容格式接收焊接状态

## 1. 构建与环境

前置条件：
- 已安装并可用的 ROS 2 Jazzy
- 已安装 `colcon` 构建工具

构建与加载环境：
```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select fanuc_driver --symlink-install
source ~/ros2_ws/install/setup.bash
```

验证：
```bash
ros2 interface show fanuc_driver/msg/WeldCommand
ros2 pkg executables fanuc_driver
```

## 2. 启动

推荐使用自带 launch（会自动解析参数文件的绝对路径）：
```bash
ros2 launch fanuc_driver weld_control.launch.py
```
该启动文件默认启动 `weld_command_node` 与 `weld_state_node_tcp`。如需启用 SimpleMessage 版本，可编辑
`launch/weld_control.launch.py` 中的注释段，将可执行切换为 `weld_state_node_simple`。

如需直接运行二进制（不通过 launch）：
```bash
ros2 run fanuc_driver weld_command_node
```
建议配合参数文件：`--ros-args --params-file <your.yaml>`，否则使用节点内默认参数。

## 3. 参数（Parameters）

参数默认由 `config/weld_command.yaml` 与 `config/weld_state.yaml` 提供（已随包安装）。示例（命令）：
```yaml
/**:
  ros__parameters:
    robot_ip: "127.0.0.1"   # 机器人控制器 IP
    robot_port: 11002        # 焊接命令 TCP 端口
    byte_order: "little"     # 字节序："little" 或 "big"
    enable_heartbeat: false  # 是否启用 3s 心跳（No-Change）
```
示例（状态 TCP）：
```yaml
/**:
  ros__parameters:
    robot_ip: "127.0.0.1"
    robot_port: 11002
    byte_order: "little"
    debug: false
    dump_raw: false
    expected_standard_length: 53        # TCP 版本
    expected_msg_type: 15
    expected_comm_type: 1
    payload_fields: 9
    # SimpleMessage 版本额外参数
    payload_only_length: 32            # 仅负载格式：32 (8×int32? 实际为9×int32=36，注意机器人端实现差异)
    standard_format_length: 53         # 标准格式：4(seq)+36(payload)+可选CR/LF
    num_weld_fields: 9
    # 可选：统一 IO 模板文件（供 C++ 节点覆盖 IP/端口/字节序）
    # io_map_file: "/absolute/path/to/weld_io_map.yaml"
    # 可选：端口覆盖（来自统一模板）
    # port_weld_state: 11001
```
说明：
- 使用 `/**:` 通配，可以在节点名变化时仍然生效。
- 请将 `robot_ip` 改为你的机器人地址。WSL2 环境下，请确保网络与防火墙策略允许访问该 IP:port。

### 统一 IO 配置（模板）
- 新增 `config/weld_io_map.yaml` 作为统一模板，包含：
  - `robot.ip`、`robot.byte_order`、`robot.ports.{weld_state_server,weld_command_server}`
  - `io_map.inputs/outputs` 的 DI/DO/GI/GO/R 定义
- 当前阶段（KAREL 改造未启用前）：
  - C++ 节点可通过参数 `io_map_file` 读取该文件并覆盖 `robot_ip/robot_port/byte_order`；
  - 也可单独使用 `port_weld_state` / `port_weld_command` 参数覆盖端口；
  - IO 映射内容仅用于将来 KAREL 侧自动生成与读取，C++ 节点不直接操作 I/O。

## 4. 话题（Topics）

- 订阅：`/weld_command`（类型 `fanuc_driver/msg/WeldCommand`）
- 发布：`/weld_state`（类型 `fanuc_driver/msg/WeldState`）

消息定义（`fanuc_driver/msg/WeldCommand.msg`）：
```
int32 target_wire_spd
int32 correction_val
int32 dyn_setting
int32 operation_mode
int32 std_pulse_val
int32 program_number
int32 arc_start_cmd
int32 gas_control
int32 jog_feed_cmd
int32 jog_retract_cmd
std_msgs/Header header
```

## 5. 使用示例

- 以 2 Hz 发布测试指令：
```bash
ros2 topic pub -r 2 /weld_command fanuc_driver/msg/WeldCommand \
"{target_wire_spd: 100, correction_val: 0, dyn_setting: 50, operation_mode: 0, std_pulse_val: 0, program_number: 1, arc_start_cmd: 1, gas_control: 1, jog_feed_cmd: 0, jog_retract_cmd: 0}"
```
- 观察话题：
```bash
ros2 topic echo /weld_command
```

- 观察焊接状态：
```bash
ros2 topic echo /weld_state
```

### SimpleMessage 与 TCP 版本差异
- TCP 版：固定头+可变剩余长度，按 `expected_*` 与 `payload_fields` 校验。
- SimpleMessage 版：支持两种格式（payload-only、standard with seq），容忍结尾 CR/LF。

期望节点日志（收到消息时）：
- `--- Received WeldCommand ROS message ---`
- 各字段值的打印
- 若与机器人成功建立 TCP 连接：`Connected to robot at <ip>:<port>` 以及发送/ACK 日志

## 6. 行为概述

- 节点接收 `WeldCommand` 后，若未连接会尝试与 `robot_ip:robot_port` 建立 TCP 连接。
- 将消息序列化为 Fanuc 协议数据包，按配置字节序发送。
- 若启用 `enable_heartbeat`，每 3 秒发送一次 No-Change 心跳包。
- 简单读取 ACK（非阻塞），异常会记录日志并可能关闭连接。

## 7. 常见问题排查

- 启动警告 “Parameter file path is not a file”：
  - 使用提供的 launch；其通过 `get_package_share_directory` 解析绝对路径。
  - 确认 `src/fanuc_driver/config/weld_command.yaml` 存在，修改后需要重建安装。
- 节点似乎未收到消息：
  - 两个终端均需 source ROS 2 overlay 后再启动/发布。
  - `ros2 topic info /weld_command --verbose` 应显示 `Subscription count: 1`。
  - 避免一次性发布与初始化竞态，建议 `-r 2` 连续发布。
- 无法连接机器人：
  - 确认主机/WSL2 到 `robot_ip:robot_port` 网络可达与防火墙放行。
  - 检查 `byte_order` 是否与控制器一致。
- DDS 域不一致：
  - 确认两个终端 `ROS_DOMAIN_ID` 一致（或均未设置）。

## 8. 与 ROS 1 版本差异

- 构建系统由 catkin 迁移为 ament_cmake，通过 `colcon` 构建。
- 参数管理采用 ROS 2 YAML 与 Launch，使用 `/**:` 提升对节点重命名的鲁棒性。
- 话题与消息字段尽量保持一致，便于工具链迁移。

## 9. License & Maintainer

- 许可证：BSD
- 维护者：robopath <robopath@example.com>

---
已实现：`weld_state` 的 TCP 与 SimpleMessage 两个版本，含测试。
