# Fanuc 焊接指令控制模块

## 概述

本系统提供了一个专门的焊接指令控制解决方案。它允许用户通过 ROS 发布指令，控制 Fanuc 机器人和 EWM 焊机执行焊接任务。指令通过一个专门的 ROS 话题发送，由机器人侧的 KAREL 程序接收并执行，实现对焊接参数（如送丝速度、焊接模式、程序号）和焊接动作（如起弧、关弧、点动送丝）的精确控制。

## 系统架构

### ROS 侧 (C++)
- **文件**: `fanuc_driver/src/fanuc_weld_command_node.cpp`
- **节点名**: `weld_command_sender`
- **功能**:
  - 订阅 `/weld_command` 话题，接收 `fanuc_driver/WeldCommand` 类型的消息。
  - 将接收到的指令打包成 ROS-Industrial Simple Message 兼容的 TCP 数据包。
  - 连接到机器人控制器并发送该数据包。
- **订阅话题**: `/weld_command` (`fanuc_driver/WeldCommand`)

### 机器人侧 (KAREL)
- **文件**: `fanuc_driver/karel/ros_weld_relay.kl`
- **端口**: 11002 (默认)
- **TAG**: 3 (默认)
- **功能**:
  - 在指定端口上监听来自 ROS 节点的 TCP 连接。
  - 接收并解析指令数据包。
  - 将指令数据写入对应的 `GO` (Group Outputs) 和 `DO` (Digital Outputs)，从而控制焊机执行相应操作。
  - 发送简单的成功/失败应答给 ROS 节点。

## I/O 映射配置

指令参数与机器人 I/O 的默认映射关系如下（可在 KAREL 程序 `ros_weld_relay.kl` 中修改）：

### Group Outputs (GO)
- `GO[2]`: **焊接程序号 (Program Number)** - `program_number`
- `GO[3]`: **目标送丝速度 (Target Wire Speed)** - `target_wire_spd`
- `GO[4]`: **修正值 (Correction Value)** - `correction_val`
- `GO[5]`: **动态特性 (Dynamic Setting)** - `dyn_setting`
- `GO[6]`: **操作模式 (Operation Mode)** - `operation_mode` (例如：0=标准, 1=脉冲)
- `GO[7]`: **标准/脉冲值 (Std/Pulse Value)** - `std_pulse_val`

### Digital Outputs (DO)
- `DO[253]`: **起弧指令 (Arc Start Command)** - `arc_start_cmd` (1=ON, 0=OFF)
- `DO[255]`: **气体控制 (Gas Control)** - `gas_control` (1=ON, 0=OFF)
- `DO[257]`: **点动送丝 (Jog Wire Feed)** - `jog_feed_cmd` (1=ON, 0=OFF)
- `DO[259]`: **点动收丝 (Jog Wire Retract)** - `jog_retract_cmd` (1=ON, 0=OFF)

## 消息格式

### `WeldCommand.msg` (指令: ROS -> Robot)
```msg
std_msgs/Header header

# Group Output (GO) Commands
uint32 target_wire_spd      # GO[3]
uint32 correction_val       # GO[4]
uint32 dyn_setting          # GO[5]
uint32 operation_mode       # GO[6]
uint32 std_pulse_val        # GO[7]
uint32 program_number       # GO[2]

# Digital Output (DO) Commands
uint32 arc_start_cmd        # DO[253]
uint32 gas_control          # DO[255]
uint32 jog_feed_cmd         # DO[257]
uint32 jog_retract_cmd      # DO[259]
```

### TCP Packet Structure
ROS 节点与 KAREL 程序之间通过一个固定结构的 TCP 数据包进行通信，总长度为 60 字节。
- **Header** (16 bytes): `ind_hdr_t` 结构，包含长度、消息类型等。
- **Sequence Number** (4 bytes): `seq_nr_`，用于追踪指令。
- **Payload** (40 bytes): `ind_weld_cmd_data_t` 结构，包含 10 个 `uint32` (KAREL 中为 `INTEGER`) 类型的焊接指令字段。

## 使用步骤

### 1. 编译系统
```bash
cd ~/catkin_ws  # 切换到你的工作空间
catkin_make
source devel/setup.bash
```

### 2. 机器人端配置

#### 上传并配置 KAREL 程序
1. 将 `ros_weld_relay.kl` 上传到机器人控制器。
2. 在机器人上编译该 KAREL 程序。
3. **重要**: 在 KAREL 程序的 `DATA` -> `Detail` 菜单中，找到 `cfg_` 变量，将其中的 `checked` 字段设置为 `TRUE`。
4. （可选）根据实际的现场总线配置修改 `ros_weld_relay.kl` 中的 I/O 映射默认值。

#### KAREL 配置参数 (`weld_cfg_t`)
```karel
TYPE
	weld_cfg_t = STRUCTURE
		checked      : BOOLEAN  -- 必须设置为 TRUE
		loop_hz      : INTEGER  -- 主循环更新率 (默认: 10Hz)
		s_tcp_nr     : INTEGER  -- TCP 监听端口 (默认: 11002)
		s_tag_nr     : INTEGER  -- 服务器 TAG (默认: 3)
		um_clear     : BOOLEAN  -- 启动时清除用户菜单 (默认: TRUE)
		-- GO/DO 映射
		wire_go      : INTEGER
		correction_go: INTEGER
        ...
	ENDSTRUCTURE
```

### 3. 启动 ROS 节点
使用 `weld_control.launch` 文件启动焊接指令发送节点。
```bash
roslaunch fanuc_driver weld_control.launch robot_ip:=<YOUR_ROBOT_IP>
```
- **`robot_ip`**: 机器人控制器的 IP 地址。
- **`robot_port`**: KAREL 程序监听的端口 (默认: 11002)。
- **`debug`**: 是否开启调试模式 (默认: false)。

### 4. 发送焊接指令
可通过多种方式发送指令：

#### a) 使用测试脚本 (推荐)
`fanuc_driver` 包提供了一个功能丰富的 Python 测试脚本。
- **进入交互模式**:
  ```bash
  rosrun fanuc_driver weld_command_test.py --interactive
  ```
  此模式下会列出预设指令，方便测试。

- **发送单条指令**:
  ```bash
  # 设置程序号为1，送丝速度为50
  rosrun fanuc_driver weld_command_test.py --program 1 --wire-speed 50
  
  # 发送起弧指令
  rosrun fanuc_driver weld_command_test.py --arc-start --gas-on
  
  # 发送停弧指令
  rosrun fanuc_driver weld_command_test.py --arc-stop --gas-off
  ```

#### b) 手动发布 ROS 话题
```bash
rostopic pub /weld_command fanuc_driver/WeldCommand "{
  target_wire_spd: 50, 
  program_number: 1, 
  arc_start_cmd: 1, 
  gas_control: 1
}" -1
```

## 如何新增一个指令变量

下述步骤以新增一个 32 bit 整数 `weld_schedule` (焊接规范号) 为例进行说明。

1. **ROS 消息文件** (`msg/WeldCommand.msg`)
   1. 在文件中追加新字段：
      ```msg
      uint32 weld_schedule      # GO[8] for weld schedule
      ```
   2. 运行 `catkin_make` 重新生成 C++ 和 Python 的消息定义。

2. **KAREL 程序** (`karel/ros_weld_relay.kl`)
   1. 添加新的 GO 映射常量：`DEF_SCHEDULE_GO = 8`。
   2. 在 `weld_cfg_t` 结构体中添加 `schedule_go` 字段，并在 `check_cfg_` 中为其设置默认值。
   3. 在 `ind_weld_cmd_data_t` 结构体中添加 `weld_schedule` 字段。
   4. **修改报文长度**: `hdr_.length_` 需要增加 4 字节。
      ```karel
      -- 原: (RI_SZ_HDR + 4 + 40) -> 16 + 4 + 40 = 60
      -- 新: (RI_SZ_HDR + 4 + 44) -> 16 + 4 + 44 = 64
      hdr_.length_ = (RI_SZ_HDR + 4 + 44) 
      ```
   5. 在 `iwd_dslice_cmd` 中，按顺序 `READ` 新增的字段。
   6. 在 `exec_weld_cmd_` 中，将新值写入对应的 `GOUT`: `GOUT[cfg.schedule_go] = cmd.weld_cmd_data_.weld_schedule`。

3. **C++ 发送节点** (`fanuc_weld_command_node.cpp`)
   1. 在 `WeldCommandPacket` 结构体中，按正确顺序添加新字段 `uint32_t weld_schedule;`。
   2. 在 `commandCallback` 函数中，从 ROS 消息中读取新字段并赋值给 `packet`。
      ```cpp
      packet.weld_schedule = msg->weld_schedule;
      ```
   3. `sizeof(packet)` 会自动更新，`send()` 函数无需修改。

4. **测试脚本** (`scripts/weld_command_test.py`) (可选)
   - 在 `send_command` 函数和 `argparse` 参数中添加对 `weld_schedule` 的支持。

5. **重新编译与部署**
   - 运行 `catkin_make` 编译 ROS 节点。
   - 将修改后的 `.kl` 文件上传并编译到机器人。

## 故障排除

1. **无法连接到机器人** (`Failed to connect`)
   - **网络检查**: `ping <YOUR_ROBOT_IP>` 确保网络通畅。
   - **端口检查**: `telnet <YOUR_ROBOT_IP> 11002` 确认端口是否开放且 KAREL 程序正在监听。
   - **KAREL 程序状态**: 确认 `ros_weld_relay.kl` 正在机器人上运行，并且 `cfg_.checked` 已设为 `TRUE`。
   - **防火墙**: 检查机器人或网络中是否有防火墙阻止了连接。

2. **指令发送但机器人无动作**
   - **查看 ROS 节点日志**: 启动 `weld_control.launch` 时设置 `debug:=true`，查看详细日志。
   - **查看 KAREL 日志**: 在示教器上查看 KAREL 程序的日志输出，确认是否收到数据以及是否有错误信息。
   - **验证 I/O 映射**: 在示教器上手动监控对应的 `GO` 和 `DO`，确认其值是否在你发送指令后发生改变。
   - **现场总线**: 确认机器人与 EWM 焊机之间的现场总线连接正常。

3. **收到应答失败** (`Command failed on robot` 或 `Failed to receive acknowledgment`)
   - 这通常表示 KAREL 程序在执行指令时遇到问题。检查 KAREL 日志以获取详细错误信息。
   - 可能原因包括 I/O 配置错误、机器人处于非自动模式等。

## 测试建议

1. **连接测试**: 启动 `weld_control.launch`，观察 ROS 日志，应能看到 "Connected to robot" 的信息。
2. **基础指令测试**: 使用 `weld_command_test.py` 发送简单的指令，如 `--program 5`。在示教器的 I/O 界面查看 `GO[2]` 的值是否变为 5。
3. **动作指令测试**: 发送 `--jog-feed` 指令，观察焊枪是否开始送丝。发送 `--jog-stop` 停止。
4. **交互模式测试**: 使用 `weld_command_test.py --interactive` 运行所有预设指令，全面测试系统功能。

## 扩展功能

- **增强应答机制**: 让 KAREL 程序返回更详细的执行结果，而不仅仅是成功/失败。
- **与状态监控结合**: 将本指令系统与 `ros_weld_state_reader` 状态监控系统结合，形成闭环控制。
- **动态配置**: 允许通过 ROS 服务或参数服务器在运行时修改 I/O 映射。 