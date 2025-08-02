
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
  - **心跳机制**: 每3秒自动发送心跳包（序列号 -110）以维持长期连接稳定。
- **订阅话题**: `/weld_command` (`fanuc_driver/WeldCommand`)

### 机器人侧 (KAREL)
- **文件**: `fanuc_driver/karel/ros_weld_relay.kl`
- **端口**: 11003 (默认)
- **TAG**: 4 (默认)
- **功能**:
  - 在指定端口上监听来自 ROS 节点的 TCP 连接。
  - 接收并解析指令数据包。
  - **心跳处理**: 识别心跳帧（序列号 -110）并重置空闲计时器，不执行任何I/O操作。
  - 将指令数据写入对应的 `GO` (Group Outputs) 和 `DO` (Digital Outputs)，从而控制焊机执行相应操作。
  - 发送简单的成功/失败应答给 ROS 节点。
  - **连接管理**: 具有10秒空闲超时机制，确保断线后能安全重连。

## 连接管理与稳定性

### 心跳机制
本系统采用了专门的心跳机制来确保长期连接的稳定性：

- **ROS端心跳**: C++节点每3秒自动发送一个心跳包（序列号为 -110），所有payload字段设为哨兵值 -1
- **KAREL端处理**: 机器人接收到心跳包后：
  - 重置空闲计时器 (`idle_counter_ = 0`)
  - 发送成功应答
  - 不执行任何焊接I/O操作
- **优势**: 
  - 保持长期稳定连接，避免因长时间无业务数据而误断开
  - 心跳包完全透明，不影响焊接参数
  - 自动检测和恢复连接异常

### 空闲超时保护
- **超时阈值**: 10秒无任何数据（包括心跳和业务指令）
- **触发条件**: 当ROS节点异常退出（如Ctrl+C）时，心跳停止
- **保护机制**: KAREL程序检测到超时后自动断开连接并准备重连
- **避免卡死**: 解决了早期版本中节点中断导致KAREL程序卡死的问题

### 连接恢复
- **自动重连**: ROS节点重启后会自动尝试连接机器人
- **状态清理**: KAREL程序在每次新连接时会重置所有内部状态
- **无状态设计**: 连接中断不会影响机器人上已设置的焊接参数

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

## 独立参数控制 (哨兵值机制)

为了实现对单个焊接参数的独立控制，避免一个指令覆盖所有未指定的参数（例如在示教器上进行的临时修改），本系统采用“哨兵值”机制。

- **哨兵值**: `-1`
- **核心规则**: 当您通过 ROS 发送指令时，对于**任何不想修改的参数，都必须在消息中将其值显式设置为 `-1`**。机器人侧的 KAREL 程序会识别这个特殊值，并跳过对该参数的更新，从而保留其当前值。

**重要**: 如果在 ROS 消息中将一个字段留空或设置为 `0`，系统会将其视为一个有效的目标值（例如，程序号0或送丝速度0），并会覆盖机器人上的现有设置。

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

### Endianness (Byte Order)

默认情况下，KAREL 通过 `WRITE`/`SOCKET` 指令输出 **小端 (Little-Endian)** 序列，
而 C++ 节点直接使用 `*reinterpret_cast<uint32_t*>(buf)` 解析，这意味着
**双方必须使用相同的字节序**。

* **验证**：可将 `seq_nr_` 暂时设为 `16#11223344`，在 ROS 端打开 `--debug`，
  如果看到 `44 33 22 11` 则表示链路为小端；若顺序反转，则说明处于大端模式。
* **大端兼容**：若 ROS 节点运行在大端 CPU 或网络环境强制转换了字节序，
  请参照 `README_WELD_STATE.md` 中的「大小端对齐」章节，对 KAREL 端使用
  `SWAP32()`，并在 C++ 端使用 `htonl()/ntohl()` 或 `std::byteswap` 进行转换。

### 大小端 (Byte Order) 说明
KAREL 端发送的所有 `INTEGER` 字段均采用 **小端序 (Little-Endian)**。ROS C++ 节点在常见的 PC/ARM 平台上也默认按小端解析，两端即可直接通信。

如果需在 **大端序** 设备上运行 ROS 节点，或者你希望将网络流改为 **大端 (Network / Big-Endian)**，请遵循以下原则：

1. **两端字节序必须保持一致**。任何一端修改后，都必须同步修改另一端。
2. **在 C++ 端交换字节** (推荐)：
   ```cpp
   uint32_t le = *reinterpret_cast<uint32_t*>(&buf[pos]);
   #if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
     le = __builtin_bswap32(le);
   #endif
   ```
3. **在 KAREL 端交换字节**：使用自定义 `SWAP_INT` 函数或手动移位，在写入前将数据转换为大端格式，并在 C++ 端直接读取。

完成修改后，请重新 `catkin_make` 并重新部署 `.kl` 程序，以确保新字节序设置生效。

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
`fanuc_driver` 包提供了一个功能丰富的 Python 测试脚本，**该脚本已内置哨兵值逻辑**，是进行测试和控制的首选方式。当您使用特定参数时，脚本会自动将所有其他参数设置为 `-1`。
- **进入交互模式**:
  ```bash
  rosrun fanuc_driver weld_command_test.py --interactive
  ```
  此模式下会列出预设指令，方便测试。

- **发送单条指令**:
  ```bash
  # 设置程序号为1，送丝速度为50
  rosrun fanuc_driver weld_command_test.py --program 1 --wire_speed 50
  
  # 发送起弧指令
  rosrun fanuc_driver weld_command_test.py --arc_start --gas_on
  
  # 发送停弧指令
  rosrun fanuc_driver weld_command_test.py --arc_stop --gas_off
  ```

#### b) 手动发布 ROS 话题
**注意**: 使用 `rostopic pub` 时，所有未指定的 `uint32` 字段默认为 0。根据哨兵值规则，这会把所有未指定的参数重置为 0。因此，您必须手动将不想更改的参数设为 `-1`。

**示例：仅设置程序号为 5，其他参数保持不变**
```bash
rostopic pub /weld_command fanuc_driver/WeldCommand "{
  target_wire_spd: -1, 
  correction_val: -1,
  dyn_setting: -1,
  operation_mode: -1,
  std_pulse_val: -1,
  program_number: 5,         # <-- The only value we want to change
  arc_start_cmd: -1, 
  gas_control: -1,
  jog_feed_cmd: -1,
  jog_retract_cmd: -1
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

4. **参数被意外覆盖**
   - **问题**: 在发送一个指令（例如只设置程序号）后，其他所有焊接参数（如送丝速度）都被重置为0。
   - **原因**: 您发送的 ROS 消息中，未改变的参数没有被设置为哨兵值 `-1`。ROS 消息字段的默认值 `0` 是一个有效的指令值，因此会覆盖机器人上的设置。
   - **解决方案**: 确保您的发布代码或 `rostopic pub` 指令将所有不想修改的参数字段的值都设置为 `-1`。

5. **连接频繁断开**
   - **问题**: 连接建立后很快就断开，特别是在长时间无指令发送时。
   - **原因**: 这在旧版本中可能发生，现版本已通过心跳机制解决。
   - **解决方案**: 
     - 确保使用最新版本的代码（包含心跳机制）
     - 检查ROS日志中是否有心跳发送失败的警告
     - 如果问题持续，可以在KAREL日志中查看 "Received heartbeat frame" 信息来确认心跳是否正常

6. **节点中断后KAREL程序卡死**
   - **问题**: 用Ctrl+C中断ROS节点后，KAREL程序无响应，示教器报错 "libind_log 参数还没有设定"。
   - **原因**: 这是旧版本的问题，现版本已通过空闲超时机制解决。
   - **解决方案**: 
     - 使用最新版本的 `ros_weld_relay.kl`
     - 如果问题仍存在，在示教器上手动停止并重启KAREL程序
     - 检查KAREL日志中的 "Idle timeout, assuming disconnect" 信息

## 测试建议

1. **连接测试**: 启动 `weld_control.launch`，观察 ROS 日志，应能看到 "Connected to robot" 的信息。
2. **心跳验证**: 连接建立后，在KAREL日志中应能定期看到 "Received heartbeat frame" 信息（每3秒一次）。
3. **基础指令测试**: 使用 `weld_command_test.py` 发送简单的指令，如 `--program 5`。在示教器的 I/O 界面查看 `GO[2]` 的值是否变为 5。
4. **动作指令测试**: 发送 `--jog_feed` 指令，观察焊枪是否开始送丝。发送 `--jog_stop` 停止。
5. **长期连接测试**: 保持连接10分钟以上，确认连接稳定，无意外断开。
6. **断线恢复测试**: 用Ctrl+C中断ROS节点，等待10秒后重启，确认能正常重连。
7. **交互模式测试**: 使用 `weld_command_test.py --interactive` 运行所有预设指令，全面测试系统功能。

## 扩展功能

- **增强应答机制**: 让 KAREL 程序返回更详细的执行结果，而不仅仅是成功/失败。
- **与状态监控结合**: 将本指令系统与 `ros_weld_state_reader` 状态监控系统结合，形成闭环控制。
- **动态配置**: 允许通过 ROS 服务或参数服务器在运行时修改 I/O 映射。 
