# Fanuc 焊接状态监控模块

## 概述

本系统提供了一个专门的焊接状态监控解决方案。它通过现场总线（如 PROFINET, EtherNet/IP）读取 EWM 焊机的 `GI/GO` (Group Input/Output) 和 `DI/DO` (Digital Input/Output) 信号，并通过 ROS 话题进行发布，实现对焊接过程的监控。

## 系统架构

### 机器人侧 (KAREL)
- **文件**: `fanuc_driver/karel/ros_weld_state_reader.kl`
- **端口**: 11002 (默认)
- **TAG**: 3 (默认)
- **功能**:
- **状态发送**: 从 `GI` 和 `DI` 读取焊机状态（如实际电压、电流、电弧状态等），打包成 `WeldState` 消息并通过 TCP 发送到 ROS。
- **循环频率**: 10Hz (默认)

### ROS 侧 (C++)
有两个版本的 ROS 节点可供选择：

#### 1. 标准版本 (Simple Message 协议)
- **文件**: `fanuc_driver/src/fanuc_weld_state_node.cpp`
- **类名**: `FanucWeldStateNode`
- **节点名**: `fanuc_weld_state_node`
- **特点**:
  - 使用 ROS-Industrial 的 Simple Message 协议
  - 使用 `TcpClient` 和 `SimpleMessage` 类
  - 兼容 ROS-Industrial 标准架构
  - 自动重连功能

#### 2. Raw版本 (直接TCP连接)
- **文件**: `src/fanuc_driver/src/fanuc_weld_state_node.cpp`
- **类名**: `FanucWeldStateNodeRaw`
- **节点名**: `fanuc_weld_state_node_raw`
- **特点**:
  - 直接使用标准 socket API 进行 TCP 连接
  - 详细的调试日志输出（带 [MSG], [DATA], [PUB] 前缀）
  - 更直接的数据处理和错误处理
  - 适合调试和问题排查

**共同功能**:
- **状态发布**: 接收 KAREL 程序发送的 TCP 消息，解析后发布到 `/weld_state` 话题。
- **发布话题**: `/weld_state` (`fanuc_driver/WeldState`)

## I/O 映射配置

当前实现中的默认 I/O 映射（可在 KAREL 程序中修改）：

### 状态信号 (焊机 -> 机器人 -> ROS)
```karel
-- Default I/O mappings in KAREL program
DEF_VOLT_GI      = 2   -- GI[2] for actual voltage (raw value)
DEF_CURR_GI      = 3   -- GI[3] for actual current (raw value)
DEF_ARC_DI       = 249 -- DI[249] for arc detection
DEF_POWER_ERR_DI = 252 -- DI[252] for power error
DEF_DEPOS_DI     = 251 -- DI[251] for deposition detection
```

### 实际读取的数据
- **实际电压**: `GIN[cfg_.volt_gi]` (GI[2])
- **实际电流**: `GIN[cfg_.curr_gi]` (GI[3])
- **电弧状态**: `DIN[cfg_.arc_di]` (DI[249])
- **电源错误**: `DIN[cfg_.power_err_di]` (DI[252])
- **熔敷检测**: `DIN[cfg_.depos_di]` (DI[251])
- **就绪状态**: 固定为 `TRUE` (测试用)
- **粘丝错误**: 固定为 `FALSE` (测试用)
- **送丝速度**: 固定为 `0` (当前版本未使用)

## 使用步骤

### 1. 编译系统
```bash
cd ~/fanuc_ws
catkin_make
source devel/setup.bash
```

### 2. 机器人端配置

#### 上传并配置 KAREL 程序
1. 将 `ros_weld_state_reader.kl` 上传到机器人控制器
2. 编译 KAREL 程序
3. 在程序中设置 `cfg_.checked = TRUE`
4. 根据实际现场总线配置修改 I/O 映射（如需要）

#### KAREL 配置参数
```karel
TYPE
	weld_cfg_t = STRUCTURE
		checked      : BOOLEAN  -- 必须设置为 TRUE
		loop_hz      : INTEGER  -- 主循环频率 (默认: 10Hz)
		s_tcp_nr     : INTEGER  -- TCP 端口 (默认: 11002)
		s_tag_nr     : INTEGER  -- 服务器 TAG (默认: 3)
		um_clear     : BOOLEAN  -- 清除用户菜单 (默认: TRUE)
		-- I/O 映射
		volt_gi      : INTEGER  -- 电压 GI 地址 (默认: 2)
		curr_gi      : INTEGER  -- 电流 GI 地址 (默认: 3)
		arc_di       : INTEGER  -- 电弧检测 DI 地址 (默认: 249)
		power_err_di : INTEGER  -- 电源错误 DI 地址 (默认: 252)
		depos_di     : INTEGER  -- 熔敷检测 DI 地址 (默认: 251)
	ENDSTRUCTURE
```

### 3. 启动 ROS 节点

#### 使用标准版本 (推荐)
```bash
roslaunch fanuc_driver weld_state.launch robot_ip:=192.168.1.31
```

#### 使用原始版本 (调试用)
```bash
roslaunch fanuc_driver weld_state.launch robot_ip:=192.168.1.31 use_bswap:=false
```

#### 可用参数
- `robot_ip`: 机器人控制器 IP 地址
- `robot_port`: TCP 端口 (默认: 11002)
- `use_bswap`: 是否使用字节交换版本 (默认: false)

### 4. 监控数据
```bash
# 查看话题列表
rostopic list | grep weld

# 监控焊接状态
rostopic echo /weld_state

# 查看话题信息
rostopic info /weld_state
```

## 数据缩放

根据 EWM 手册，原始值需要缩放才能得到物理单位：
```cpp
// 缩放因子 (在 C++ 节点中定义)
const double VOLTAGE_SCALE = 100.0 / 32767.0;   // 原始值 -> 伏特 (V)
const double CURRENT_SCALE = 1000.0 / 32767.0;  // 原始值 -> 安培 (A)
const double WIRE_SPEED_SCALE = 40.0 / 32767.0; // 原始值 -> 米/分钟 (m/min)
```

## 消息格式

### `WeldState.msg` (状态: Robot -> ROS)
```msg
Header header
bool arc_ok          # 电弧检测状态 (来自 DI[249])
bool ready           # 焊机准备就绪 (固定为 TRUE)
bool stick_err       # 送丝粘连错误 (固定为 FALSE)
bool general_err     # 一般错误 (来自 DI[252])
uint8 err_code       # 故障代码 (基于 DI[251]: 0=正常熔敷, 1=无熔敷)
int16 act_voltage    # 实际电压 (来自 GI[2], 原始值)
int16 act_current    # 实际电流 (来自 GI[3], 原始值)
int16 act_wire_spd   # 实际送丝速度 (固定为 0)
```

### 如何新增一个变量 (示例: `act_power` 实际功率)

下述步骤以 **新增一个 16 bit 整数 (`int16`) 变量 `act_power`** 为例进行说明，其他类型（`bool` / `uint8` / `float`）原则相同，关键是**保持发送端与接收端的字节数一致**。

1. **ROS 消息文件** (`msg/WeldState.msg`)
   1) 追加字段
   ```msg
   int16 act_power         # 实际功率 (来自 GI[4]，原始值)
   ```
   2) `catkin_make` 重新生成消息头。

2. **KAREL 程序** (`karel/ros_weld_state_reader.kl`)
   1) 在 I/O 映射常量区添加默认地址，例如：
   ```karel
   DEF_POWER_GI = 4  -- GI[4] for actual power
   ```
   2) 在 `weld_cfg_t` 结构体与 `check_cfg_()` 中加入 `power_gi` 字段并设置默认值。
   3) 在主循环 `USING weld_pkt_out_.weld_data_ DO` 内读取 GI 并填充：
   ```karel
   act_power = GIN[cfg_.power_gi]  -- GI[4]
   ```
   4) **扩展报文长度**：当前实现的载荷包含 8×`INT32` (=32 字节)。添加新变量后为 **9×`INT32` (=36 字节)**。
      * 修改头部长度：
      ```karel
      hdr_.length_ = 53  -- 原先 49 + 4 = 53
      ```
      * `iwd_srlise()` 中按原顺序 `WRITE` 新字段（仍然作为 32 bit 整数写出以保持 4 字节对齐）。

3. **C++ 接收节点**
   * **固定长度校验**
     * `fanuc_weld_state_node_tcp.cpp` 第 129–133 行和
       `fanuc_weld_state_node_simple.cpp` 第 233–286 行都校验 `length == 49`。
       将其改为 **`length == 53`**。
   * **解析载荷字节数**
     * 将 `payload_buf` 大小由 `32` 改为 `36`；循环解析数组大小由 `8` 改为 `9`。
   * **发布 ROS 消息**
     * 读取第 8 / 9 个（0-based）元素并赋值：
       ```cpp
       weld_msg.act_power = static_cast<int16_t>(weld_ints[8]);
       ```

4. **Launch 文件** (`launch/weld_state.launch`)
   * 无需改动；节点参数不变。

5. **确保大小端一致 & 重新编译**
   * KAREL 默认使用小端写入；C++ 端按 `*((int32_t*)&buf[i*4])` 解析同样为小端。只要双方一致即可。
   * 完成上述修改后，重新 `catkin_make`，部署新 `.kl` 程序并启动节点。

**`length` 字段始终 = `msg_type` 之后所有字节数**。每新增一个 `INT32` 字段，`length` 与 `payload` 均需 **+4**，否则接收端会提示 *"Invalid header format"*。

## 故障排除

### 连接问题
1. **检查网络连接**:
   ```bash
   ping 192.168.1.31  # 替换为实际机器人IP
   ```

2. **检查端口连通性**:
   ```bash
   telnet 192.168.1.31 11002
   ```

3. **检查 KAREL 程序状态**:
   - 确认程序正在运行
   - 检查 `cfg_.checked = TRUE`
   - 查看示教器上的日志输出

### 数据问题
1. **验证 I/O 映射**:
   - 在示教器上检查 `GI[2]`, `GI[3]` 的值
   - 确认 `DI[249]`, `DI[251]`, `DI[252]` 的状态

2. **检查现场总线配置**:
   - 确认 EWM 焊机与机器人的现场总线连接正常
   - 验证 I/O 地址映射是否正确

3. **监控原始数据**:
   ```bash
   # 启用调试日志
   rosrun fanuc_driver weld_state __log_level:=debug
   ```

### 常见错误及解决方案

#### 1. "Failed to connect" 错误
- 检查机器人 IP 地址是否正确
- 确认机器人防火墙设置
- 验证 KAREL 程序是否运行

#### 2. "Invalid header format" 警告
- 检查 KAREL 程序的消息格式
- 确认使用的是正确版本的程序

#### 3. 数据值异常
- 在示教器上手动检查对应 I/O 的值
- 确认现场总线配置正确
- 检查 EWM 焊机的连接状态

## 测试建议

### 1. 基本连接测试
```bash
# 启动节点
roslaunch fanuc_driver weld_state.launch robot_ip:=YOUR_ROBOT_IP

# 检查话题
rostopic list | grep weld_state
# 应能看到 /weld_state

# 检查消息
rostopic echo /weld_state -n 1
```

### 2. 数据验证测试
```bash
# 持续监控状态
rostopic echo /weld_state

# 在示教器上手动改变 GI[2], GI[3] 的值
# 观察 ROS 话题中 act_voltage, act_current 是否相应变化

# 手动改变 DI[249], DI[251], DI[252] 的状态
# 观察 arc_ok, err_code, general_err 是否相应变化
```

### 3. 性能测试
```bash
# 检查消息频率
rostopic hz /weld_state
# 应显示约 10Hz (与 KAREL 程序中的 LOOP_HZ 一致)

# 检查消息延迟
rostopic delay /weld_state
```

## 扩展功能

当前版本为基础监控版本。未来可扩展的功能包括：

1. **控制指令支持**: 添加 ROS -> Robot 的控制指令
2. **更多 I/O 映射**: 支持更多焊接参数的读取和控制
3. **自动缩放**: 在 ROS 节点中自动应用物理单位缩放
4. **故障诊断**: 更详细的错误代码解析和故障诊断
5. **参数动态配置**: 支持运行时修改 I/O 映射和其他参数 