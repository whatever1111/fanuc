# Fanuc机器人端口配置指南

## 📋 概述

Fanuc机器人的ROS通信需要在控制器上运行KAREL程序来监听特定端口。标准配置使用：
- **端口 11000**: 运动控制（Motion/Command）
- **端口 11002**: 状态反馈（State）

## 🔧 前置要求

1. **KAREL选项**: 控制器必须安装KAREL选项（R632）
2. **用户Socket通信选项**: 需要User Socket Messaging选项（R636）
3. **控制器访问权限**: 需要能够加载和运行KAREL程序

## 📝 步骤1：检查KAREL程序

### 1.1 查看已安装的KAREL程序

在示教器上：
1. 按 **MENU**
2. 选择 **FILE**
3. 查找以下文件：
   - `ROS_RELAY.kl` 或 `ros_relay.kl`
   - `ROS_STATE.kl` 或 `ros_state.kl`

如果这些文件不存在，需要先传输和编译KAREL源代码。

## 📝 步骤2：KAREL程序配置

### 2.1 ROS_STATE程序（端口11002）

查看KAREL源代码中的端口配置：

```karel
-- ros_state.kl
PROGRAM ros_state
%STACKSIZE = 4000
%NOLOCKGROUP
%NOPAUSE = ERROR + COMMAND + TPENABLE
%COMMENT = 'ROS State Server'

CONST
  -- 配置状态服务器端口
  STATE_PORT = 11002  -- 这里定义端口号
  LOOP_HZ = 40        -- 发送频率

VAR
  sock_fd : FILE
  status : INTEGER
  
BEGIN
  -- 创建服务器socket
  SET_FILE_ATR(sock_fd, ATR_IA)
  -- 绑定到端口11002
  OPEN FILE sock_fd ('S', 'tcp_server:11002')
  
  -- 主循环
  WHILE TRUE DO
    -- 发送关节位置
    -- 发送机器人状态
  ENDWHILE
END ros_state
```

### 2.2 ROS_RELAY程序（端口11000）

```karel
-- ros_relay.kl
PROGRAM ros_relay
CONST
  -- 配置运动服务器端口
  MOTION_PORT = 11000  -- 这里定义端口号
  
BEGIN
  -- 创建服务器socket
  OPEN FILE sock_fd ('S', 'tcp_server:11000')
  -- 接收运动命令
END ros_relay
```

## 📝 步骤3：修改端口号（如需要）

### 3.1 使用示教器编辑

如果需要修改端口号：

1. **打开程序编辑器**
   - MENU → SELECT → 选择KAREL程序
   - 按 **EDIT**

2. **找到端口定义**
   - 查找 `STATE_PORT` 或 `11002`
   - 修改为所需端口号

3. **重新编译**
   - 保存修改
   - 需要重新编译KAREL程序

### 3.2 使用KCL命令行

在示教器KCL界面：
```
KCL> SET PORT STATE_PORT 11002
KCL> SET PORT MOTION_PORT 11000
```

## 📝 步骤4：启动KAREL程序

### 4.1 手动启动

1. 按 **SELECT**
2. 选择 `ROS_STATE`
3. 按 **SHIFT+RESET** （清除故障）
4. 按 **SHIFT+FWD** （启动程序）

重复上述步骤启动 `ROS_RELAY`

### 4.2 自动启动配置

配置程序自动启动：

1. **MENU** → **SETUP** → **PROG RUN SETUP**
2. 找到 **AUTO START**
3. 添加程序：
   ```
   1. ROS_STATE
   2. ROS_RELAY
   ```

## 📝 步骤5：网络配置

### 5.1 检查TCP/IP设置

1. **MENU** → **SETUP** → **Host Comm** → **TCP/IP**
2. 配置：
   ```
   Robot name: ROBOT
   IP address: 192.168.0.99  (您的机器人IP)
   Subnet mask: 255.255.255.0
   Router IP: 192.168.0.1
   ```

### 5.2 启用Socket通信

1. **MENU** → **SETUP** → **Host Comm** → **SETUP**
2. 确保以下设置：
   ```
   TCP/IP: ENABLE
   Socket Msg: ENABLE
   ```

## 📝 步骤6：防火墙设置

### 6.1 检查端口访问

在控制器设置中确保端口未被阻止：

1. **MENU** → **SETUP** → **Host Comm** → **FIREWALL**
2. 添加规则（如果有防火墙功能）：
   ```
   Allow IN TCP 11000
   Allow IN TCP 11002
   ```

## 🔍 故障排除

### 问题1：端口无法绑定

**错误信息**: "Socket bind error" 或 "Port already in use"

**解决方法**:
- 检查是否有其他程序使用该端口
- 尝试使用其他端口号
- 重启控制器

### 问题2：KAREL程序异常终止

**错误信息**: "ABORTED" 状态

**解决方法**:
1. 检查错误日志：**MENU** → **ALARM** → **HIST**
2. 常见原因：
   - Socket选项未安装
   - 内存不足
   - 语法错误

### 问题3：连接被拒绝

**症状**: PC可以ping通机器人但连接端口失败

**检查清单**:
- [ ] KAREL程序正在运行
- [ ] 端口号正确
- [ ] 机器人在AUTO模式
- [ ] 网络配置正确
- [ ] Socket通信已启用

## 📦 完整的KAREL程序示例

如果需要创建新的KAREL程序，这里是最小示例：

### 简单的状态服务器（ros_state_simple.kl）

```karel
PROGRAM ros_state_simple
%STACKSIZE = 4000
%NOLOCKGROUP
%NOPAUSE = ERROR + COMMAND + TPENABLE

VAR
  sock_fd : FILE
  client_fd : FILE
  status : INTEGER
  joint_pos : JOINTPOS
  data : STRING[254]
  
BEGIN
  -- 打开服务器socket
  SET_FILE_ATR(sock_fd, ATR_IA)
  SET_FILE_ATR(sock_fd, ATR_UF)
  
  -- 监听端口11002
  OPEN FILE sock_fd ('S', 'tcp_server:11002')
  
  WRITE('State server started on port 11002', CR)
  
  -- 接受连接
  WHILE TRUE DO
    -- 等待客户端连接
    OPEN FILE client_fd ('S', sock_fd)
    WRITE('Client connected', CR)
    
    -- 发送数据循环
    WHILE TRUE DO
      -- 获取当前关节位置
      joint_pos = CURJPOS(0,0)
      
      -- 格式化并发送数据
      -- 这里需要实现SimpleMessage协议
      
      DELAY 25  -- 40Hz
    ENDWHILE
  ENDWHILE
  
  CLOSE FILE client_fd
  CLOSE FILE sock_fd
END ros_state_simple
```

## 🚀 快速测试

### 从PC测试连接

1. **使用netcat测试**:
   ```bash
   nc -v 192.168.0.99 11002
   ```

2. **使用telnet测试**:
   ```bash
   telnet 192.168.0.99 11002
   ```

3. **使用Python测试**:
   ```python
   import socket
   
   sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
   try:
       sock.connect(('192.168.0.99', 11002))
       print("Connected successfully!")
       # 尝试接收数据
       data = sock.recv(100)
       print(f"Received: {data}")
   except Exception as e:
       print(f"Connection failed: {e}")
   finally:
       sock.close()
   ```

## 📚 参考资料

### Fanuc手册
- **R-30iA/R-30iB KAREL Reference Manual** (MARRC75KR07091E)
- **Socket Messaging Option Manual** (MAROCSKT)
- **Internet Connectivity Option Manual** (MAROCINC)

### 端口约定
ROS-Industrial标准端口分配：
- 11000: Motion/Command Interface
- 11001: System Interface  
- 11002: State Interface
- 11003: I/O Interface

### KAREL Socket编程关键函数
- `OPEN FILE ... ('S', 'tcp_server:port')` - 创建服务器
- `SET_FILE_ATR()` - 设置socket属性
- `WRITE_DICT()` - 发送二进制数据
- `READ_DICT()` - 接收二进制数据

## ⚠️ 重要提示

1. **控制器重启**: 修改KAREL程序后可能需要重启控制器
2. **程序优先级**: 确保ROS程序有足够的优先级
3. **错误处理**: KAREL程序应包含适当的错误处理
4. **资源限制**: 注意控制器的连接数限制（通常最多4-8个并发连接）

---

如需更多帮助，请查阅Fanuc官方文档或联系技术支持。