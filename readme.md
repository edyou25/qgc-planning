# Planning-QGC-PX4-AirSim
## 通信架构概述

1. **PX4 (WSL)** → 向QGC发送飞行数据
2. **QGC (WSL)** → 处理数据并转发到Windows
3. **Windows监听程序** → 接收、分析和优化航点数据
4. **AirSim (Windows)** → 可能通过GeoJSON文件或直接MAVLink连接获取数据

## Mavlink字段

# MAVLink 主要消息类型

MAVLink协议包含大量消息类型，用于无人机系统的通信。以下是一些最常用的消息类型分类：

## 基础状态与控制消息

1. **HEARTBEAT** (ID=0)
   - 系统心跳，包含系统类型、自动驾驶仪类型、系统状态等
   - 所有MAVLink系统必须每秒发送至少1次

2. **SYS_STATUS** (ID=1)
   - 系统状态，包含电池、通信和传感器状态

3. **SYSTEM_TIME** (ID=2)
   - 同步系统时间

4. **PING** (ID=4)
   - 测试通信延迟和可靠性

### 导航与位置消息

1. **GLOBAL_POSITION_INT** (ID=33)
   - 包含全球位置、高度和速度
   - 纬度/经度以1E7度为单位，高度以毫米为单位

2. **LOCAL_POSITION_NED** (ID=32)
   - 北-东-下坐标系中的位置

3. **GPS_RAW_INT** (ID=24)
   - GPS接收机的原始数据

4. **ATTITUDE** (ID=30)
   - 飞行器姿态(滚转、俯仰、偏航)

5. **ALTITUDE** (ID=141)
   - 不同参考系的高度数据

### 任务规划消息

1. **MISSION_COUNT** (ID=44)
   - 指示任务包含多少个航点

2. **MISSION_ITEM** (ID=39)
   - 单个航点信息
   - 包含位置、命令类型、参数等

3. **MISSION_REQUEST** (ID=40)
   - 请求特定序号的航点

4. **MISSION_ACK** (ID=47)
   - 确认任务操作完成

5. **MISSION_CURRENT** (ID=42)
   - 当前执行的航点序号

6. **MISSION_ITEM_REACHED** (ID=46)
   - 到达指定航点的通知

### 命令消息

1. **COMMAND_LONG** (ID=76)
   - 发送命令到飞行器
   - 包含命令ID和最多7个参数

2. **COMMAND_ACK** (ID=77)
   - 命令确认回复

### 参数操作消息

1. **PARAM_REQUEST_LIST** (ID=21)
   - 请求所有参数列表

2. **PARAM_VALUE** (ID=22)
   - 单个参数的值

3. **PARAM_SET** (ID=23)
   - 设置参数值

### 数据流控制

1. **REQUEST_DATA_STREAM** (ID=66)
   - 请求特定类型的数据流
   - 可以设置数据流频率

2. **DATA_STREAM** (ID=67)
   - 数据流设置信息

### 扩展状态消息

1. **VFR_HUD** (ID=74)
   - 飞行数据，包括空速、地速、高度等

2. **STATUSTEXT** (ID=253)
   - 状态文本消息，用于日志和警告

3. **RC_CHANNELS** (ID=65)
   - 遥控通道值
