# Planning-QGC-PX4-AirSim


## 通信架构概述

1. **PX4 (WSL)** → 向QGC发送飞行数据
2. **QGC (WSL)** → 处理数据并转发到Python
3. **Python监听程序** → 接收、分析和优化航点数据
4. **AirSim (Windows)** → 可能通过GeoJSON文件或直接MAVLink连接获取数据


## 核心 MAVLink 消息速览
名称(ID)  关键字段 / 作用

### 基础 / 系统
- HEARTBEAT(0)  type, autopilot, base_mode, system_status  心跳/状态机基准
- SYS_STATUS(1)  sensors_present/active, voltage_battery, load  基本健康
- SYSTEM_TIME(2)  time_unix_usec  时间同步
- STATUSTEXT(253)  severity, text  文本告警

### 时延/测试
- PING(4)  seq, time_usec  往返延迟测量

### 参数
- PARAM_REQUEST_LIST(21)  请求全部参数
- PARAM_VALUE(22)  param_id, value, index, count  返回参数条目
- PARAM_SET(23)  修改参数

### 位置 / 姿态 / 传感
- LOCAL_POSITION_NED(32)  x,y,z,vx,vy,vz  本地 NED 位置速度
- GLOBAL_POSITION_INT(33)  lat,lon,alt,relative_alt,vx,vy,vz  全球+相对高
- ATTITUDE(30)  roll,pitch,yaw, *rate  欧拉姿态
- ALTITUDE(141)  altitude_amsl, altitude_relative  多参考高度
- HIGHRES_IMU(105)  加速度/陀螺/磁/气压/温度
- GPS_RAW_INT(24)  lat,lon,alt, eph,epv, fix_type, satellites_visible  原始 GPS
- ODOMETRY(331)  姿态+位置+速度融合

### 任务 (Mission)
- MISSION_COUNT(44)  count  航点总数
- MISSION_ITEM(39) / MISSION_ITEM_INT(73)  seq, command, x,y,z  单航点(后者高精度)
- MISSION_REQUEST(40)  seq  请求指定航点
- MISSION_CURRENT(42)  seq  当前执行航点
- MISSION_ITEM_REACHED(46)  seq  已到达航点
- MISSION_ACK(47)  type  任务处理结果

### 控制 / 期望
- POSITION_TARGET_LOCAL_NED(85)  setpoint  本地位置/速度/加速度/航向期望
- ATTITUDE_TARGET(83)  quaternion, body_rates, thrust  姿态期望
- SERVO_OUTPUT_RAW(36)  servo1_raw..  PWM 输出
- RC_CHANNELS(65)  chan_raw  遥控输入

### 指令
- COMMAND_LONG(76)  command, param1..7  通用命令
- COMMAND_ACK(77)  result  指令反馈

### HUD / 状态补充
- VFR_HUD(74)  airspeed, groundspeed, alt, climb, heading, throttle

### 数据流/时间同步 (较少直接用)
- REQUEST_DATA_STREAM(66)  请求旧式数据流 (PX4 新版多用 MESSAGE_INTERVAL 指令)
- TIMESYNC(111)  高频时间基准

## 常见用法提示
- 高频本地位置: 订阅 LOCAL_POSITION_NED 与 ODOMETRY 任选一主用，避免重复绘制。
- 任务下载顺序: MISSION_REQUEST_LIST → MISSION_COUNT → 逐个 MISSION_REQUEST → MISSION_ITEM(_INT) → MISSION_ACK。
- 调整输出频率: 使用 COMMAND_LONG + MAV_CMD_SET_MESSAGE_INTERVAL (取代 REQUEST_DATA_STREAM)。
- 高度一致性: 若 GLOBAL_POSITION_INT 与 ALTITUDE 同时使用，统一 relative_alt 为主图，ALTITUDE 提供 AMSL 校验。

## 典型调试命令 (PX4 Shell)
```
mavlink status
listener vehicle_local_position
listener sensor_combined
mavlink stream -u 14550 -s LOCAL_POSITION_NED -r 30
```

## 可视化脚本参数简表
--show-vel / --show-imu / --show-alt / --show-gps / --show-servo 按需开启子图
--mission-request  启动时请求全任务
--time-window N    滑动窗口 (秒)
--window N         缓冲最大点数

## 最小启动流程
1) 启动 PX4:  make px4_sitl_default none_iris
2) (可选) 开第二链路: mavlink start -u 14552 -o 14551 -m onboard -r 400000 -t 127.0.0.1 -f
3) 启动可视化: python main.py --conn udp:0.0.0.0:14551 --show-vel --show-imu --show-alt --mission-request

