# Planning-QGC-PX4-AirSim
## 通信架构概述

您的系统使用MAVLink协议通过UDP进行通信，形成了如下结构：

1. **PX4 (WSL)** → 向QGC发送飞行数据
2. **QGC (WSL)** → 处理数据并转发到Windows
3. **Windows监听程序** → 接收、分析和优化航点数据
4. **AirSim (Windows)** → 可能通过GeoJSON文件或直接MAVLink连接获取数据

## 关键通信机制分析

### 1. UDP套接字连接

代码建立了UDP监听连接，监听来自QGC的MAVLink消息：

```python
conn = mavutil.mavlink_connection(f'udp:{WSL_IP}:{port}')
```

关键配置参数包括：
- `WSL_IP = "0.0.0.0"` - 在所有网络接口上监听
- `QGC_PORT = 14550` - QGC的默认端口
- `WINDOWS_IP = "172.26.48.1"` - Windows在WSL网络中的IP地址

### 2. 多端口监听机制

代码支持同时监听多个端口，增加数据接收的成功率：

```python
if MONITOR_MULTIPLE_PORTS:
    ports_to_try = [QGC_PORT, SIM_PORT]
```

这样可以同时捕获来自QGC(14550)和PX4 SITL(14540)的数据。

### 3. 数据流请求

为了确保获取实时数据，代码主动向PX4/QGC请求特定类型的数据流：

```python
conn.mav.request_data_stream_send(
    conn.target_system,
    conn.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION,  # 位置数据
    10,  # 10 Hz频率
    1    # 启用
)
```

这确保了位置、状态和其他重要数据能以足够的频率传输。

### 4. 多线程监听与处理

代码使用多线程架构确保实时性：

1. **位置监听线程**：持续接收位置更新
   ```python
   position_thread = threading.Thread(target=position_listener, args=(connection,))
   ```

2. **诊断线程**：监控通信状态和数据流
   ```python
   diagnostic_t = threading.Thread(target=diagnostic_thread_func)
   ```

3. **模拟线程**：在实际数据不可用时提供模拟位置
   ```python
   simulation_thread = threading.Thread(target=simulation_position_updater)
   ```

### 5. 航点数据交换

监听程序通过MAVLink请求和接收航点数据：

```python
conn.mav.mission_request_list_send(conn.target_system, conn.target_component)
```

然后处理响应：
```python
if msg.get_type() == 'MISSION_COUNT':
    mission_count = msg.count
elif msg.get_type() == 'MISSION_ITEM':
    # 处理航点数据
```

### 6. 线程同步机制

使用线程锁确保多线程环境下的数据一致性：

```python
with lock:
    # 安全访问和修改共享数据
    waypoints_global = waypoints.copy()
```