from pymavlink import mavutil
import geojson
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import threading
import time

# --------------------------
# 配置
# --------------------------
WSL_IP = "0.0.0.0"   # 监听所有接口
QGC_PORT = 14550
WINDOWS_IP = "172.26.48.1"  # Windows 在 WSL 网络中的 IP 地址
DEBUG = True  # 启用调试模式
REALTIME_PLOT = True  # 是否启用实时可视化
UPDATE_INTERVAL = 500  # 更新间隔（毫秒）
LISTEN_POSITION = True  # 是否监听位置信息
SIMULATION_MODE = True  # 启用模拟模式，不等待真实的心跳
SKIP_HEARTBEAT = True  # 跳过心跳等待

# 全局变量
current_position = {'lat': None, 'lon': None, 'alt': None}
position_history = {'lat': [], 'lon': [], 'alt': [], 'time': []}
waypoints_global = []
waypoints_optimized_global = []
fig = None
ax = None
animation = None
lock = threading.Lock()  # 线程锁，用于安全访问共享数据

# --------------------------
# 示例路径优化函数
# --------------------------
def optimize_waypoints(wps):
    # 示例：高度增加 5m
    return [{**wp, 'z': wp['z'] + 5} for wp in wps]

# --------------------------
# 可视化函数
# --------------------------
def init_plot():
    """初始化matplotlib图形"""
    global fig, ax
    plt.ion()  # 启用交互模式
    fig, ax = plt.subplots(figsize=(10, 8))
    fig.canvas.manager.set_window_title('QGC航点可视化')
    
    # 添加图例和标签
    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')
    ax.set_title('QGC Mission Waypoints & Real-time Position')
    ax.grid(True)
    
    return fig, ax

def update_plot(frame):
    """更新图形数据"""
    global ax, waypoints_global, waypoints_optimized_global, position_history, lock
    
    with lock:
        # 清除当前图形
        ax.clear()
        
        # 绘制航点
        if waypoints_global:
            waypoint_x = [wp['x'] for wp in waypoints_global]
            waypoint_y = [wp['y'] for wp in waypoints_global]
            ax.plot(waypoint_x, waypoint_y, 'bo-', label='原始航点')
            
            # 标记航点序号
            for i, (x, y) in enumerate(zip(waypoint_x, waypoint_y)):
                ax.annotate(str(i), (x, y), textcoords="offset points", 
                            xytext=(0, 10), ha='center')
        
        # 绘制优化后的航点
        if waypoints_optimized_global:
            opt_waypoint_x = [wp['x'] for wp in waypoints_optimized_global]
            opt_waypoint_y = [wp['y'] for wp in waypoints_optimized_global]
            ax.plot(opt_waypoint_x, opt_waypoint_y, 'ro--', label='优化航点')
        
        # 绘制实时位置历史轨迹
        if position_history['lat'] and position_history['lon']:
            ax.plot(position_history['lon'], position_history['lat'], 'g-', alpha=0.5, label='飞行轨迹')
            
            # 绘制当前位置
            if position_history['lat'][-1] and position_history['lon'][-1]:
                ax.plot(position_history['lon'][-1], position_history['lat'][-1], 'g*', markersize=10, label='当前位置')
        
        # 添加图例和网格
        ax.legend(loc='best')
        ax.grid(True)
        
        # 自动调整坐标范围，确保所有点都可见
        if waypoints_global or position_history['lat']:
            ax.set_xlabel('Longitude')
            ax.set_ylabel('Latitude')
            ax.set_title(f'QGC Mission Waypoints & Position (Alt: {current_position["alt"]} m)')
    
    return ax,

def position_listener(connection):
    """在单独的线程中监听位置更新"""
    global current_position, position_history, lock
    
    print("开始监听位置信息...")
    while True:
        try:
            # 接收GLOBAL_POSITION_INT消息
            msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if msg:
                with lock:
                    # 更新当前位置（转换到度）
                    current_position['lat'] = msg.lat / 1e7  # 纬度，从整数转换为度
                    current_position['lon'] = msg.lon / 1e7  # 经度，从整数转换为度
                    current_position['alt'] = msg.alt / 1000  # 高度，从毫米转换为米
                    
                    # 添加到历史记录
                    position_history['lat'].append(current_position['lat'])
                    position_history['lon'].append(current_position['lon'])
                    position_history['alt'].append(current_position['alt'])
                    position_history['time'].append(time.time())
                    
                    # 只保留最近的100个点
                    if len(position_history['lat']) > 100:
                        position_history['lat'] = position_history['lat'][-100:]
                        position_history['lon'] = position_history['lon'][-100:]
                        position_history['alt'] = position_history['alt'][-100:]
                        position_history['time'] = position_history['time'][-100:]
                    
                    if DEBUG:
                        print(f"Position: Lat={current_position['lat']:.6f}, Lon={current_position['lon']:.6f}, Alt={current_position['alt']:.2f}m")
        except Exception as e:
            if DEBUG:
                print(f"Position listener error: {e}")
            time.sleep(0.5)

# --------------------------
# 连接 QGC (WSL 中)
# --------------------------
print(f"Connecting to QGC, listening on {WSL_IP}:{QGC_PORT} ...")
print(f"IMPORTANT: In QGC, follow these steps:")
print(f"1. Click the gear icon (⚙️) at the top of the QGC window")
print(f"2. Select 'Comm Links' from the left menu")
print(f"3. Click 'Add' to create a new link")
print(f"4. Set Type: UDP")
print(f"5. Add server URL - Host: {WINDOWS_IP}, Port: {QGC_PORT}")
print(f"6. Click OK and then Connect to this link")
print("\nWaiting for QGC to connect...\n")

# 初始化可视化（如果启用）
if REALTIME_PLOT:
    init_plot()

# 尝试不同的连接方法
try:
    print("Trying udp connection...")
    connection = mavutil.mavlink_connection(f'udp:{WSL_IP}:{QGC_PORT}')
    print("UDP connection established")
except Exception as e:
    print(f"UDP connection failed: {e}")
    print("Trying udpin connection...")
    try:
        connection = mavutil.mavlink_connection(f'udpin:{WSL_IP}:{QGC_PORT}')
        print("UDPIN connection established")
    except Exception as e:
        print(f"UDPIN connection failed: {e}")
        raise Exception("Could not establish any connection. Please verify QGC settings.")

print("Waiting for heartbeat from QGC...")
print("This could take a minute. Make sure QGC is connected to the comm link.")

# 添加超时机制
import time
start_time = time.time()
timeout = 60  # 60秒超时
got_heartbeat = False

# 如果设置跳过心跳等待，直接假设心跳已收到
if SKIP_HEARTBEAT:
    print("跳过心跳等待，直接继续...")
    got_heartbeat = True
else:
    while time.time() - start_time < timeout:
        if DEBUG:
            print(f"Waiting for heartbeat... (elapsed: {int(time.time() - start_time)}s)")
        try:
            msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
            if msg:
                print("Heartbeat received!")
                got_heartbeat = True
                break
        except Exception as e:
            print(f"Error waiting for heartbeat: {e}")
            time.sleep(1)

if not got_heartbeat:
    print("\nNo heartbeat received within timeout period.")
    print("Possible issues:")
    print("1. QGC is not configured to send MAVLink data to your Windows IP")
    print("2. A firewall is blocking the connection")
    print("3. Wrong IP address or port")
    print("\nTrying to continue anyway...\n")
else:
    print("Heartbeat received! Connection established.")

# 如果启用位置监听，启动监听线程
if LISTEN_POSITION and got_heartbeat:
    position_thread = threading.Thread(target=position_listener, args=(connection,), daemon=True)
    position_thread.start()
    print("Position listener started in background")

# --------------------------
# 请求 Mission 列表
# --------------------------
print("Requesting mission list from QGC...")
# 设置一个默认的系统和组件ID，如果没有收到心跳
if not hasattr(connection, 'target_system') or connection.target_system == 0:
    print("No target system set, using default (1, 1)")
    connection.target_system = 1
    connection.target_component = 1

connection.mav.mission_request_list_send(connection.target_system, connection.target_component)

waypoints = []
mission_count = None
mission_timeout = 30  # 30秒超时
start_time = time.time()

while time.time() - start_time < mission_timeout:
    try:
        msg = connection.recv_match(type=['MISSION_ITEM', 'MISSION_COUNT'], blocking=True, timeout=1)
        if msg is None:
            continue

        if msg.get_type() == 'MISSION_COUNT':
            mission_count = msg.count
            print(f"Mission has {mission_count} waypoints.")
            for seq in range(mission_count):
                connection.mav.mission_request_send(connection.target_system, connection.target_component, seq)

        elif msg.get_type() == 'MISSION_ITEM':
            wp = {
                'seq': msg.seq,
                'frame': msg.frame,
                'command': msg.command,
                'x': msg.x,
                'y': msg.y,
                'z': msg.z,
            }
            waypoints.append(wp)
            print(f"Received waypoint: {wp}")
            
            # 更新全局航点列表（用于可视化）
            with lock:
                waypoints_global = waypoints.copy()

            if mission_count is not None and msg.seq == mission_count - 1:
                print("\nAll waypoints received!")
                break
    except Exception as e:
        print(f"Error receiving mission items: {e}")
        time.sleep(0.5)

if not waypoints:
    print("\nNo waypoints received within timeout period.")
    print("Possible issues:")
    print("1. No mission is loaded in QGC")
    print("2. Communication link issues")
    print("3. QGC is not configured correctly")
    print("\nCreating test waypoints for demonstration...")
    
    # 创建一些示例航点
    test_waypoints = [
        {'seq': 0, 'frame': 0, 'command': 16, 'x': 47.3977419, 'y': 8.5455939, 'z': 50.0},
        {'seq': 1, 'frame': 0, 'command': 16, 'x': 47.3976419, 'y': 8.5465939, 'z': 50.0},
        {'seq': 2, 'frame': 0, 'command': 16, 'x': 47.3975419, 'y': 8.5475939, 'z': 50.0},
    ]
    waypoints = test_waypoints
    print("Created test waypoints.")
    
    # 更新全局航点列表（用于可视化）
    with lock:
        waypoints_global = waypoints.copy()

# --------------------------
# 优化路径
# --------------------------
optimized_waypoints = optimize_waypoints(waypoints)
print("\nOptimized waypoints:")
for wp in optimized_waypoints:
    print(wp)

# 更新全局优化航点列表（用于可视化）
with lock:
    waypoints_optimized_global = optimized_waypoints.copy()

# --------------------------
# 启动实时可视化动画
# --------------------------
if REALTIME_PLOT:
    print("\n启动实时可视化...")
    animation = FuncAnimation(fig, update_plot, interval=UPDATE_INTERVAL, blit=True)
    plt.show()
    
# --------------------------
# 输出 GeoJSON
# --------------------------
features = [
    geojson.Feature(geometry=geojson.Point((wp['x'], wp['y'])), properties={"seq": wp['seq'], "z": wp['z']})
    for wp in optimized_waypoints
]

feature_collection = geojson.FeatureCollection(features)
with open("optimized_waypoints.geojson", "w") as f:
    geojson.dump(feature_collection, f, indent=2)

print("\nGeoJSON saved to optimized_waypoints.geojson")

# --------------------------
# 保持程序运行，直到用户退出
# --------------------------
if REALTIME_PLOT:
    print("\n实时可视化已启动。按 Ctrl+C 退出程序...")
    try:
        # 等待用户手动终止程序
        plt.show(block=True)
    except KeyboardInterrupt:
        print("\n用户终止程序.")
    finally:
        print("程序结束.")
