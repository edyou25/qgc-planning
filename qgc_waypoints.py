from pymavlink import mavutil
import geojson
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import threading
import time
import matplotlib
from datetime import datetime
import sys  # 添加系统模块导入

# 设置matplotlib支持中文
matplotlib.rcParams['font.sans-serif'] = ['Arial', 'DejaVu Sans', 'SimHei']  # 尝试先使用通用英文字体
matplotlib.rcParams['axes.unicode_minus'] = False
matplotlib.rcParams['figure.figsize'] = [14, 10]
matplotlib.rcParams['figure.autolayout'] = True

# 设置backend为TkAgg，提高绘图响应速度
try:
    matplotlib.use('TkAgg')  # 这个后端在交互性和响应速度方面表现更好
except:
    pass  # 如果已经初始化了其他后端，可能会失败，但不影响程序

# 配置
WSL_IP = "0.0.0.0"   # 监听所有接口
QGC_PORT = 14550      # 默认QGC端口
SIM_PORT = 14540      # PX4 SITL默认端口
WINDOWS_IP = "172.26.48.1"  # Windows 在 WSL 网络中的 IP 地址
UPDATE_INTERVAL = 300  # 更新间隔（毫秒）降低刷新率以减少CPU负载

# 增加一个终止标志，用于在主循环中优雅退出
terminate = False

# 全局变量
current_position = {'lat': None, 'lon': None, 'alt': None}
position_history = {'lat': [], 'lon': [], 'alt': [], 'time': []}
waypoints_global = []
waypoints_optimized_global = []
fig = None
ax = None
info_ax = None
lock = threading.Lock()
start_time = time.time()
status_messages = []
max_status_messages = 10

def handle_close(evt):
    """处理窗口关闭事件"""
    global terminate
    terminate = True
    print("Window closed, program will exit...")
    plt.close('all')

# 路径优化函数
def optimize_waypoints(wps):
    # 示例：高度增加 5m
    return [{**wp, 'z': wp['z'] + 5} for wp in wps]

# 可视化函数
def init_plot():
    """初始化matplotlib图形"""
    global fig, ax, info_ax
    plt.ion()
    
    # 创建适当大小的图形，确保响应性
    fig = plt.figure(figsize=(12, 8), dpi=100)
    fig.set_tight_layout(True)
    
    gs = fig.add_gridspec(1, 4)
    ax = fig.add_subplot(gs[0, :3])  # 主图占3/4
    info_ax = fig.add_subplot(gs[0, 3])  # 信息面板占1/4
    
    fig.canvas.manager.set_window_title('QGC Waypoint Visualization System')
    
    # 添加窗口关闭事件处理
    fig.canvas.mpl_connect('close_event', handle_close)
    
    # 初始化坐标轴
    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')
    ax.set_title('QGC Waypoints & Live Position')
    ax.grid(True)
    
    info_ax.axis('off')
    
    # 确保画布刷新
    fig.canvas.draw()
    plt.pause(0.1)
    
    return fig, ax, info_ax

def add_status_message(message):
    """添加状态消息"""
    global status_messages, max_status_messages
    timestamp = datetime.now().strftime("%H:%M:%S")
    status_messages.append(f"[{timestamp}] {message}")
    if len(status_messages) > max_status_messages:
        status_messages = status_messages[-max_status_messages:]
    print(f"Status: {message}")

def update_plot(frame):
    """更新图形数据"""
    global ax, info_ax, waypoints_global, waypoints_optimized_global, position_history, lock
    
    try:
        current_time = time.time()
        elapsed_time = max(0.001, current_time - start_time)
        
        with lock:
            # 清除之前的内容
            ax.clear()
            info_ax.clear()
            
            # 只有在有航点的情况下才绘制航点
            if waypoints_global:
                # 绘制航点
                waypoint_x = [wp['x'] for wp in waypoints_global]
                waypoint_y = [wp['y'] for wp in waypoints_global]
                ax.plot(waypoint_x, waypoint_y, 'bo-', label='Original WP')
                
                # 标记航点序号 (简化处理，只标记前10个点，减少渲染压力)
                for i, (x, y) in enumerate(zip(waypoint_x[:min(10, len(waypoint_x))], waypoint_y[:min(10, len(waypoint_y))])):
                    ax.annotate(str(i), (x, y), textcoords="offset points", 
                                xytext=(0, 10), ha='center')
            
                # 绘制优化后的航点
                if waypoints_optimized_global:
                    opt_waypoint_x = [wp['x'] for wp in waypoints_optimized_global]
                    opt_waypoint_y = [wp['y'] for wp in waypoints_optimized_global]
                    ax.plot(opt_waypoint_x, opt_waypoint_y, 'ro--', label='Optimized WP')
                
                # 绘制实时位置 - 简化绘图以提高性能
                if position_history['lat'] and position_history['lon']:
                    # 使用更大的步长简化轨迹绘制，减少图形元素数量
                    step = max(1, len(position_history['lat']) // 10)
                    ax.plot(position_history['lon'][::step], position_history['lat'][::step], 
                            'g-', alpha=0.5, linewidth=1.5, label='Flight Path')
                    
                    # 绘制当前位置
                    ax.plot(position_history['lon'][-1], position_history['lat'][-1], 
                            'g*', markersize=10, label='Current Pos')
                
                # 添加图例，但位置更合理
                ax.legend(loc='upper right', fontsize='small')
                
                # 坐标轴标签
                ax.set_xlabel('Longitude')
                ax.set_ylabel('Latitude')
                alt_text = "Unknown" if current_position["alt"] is None else f"{current_position['alt']:.1f}"
                ax.set_title(f'QGC Waypoints & Live Position (Alt: {alt_text} m)', fontsize=10)
            else:
                # 当没有航点时，只显示基本坐标系
                ax.set_title('Waiting for waypoint data...')
                ax.set_xlabel('Longitude')
                ax.set_ylabel('Latitude')
            
            # 添加网格线，提高可读性
            ax.grid(True, linestyle='--', alpha=0.7)
            
            # 更新信息面板 - 简化信息显示
            info_ax.axis('off')
            
            # 构建信息文本，但不要太复杂
            info_text = [
                "System Status",
                "="*20,
                f"Runtime: {int(elapsed_time//3600):02d}:{int((elapsed_time%3600)//60):02d}:{int(elapsed_time%60):02d}",
            ]
            
            # 添加航点信息
            if waypoints_global:
                info_text.append(f"WP Count: {len(waypoints_global)}")
            else:
                info_text.extend([
                    f"* Waiting for WP data...",
                    f"* Wait time: {int(elapsed_time)}s",
                    "="*20,
                    "Tip: Create mission in QGC",
                    "Setup comm link as instructed"
                ])
            
            # 添加当前位置信息
            if current_position['lat'] is not None:
                info_text.extend([
                    "="*20,
                    "* Current Position:",
                    f"* Lat: {current_position['lat']:.6f}",
                    f"* Lon: {current_position['lon']:.6f}", 
                    f"* Alt: {current_position['alt']:.1f} m",
                ])
            else:
                info_text.extend([
                    "="*20,
                    "* No position data",
                    "Ensure UAV connected to QGC"
                ])
            
            # 添加状态消息 (只显示最后一条以减少渲染负担)
            if status_messages:
                info_text.extend([
                    "="*20,
                    "* Latest Status:"
                ])
                info_text.append(status_messages[-1])
            
            # 绘制信息文本 (简化文本显示)
            info_ax.text(0.05, 0.95, '\n'.join(info_text), 
                    verticalalignment='top', horizontalalignment='left',
                    transform=info_ax.transAxes, fontsize=9,
                    bbox={'boxstyle': 'round', 'facecolor': 'wheat', 'alpha': 0.5})
        
        # 不使用blit模式，返回空列表
        return []
    except Exception as e:
        print(f"Plot update error: {e}")
        return []

def position_listener(connection):
    """监听位置更新"""
    global current_position, position_history, lock
    
    print("Starting position monitoring...")
    add_status_message("Starting position monitoring")
    
    # 请求位置数据流
    try:
        # 请求位置数据
        connection.mav.request_data_stream_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            10,  # 10 Hz
            1    # 启用
        )
        
        # 请求扩展状态数据
        connection.mav.request_data_stream_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            5,  # 5 Hz
            1   # 启用
        )
        
        # 请求基本信息流
        connection.mav.request_data_stream_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            2,  # 2 Hz
            1   # 启用
        )
        print("Position and status data streams requested")
        add_status_message("All data streams requested")
    except Exception as e:
        print(f"Error requesting data streams: {e}")
        add_status_message(f"Data stream request failed: {str(e)[:30]}")
    
    message_count = 0
    last_status_time = time.time()
    
    while not terminate:
        try:
            # 尝试接收任何类型的消息，了解连接状态
            any_msg = connection.recv_match(blocking=False)
            if any_msg:
                message_count += 1
                
                # 每10秒打印一次状态
                current_time = time.time()
                if current_time - last_status_time > 10:
                    print(f"Received {message_count} messages")
                    last_status_time = current_time
            
            # 接收位置信息
            msg = connection.recv_match(
                type=['GLOBAL_POSITION_INT', 'GPS_RAW_INT', 'LOCAL_POSITION_NED'], 
                blocking=True, 
                timeout=0.5  # 减少超时时间，提高响应性
            )
            
            if msg:
                with lock:
                    if msg.get_type() == 'GLOBAL_POSITION_INT':
                        current_position['lat'] = msg.lat / 1e7
                        current_position['lon'] = msg.lon / 1e7
                        current_position['alt'] = msg.alt / 1000
                        # 减少位置打印，降低控制台输出量
                        if message_count % 20 == 0:  # 每20条消息打印一次
                            print(f"Position: Lat={current_position['lat']:.6f}, Lon={current_position['lon']:.6f}")
                    
                    elif msg.get_type() == 'GPS_RAW_INT':
                        current_position['lat'] = msg.lat / 1e7
                        current_position['lon'] = msg.lon / 1e7
                        current_position['alt'] = msg.alt / 1000
                    
                    # 添加到历史记录
                    if current_position['lat'] is not None:
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
        except Exception as e:
            if not str(e).startswith('timeout on') and not str(e) == 'None':  # 忽略常见的超时错误
                print(f"Position monitoring error: {e}")
            time.sleep(0.1)  # 减少CPU使用

# 主程序
print(f"Connecting to QGC, listening on {WSL_IP}:{QGC_PORT} ...")
print(f"In QGC, please configure communication link:")
print(f"1. Click gear icon (⚙️)")
print(f"2. Select 'Comm Links' menu")
print(f"3. Click 'Add' to create a new link")
print(f"4. Set type: UDP")
print(f"5. Add server URL - Host: {WINDOWS_IP}, Port: {QGC_PORT}")
print(f"6. Click OK and connect this link")

# 初始化可视化
init_plot()
add_status_message("Visualization system initialized")

# 建立连接
connections = []
ports = [QGC_PORT, SIM_PORT]
for port in ports:
    try:
        conn = mavutil.mavlink_connection(f'udp:{WSL_IP}:{port}')
        connections.append(conn)
        print(f"Connection to port {port} successful")
    except Exception as e:
        print(f"Connection to port {port} failed: {e}")

if not connections:
    print("Warning: Unable to establish connection")
    add_status_message("Warning: Unable to establish connection")
else:
    connection = connections[0]  # 使用第一个连接作为主连接
    print("Connection established")
    add_status_message("Connection established")

    # 设置目标系统和组件ID
    connection.target_system = 1
    connection.target_component = 1

    # 启动位置监听线程
    position_thread = threading.Thread(target=position_listener, args=(connection,), daemon=True)
    position_thread.start()
    print("Position monitoring thread started")

    # 请求航点
    print("Requesting mission waypoint list...")
    connection.mav.mission_request_list_send(connection.target_system, connection.target_component)

    # 接收航点
    waypoints = []
    mission_count = None
    mission_timeout = 1  # 减少超时时间，加快界面显示
    start_time = time.time()

    print("Attempting to receive waypoint data...")
    add_status_message("Attempting to receive waypoint data")

    while time.time() - start_time < mission_timeout:
        msg = connection.recv_match(type=['MISSION_ITEM', 'MISSION_COUNT'], blocking=False)
        if msg is None:
            time.sleep(0.1)
            continue

        if msg.get_type() == 'MISSION_COUNT':
            mission_count = msg.count
            print(f"Mission contains {mission_count} waypoints")
            add_status_message(f"Detected {mission_count} waypoints")
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
            print(f"Received waypoint #{msg.seq}")
            
            with lock:
                waypoints_global = waypoints.copy()

            if mission_count is not None and len(waypoints) == mission_count:
                break

    # 优化路径
    if waypoints:
        optimized_waypoints = optimize_waypoints(waypoints)
        print("Path optimization completed")
        
        with lock:
            waypoints_optimized_global = optimized_waypoints.copy()
        
        # 保存为GeoJSON
        features = [
            geojson.Feature(geometry=geojson.Point((wp['x'], wp['y'])), properties={"seq": wp['seq'], "z": wp['z']})
            for wp in optimized_waypoints
        ]
        feature_collection = geojson.FeatureCollection(features)
        with open("optimized_waypoints.geojson", "w") as f:
            geojson.dump(feature_collection, f, indent=2)
        print("GeoJSON saved to optimized_waypoints.geojson")
    else:
        print("No waypoints received")

# 启动可视化
try:
    print("Starting real-time visualization...")
    add_status_message("Visualization interface started")
    
    # 重置开始时间，使计时从可视化启动时开始
    start_time = time.time()
    frame_count = 0
    
    # 使用非阻塞模式显示图形
    plt.ion()  # 交互模式打开
    
    # 使用更简单的手动更新循环，减少绘图负担
    plt.show(block=False)
    
    # 保持程序运行直到用户关闭窗口或按Ctrl+C
    print("Visualization interface running, press Ctrl+C to exit...")
    last_update = time.time()
    
    try:
        while plt.get_fignums() and not terminate:  # 当图形窗口存在且未终止时
            current_time = time.time()
            
            # 控制更新频率
            if current_time - last_update >= UPDATE_INTERVAL / 1000.0:
                update_plot(frame_count)
                frame_count += 1
                last_update = current_time
                
                # 确保处理所有挂起的GUI事件，使界面保持响应
                fig.canvas.start_event_loop(0.001)  # 让GUI处理事件但不阻塞
            
            # 短暂睡眠，减少CPU使用
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("User terminated program")
        
except Exception as e:
    print(f"Visualization error: {e}")
    import traceback
    traceback.print_exc()
    
    # 尝试使用备选方案显示界面
    try:
        plt.ioff()  # 关闭交互模式
        plt.show()  # 尝试阻塞模式显示
    except:
        pass
    
finally:
    # 确保程序正常退出
    print("Program exiting")
    terminate = True