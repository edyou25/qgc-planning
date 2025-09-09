from pymavlink import mavutil
import geojson
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import threading
import time
import matplotlib
from datetime import datetime

# 设置matplotlib支持中文
matplotlib.rcParams['font.sans-serif'] = ['SimHei']  # 用来正常显示中文标签
matplotlib.rcParams['axes.unicode_minus'] = False  # 用来正常显示负号
matplotlib.rcParams['figure.figsize'] = [14, 10]   # 设置默认图形大小
matplotlib.rcParams['figure.autolayout'] = True    # 自动调整布局
matplotlib.rcParams['toolbar'] = 'toolbar2'        # 显示工具栏

# --------------------------
# 配置
# --------------------------
WSL_IP = "0.0.0.0"   # 监听所有接口
QGC_PORT = 14550      # 默认QGC端口
SIM_PORT = 14540      # PX4 SITL默认端口
WINDOWS_IP = "172.26.48.1"  # Windows 在 WSL 网络中的 IP 地址
DEBUG = True  # 启用调试模式
REALTIME_PLOT = True  # 是否启用实时可视化
UPDATE_INTERVAL = 500  # 更新间隔（毫秒）
LISTEN_POSITION = True  # 是否监听位置信息
SIMULATION_MODE = True  # 启用模拟模式，不等待真实的心跳
SKIP_HEARTBEAT = True  # 跳过心跳等待
USE_SIM_PORT = False   # 是否使用仿真端口而不是QGC端口
MONITOR_MULTIPLE_PORTS = True  # 是否监听多个端口
REQUEST_DATA_STREAM = True  # 是否主动请求数据流

# 全局变量
current_position = {'lat': None, 'lon': None, 'alt': None}
position_history = {'lat': [], 'lon': [], 'alt': [], 'time': []}
waypoints_global = []
waypoints_optimized_global = []
fig = None
ax = None
info_ax = None  # 信息显示子图
animation = None
lock = threading.Lock()  # 线程锁，用于安全访问共享数据
connections = []  # 存储多个连接
frame_count = 0  # 帧计数器
start_time = time.time()  # 程序启动时间
status_messages = []  # 状态消息列表
max_status_messages = 10  # 最多显示的状态消息数

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
    global fig, ax, info_ax
    plt.ion()  # 启用交互模式
    
    # 创建带有两个子图的Figure
    fig = plt.figure(figsize=(14, 10))
    
    # 创建网格布局：主图占左侧和中间，信息面板占右侧
    gs = fig.add_gridspec(1, 4)
    ax = fig.add_subplot(gs[0, :3])  # 主图占3/4
    info_ax = fig.add_subplot(gs[0, 3])  # 信息面板占1/4
    
    fig.canvas.manager.set_window_title('QGC航点可视化与路径规划系统')
    
    # 主图设置
    ax.set_xlabel('经度')
    ax.set_ylabel('纬度')
    ax.set_title('QGC任务航点与实时位置')
    ax.grid(True)
    
    # 信息面板设置
    info_ax.axis('off')  # 关闭坐标轴
    
    # 阻止窗口关闭
    def on_close(event):
        print("窗口关闭事件被触发，但窗口将保持打开")
        print("请使用键盘Ctrl+C终止程序")
        # 阻止窗口关闭
        plt.show()
        return False
    
    # 注册窗口关闭事件处理器
    fig.canvas.mpl_connect('close_event', on_close)
    
    return fig, ax, info_ax

def add_status_message(message):
    """添加状态消息到状态消息列表"""
    global status_messages, max_status_messages
    timestamp = datetime.now().strftime("%H:%M:%S")
    status_messages.append(f"[{timestamp}] {message}")
    # 只保留最近的N条消息
    if len(status_messages) > max_status_messages:
        status_messages = status_messages[-max_status_messages:]
    # 输出到控制台便于调试
    print(f"状态: {message}")

def update_plot(frame):
    """更新图形数据"""
    global ax, info_ax, waypoints_global, waypoints_optimized_global, position_history, lock, frame_count
    
    try:
        frame_count += 1
        current_time = time.time()
        elapsed_time = max(0.001, current_time - start_time)  # 确保不会除以零
        
        with lock:
            # 清除当前图形
            ax.clear()
            info_ax.clear()
            
            # 只有在有航点的情况下才在左侧绘图
            if waypoints_global:
                # 绘制航点
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
                ax.set_xlabel('经度')
                ax.set_ylabel('纬度')
                alt_text = "未知" if current_position["alt"] is None else f"{current_position['alt']:.1f}"
                ax.set_title(f'QGC航点任务与实时位置 (高度: {alt_text} m)')
            else:
                # 当没有航点时，只显示基本坐标系，不绘制任何内容
                ax.set_title('等待航点数据...')
                ax.set_xlabel('经度')
                ax.set_ylabel('纬度')
                ax.grid(True)
                
                # 如果有位置信息，仍然显示当前位置
                if position_history['lat'] and position_history['lon']:
                    ax.plot(position_history['lon'][-1], position_history['lat'][-1], 'g*', markersize=10, label='当前位置')
                    ax.legend(loc='best')
            
            # 更新信息面板
            info_ax.axis('off')  # 关闭坐标轴
            
            # 构建信息文本
            info_text = [
                "系统状态信息",
                "="*20,
                f"运行时间: {int(elapsed_time//3600):02d}:{int((elapsed_time%3600)//60):02d}:{int(elapsed_time%60):02d}",
                f"帧数: {frame_count}",
                f"刷新率: {frame_count/elapsed_time:.1f} 帧/秒",
                "="*20,
            ]
            
            # 添加航点信息
            if waypoints_global:
                info_text.append(f"航点数量: {len(waypoints_global)}")
            else:
                info_text.extend([
                    f"📡 等待航点数据...",
                    f"⏱️ 已等待帧数: {frame_count}",
                    f"⏰ 等待时间: {int(elapsed_time)}秒"
                ])
                
                # 如果长时间没有收到数据，添加提示信息
                if int(elapsed_time) > 30:
                    info_text.extend([
                        "="*20,
                        "🔍 排查提示:",
                        "1. 确认QGC已连接到无人机",
                        "2. 检查QGC通信链接配置",
                        "3. 确认已加载任务航点"
                    ])
            
            # 添加当前位置信息
            if current_position['lat'] is not None:
                info_text.extend([
                    "="*20,
                    "📍 当前位置:",
                    f"🌐 纬度: {current_position['lat']:.6f}",
                    f"🌐 经度: {current_position['lon']:.6f}", 
                    f"🔼 高度: {current_position['alt']:.1f} m",
                ])
            elif int(elapsed_time) > 10:  # 如果超过10秒没有位置信息，显示等待提示
                info_text.extend([
                    "="*20,
                    "📍 等待位置数据...",
                    f"⏱️ 已等待: {int(elapsed_time)}秒",
                ])
            
            # 添加状态消息
            if status_messages:
                info_text.extend([
                    "="*20,
                    "📝 最近状态消息:"
                ])
                info_text.extend(status_messages)
            
            # 绘制信息文本和等待指示器
            info_ax.text(0.05, 0.95, '\n'.join(info_text), 
                    verticalalignment='top', horizontalalignment='left',
                    transform=info_ax.transAxes, fontsize=9,
                    bbox={'boxstyle': 'round', 'facecolor': 'wheat', 'alpha': 0.5})
            
            # 如果没有航点数据，添加醒目的等待指示
            if not waypoints_global:
                # 计算闪烁效果
                blink = (int(elapsed_time * 2) % 2 == 0)  # 每0.5秒切换一次
                color = 'red' if blink else 'orange'
                fontsize = 12
                
                # 在信息面板底部添加醒目提示
                info_ax.text(0.5, 0.05, "正在等待航点数据...", 
                          verticalalignment='bottom', horizontalalignment='center',
                          transform=info_ax.transAxes, fontsize=fontsize, fontweight='bold',
                          color=color, bbox={'boxstyle': 'round', 'facecolor': 'lightgray', 'alpha': 0.7})
        
        return ax, info_ax
    except Exception as e:
        print(f"更新图形时出错: {e}")
        add_status_message(f"图形更新错误: {str(e)[:50]}")
        # 返回空列表，避免可视化崩溃
        return []

def position_listener(connection):
    """在单独的线程中监听位置更新"""
    global current_position, position_history, lock
    
    print("开始监听位置信息...")
    add_status_message("开始监听位置信息")
    
    # 尝试显式订阅位置信息
    try:
        connection.mav.request_data_stream_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,  # 位置数据流
            10,  # 10 Hz
            1   # 启用
        )
        print("已请求位置数据流，频率10Hz")
        add_status_message("已请求位置数据流")
    except Exception as e:
        print(f"请求数据流出错: {e}")
        add_status_message(f"请求数据流出错: {str(e)[:50]}")
    
    last_message_time = time.time()
    message_count = 0
    
    while True:
        try:
            # 尝试接收多种类型的位置消息
            msg = connection.recv_match(
                type=['GLOBAL_POSITION_INT', 'GPS_RAW_INT', 'LOCAL_POSITION_NED'], 
                blocking=True, 
                timeout=1
            )
            
            current_time = time.time()
            if current_time - last_message_time > 10:  # 每10秒输出一次状态
                print(f"位置监听中... 已接收 {message_count} 条消息")
                if message_count == 0:
                    print("提示: 尝试在QGC的通信链接中检查数据传输是否已启用")
                    add_status_message("未收到位置数据，请检查QGC设置")
                last_message_time = current_time
            
            if msg:
                message_count += 1
                with lock:
                    if msg.get_type() == 'GLOBAL_POSITION_INT':
                        # 更新当前位置（转换到度）
                        current_position['lat'] = msg.lat / 1e7  # 纬度，从整数转换为度
                        current_position['lon'] = msg.lon / 1e7  # 经度，从整数转换为度
                        current_position['alt'] = msg.alt / 1000  # 高度，从毫米转换为米
                        
                        print(f"收到全球位置消息: 纬度={current_position['lat']:.6f}, 经度={current_position['lon']:.6f}")
                        add_status_message(f"位置更新: 高度={current_position['alt']:.1f}m")
                    
                    elif msg.get_type() == 'GPS_RAW_INT':
                        # 从GPS消息更新位置
                        current_position['lat'] = msg.lat / 1e7
                        current_position['lon'] = msg.lon / 1e7
                        current_position['alt'] = msg.alt / 1000
                        
                        print(f"收到GPS消息: 纬度={current_position['lat']:.6f}, 经度={current_position['lon']:.6f}")
                        add_status_message(f"GPS更新: 卫星={msg.satellites_visible}")
                    
                    elif msg.get_type() == 'LOCAL_POSITION_NED':
                        # 本地位置消息 - 记录但不直接更新坐标
                        print(f"收到本地位置消息: x={msg.x:.1f}, y={msg.y:.1f}, z={msg.z:.1f}")
                        add_status_message(f"本地位置更新: z={msg.z:.1f}m")
                    
                    # 如果成功接收到位置，添加到历史记录
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
                        
                        if DEBUG and message_count % 5 == 0:  # 减少输出频率
                            print(f"位置: 纬度={current_position['lat']:.6f}, 经度={current_position['lon']:.6f}, 高度={current_position['alt']:.2f}m")
        except Exception as e:
            if DEBUG:
                print(f"位置监听错误: {e}")
            time.sleep(0.5)

def simulation_position_updater():
    """模拟飞行器位置更新的线程函数"""
    global current_position, position_history, waypoints_global, lock
    
    print("启动模拟位置更新...")
    add_status_message("启动模拟位置更新")
    
    # 使用航点创建飞行路径
    waypoints = []
    with lock:
        waypoints = waypoints_global.copy()
    
    if not waypoints:
        print("无航点数据，使用默认位置进行模拟...")
        add_status_message("无航点数据，使用默认位置")
        
        # 设置一个默认位置但不创建航点
        default_lat = 39.9042  # 北京天安门附近
        default_lon = 116.4074
        default_alt = 100.0
        
        # 更新当前位置信息，但不修改waypoints_global
        with lock:
            current_position['lat'] = default_lat
            current_position['lon'] = default_lon
            current_position['alt'] = default_alt
        
        # 循环执行，更新状态消息但不更新位置
        while True:
            try:
                with lock:
                    # 不添加到历史记录，保持位置不变
                    if len(position_history['lat']) == 0:
                        # 仅在第一次添加位置，后续不更新
                        position_history['lat'].append(default_lat)
                        position_history['lon'].append(default_lon)
                        position_history['alt'].append(default_alt)
                        position_history['time'].append(time.time())
                
                # 每10秒更新一次状态消息
                if int(time.time()) % 10 == 0:
                    add_status_message("等待航点数据中...")
                
                time.sleep(1.0)
            except Exception as e:
                print(f"模拟位置更新错误: {e}")
                add_status_message(f"模拟错误: {str(e)[:50]}")
                time.sleep(1.0)
        
        return
    
    # 为了平滑路径，在每两个航点之间插入多个点
    smooth_path = []
    for i in range(len(waypoints) - 1):
        wp1 = waypoints[i]
        wp2 = waypoints[i + 1]
        
        # 每两个航点之间插入20个点
        for j in range(21):
            ratio = j / 20.0
            lat = wp1['x'] + (wp2['x'] - wp1['x']) * ratio
            lon = wp1['y'] + (wp2['y'] - wp1['y']) * ratio
            alt = wp1['z'] + (wp2['z'] - wp1['z']) * ratio
            smooth_path.append((lat, lon, alt))
    
    # 如果只有一个航点，创建围绕它的圆形路径
    if len(waypoints) == 1:
        wp = waypoints[0]
        radius = 0.0001  # 大约10米的半径
        for angle in range(0, 360, 5):
            rad = angle * 3.14159 / 180.0
            lat = wp['x'] + radius * np.cos(rad)
            lon = wp['y'] + radius * np.sin(rad)
            smooth_path.append((lat, lon, wp['z']))
    
    add_status_message(f"模拟路径创建完成，共{len(smooth_path)}个点")
    
    # 在整个路径上循环移动
    path_index = 0
    while True:
        try:
            # 获取当前路径点
            if smooth_path:
                lat, lon, alt = smooth_path[path_index]
                
                # 更新当前位置和历史
                with lock:
                    current_position['lat'] = lat
                    current_position['lon'] = lon
                    current_position['alt'] = alt
                    
                    position_history['lat'].append(lat)
                    position_history['lon'].append(lon)
                    position_history['alt'].append(alt)
                    position_history['time'].append(time.time())
                    
                    # 保持历史记录在合理大小
                    if len(position_history['lat']) > 100:
                        position_history['lat'] = position_history['lat'][-100:]
                        position_history['lon'] = position_history['lon'][-100:]
                        position_history['alt'] = position_history['alt'][-100:]
                        position_history['time'] = position_history['time'][-100:]
                
                # 移动到下一个点
                path_index = (path_index + 1) % len(smooth_path)
                
                # 当完成一次循环时，记录一条状态消息
                if path_index == 0:
                    add_status_message("模拟：完成一次路径循环")
                
                if DEBUG and path_index % 5 == 0:  # 减少输出频率
                    print(f"模拟位置: 纬度={lat:.6f}, 经度={lon:.6f}, 高度={alt:.2f}m")
            
            # 控制更新频率
            time.sleep(0.5)  # 每0.5秒更新一次位置
            
        except Exception as e:
            print(f"模拟位置更新错误: {e}")
            add_status_message(f"模拟错误: {str(e)[:50]}")
            time.sleep(1.0)

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
    add_status_message("可视化系统初始化")

# 建立连接（尝试多种端口和连接方式）
connection = None  # 主连接
ports_to_try = [QGC_PORT]
if USE_SIM_PORT:
    ports_to_try = [SIM_PORT]
if MONITOR_MULTIPLE_PORTS:
    ports_to_try = [QGC_PORT, SIM_PORT]

for port in ports_to_try:
    try:
        print(f"尝试连接到端口 {port}...")
        conn = mavutil.mavlink_connection(f'udp:{WSL_IP}:{port}')
        connections.append(conn)
        if connection is None:  # 第一个成功的连接成为主连接
            connection = conn
            print(f"主连接建立在端口 {port}")
            add_status_message(f"主连接建立在端口 {port}")
        else:
            print(f"额外连接建立在端口 {port}")
            add_status_message(f"额外连接建立在端口 {port}")
    except Exception as e:
        print(f"连接到端口 {port} 失败: {e}")
        # 尝试udpin连接
        try:
            conn = mavutil.mavlink_connection(f'udpin:{WSL_IP}:{port}')
            connections.append(conn)
            if connection is None:
                connection = conn
                print(f"主UDPIN连接建立在端口 {port}")
                add_status_message(f"主UDPIN连接建立在端口 {port}")
            else:
                print(f"额外UDPIN连接建立在端口 {port}")
                add_status_message(f"额外UDPIN连接建立在端口 {port}")
        except Exception as e2:
            print(f"UDPIN连接到端口 {port} 失败: {e2}")
            add_status_message(f"连接端口 {port} 失败")

# 检查是否建立了至少一个连接
if not connection:
    print("警告: 无法建立任何连接。将使用模拟模式继续。")
    add_status_message("警告: 无法建立连接，使用模拟模式")
else:
    print("成功建立至少一个连接。")
    add_status_message("成功建立连接")

print("等待心跳信号...")
print("提示: 如果在仿真环境中，您可以启用SKIP_HEARTBEAT直接跳过等待")
add_status_message("等待心跳信号...")

# 添加超时机制
import time
start_time = time.time()
timeout = 60  # 60秒超时
got_heartbeat = False

# 如果设置跳过心跳等待，直接假设心跳已收到
if SKIP_HEARTBEAT:
    print("跳过心跳等待，直接继续...")
    add_status_message("跳过心跳等待")
    got_heartbeat = True
elif SIMULATION_MODE and not connection:
    print("模拟模式启用，跳过心跳等待...")
    add_status_message("模拟模式：跳过心跳等待")
    got_heartbeat = True
else:
    while time.time() - start_time < timeout:
        if DEBUG:
            print(f"等待心跳... (已等待: {int(time.time() - start_time)}秒)")
        
        # 检查所有连接
        for conn in connections:
            try:
                msg = conn.recv_match(type='HEARTBEAT', blocking=False)
                if msg:
                    print(f"通过连接 {conn.port} 收到心跳!")
                    add_status_message(f"收到心跳：端口 {conn.port}")
                    connection = conn  # 使用收到心跳的连接作为主连接
                    got_heartbeat = True
                    break
            except Exception as e:
                if DEBUG:
                    print(f"等待心跳时出错: {e}")
        
        if got_heartbeat:
            break
            
        time.sleep(1)

if not got_heartbeat and not SIMULATION_MODE:
    print("\n超时未收到心跳信号.")
    print("可能的问题:")
    print("1. QGC未配置发送MAVLink数据到Windows IP")
    print("2. 防火墙阻止了连接")
    print("3. IP地址或端口错误")
    print("\n将尝试继续运行...\n")
    add_status_message("警告：未收到心跳信号")
    
    # 询问用户是否使用模拟模式
    print("是否启用模拟模式以显示演示数据? (Y/N)")
    choice = input().strip().upper()
    if choice == 'Y':
        SIMULATION_MODE = True
        print("已启用模拟模式")
        add_status_message("用户选择启用模拟模式")
    
else:
    if got_heartbeat:
        print("心跳接收成功! 连接已建立.")
        add_status_message("心跳接收成功！连接已建立")

# 如果启用位置监听，启动监听线程
if LISTEN_POSITION and connection and got_heartbeat:
    position_thread = threading.Thread(target=position_listener, args=(connection,), daemon=True)
    position_thread.start()
    print("实时位置监听线程已启动")
    add_status_message("实时位置监听线程已启动")

# 如果是模拟模式，启动模拟位置更新线程
if SIMULATION_MODE and not current_position['lat']:  # 只有在没有实际位置数据时才启动模拟
    simulation_thread = threading.Thread(target=simulation_position_updater, daemon=True)
    simulation_thread.start()
    print("模拟位置更新线程已启动")
    add_status_message("模拟位置更新线程已启动")
else:
    print("实际位置监听已启动，不使用模拟位置")
    add_status_message("使用实际位置数据")

# --------------------------
# 请求 Mission 列表
# --------------------------
print("请求任务航点列表...")
add_status_message("请求任务航点列表")

# 为所有连接设置默认系统和组件ID
for conn in connections:
    if not hasattr(conn, 'target_system') or conn.target_system == 0:
        if DEBUG:
            print(f"为连接 {conn.port} 设置默认目标系统 (1, 1)")
        conn.target_system = 1
        conn.target_component = 1
    
    # 向每个连接发送任务请求
    conn.mav.mission_request_list_send(conn.target_system, conn.target_component)
    add_status_message(f"向连接 {conn.port} 请求任务列表")
    
    # 如果配置为请求数据流，则发送请求
    if REQUEST_DATA_STREAM:
        try:
            # 请求位置信息
            conn.mav.request_data_stream_send(
                conn.target_system,
                conn.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                10,  # 10 Hz
                1    # 启用
            )
            print(f"向连接 {conn.port} 请求位置数据流")
            add_status_message(f"请求位置数据: {conn.port}")
            
            # 请求扩展状态
            conn.mav.request_data_stream_send(
                conn.target_system,
                conn.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
                5,   # 5 Hz
                1    # 启用
            )
            print(f"向连接 {conn.port} 请求扩展状态数据流")
            
            # 请求额外数据
            conn.mav.request_data_stream_send(
                conn.target_system,
                conn.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
                2,   # 2 Hz
                1    # 启用
            )
        except Exception as e:
            print(f"请求数据流出错: {e}")
            add_status_message(f"请求数据流错误: {str(e)[:30]}")

# 如果没有连接，但使用模拟模式，则跳过请求
if not connections and SIMULATION_MODE:
    print("模拟模式：跳过任务请求")
    add_status_message("模拟模式：跳过任务请求")

waypoints = []
mission_count = None
mission_timeout = 30  # 30秒超时
start_time = time.time()

while time.time() - start_time < mission_timeout and not SIMULATION_MODE:
    try:
        # 检查所有连接
        for conn in connections:
            msg = conn.recv_match(type=['MISSION_ITEM', 'MISSION_COUNT'], blocking=False)
            if msg is None:
                continue

            if msg.get_type() == 'MISSION_COUNT':
                mission_count = msg.count
                print(f"任务包含 {mission_count} 个航点")
                add_status_message(f"任务包含 {mission_count} 个航点")
                for seq in range(mission_count):
                    conn.mav.mission_request_send(conn.target_system, conn.target_component, seq)

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
                print(f"接收到航点: {wp}")
                add_status_message(f"接收到航点 #{msg.seq}")
                
                # 更新全局航点列表（用于可视化）
                with lock:
                    waypoints_global = waypoints.copy()

                if mission_count is not None and msg.seq == mission_count - 1:
                    print("\n所有航点接收完成!")
                    add_status_message("所有航点接收完成！")
                    break
                    
        # 如果接收完所有航点，跳出外层循环
        if mission_count is not None and len(waypoints) == mission_count:
            break
            
        time.sleep(0.1)  # 短暂等待，减少CPU使用
    except Exception as e:
        print(f"接收任务项时出错: {e}")
        time.sleep(0.5)

if not waypoints and not SIMULATION_MODE:
    print("\n超时未接收到航点.")
    print("可能的问题:")
    print("1. QGC中未加载任务")
    print("2. 通信链路问题")
    print("3. QGC配置不正确")
    add_status_message("警告：未接收到航点")
    
# 仅在模拟模式下创建测试航点
if not waypoints and SIMULATION_MODE:
    print("\n模拟模式：但不创建测试航点...")
    add_status_message("模拟模式：不创建测试航点")
    print("未接收到航点数据")
    add_status_message("未接收到航点数据，等待中...")
    # 不要创建测试航点，保持 waypoints 为空

# --------------------------
# 优化路径
# --------------------------
if waypoints:
    optimized_waypoints = optimize_waypoints(waypoints)
    print("\n优化后的航点:")
    add_status_message("路径优化完成")
    for wp in optimized_waypoints:
        print(wp)

    # 更新全局优化航点列表（用于可视化）
    with lock:
        waypoints_optimized_global = optimized_waypoints.copy()

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

    print("\nGeoJSON 已保存到 optimized_waypoints.geojson")
    add_status_message("GeoJSON 文件已保存")
else:
    print("\n未接收到航点，跳过路径优化和GeoJSON输出")
    add_status_message("未接收到航点，跳过路径优化")

# --------------------------
# 启动实时可视化动画
# --------------------------
if REALTIME_PLOT:
    try:
        print("\n启动实时可视化...")
        add_status_message("启动实时可视化")
        # 确保刚开始时 start_time 不会太接近当前时间，避免除以零错误
        time.sleep(0.1)  # 短暂延迟确保有非零的时间差
        
        # 创建动画并确保窗口保持打开
        animation = FuncAnimation(fig, update_plot, interval=UPDATE_INTERVAL, blit=False, cache_frame_data=False)
        print("可视化窗口已启动。窗口将保持打开状态，请使用键盘Ctrl+C终止程序")
        add_status_message("可视化窗口已启动，正在等待数据...")
        
        # 使用显式循环保持窗口打开
        plt.show(block=False)  # 非阻塞模式
        
        # 主循环
        try:
            while True:
                plt.pause(0.5)  # 短暂暂停，保持动画更新
        except KeyboardInterrupt:
            print("\n用户按下Ctrl+C，程序终止")
        finally:
            print("程序结束")
            
    except Exception as e:
        print(f"启动可视化时出错: {e}")
        add_status_message(f"可视化错误: {str(e)}")
        
        # 尝试不使用blit模式重新启动可视化
        try:
            print("尝试使用兼容模式重新启动可视化...")
            animation = FuncAnimation(fig, update_plot, interval=UPDATE_INTERVAL, blit=False, cache_frame_data=False)
            plt.show(block=False)
            
            try:
                while True:
                    plt.pause(0.5)
            except KeyboardInterrupt:
                print("\n用户按下Ctrl+C，程序终止")
            finally:
                print("程序结束")
                
        except Exception as e2:
            print(f"兼容模式下仍然出错: {e2}")
            print("程序将继续运行，但无法显示可视化界面")
            
            # 如果可视化失败，至少打印状态更新
            while True:
                try:
                    time.sleep(5)
                    current_time = time.time()
                    elapsed_time = max(0.001, current_time - start_time)
                    print(f"运行时间: {int(elapsed_time//3600):02d}:{int((elapsed_time%3600)//60):02d}:{int(elapsed_time%60):02d}")
                    if current_position['lat'] is not None:
                        print(f"当前位置: 纬度={current_position['lat']:.6f}, 经度={current_position['lon']:.6f}, 高度={current_position['alt']:.1f}m")
                except KeyboardInterrupt:
                    print("用户终止程序")
                    break
                except Exception:
                    pass
