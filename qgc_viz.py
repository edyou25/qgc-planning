import matplotlib
matplotlib.use('Qt5Agg')

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from pymavlink import mavutil
from collections import deque
import time
import numpy as np
import signal
import sys
import logging
# 创建 logger
logger = logging.getLogger("my_logger")
logger.setLevel(logging.DEBUG)  # 设置全局最低日志级别

# 控制台 handler
console_handler = logging.StreamHandler()
console_handler.setLevel(logging.INFO)  # 控制台显示 INFO 及以上

# 文件 handler
file_handler = logging.FileHandler("/home/hw/qgc-planning/logs/mavviz.log", mode='w')
file_handler.setLevel(logging.DEBUG)    # 文件记录 DEBUG 及以上

# 日志格式
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
console_handler.setFormatter(formatter)
file_handler.setFormatter(formatter)

# 添加 handler 到 logger
logger.addHandler(console_handler)
logger.addHandler(file_handler)
# -------------------------
# 配置参数
# -------------------------
CONN = 'udp:127.0.0.1:14550'   # MAVLink UDP 连接
WINDOW = 200                   # 滑动窗口长度
PLOT_INTERVAL = 50             # 刷新间隔(ms)
TIME_WINDOW = 10               # x轴显示时间窗口(s)

# -------------------------
# 数据缓存
# -------------------------
times = deque(maxlen=WINDOW)
xs = deque(maxlen=WINDOW)
ys = deque(maxlen=WINDOW)
zs = deque(maxlen=WINDOW)
rolls = deque(maxlen=WINDOW)
pitchs = deque(maxlen=WINDOW)
yaws = deque(maxlen=WINDOW)

missions = []  # mission item list: [(seq, x, y, z)]
running = True  # global flag for graceful shutdown

# -------------------------
# 建立 MAVLink 连接
# -------------------------
try:
    m = mavutil.mavlink_connection(CONN)
    m.wait_heartbeat(timeout=5)
    print(f"✅ Connected to system {m.target_system}, component {m.target_component}")
except Exception as e:
    print(f"❌ MAVLink connection failed: {e}")
    sys.exit(1)

# -------------------------
# 初始化绘图
# -------------------------
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

# 左边 subplot: 位置 + 姿态
line_x, = ax1.plot([], [], label='X [m]')
line_y, = ax1.plot([], [], label='Y [m]')
line_z, = ax1.plot([], [], label='Z [m]')
line_roll, = ax1.plot([], [], label='Roll [rad]', linestyle='--')
line_pitch, = ax1.plot([], [], label='Pitch [rad]', linestyle='--')
line_yaw, = ax1.plot([], [], label='Yaw [rad]', linestyle='--')

ax1.set_xlabel("Time [s]")
ax1.set_ylabel("Position / Attitude")
ax1.legend()
ax1.grid(True)

# 右边 subplot: mission (keep a persistent line to avoid full clear each frame)
ax2.set_title("Mission Waypoints")
ax2.set_xlabel("X [m]")
ax2.set_ylabel("Y [m]")
ax2.grid(True)
mission_line, = ax2.plot([], [], "o-", label="Waypoints")
ax2.legend()

start_time = time.time()

def init():
    ax1.set_xlim(0, TIME_WINDOW)
    ax1.set_ylim(-10, 10)
    return (line_x, line_y, line_z, line_roll, line_pitch, line_yaw)

def update(frame):
    if not running:
        return ()

    msg = m.recv_match(blocking=False)
    now = time.time() - start_time

    # -------------------------
    # 处理飞行数据
    # -------------------------
    if msg:
        tname = msg.get_type()
        if tname in ("LOCAL_POSITION_NED", "LOCAL_POSITION"):
            xs.append(msg.x)
            ys.append(msg.y)
            zs.append(msg.z)
            # 保持队列长度一致
            rolls.append(None)
            pitchs.append(None)
            yaws.append(None)
            times.append(now)
        elif tname == "ATTITUDE":
            rolls.append(msg.roll)
            pitchs.append(msg.pitch)
            yaws.append(msg.yaw)
            xs.append(None)
            ys.append(None)
            zs.append(None)
            times.append(now)
        elif tname == "MISSION_ITEM":
            missions.append((msg.seq, msg.x, msg.y, msg.z))
            logging.info(f"Mission item added: seq={msg.seq}, x={msg.x}, y={msg.y}, z={msg.z}")

    # -------------------------
    # 更新左边 subplot
    # -------------------------
    # 取最小长度，避免 shape mismatch
    min_len = min(len(times), len(xs), len(ys), len(zs), len(rolls), len(pitchs), len(yaws))
    if min_len > 0:
        tdata = list(times)[-min_len:]
        line_x.set_data(tdata, list(xs)[-min_len:])
        line_y.set_data(tdata, list(ys)[-min_len:])
        line_z.set_data(tdata, list(zs)[-min_len:])
        line_roll.set_data(tdata, list(rolls)[-min_len:])
        line_pitch.set_data(tdata, list(pitchs)[-min_len:])
        line_yaw.set_data(tdata, list(yaws)[-min_len:])

        # 动态 x 轴
        tmax = tdata[-1]
        ax1.set_xlim(max(0, tmax - TIME_WINDOW), tmax + 0.1)

        # 动态 y 轴
        all_vals = [v for v in list(xs)[-min_len:] + list(ys)[-min_len:] + list(zs)[-min_len:] +
                    list(rolls)[-min_len:] + list(pitchs)[-min_len:] + list(yaws)[-min_len:] if v is not None]
        if all_vals:
            ymin, ymax = min(all_vals), max(all_vals)
            if ymin == ymax:
                ymin -= 1; ymax += 1
            ax1.set_ylim(ymin - 0.5, ymax + 0.5)

    # -------------------------
    # 更新右边 subplot (mission) - only data change
    # -------------------------
    if missions:
        mx = [mi[1] for mi in missions]
        my = [mi[2] for mi in missions]
        mission_line.set_data(mx, my)
        ax2.relim()
        ax2.autoscale_view()
    else:
        mission_line.set_data([], [])

    return (line_x, line_y, line_z, line_roll, line_pitch, line_yaw, mission_line)

# -------------------------
# 动画
# -------------------------
def shutdown(*_):
    global running
    if not running:
        return
    running = False
    try:
        ani.event_source.stop()
    except Exception:
        pass
    try:
        m.close()
    except Exception:
        pass
    plt.close(fig)

signal.signal(signal.SIGINT, shutdown)
fig.canvas.mpl_connect('close_event', lambda evt: shutdown())

# cache_frame_data disabled to silence warning when frames is infinite/None
ani = animation.FuncAnimation(
    fig,
    update,
    init_func=init,
    interval=PLOT_INTERVAL,
    blit=False,
    cache_frame_data=False,
)
plt.show()
