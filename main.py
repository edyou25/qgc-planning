import matplotlib
matplotlib.use('Qt5Agg')  # must be set before importing pyplot

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time, signal, sys, logging, argparse
from state import create_state, AppState
from mavlink_client import MavlinkClient
from plotter import build_layout, update_plots
from logutil import setup_logger  # new

# -------------------------
# 参数解析
# -------------------------
def parse_args():
    p = argparse.ArgumentParser(description="Realtime PX4 MAVLink visualization")
    p.add_argument('--conn', default='udp:127.0.0.1:14550', help='MAVLink connection string')
    p.add_argument('--window', type=int, default=200, help='Max data points stored')
    p.add_argument('--time-window', type=float, default=10.0, help='Seconds shown on X axis')
    p.add_argument('--interval', type=int, default=50, help='Plot update interval ms')
    p.add_argument('--log', default='/home/hw/qgc-planning/logs/mavviz.log', help='Log file path')
    p.add_argument('--mission-request', action='store_true', help='Request mission list on start')
    p.add_argument('--no-attitude', action='store_true', help='Disable attitude plotting')
    p.add_argument('--no-mission', action='store_true', help='Disable mission plotting')
    p.add_argument('--show-vel', action='store_true', help='Show velocity subplot')
    p.add_argument('--show-imu', action='store_true', help='Show IMU accel/gyro subplot')
    p.add_argument('--show-alt', action='store_true', help='Show altitude subplot')
    p.add_argument('--show-gps', action='store_true', help='Show GPS status subplot')
    p.add_argument('--show-servo', action='store_true', help='Show servo PWM subplot')
    return p.parse_args()

args = parse_args()

# -------------------------
# Logging (refactored)
# -------------------------
logger = setup_logger(args.log)

# -------------------------
CONN = args.conn
WINDOW = args.window
PLOT_INTERVAL = args.interval
TIME_WINDOW = args.time_window
state: AppState = create_state(WINDOW)
client = MavlinkClient(CONN, logger, state, args)
try:
    client.connect()
except Exception as e:
    logger.error(f"❌ MAVLink connection failed: {e}")
    sys.exit(1)
if args.mission_request and not args.no_mission:
    client.request_mission()

ctx = build_layout(args, TIME_WINDOW)
running = True

# -------------------------
# 动画回调
# -------------------------
start_time = time.time()

def init():
    ctx.ax_main.set_xlim(0, TIME_WINDOW)
    return ()

def update(_):
    if not running:
        return ()
    client.poll()
    return update_plots(ctx, state, args, TIME_WINDOW)

# -------------------------
# 关闭处理
# -------------------------
def shutdown(*_):
    global running
    if not running: return
    running = False
    try: ani.event_source.stop()
    except Exception: pass
    client.close()
    plt.close(ctx.fig)

signal.signal(signal.SIGINT, shutdown)
ctx.fig.canvas.mpl_connect('close_event', lambda evt: shutdown())

ani = animation.FuncAnimation(ctx.fig, update, init_func=init, interval=PLOT_INTERVAL, blit=False, cache_frame_data=False)
plt.show()
