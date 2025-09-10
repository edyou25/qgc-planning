import matplotlib
matplotlib.use('Qt5Agg')  # must be set before importing pyplot

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time, signal, sys, logging
from state import create_state, AppState
from mavlink_client import MavlinkClient
from plotter import build_layout, update_plots
from logutil import setup_logger
from cli import get_args

# -------------------------
# 参数解析
# -------------------------
args = get_args()

# -------------------------
# Logging
# -------------------------
logger = setup_logger(args.log)

# -------------------------
CONN_ACTIVE = args.conn_active
CONN_PASSIVE = args.conn_passive
WINDOW = args.window; PLOT_INTERVAL = args.interval; TIME_WINDOW = args.time_window
state: AppState = create_state(WINDOW)
# FIX: supply passive link
client = MavlinkClient(CONN_ACTIVE, logger, state, args, conn_passive=CONN_PASSIVE)
try:
    client.connect()
except Exception as e:
    logger.error(f"❌ MAVLink connection failed: {e}"); sys.exit(1)
# Sanity warnings
if args.mission_request and (CONN_ACTIVE == CONN_PASSIVE):
    logger.warning("Active & passive connection are identical; OK but no separation of mission traffic.")
if args.mission_request and '14551' in CONN_ACTIVE and '14552' in CONN_PASSIVE:
    logger.warning("It looks like active/passive might be reversed (active=14551, passive=14552). Ensure active uses PX4 -u port (e.g., 14552).")

if args.mission_request and not args.no_mission and not args.passive_mission:
    client.request_mission()
elif args.passive_mission:
    logger.info("Passive mission mode enabled (no active requests)")

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
