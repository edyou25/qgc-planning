import argparse

def _build_parser():
    p = argparse.ArgumentParser(description="Realtime PX4 MAVLink visualization (simplified CLI)")
    p.add_argument('--conn', default='udp:127.0.0.1:14551', help='Default MAVLink connection (used if active/passive not set)')
    p.add_argument('--conn-active', default='', help='Active (bidirectional) connection for mission requests')
    p.add_argument('--conn-passive', default='', help='Passive/telemetry connection (receive only)')
    p.add_argument('--window', type=int, default=300, help='Max data points stored')
    p.add_argument('--time-window', type=float, default=15.0, help='Seconds shown on X axis')
    p.add_argument('--interval', type=int, default=60, help='Plot update interval ms')
    p.add_argument('--log', default='/home/hw/qgc-planning/logs/mavviz.log', help='Log file path')
    p.add_argument('--plots', choices=['basic','nav','imu','full'], default='full', help='Preset plot set')
    p.add_argument('--mission-mode', choices=['off','passive','active'], default='active', help='Mission handling mode')
    return p


def _expand_presets(args):
    setattr(args, 'no_attitude', False)
    setattr(args, 'no_mission', args.mission_mode == 'off')
    setattr(args, 'passive_mission', args.mission_mode == 'passive')
    setattr(args, 'mission_request', args.mission_mode == 'active')
    setattr(args, 'mission_forward', '')
    show_vel = show_imu = show_alt = show_gps = show_servo = False
    if args.plots == 'nav':
        show_vel = show_alt = True
    elif args.plots == 'imu':
        show_imu = True
    elif args.plots == 'full':
        show_vel = show_imu = show_alt = show_gps = show_servo = True
    setattr(args, 'show_vel', show_vel)
    setattr(args, 'show_imu', show_imu)
    setattr(args, 'show_alt', show_alt)
    setattr(args, 'show_gps', show_gps)
    setattr(args, 'show_servo', show_servo)
    # Resolve dual connection defaults
    if not args.conn_active and not args.conn_passive:
        args.conn_active = args.conn
        args.conn_passive = args.conn
    elif args.conn_active and not args.conn_passive:
        args.conn_passive = args.conn_active
    elif args.conn_passive and not args.conn_active:
        args.conn_active = args.conn_passive
    return args


def get_args():
    parser = _build_parser()
    args = parser.parse_args()
    return _expand_presets(args)
