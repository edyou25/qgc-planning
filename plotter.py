import matplotlib.pyplot as plt

class PlotContext:
    def __init__(self):
        self.fig = None
        self.ax_main = None
        self.ax_mission = None
        self.axes_extra = {}
        self.lines = {}
        self.mission_line = None
        self.servo_art = None


def build_layout(args, TIME_WINDOW):
    ctx = PlotContext()
    subplot_list = []
    if args.show_vel: subplot_list.append('vel')
    if args.show_imu: subplot_list.append('imu')
    if args.show_alt: subplot_list.append('alt')
    if args.show_gps: subplot_list.append('gps')
    if args.show_servo: subplot_list.append('servo')
    extra_n = len(subplot_list)
    cols = 2
    rows = 1 + (extra_n + (0 if args.no_mission else 0) + 1) // cols
    ctx.fig = plt.figure(figsize=(14, 4 + 3*max(0, rows-1)))
    gs = ctx.fig.add_gridspec(rows, cols)
    ctx.ax_main = ctx.fig.add_subplot(gs[0,0])
    ctx.ax_mission = ctx.fig.add_subplot(gs[0,1]) if not args.no_mission else None
    row_i = 1; col_i = 0
    for name in subplot_list:
        ax = ctx.fig.add_subplot(gs[row_i, col_i])
        ctx.axes_extra[name] = ax
        col_i += 1
        if col_i >= cols:
            col_i = 0; row_i += 1
    # main lines
    axm = ctx.ax_main
    ctx.lines['x'], = axm.plot([],[],label='X')
    ctx.lines['y'], = axm.plot([],[],label='Y')
    ctx.lines['z'], = axm.plot([],[],label='Z')
    ctx.lines['roll'] = ctx.lines['pitch'] = ctx.lines['yaw'] = None
    if not args.no_attitude:
        ctx.lines['roll'], = axm.plot([],[],label='Roll', linestyle='--')
        ctx.lines['pitch'], = axm.plot([],[],label='Pitch', linestyle='--')
        ctx.lines['yaw'], = axm.plot([],[],label='Yaw', linestyle='--')
    axm.set_xlabel('Time [s]'); axm.set_ylabel('Pos/Att'); axm.legend(); axm.grid(True)
    if ctx.ax_mission is not None:
        ctx.ax_mission.set_title('Mission Waypoints')
        ctx.ax_mission.set_xlabel('X'); ctx.ax_mission.set_ylabel('Y'); ctx.ax_mission.grid(True)
        ctx.mission_line, = ctx.ax_mission.plot([],[], 'o-', label='Waypoints')
        ctx.ax_mission.legend()
    # velocity
    if 'vel' in ctx.axes_extra:
        ax = ctx.axes_extra['vel']; ax.set_title('Velocity'); ax.grid(True)
        for n in ('vx','vy','vz','vx_sp','vy_sp','vz_sp'):
            ctx.lines[n], = ax.plot([],[], '--' if 'sp' in n else '-', label=n)
        ax.legend()
    if 'imu' in ctx.axes_extra:
        ax = ctx.axes_extra['imu']; ax.set_title('IMU'); ax.grid(True)
        for n in ('ax','ay','az','gx','gy','gz'):
            ctx.lines[n], = ax.plot([],[], label=n)
        ax.legend()
    if 'alt' in ctx.axes_extra:
        ax = ctx.axes_extra['alt']; ax.set_title('Altitude'); ax.grid(True)
        for n in ('amsl','rel'):
            ctx.lines[f'alt_{n}'], = ax.plot([],[], label=n)
        ax.legend()
    if 'gps' in ctx.axes_extra:
        ax = ctx.axes_extra['gps']; ax.set_title('GPS'); ax.grid(True)
        for n in ('sats','eph','epv'):
            ctx.lines[f'gps_{n}'], = ax.plot([],[], label=n)
        ax.legend()
    if 'servo' in ctx.axes_extra:
        ax = ctx.axes_extra['servo']; ax.set_title('Servo PWM (ch1-8)')
        ax.set_ylim(800,2200); ax.set_ylabel('PWM'); ax.set_xticks(range(8)); ax.set_xticklabels([str(i+1) for i in range(8)])
        ax.grid(True, axis='y')
        ctx.servo_art = ax.bar(range(8), [1500]*8)
    return ctx


def update_plots(ctx, state, args, TIME_WINDOW):
    artists = []
    pos = state.pos; att = state.att; vel = state.vel; imu = state.imu; alt = state.alt; gps = state.gps; servo = state.servo
    lines = ctx.lines
    # time window reference
    latest_t = 0
    earliest_t = None
    for seq in (pos.t, att.t, vel.t, imu.t, alt.t, gps.t):
        if seq:
            if earliest_t is None or seq[0] < earliest_t:
                earliest_t = seq[0]
            if seq[-1] > latest_t:
                latest_t = seq[-1]
    if earliest_t is None:
        earliest_t = 0
    span = latest_t - earliest_t
    # desired left bound so curve occupies whole axis when span < TIME_WINDOW
    if span < TIME_WINDOW:
        left_main = earliest_t
    else:
        left_main = latest_t - TIME_WINDOW
    right_main = latest_t if span < TIME_WINDOW else latest_t
    right_main += 0.05 * (right_main - left_main + 1e-6)
    # main pos
    if pos.t:
        tpos = list(pos.t)
        lines['x'].set_data(tpos, list(pos.x)); lines['y'].set_data(tpos, list(pos.y)); lines['z'].set_data(tpos, list(pos.z))
        artists.extend([lines['x'], lines['y'], lines['z']])
    if lines.get('roll') and att.t:
        tatt = list(att.t)
        lines['roll'].set_data(tatt, list(att.roll)); lines['pitch'].set_data(tatt, list(att.pitch)); lines['yaw'].set_data(tatt, list(att.yaw))
        artists.extend([lines['roll'], lines['pitch'], lines['yaw']])
    # adjust main axes
    ctx.ax_main.set_xlim(left_main, right_main)
    # altitude
    if args.show_alt and alt.t:
        ta = list(alt.t)
        lines['alt_amsl'].set_data(ta, list(alt.alt_amsl)); lines['alt_rel'].set_data(ta, list(alt.alt_rel))
        ax = ctx.axes_extra['alt']
        if alt.t:
            a_left = alt.t[0] if (alt.t[-1]-alt.t[0]) < TIME_WINDOW else alt.t[-1]-TIME_WINDOW
            a_right = alt.t[-1] + 0.05*(alt.t[-1]-alt.t[0] + 1e-6)
            ax.set_xlim(a_left, a_right)
        vals = list(alt.alt_amsl)+list(alt.alt_rel)
        if vals:
            mn, mx = min(vals), max(vals)
            if mn == mx: mn -= 1; mx += 1
            ax.set_ylim(mn-0.5, mx+0.5)
        artists.extend([lines['alt_amsl'], lines['alt_rel']])
    # velocity
    if args.show_vel and vel.t:
        tv = list(vel.t)
        for n in ('vx','vy','vz','vx_sp','vy_sp','vz_sp'):
            lines[n].set_data(tv, list(getattr(vel,n)))
        vv = list(vel.vx)+list(vel.vy)+list(vel.vz)+list(vel.vx_sp)+list(vel.vy_sp)+list(vel.vz_sp)
        ax = ctx.axes_extra['vel']
        # compute axis specific left/right
        if vel.t:
            v_left_span = vel.t[0] if (vel.t[-1]-vel.t[0]) < TIME_WINDOW else vel.t[-1]-TIME_WINDOW
            v_right = vel.t[-1] + 0.05*(vel.t[-1]-vel.t[0] + 1e-6)
            ax.set_xlim(v_left_span, v_right)
        if vv:
            vmin, vmax = min(vv), max(vv)
            if vmin == vmax: vmin -= 0.5; vmax += 0.5
            ax.set_ylim(vmin-0.2, vmax+0.2)
        for n in ('vx','vy','vz','vx_sp','vy_sp','vz_sp'): artists.append(lines[n])
    # imu
    if args.show_imu and imu.t:
        ti = list(imu.t)
        for n in ('ax','ay','az','gx','gy','gz'):
            lines[n].set_data(ti, list(getattr(imu,n)))
        av = list(imu.ax)+list(imu.ay)+list(imu.az)+list(imu.gx)+list(imu.gy)+list(imu.gz)
        ax = ctx.axes_extra['imu']
        if imu.t:
            i_left = imu.t[0] if (imu.t[-1]-imu.t[0]) < TIME_WINDOW else imu.t[-1]-TIME_WINDOW
            i_right = imu.t[-1] + 0.05*(imu.t[-1]-imu.t[0] + 1e-6)
            ax.set_xlim(i_left, i_right)
        if av:
            mn, mx = min(av), max(av)
            if mn == mx: mn -= 0.1; mx += 0.1
            ax.set_ylim(mn-0.1, mx+0.1)
        for n in ('ax','ay','az','gx','gy','gz'): artists.append(lines[n])
    # gps
    if args.show_gps and gps.t:
        tg = list(gps.t)
        lines['gps_sats'].set_data(tg, list(gps.sats)); lines['gps_eph'].set_data(tg, list(gps.eph)); lines['gps_epv'].set_data(tg, list(gps.epv))
        ax = ctx.axes_extra['gps']
        if gps.t:
            g_left = gps.t[0] if (gps.t[-1]-gps.t[0]) < TIME_WINDOW else gps.t[-1]-TIME_WINDOW
            g_right = gps.t[-1] + 0.05*(gps.t[-1]-gps.t[0] + 1e-6)
            ax.set_xlim(g_left, g_right)
        gvals = list(gps.sats)+list(gps.eph)+list(gps.epv)
        if gvals:
            mn, mx = min(gvals), max(gvals)
            if mn == mx: mn -= 1; mx += 1
            ax.set_ylim(mn-0.5, mx+0.5)
        artists.extend([lines['gps_sats'], lines['gps_eph'], lines['gps_epv']])
    # servo
    if args.show_servo and servo.t and ctx.servo_art:
        latest = [ch[-1] if ch else 1500 for ch in servo.ch]
        for rect, val in zip(ctx.servo_art, latest): rect.set_height(val)
        artists.extend(list(ctx.servo_art))
    # mission
    if ctx.mission_line is not None:
        missions = state.mission.missions[:-1] # exclude last two dummy items
        if missions:
            mx = [mi[1] for mi in missions]; my = [mi[2] for mi in missions]
            ctx.mission_line.set_data(mx, my)
            ctx.ax_mission.relim(); ctx.ax_mission.autoscale_view()
        else:
            ctx.mission_line.set_data([], [])
        artists.append(ctx.mission_line)
    return tuple(artists)
