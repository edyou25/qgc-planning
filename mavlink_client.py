import time
import numpy as np
from pymavlink import mavutil

class MavlinkClient:
    def __init__(self, conn_str, logger, state, args):
        self.conn_str = conn_str
        self.logger = logger
        self.state = state
        self.args = args
        self.m = None
        self.start_time = None
        self.last_t = 0.0  # ensure strictly increasing time axis

    def connect(self, timeout=5):
        self.m = mavutil.mavlink_connection(self.conn_str)
        self.m.wait_heartbeat(timeout=timeout)
        self.start_time = time.time()
        self.logger.info(f"✅ MAVLink connected: {self.conn_str} (sys={self.m.target_system} comp={self.m.target_component})")

    def request_mission(self):
        if self.m and not self.args.no_mission:
            try:
                self.logger.info("Requesting mission list...")
                self.m.mav.mission_request_list_send(self.m.target_system, self.m.target_component)
            except Exception as e:
                self.logger.warning(f"Mission request failed: {e}")

    def _next_time(self):
        if self.start_time is None:
            self.start_time = time.time()
        t = time.time() - self.start_time
        # enforce strictly increasing (avoid duplicate timestamps leading to visual artifacts)
        if t <= self.last_t:
            t = self.last_t + 1e-3
        self.last_t = t
        return t

    def poll(self):
        """Fetch all pending messages and update state using host-relative monotonic time."""
        if not self.m:
            return
        pos_buf = self.state.pos
        att_buf = self.state.att
        vel_buf = self.state.vel
        imu_buf = self.state.imu
        alt_buf = self.state.alt
        gps_buf = self.state.gps
        servo_buf = self.state.servo
        missions = self.state.mission.missions
        args = self.args
        while True:
            msg = self.m.recv_match(blocking=False)
            if not msg:
                break
            tname = msg.get_type()
            t = self._next_time()
            # Position & velocity
            if tname == "LOCAL_POSITION_NED":
                if all(hasattr(msg, a) for a in ('x','y','z')):
                    pos_buf.t.append(t); pos_buf.x.append(msg.x); pos_buf.y.append(msg.y); pos_buf.z.append(msg.z)
                if args.show_vel and all(hasattr(msg,a) for a in ('vx','vy','vz')):
                    vel_buf.t.append(t); vel_buf.vx.append(msg.vx); vel_buf.vy.append(msg.vy); vel_buf.vz.append(msg.vz)
                    vel_buf.vx_sp.append(vel_buf.vx_sp[-1] if vel_buf.vx_sp else 0)
                    vel_buf.vy_sp.append(vel_buf.vy_sp[-1] if vel_buf.vy_sp else 0)
                    vel_buf.vz_sp.append(vel_buf.vz_sp[-1] if vel_buf.vz_sp else 0)
            elif tname == "ODOMETRY":
                if all(hasattr(msg, a) for a in ('x','y','z')):
                    pos_buf.t.append(t); pos_buf.x.append(getattr(msg,'x',0)); pos_buf.y.append(getattr(msg,'y',0)); pos_buf.z.append(getattr(msg,'z',0))
                if args.show_vel and all(hasattr(msg,a) for a in ('vx','vy','vz')):
                    vel_buf.t.append(t); vel_buf.vx.append(getattr(msg,'vx',0)); vel_buf.vy.append(getattr(msg,'vy',0)); vel_buf.vz.append(getattr(msg,'vz',0))
                    vel_buf.vx_sp.append(vel_buf.vx_sp[-1] if vel_buf.vx_sp else 0)
                    vel_buf.vy_sp.append(vel_buf.vy_sp[-1] if vel_buf.vy_sp else 0)
                    vel_buf.vz_sp.append(vel_buf.vz_sp[-1] if vel_buf.vz_sp else 0)
            elif tname == "POSITION_TARGET_LOCAL_NED" and args.show_vel:
                vel_buf.t.append(t)
                vel_buf.vx.append(vel_buf.vx[-1] if vel_buf.vx else 0)
                vel_buf.vy.append(vel_buf.vy[-1] if vel_buf.vy else 0)
                vel_buf.vz.append(vel_buf.vz[-1] if vel_buf.vz else 0)
                vel_buf.vx_sp.append(getattr(msg,'vx',0)); vel_buf.vy_sp.append(getattr(msg,'vy',0)); vel_buf.vz_sp.append(getattr(msg,'vz',0))
            elif tname == "ATTITUDE" and not args.no_attitude:
                att_buf.t.append(t); att_buf.roll.append(msg.roll); att_buf.pitch.append(msg.pitch); att_buf.yaw.append(msg.yaw)
            elif tname == "HIGHRES_IMU" and args.show_imu:
                imu_buf.t.append(t)
                imu_buf.ax.append(msg.xacc); imu_buf.ay.append(msg.yacc); imu_buf.az.append(msg.zacc)
                imu_buf.gx.append(msg.xgyro); imu_buf.gy.append(msg.ygyro); imu_buf.gz.append(msg.zgyro)
            elif tname == "ALTITUDE" and args.show_alt:
                alt_buf.t.append(t)
                alt_buf.alt_amsl.append(getattr(msg,'altitude_amsl', np.nan))
                alt_buf.alt_rel.append(getattr(msg,'altitude_relative', np.nan))
            elif tname == "GLOBAL_POSITION_INT" and args.show_alt:
                alt_buf.t.append(t)
                alt_buf.alt_amsl.append(getattr(msg,'alt',0)/1000.0)
                alt_buf.alt_rel.append(getattr(msg,'relative_alt',0)/1000.0)
            elif tname == "GPS_RAW_INT" and args.show_gps:
                gps_buf.t.append(t)
                gps_buf.sats.append(getattr(msg,'satellites_visible',0))
                gps_buf.eph.append(getattr(msg,'eph',0)/100.0)
                gps_buf.epv.append(getattr(msg,'epv',0)/100.0)
                gps_buf.fix.append(getattr(msg,'fix_type',0))
            elif tname == "SERVO_OUTPUT_RAW" and args.show_servo:
                servo_buf.t.append(t)
                for i in range(8):
                    servo_buf.ch[i].append(getattr(msg,f'servo{i+1}_raw',1500))
            elif tname == "MISSION_ITEM" and not self.args.no_mission:
                missions.append((msg.seq, msg.x, msg.y, msg.z))
            elif tname == "MISSION_ITEM_INT" and not self.args.no_mission:
                lat = msg.x/1e7 if hasattr(msg,'x') else 0
                lon = msg.y/1e7 if hasattr(msg,'y') else 0
                missions.append((msg.seq, lat, lon, getattr(msg,'z',0)))
            elif tname == "MISSION_COUNT" and not self.args.no_mission:
                self.logger.info(f"Mission count: {getattr(msg,'count','?')}")

    def close(self):
        try:
            if self.m:
                self.m.close()
        except Exception:
            pass
