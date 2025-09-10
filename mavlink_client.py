import time
import numpy as np
from pymavlink import mavutil

class MavlinkClient:
    def __init__(self, conn_active, logger, state, args, conn_passive=None):
        self.conn_active_str = conn_active
        self.conn_passive_str = conn_passive if conn_passive and conn_passive != conn_active else None
        self.logger = logger
        self.state = state
        self.args = args
        self.m_active = None
        self.m_passive = None
        self.start_time = None
        self.last_t = 0.0
        # Mission download state
        self.mdl_active = False
        self.mdl_expected = 0
        self.mdl_next_seq = 0
        self.mdl_received = {}
        self.mdl_last_req_ts = 0.0
        self.mdl_retry = 0
        self.mdl_max_retry = 5
        # IDs
        self.autopilot_sysid = None
        self.autopilot_compid = None
        self._hb_last = 0.0

    # ----------------- Connection -----------------
    def connect(self, timeout=5):
        # Active connection (bidirectional)
        self.m_active = mavutil.mavlink_connection(
            self.conn_active_str,
            source_system=getattr(self.args, 'source_system', 252),
            source_component=getattr(self.args, 'source_component', 191)
        )
        hb = self.m_active.wait_heartbeat(timeout=timeout)
        self.autopilot_sysid = hb.get_srcSystem()
        self.autopilot_compid = hb.get_srcComponent() or 1
        self.m_active.target_system = self.autopilot_sysid
        self.m_active.target_component = self.autopilot_compid
        try: self.m_active.mav.set_proto_version(2)
        except Exception: pass
        if self.conn_passive_str:
            # Passive (receive only) use different component id to avoid confusing autopilot; suppress heartbeats
            self.m_passive = mavutil.mavlink_connection(
                self.conn_passive_str,
                source_system=getattr(self.args, 'source_system', 252),
                source_component=200  # distinct passive component id
            )
            try:
                self.m_passive.mav.set_proto_version(2)
            except Exception:
                pass
            self.logger.info(f"📡 Passive link attached: {self.conn_passive_str}")
        self.start_time = time.time()
        self.logger.info(f"✅ Active link: {self.conn_active_str} (local={self.m_active.mav.srcSystem}/{self.m_active.mav.srcComponent} target={self.autopilot_sysid}/{self.autopilot_compid})")

    def _send_heartbeat(self):
        now = time.time()
        if now - self._hb_last >= 1.0 and self.m_active:
            try:
                self.m_active.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
                self._hb_last = now
            except Exception:
                pass

    # ----------------- Mission Download -----------------
    def request_mission(self):
        if not self.m_active or self.args.no_mission or self.args.passive_mission:
            return
        if self.mdl_active:
            self.logger.debug("Mission download already active, skip.")
            return
        self.mdl_active = True
        self.mdl_expected = 0
        self.mdl_next_seq = 0
        self.mdl_received.clear()
        self.mdl_retry = 0
        try:
            self.logger.info("Send MISSION_REQUEST_LIST")
            self.m_active.mav.mission_request_list_send(self.autopilot_sysid, self.autopilot_compid)
            self.mdl_last_req_ts = time.time()
        except Exception as e:
            self.logger.warning(f"mission_request_list send failed: {e}")
            self.mdl_active = False

    def _mission_request_seq(self, seq):
        try:
            self.m_active.mav.mission_request_int_send(self.autopilot_sysid, self.autopilot_compid, seq)
        except AttributeError:
            self.m_active.mav.mission_request_send(self.autopilot_sysid, self.autopilot_compid, seq)
        self.mdl_last_req_ts = time.time()
        self.logger.debug(f"Requesting mission seq {seq}")

    def _mission_ack(self, result=0):
        try:
            self.m_active.mav.mission_ack_send(self.autopilot_sysid, self.autopilot_compid, result)
        except Exception as e:
            self.logger.warning(f"Send mission ACK failed: {e}")

    def _mission_download_tick(self):
        if not self.mdl_active:
            return
        now = time.time()
        if self.mdl_expected == 0:
            if now - self.mdl_last_req_ts > 1.2:
                if self.mdl_retry >= self.mdl_max_retry:
                    self.logger.error("MISSION_COUNT timeout, abort")
                    self._mission_ack(1); self.mdl_active = False; return
                self.mdl_retry += 1
                self.logger.warning(f"Resend MISSION_REQUEST_LIST (attempt {self.mdl_retry})")
                try:
                    self.m_active.mav.mission_request_list_send(self.autopilot_sysid, self.autopilot_compid)
                    self.mdl_last_req_ts = now
                except Exception as e:
                    self.logger.warning(f"Resend failed: {e}")
            return
        if self.mdl_next_seq < self.mdl_expected:
            if now - self.mdl_last_req_ts > 1.0:
                if self.mdl_retry >= self.mdl_max_retry:
                    self.logger.error("Mission item timeout, abort")
                    self._mission_ack(1); self.mdl_active = False; return
                self.mdl_retry += 1
                self.logger.warning(f"Retry mission seq {self.mdl_next_seq} (attempt {self.mdl_retry})")
                self._mission_request_seq(self.mdl_next_seq)
        else:
            self.logger.info(f"Mission download complete: {len(self.mdl_received)}/{self.mdl_expected}")
            res = 0 if len(self.mdl_received)==self.mdl_expected else 1
            self._mission_ack(res)
            missions_list = [self.mdl_received[i] for i in sorted(self.mdl_received)]
            self.state.mission.missions[:] = missions_list
            for m in missions_list:
                self.logger.info(f"  Mission item: seq={m[0]} x={m[1]} y={m[2]} z={m[3]}")
            self.mdl_active = False

    # ----------------- Timing -----------------
    def _next_time(self):
        if self.start_time is None:
            self.start_time = time.time()
        t = time.time() - self.start_time
        if t <= self.last_t:
            t = self.last_t + 1e-3
        self.last_t = t
        return t

    # ----------------- Polling -----------------
    def poll(self):
        if not self.m_active:
            return
        self._send_heartbeat()
        pos_buf = self.state.pos; att_buf = self.state.att; vel_buf = self.state.vel
        imu_buf = self.state.imu; alt_buf = self.state.alt; gps_buf = self.state.gps
        servo_buf = self.state.servo; missions = self.state.mission.missions; args = self.args

        def drain(conn):
            while True:
                msg = conn.recv_match(blocking=False)
                if not msg: break
                tname = msg.get_type(); t = self._next_time()
                if tname == "LOCAL_POSITION_NED":
                    if all(hasattr(msg,a) for a in ('x','y','z')):
                        pos_buf.t.append(t); pos_buf.x.append(msg.x); pos_buf.y.append(msg.y); pos_buf.z.append(msg.z)
                    if args.show_vel and all(hasattr(msg,a) for a in ('vx','vy','vz')):
                        vel_buf.t.append(t); vel_buf.vx.append(msg.vx); vel_buf.vy.append(msg.vy); vel_buf.vz.append(msg.vz)
                        vel_buf.vx_sp.append(vel_buf.vx_sp[-1] if vel_buf.vx_sp else 0)
                        vel_buf.vy_sp.append(vel_buf.vy_sp[-1] if vel_buf.vy_sp else 0)
                        vel_buf.vz_sp.append(vel_buf.vz_sp[-1] if vel_buf.vz_sp else 0)
                elif tname == "ODOMETRY":
                    if all(hasattr(msg,a) for a in ('x','y','z')):
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
                    imu_buf.t.append(t); imu_buf.ax.append(msg.xacc); imu_buf.ay.append(msg.yacc); imu_buf.az.append(msg.zacc)
                    imu_buf.gx.append(msg.xgyro); imu_buf.gy.append(msg.ygyro); imu_buf.gz.append(msg.zgyro)
                elif tname == "ALTITUDE" and args.show_alt:
                    alt_buf.t.append(t); alt_buf.alt_amsl.append(getattr(msg,'altitude_amsl', np.nan)); alt_buf.alt_rel.append(getattr(msg,'altitude_relative', np.nan))
                elif tname == "GLOBAL_POSITION_INT" and args.show_alt:
                    alt_buf.t.append(t); alt_buf.alt_amsl.append(getattr(msg,'alt',0)/1000.0); alt_buf.alt_rel.append(getattr(msg,'relative_alt',0)/1000.0)
                elif tname == "GPS_RAW_INT" and args.show_gps:
                    gps_buf.t.append(t); gps_buf.sats.append(getattr(msg,'satellites_visible',0)); gps_buf.eph.append(getattr(msg,'eph',0)/100.0); gps_buf.epv.append(getattr(msg,'epv',0)/100.0); gps_buf.fix.append(getattr(msg,'fix_type',0))
                elif tname == "SERVO_OUTPUT_RAW" and args.show_servo:
                    servo_buf.t.append(t)
                    for i in range(8): servo_buf.ch[i].append(getattr(msg,f'servo{i+1}_raw',1500))
                elif tname == "MISSION_COUNT" and not self.args.no_mission:
                    if conn is self.m_active and self.mdl_active:
                        self.mdl_expected = getattr(msg,'count',0)
                        self.logger.info(f"Mission count: {self.mdl_expected}")
                        self.mdl_next_seq = 0
                        if self.mdl_expected > 0:
                            self._mission_request_seq(0)
                elif tname in ("MISSION_ITEM","MISSION_ITEM_INT") and not self.args.no_mission:
                    seq = getattr(msg,'seq', None)
                    if seq is None: continue
                    if self.mdl_active and conn is self.m_active:
                        if tname == "MISSION_ITEM":
                            self.mdl_received[seq] = (seq, getattr(msg,'x',0), getattr(msg,'y',0), getattr(msg,'z',0))
                        else:
                            self.mdl_received[seq] = (seq, (msg.x/1e7 if hasattr(msg,'x') else 0), (msg.y/1e7 if hasattr(msg,'y') else 0), getattr(msg,'z',0))
                        if seq == self.mdl_next_seq:
                            self.mdl_next_seq += 1
                            if self.mdl_next_seq < self.mdl_expected:
                                self._mission_request_seq(self.mdl_next_seq)
                        self.mdl_retry = 0
                    else:
                        # passive append (ensure not duplicate existing active reception)
                        if not any(m[0]==seq for m in missions):
                            missions.append((seq, getattr(msg,'x',0), getattr(msg,'y',0), getattr(msg,'z',0)))
                elif tname == "MISSION_CURRENT" and not self.args.no_mission:
                    pass
        # Drain active first (authoritative for mission protocol), then passive
        drain(self.m_active)
        if self.m_passive:
            drain(self.m_passive)
        self._mission_download_tick()

    def close(self):
        for c in (self.m_active, self.m_passive):
            try:
                if c: c.close()
            except Exception:
                pass
