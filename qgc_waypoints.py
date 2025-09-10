from pymavlink import mavutil

# 连接到 SITL 或 PX4
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')

print("Listening to all MAVLink traffic...\n")

while True:
    msg = master.recv_match(blocking=True)
    if msg:
        sysid, compid = msg.get_srcSystem(), msg.get_srcComponent()
        print(f"[sysid={sysid}, compid={compid}] {msg}")
