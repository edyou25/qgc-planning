from collections import deque
from dataclasses import dataclass
from typing import Deque, List

@dataclass
class Buffer:
    t: Deque[float]; x: Deque[float]; y: Deque[float]; z: Deque[float]
@dataclass
class AttitudeBuffer:
    t: Deque[float]; roll: Deque[float]; pitch: Deque[float]; yaw: Deque[float]
@dataclass
class VelBuffer:
    t: Deque[float]; vx: Deque[float]; vy: Deque[float]; vz: Deque[float]; vx_sp: Deque[float]; vy_sp: Deque[float]; vz_sp: Deque[float]
@dataclass
class ImuBuffer:
    t: Deque[float]; ax: Deque[float]; ay: Deque[float]; az: Deque[float]; gx: Deque[float]; gy: Deque[float]; gz: Deque[float]
@dataclass
class AltBuffer:
    t: Deque[float]; alt_amsl: Deque[float]; alt_rel: Deque[float]
@dataclass
class GpsBuffer:
    t: Deque[float]; sats: Deque[int]; eph: Deque[float]; epv: Deque[float]; fix: Deque[int]
@dataclass
class ServoBuffer:
    t: Deque[float]; ch: List[Deque[int]]

@dataclass
class MissionState:
    missions: List[tuple]

@dataclass
class AppState:
    pos: Buffer
    att: AttitudeBuffer
    vel: VelBuffer
    imu: ImuBuffer
    alt: AltBuffer
    gps: GpsBuffer
    servo: ServoBuffer
    mission: MissionState


def create_state(window: int) -> AppState:
    dq = lambda: deque(maxlen=window)
    pos = Buffer(dq(), dq(), dq(), dq())
    att = AttitudeBuffer(dq(), dq(), dq(), dq())
    vel = VelBuffer(*(dq() for _ in range(7)))
    imu = ImuBuffer(*(dq() for _ in range(7)))
    alt = AltBuffer(dq(), dq(), dq())
    gps = GpsBuffer(dq(), dq(), dq(), dq(), dq())
    servo = ServoBuffer(dq(), [dq() for _ in range(8)])
    mission = MissionState([])
    return AppState(pos, att, vel, imu, alt, gps, servo, mission)
