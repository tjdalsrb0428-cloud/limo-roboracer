#!/usr/bin/env python3
"""측정 스크립트 공용: 노드 생성, 센서 구독, 시간 대기, Ctrl+C 안전 정지.

로봇 호스트 기본 ROS(rclpy) + numpy + PyYAML만 사용한다(설치 불필요).
ROS_LOCALHOST_ONLY / ROS_DOMAIN_ID 는 드라이버를 띄운 셸과 같게 export 하고 실행할 것.
"""
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

from robot_profile import CmdSender, load_profile, scan_valid  # noqa: F401


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Rt(Node):
    """프로파일의 토픽을 구독해 최신값·수신 시각을 모아 둔다."""

    def __init__(self, name, prof, want=('scan', 'odom', 'imu', 'status'), cmd=False):
        super().__init__(name)
        self.prof = prof
        self.msgs = {}     # key -> 최신 메시지
        self.stamps = {}   # key -> 수신 시각(time.monotonic) 목록
        self.cb = {}       # key -> 추가 콜백
        T = prof['topics']
        types = {}
        from sensor_msgs.msg import LaserScan, Imu
        from nav_msgs.msg import Odometry
        types.update(scan=LaserScan, odom=Odometry, odom_filtered=Odometry, imu=Imu)
        try:
            from limo_msgs.msg import LimoStatus
            types['status'] = LimoStatus
        except ImportError:
            pass
        for k in want:
            if k in types and T.get(k):
                self.create_subscription(types[k], T[k], self._mk(k), qos_profile_sensor_data)
        self.cmd = CmdSender(self, prof) if cmd else None

    def _mk(self, key):
        def f(m):
            self.msgs[key] = m
            self.stamps.setdefault(key, []).append(time.monotonic())
            if key in self.cb:
                self.cb[key](m)
        return f

    def spin_for(self, sec, every=None, fn=None):
        """sec 초 동안 콜백 처리. every 초마다 fn() 호출(명령 발행 등). fn이 False 반환하면 중단."""
        t0 = time.monotonic()
        nxt = t0
        while time.monotonic() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.005)
            if every and time.monotonic() >= nxt:
                nxt += every
                if fn() is False:
                    return False
        return True

    def wait_for(self, keys, timeout=5.0):
        """keys 메시지가 모두 올 때까지 대기. 못 받은 키 목록 반환."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(k in self.msgs for k in keys):
                return []
        return [k for k in keys if k not in self.msgs]

    def front_range(self, half_deg=5.0):
        """정면 ±half_deg 안 유효 점의 중앙값 거리(없으면 None)."""
        import numpy as np
        m = self.msgs.get('scan')
        if m is None:
            return None
        a, r = scan_valid(m, self.prof)
        sel = np.abs(a) <= math.radians(half_deg)
        return float(np.median(r[sel])) if sel.any() else None


def start(name, args, want=('scan', 'odom', 'imu', 'status'), cmd=False):
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # Ctrl+C → KeyboardInterrupt (정지 명령 보낼 수 있게)
    prof = load_profile(args.robot, args.calib)
    node = Rt(name, prof, want, cmd)
    print(f"[{name}] robot={prof['name']} cmd={prof['topics']['cmd']} "
          f"ROS_LOCALHOST_ONLY={os.environ.get('ROS_LOCALHOST_ONLY', '(없음)')} "
          f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '(없음)')}", flush=True)
    return node


def finish(node):
    if node.cmd:
        node.cmd.stop(10)
        node.spin_for(0.2)
    node.destroy_node()
    rclpy.try_shutdown()


def control_mode_note(node):
    """LIMO 조종기 SWB 상태 안내(limo_status.control_mode)."""
    s = node.msgs.get('status')
    if s is None:
        return '(limo_status 없음)'
    return (f'control_mode={s.control_mode} motion_mode={s.motion_mode} 배터리={s.battery_voltage:.1f}V '
            '(코드로 움직이려면 조종기 SWB를 위로; 안 움직이면 이 값 확인)')
