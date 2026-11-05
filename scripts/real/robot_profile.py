#!/usr/bin/env python3
"""로봇 프로파일 로더 + 명령 발행 + 스캔 필터 (LIMO1 / LIMO Pro 공용).

사용 예:
    from robot_profile import load_profile, CmdSender, scan_valid
    prof = load_profile('limo_pro')                 # 또는 'limo1', 또는 yaml 경로
    prof = load_profile('limo_pro', calib='calib_1253.yaml')   # 로봇별 보정값 덮어쓰기
    cmd = CmdSender(node, prof)
    cmd.send(0.3, 0.2)       # 속도 0.3 m/s, 중심 조향각 0.2 rad(+ = 좌회전)
    cmd.stop()
    ang, rng = scan_valid(scan_msg, prof)            # 유효한 점만 (각도 rad, 거리 m)

로봇 호스트에서 설치 없이 돌도록 표준 라이브러리 + PyYAML + numpy + rclpy 메시지만 쓴다.
"""
import math
import os

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PROFILE_DIR = os.path.normpath(os.path.join(HERE, '..', '..', 'config', 'robots'))


def _merge(base, over):
    """over의 값으로 base를 재귀적으로 덮어쓴 새 dict."""
    out = dict(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _merge(out[k], v)
        else:
            out[k] = v
    return out


def _find(path_or_name):
    if os.path.isfile(path_or_name):
        return path_or_name
    p = os.path.join(PROFILE_DIR, path_or_name if path_or_name.endswith('.yaml') else path_or_name + '.yaml')
    if os.path.isfile(p):
        return p
    names = sorted(f[:-5] for f in os.listdir(PROFILE_DIR) if f.endswith('.yaml') and not f.startswith('calib'))
    raise FileNotFoundError(f'프로파일 없음: {path_or_name} (가능: {", ".join(names)})')


def load_profile(name, calib=None):
    """프로파일 dict 반환. calib = 로봇별 보정 yaml(같은 구조의 일부만 적어도 됨)."""
    with open(_find(name)) as f:
        prof = yaml.safe_load(f)
    if calib:
        with open(_find(calib)) as f:
            prof = _merge(prof, yaml.safe_load(f))
    return prof


def add_args(parser):
    """argparse에 --robot, --calib 옵션 추가."""
    parser.add_argument('--robot', required=True, help='limo1 | limo_pro | 프로파일 yaml 경로')
    parser.add_argument('--calib', default=None, help='로봇별 보정 yaml (선택)')
    return parser


class CmdSender:
    """원하는 (속도, 중심 조향각)을 로봇 종류에 맞는 메시지로 바꿔 발행한다.

    보정: 실제 조향 = 명령 × steer_scale + steer_trim 이라고 보고 역으로 명령을 계산한다.
          실제 속도 = 명령 × speed_scale.
    """

    def __init__(self, node, prof, qos=10):
        self.prof = prof
        c = prof['command']
        self.kind = c['type']
        self.steer_max = float(c['steer_max'])
        self.speed_max = float(c['speed_max'])
        self.L = float(prof['vehicle']['wheelbase'])
        cal = prof.get('calib', {}) or {}
        self.steer_scale = float(cal.get('steer_scale', 1.0)) or 1.0
        self.steer_trim = float(cal.get('steer_trim', 0.0))
        self.speed_scale = float(cal.get('speed_scale', 1.0)) or 1.0
        topic = prof['topics']['cmd']
        if self.kind == 'twist':
            from geometry_msgs.msg import Twist
            self._Msg = Twist
        elif self.kind == 'ackermann':
            from ackermann_msgs.msg import AckermannDrive
            self._Msg = AckermannDrive
        else:
            raise ValueError(f'알 수 없는 command.type: {self.kind}')
        self.pub = node.create_publisher(self._Msg, topic, qos)
        self.last = (0.0, 0.0)   # 실제로 보낸 (속도, 조향 명령)

    def raw(self, v, steer):
        """보정·제한 적용 후 (속도 명령, 조향 명령) 계산만 한다(발행 안 함)."""
        v_cmd = max(-self.speed_max, min(self.speed_max, v / self.speed_scale))
        s_cmd = steer / self.steer_scale + self.steer_trim
        s_cmd = max(-self.steer_max, min(self.steer_max, s_cmd))
        return v_cmd, s_cmd

    def send(self, v, steer):
        """v: m/s(+ 전진), steer: 원하는 중심 조향각 rad(+ 좌회전)."""
        v_cmd, s_cmd = self.raw(v, steer)
        m = self._Msg()
        if self.kind == 'twist':
            # LIMO1 드라이버는 r = v/ω 로 조향을 정한다 → ω = v·tan(δ)/L
            # (정지 상태(v=0)에서는 조향 명령을 줄 수 없다)
            m.linear.x = v_cmd
            m.angular.z = v_cmd * math.tan(s_cmd) / self.L
        else:
            m.speed = v_cmd
            m.steering_angle = s_cmd
        self.pub.publish(m)
        self.last = (v_cmd, s_cmd)

    def stop(self, n=5):
        for _ in range(n):
            self.send(0.0, 0.0)


def scan_valid(msg, prof):
    """LaserScan에서 유효한 점만 (각도 배열, 거리 배열)로 반환.

    각도는 메시지의 angle_min/angle_increment/ranges 길이로 계산한다(점 수·범위를 가정하지 않음).
    0, inf, nan, 프로파일 range 밖, 차체 안(넓은 각도에서 자기 바퀴·몸체가 찍힘, 시뮬 Pro ±101° 밖)은 버린다.
    """
    import numpy as np
    r = np.asarray(msg.ranges, dtype=float)
    a = msg.angle_min + msg.angle_increment * np.arange(r.size)
    lo = max(float(prof['lidar']['range_min']), float(msg.range_min))
    hi = min(float(prof['lidar']['range_max']), float(msg.range_max))
    ok = np.isfinite(r) & (r >= lo) & (r <= hi)
    v = prof['vehicle']
    m = float(prof['lidar'].get('self_margin', 0.03))
    xb = float(prof['lidar'].get('x_offset', 0.0)) + r * np.cos(a)       # base_link 기준
    yb = r * np.sin(a)
    body = (np.abs(xb) <= v['length'] / 2 + m) & (np.abs(yb) <= v['width'] / 2 + m)
    ok &= ~body
    return a[ok], r[ok]


if __name__ == '__main__':
    # 프로파일 확인용: python3 robot_profile.py limo_pro
    import sys
    p = load_profile(sys.argv[1] if len(sys.argv) > 1 else 'limo_pro',
                     sys.argv[2] if len(sys.argv) > 2 else None)
    print(yaml.safe_dump(p, allow_unicode=True, sort_keys=False))
