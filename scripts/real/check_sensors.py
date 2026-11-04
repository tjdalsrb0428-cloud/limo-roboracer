#!/usr/bin/env python3
"""센서 점검: 토픽 주기, 라이다 실제 사양, IMU 정지 편향, 배터리·모드. 로봇은 움직이지 않는다.

  python3 check_sensors.py --robot limo_pro [--sec 5]
결과의 '라이다' 줄을 프로파일(lidar)과 비교하고, 다르면 프로파일을 고친다.
"""
import argparse
import math

import numpy as np

import rt_common as rc
from robot_profile import add_args, scan_valid


def main():
    ap = add_args(argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter))
    ap.add_argument('--sec', type=float, default=5.0, help='측정 시간(초)')
    a = ap.parse_args()
    node = rc.start('check_sensors', a, want=('scan', 'odom', 'odom_filtered', 'imu', 'status'))
    gz, valid_ratio = [], []
    node.cb['imu'] = lambda m: gz.append(m.angular_velocity.z)

    def on_scan(m):
        r = np.asarray(m.ranges, float)
        valid_ratio.append(scan_valid(m, node.prof)[1].size / max(1, r.size))
    node.cb['scan'] = on_scan
    try:
        node.spin_for(a.sec)
    except KeyboardInterrupt:
        pass
    P = node.prof
    print(f'\n=== 토픽 주기 ({a.sec:.0f} s) ===')
    exp = {'scan': P['lidar']['rate_hz']}
    for k in ('scan', 'odom', 'odom_filtered', 'imu', 'status'):
        st = node.stamps.get(k, [])
        hz = (len(st) - 1) / (st[-1] - st[0]) if len(st) > 2 else 0.0
        note = '' if st else '  ← 수신 없음 (드라이버·LOCALHOST_ONLY·토픽 이름 확인)'
        if k in exp and st and abs(hz - exp[k]) > 0.2 * exp[k]:
            note = f'  ← 프로파일 {exp[k]} Hz와 다름'
        print(f'{k:14s} {P["topics"].get(k, "-"):22s} {hz:6.1f} Hz{note}')

    m = node.msgs.get('scan')
    if m is not None:
        n = len(m.ranges)
        print('\n=== 라이다 (메시지 실측 / 프로파일) ===')
        print(f'frame      {m.header.frame_id}  /  {P["frames"]["laser"]}')
        print(f'angle      {math.degrees(m.angle_min):.1f} ~ {math.degrees(m.angle_max):.1f} deg  /  '
              f'{P["lidar"]["angle_min_deg"]} ~ {P["lidar"]["angle_max_deg"]}')
        print(f'points     {n}  (간격 {math.degrees(m.angle_increment):.2f} deg)  /  약 {P["lidar"]["points_approx"]}')
        print(f'range      {m.range_min:.2f} ~ {m.range_max:.1f} m (드라이버)  /  사용 {P["lidar"]["range_min"]} ~ {P["lidar"]["range_max"]}')
        ang, rng = scan_valid(m, P)
        if rng.size:
            print(f'유효 점    평균 {100 * np.mean(valid_ratio):.0f} %   최근 스캔 최소 {rng.min():.2f} m / 최대 {rng.max():.2f} m')
            k = int(np.argmin(np.abs(ang)))
            print(f'정면       {rng[k]:.2f} m (각도 {math.degrees(ang[k]):.1f} deg)')
    if gz:
        g = np.asarray(gz)
        print(f'\n=== IMU (정지 상태여야 함) ===\ngyro z 평균 {g.mean():+.4f} rad/s (편향), 표준편차 {g.std():.4f}')
    o = node.msgs.get('odom')
    if o is not None:
        print(f'\n=== odom ===\nframe {o.header.frame_id} -> {o.child_frame_id}, v={o.twist.twist.linear.x:+.3f} m/s')
    print('\n=== 상태 ===\n' + rc.control_mode_note(node))
    rc.finish(node)


if __name__ == '__main__':
    main()
