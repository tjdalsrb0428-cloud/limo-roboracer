#!/usr/bin/env python3
"""조향 보정: 직진 트림(steer_trim)과 조향 배율(steer_scale)을 IMU 회전 속도로 잰다.

  # 1) 직진 트림: 앞이 2 m 이상 비어 있는 곳에서
  python3 calib_steer.py --robot limo_pro --mode straight
  # 2) 조향 배율: 지름 2 m 원을 그릴 공간에서 (좌·우 각각 원을 돈다)
  python3 calib_steer.py --robot limo_pro --mode circle --steer 0.3

원리: 실제 곡률 κ = ω(IMU) / v(odom × odom_scale) → 실제 중심 조향각 = atan(L·κ).
calib_speed.py를 먼저 돌려 odom_scale을 넣어 두면 더 정확하다.
조종기 SWB = 위(코드 제어). 앞 0.35 m 안에 물체가 보이면 즉시 정지. Ctrl+C = 정지.
"""
import argparse
import math

import numpy as np

import rt_common as rc
from robot_profile import add_args


def drive(node, v, steer, sec, gz, vs, settle=1.0, stop_dist=0.35):
    """(v, steer)로 sec 초 주행하며 settle 이후의 IMU ω, odom v 기록. 막히면 False."""
    t = {'n': 0}
    dt = 0.05

    def tick():
        t['n'] += 1
        fr = node.front_range(20.0)
        if fr is not None and fr < stop_dist:
            print(f'  ! 앞 {fr:.2f} m → 정지', flush=True)
            return False
        node.cmd.send(v, steer)
        if t['n'] * dt >= settle:
            if 'imu' in node.msgs:
                gz.append(node.msgs['imu'].angular_velocity.z)
            if 'odom' in node.msgs:
                vs.append(node.msgs['odom'].twist.twist.linear.x)
        return True
    ok = node.spin_for(sec, every=dt, fn=tick)
    node.cmd.stop()
    node.spin_for(1.0)
    return ok


def bias(node, sec=2.0):
    g = []
    node.cb['imu'] = lambda m: g.append(m.angular_velocity.z)
    node.spin_for(sec)
    node.cb.pop('imu', None)
    return float(np.mean(g)) if g else 0.0


def actual_steer(node, gz, vs, b):
    L = float(node.prof['vehicle']['wheelbase'])
    osc = float(node.prof['calib'].get('odom_scale', 1.0))
    w = float(np.mean(gz)) - b
    v = float(np.mean(vs)) * osc
    if abs(v) < 0.05:
        return None, w, v
    return math.atan(L * w / v), w, v


def main():
    ap = add_args(argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter))
    ap.add_argument('--mode', choices=['straight', 'circle'], required=True)
    ap.add_argument('--speed', type=float, default=0.3)
    ap.add_argument('--steer', type=float, default=0.3, help='circle: 시험 조향각 rad')
    ap.add_argument('--dist', type=float, default=2.0, help='straight: 주행 거리 m')
    ap.add_argument('--time', type=float, default=7.0, help='circle: 방향별 주행 시간 s')
    a = ap.parse_args()
    node = rc.start('calib_steer', a, want=('scan', 'odom', 'imu', 'status'), cmd=True)
    try:
        miss = node.wait_for(['scan', 'odom', 'imu'])
        if miss:
            print(f'수신 안 됨: {miss} → 중단')
            return
        print(rc.control_mode_note(node))
        b = bias(node)
        print(f'IMU 편향 {b:+.4f} rad/s (정지 2 s)')
        cal = node.prof['calib']
        if a.mode == 'straight':
            gz, vs = [], []
            sec = a.dist / max(a.speed, 0.05) + 1.0
            print(f'직진 {a.dist} m @ {a.speed} m/s (현재 steer_trim {cal.get("steer_trim", 0.0):+.3f})', flush=True)
            drive(node, a.speed, 0.0, sec, gz, vs)
            d, w, v = actual_steer(node, gz, vs, b)
            if d is None:
                print('속도가 거의 0 → 측정 실패 (SWB, 배터리 확인)')
                return
            s = float(cal.get('steer_scale', 1.0))
            new_trim = float(cal.get('steer_trim', 0.0)) - d / s
            side = '왼쪽' if d > 0 else '오른쪽'
            print(f'\n결과: ω={w:+.4f} rad/s, v={v:.3f} m/s → 직진 명령에서 실제 조향 {d:+.4f} rad ({side}으로 쏠림)')
            print(f'  → calib.steer_trim: {new_trim:+.4f}   (다시 돌려서 |실제 조향| < 0.005 이면 완료)')
        else:
            res = {}
            for name, sg in (('좌', +1), ('우', -1)):
                gz, vs = [], []
                print(f'{name}회전 원: steer {sg * a.steer:+.2f} rad, {a.speed} m/s, {a.time:.0f} s', flush=True)
                if not drive(node, a.speed, sg * a.steer, a.time, gz, vs):
                    print('  중단됨(장애물) → 공간 넓은 곳에서 다시')
                    return
                d, w, v = actual_steer(node, gz, vs, b)
                if d is None:
                    print('  속도 0 → 측정 실패')
                    return
                cmd_s = node.cmd.raw(a.speed, sg * a.steer)[1]   # 실제로 보낸 조향 명령(보정·제한 적용 후)
                R = v / w if abs(w) > 1e-3 else float('inf')
                res[name] = (d, cmd_s)
                print(f'  ω={w:+.3f} rad/s v={v:.3f} m/s → 반경 {abs(R):.2f} m, 실제 조향 {d:+.3f} rad (명령 {cmd_s:+.3f})')
            # 실제 = 명령 × scale + δ0  두 점(좌·우)으로 scale, δ0 추정
            (dl, cl), (dr, cr) = res['좌'], res['우']
            scale = (dl - dr) / (cl - cr)
            d0 = dl - scale * cl
            print(f'\n결과: steer_scale ≈ {scale:.3f} (좌 {dl / cl:.3f}, 우 {dr / cr:.3f}), 0점 치우침 {d0:+.4f} rad')
            # CmdSender: 명령 = 원하는 값/scale + trim → scale = 명령→실제 배율, trim = -δ0/scale
            print(f'  → calib.steer_scale: {scale:.3f}   calib.steer_trim: {-d0 / scale:+.4f}')
            print('  좌·우 배율 차이가 10 % 넘으면 기계적 비대칭 → 기록해 둘 것')
            print(f'  최소 회전반경(추정) ≈ {float(node.prof["vehicle"]["wheelbase"]) / math.tan(abs(scale) * float(node.prof["command"]["steer_max"])):.2f} m')
    except KeyboardInterrupt:
        print('\nCtrl+C → 정지')
    finally:
        rc.finish(node)


if __name__ == '__main__':
    main()
