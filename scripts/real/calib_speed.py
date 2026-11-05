#!/usr/bin/env python3
"""속도 보정: 정면 벽까지 라이다 거리 변화로 실제 속도를 재서 speed_scale·odom_scale·반응 지연을 구한다.

  python3 calib_speed.py --robot limo_pro --speed 0.3 [--dist 1.5]
준비: 정면 2.5 m 이상 앞에 평평한 벽(또는 상자)이 보이게 똑바로 둔다. 바닥 재질(일반/잔디)별로 따로 돌린다.
조종기 SWB = 위(코드 제어). 앞 0.5 m 안이면 정지. Ctrl+C = 정지.
"""
import argparse
import time

import numpy as np

import rt_common as rc
from robot_profile import add_args


def main():
    ap = add_args(argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter))
    ap.add_argument('--speed', type=float, default=0.3)
    ap.add_argument('--dist', type=float, default=1.5, help='주행 거리 m')
    ap.add_argument('--stop', type=float, default=0.5, help='정면 이 거리 안이면 정지 m')
    a = ap.parse_args()
    node = rc.start('calib_speed', a, want=('scan', 'odom', 'status'), cmd=True)
    try:
        miss = node.wait_for(['scan', 'odom'])
        if miss:
            print(f'수신 안 됨: {miss} → 중단')
            return
        print(rc.control_mode_note(node))
        # 출발 전 정면 거리(1 s 중앙값)
        fr0 = []
        node.cb['scan'] = lambda m: fr0.append(node.front_range(3.0))
        node.spin_for(1.0)
        fr0 = [x for x in fr0 if x is not None]
        if not fr0:
            print('정면(±3°)에 유효 점 없음 → 벽을 정면에 두고 다시')
            return
        d0 = float(np.median(fr0))
        if d0 - a.dist < a.stop:
            print(f'정면 {d0:.2f} m: 주행 {a.dist} m 하기엔 가깝다 → 더 뒤에서 시작하거나 --dist 줄이기')
            return
        print(f'정면 {d0:.2f} m, {a.speed} m/s로 {a.dist} m 주행', flush=True)
        log = []   # (t, 정면 거리, odom v) - 스캔마다
        t0 = time.monotonic()
        node.cb['scan'] = lambda m: log.append((time.monotonic() - t0, node.front_range(3.0),
                                                 rc.odom_speed(node.msgs['odom'])))

        def tick():
            fr = node.front_range(3.0)
            if fr is not None and (fr < a.stop or fr < d0 - a.dist):
                return False
            node.cmd.send(a.speed, 0.0)
            return True
        node.spin_for(a.dist / max(a.speed, 0.05) * 2.0 + 2.0, every=0.05, fn=tick)
        node.cmd.stop()
        node.spin_for(1.0)
        node.cb.pop('scan', None)

        L = np.array([(t, r, v) for t, r, v in log if r is not None])
        if len(L) < 8:
            print('기록 부족 → 실패')
            return
        t, r, v = L[:, 0], L[:, 1], L[:, 2]
        moved = np.nonzero(d0 - r > 0.02)[0]
        ov = np.nonzero(v > 0.05)[0]
        lat_lidar = t[moved[0]] if moved.size else float('nan')
        lat_odom = t[ov[0]] if ov.size else float('nan')
        # 정속 구간: 출발 후 1 s ~ 정지 명령 전
        steady = (t > (lat_lidar if np.isfinite(lat_lidar) else 0) + 1.0) & (v > 0.5 * a.speed)
        if steady.sum() < 5:
            print('정속 구간이 짧다 → --dist 늘려서 다시')
            return
        v_true = -np.polyfit(t[steady], r[steady], 1)[0]          # 거리 감소 기울기 = 실제 속도
        v_odom = float(np.mean(v[steady]))
        v_cmd = node.cmd.raw(a.speed, 0.0)[0]
        print(f'\n결과 (정속 {steady.sum()}스캔):')
        print(f'  명령 {v_cmd:.3f} m/s → 실제(라이다) {v_true:.3f} m/s, odom {v_odom:.3f} m/s')
        print(f'  반응 지연: odom {lat_odom:.2f} s, 라이다 {lat_lidar:.2f} s (라이다는 스캔 주기만큼 늦게 잡힘)')
        print(f'  총 이동: 라이다 {d0 - r[-1]:.2f} m')
        print(f'  → calib.speed_scale: {v_true / v_cmd:.3f}   calib.odom_scale: {v_true / v_odom:.3f}')
    except KeyboardInterrupt:
        print('\nCtrl+C → 정지')
    finally:
        rc.finish(node)


if __name__ == '__main__':
    main()
