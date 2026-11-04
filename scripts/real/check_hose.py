#!/usr/bin/env python3
"""호스 감지 점검: 정면 부채꼴에서 물체(연속된 점 묶음)를 찾아 거리·폭·점 수·빠진 점을 1초마다 출력. 로봇은 움직이지 않는다.

  python3 check_hose.py --robot limo_pro [--sector 30]
로봇을 호스 정면 0.5 / 1 / 2 / 3 m에 차례로 놓고 본다.
  - '점'이 '기대'보다 많이 적거나 '빠짐'이 크면 → 검은 호스 반사 약함(점 누락)
  - 호스보다 가까운 곳에 작은 묶음이 계속 생기면 → 바닥 찍힘 의심
  - 호스 이음매 방향에서 거리가 갑자기 멀어지면 → 틈으로 바깥이 보임
"""
import argparse
import math

import numpy as np

import rt_common as rc
from robot_profile import add_args


def segments(ang, rng, inc):
    """각도순 유효 점을 이웃 점 사이 실제 간격(>max(10 cm, 3×점간격)) 또는 3칸 이상 빈틈에서 끊어 묶음 목록 반환.
    (거리 차이만 보면 비스듬히 보이는 벽이 점마다 끊어진다)"""
    if ang.size == 0:
        return []
    x, y = rng * np.cos(ang), rng * np.sin(ang)
    gap = np.hypot(np.diff(x), np.diff(y))
    cut = np.nonzero((gap > np.maximum(0.10, 3.0 * rng[:-1] * inc)) |
                     (np.diff(ang) > 3.5 * inc))[0] + 1
    out = []
    for idx in np.split(np.arange(ang.size), cut):
        a, r = ang[idx], rng[idx]
        span = a[-1] - a[0]
        exp = int(round(span / inc)) + 1
        out.append(dict(r=float(r.min()), a=math.degrees(float(np.mean(a))), n=idx.size, exp=exp,
                        width=float(np.hypot(x[idx[-1]] - x[idx[0]], y[idx[-1]] - y[idx[0]])), miss=exp - idx.size))
    return out


def main():
    ap = add_args(argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter))
    ap.add_argument('--sector', type=float, default=30.0, help='정면 ±각도(deg)')
    ap.add_argument('--top', type=int, default=4, help='가까운 묶음 몇 개 출력')
    a = ap.parse_args()
    node = rc.start('check_hose', a, want=('scan',))
    try:
        if node.wait_for(['scan']):
            print('/scan 수신 안 됨 → 중단')
            return
        P = node.prof
        while True:
            node.spin_for(1.0)
            m = node.msgs['scan']
            r_all = np.asarray(m.ranges, float)
            a_all = m.angle_min + m.angle_increment * np.arange(r_all.size)
            sec = np.abs(a_all) <= math.radians(a.sector)
            lo = max(float(P['lidar']['range_min']), m.range_min)
            hi = min(float(P['lidar']['range_max']), m.range_max)
            ok = sec & np.isfinite(r_all) & (r_all >= lo) & (r_all <= hi)
            bad = int(sec.sum() - ok.sum())
            segs = sorted(segments(a_all[ok], r_all[ok], m.angle_increment), key=lambda s: s['r'])
            print(f'--- 정면 ±{a.sector:.0f}°: 점 {int(sec.sum())}개 중 무효 {bad}개 ({100 * bad / max(1, sec.sum()):.0f} %)')
            for s in segs[:a.top]:
                print(f"  최근접 {s['r']:5.2f} m  방향 {s['a']:+6.1f}°  폭 {s['width']:4.2f} m  점 {s['n']:3d}/기대 {s['exp']:3d}  빠짐 {s['miss']}",
                      flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        rc.finish(node)


if __name__ == '__main__':
    main()
