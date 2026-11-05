#!/usr/bin/env python3
"""기말 시험형 덕트 트랙 world 생성 (Roboracer/F1TENTH식 주름 덕트 벽, 회색).

트랙: 약 21×7 m 안, 폭 1.0~1.8 m, 출발 (0, 1) +x 방향(roboracer_sim 기본 스폰).
덕트: 지름 0.35 m 원통을 0.125 m 마디로 이어 붙이고 반지름을 0.175/0.163 m로 번갈아 주름 흉내.
      약 6 m마다 3 cm 이음매 틈. 바깥 3 m에 방 벽(틈으로 보이는 배경).
  python3 make_duct_track.py [출력.world] [--png 미리보기.png]
"""
import argparse
import math
import os

import numpy as np
from scipy.interpolate import splev, splprep

HERE = os.path.dirname(os.path.abspath(__file__))

# 중심선 제어점 (x, y, 폭). 반시계 방향, 아래 직선 y=1 을 +x로 지나감.
CTRL = [
    (-6.0, 1.0, 1.6), (0.0, 1.0, 1.6), (6.0, 1.0, 1.6), (9.3, 1.5, 1.5),
    (10.2, 3.4, 1.4), (9.3, 5.2, 1.3), (7.2, 5.3, 1.2), (5.6, 4.0, 1.0),   # 좁은 S자
    (3.6, 3.7, 1.0), (1.9, 5.3, 1.2), (-0.5, 6.4, 1.5), (-4.5, 6.3, 1.7),
    (-8.3, 6.5, 1.5), (-10.0, 5.4, 1.3), (-10.4, 3.8, 1.2), (-9.9, 2.2, 1.3),
    (-8.6, 1.15, 1.5),
]
R_BIG, R_SMALL, PITCH = 0.175, 0.163, 0.125     # 덕트 반지름(주름 산/골), 마디 길이
SECTION, GAP = 6.0, 0.03                         # 덕트 한 토막 길이, 이음매 틈


def centerline(ds=0.05):
    c = np.array(CTRL + [CTRL[0]])
    tck, _ = splprep([c[:, 0], c[:, 1], c[:, 2]], s=0, per=True)
    u = np.linspace(0, 1, 4000, endpoint=False)
    x, y, w = splev(u, tck)
    seg = np.hypot(np.diff(x, append=x[0]), np.diff(y, append=y[0]))
    s = np.concatenate([[0], np.cumsum(seg)[:-1]])
    L = seg.sum()
    su = np.arange(0, L, ds)
    xs, ys, ws = (np.interp(su, s, v, period=L) for v in (x, y, w))
    dx, dy = np.gradient(xs), np.gradient(ys)
    n = np.hypot(dx, dy)
    return xs, ys, ws, -dy / n, dx / n, L          # 왼쪽 법선


def check(xs, ys, ws, ds=0.05):
    """서로 다른 구간의 덕트끼리 겹치지 않는지(트랙 경계 사이 최소 거리) 확인."""
    P = np.stack([xs, ys], 1)
    n = len(P)
    worst = (1e9, 0, 0)
    for i in range(0, n, 4):
        d = np.hypot(*(P - P[i]).T)
        k = np.abs(np.arange(n) - i)
        k = np.minimum(k, n - k) * ds
        far = k > 4.0
        need = (ws[i] + ws) / 2 + 4 * R_BIG + 0.05
        m = (d - need)[far]
        if m.size and m.min() < worst[0]:
            j = np.nonzero(far)[0][np.argmin(m)]
            worst = (m.min(), i, j)
    return worst


def boundary(xs, ys, ws, nx, ny, side):
    off = side * (ws / 2 + R_BIG)
    return xs + nx * off, ys + ny * off


def ducts(bx, by):
    """경계선을 따라 PITCH 마디 원통 목록 (cx, cy, yaw, r, len)."""
    seg = np.hypot(np.diff(bx, append=bx[0]), np.diff(by, append=by[0]))
    s = np.concatenate([[0], np.cumsum(seg)[:-1]])
    L = seg.sum()
    out, k, t = [], 0, 0.0
    while t + PITCH <= L:
        if (t % SECTION) + PITCH > SECTION - GAP and (t % SECTION) < SECTION:   # 이음매 틈
            t = (math.floor(t / SECTION) + 1) * SECTION + GAP / 2
            continue
        a = np.interp([t, t + PITCH], s, bx, period=L), np.interp([t, t + PITCH], s, by, period=L)
        cx, cy = (a[0][0] + a[0][1]) / 2, (a[1][0] + a[1][1]) / 2
        yaw = math.atan2(a[1][1] - a[1][0], a[0][1] - a[0][0])
        out.append((cx, cy, yaw, R_BIG if k % 2 == 0 else R_SMALL, PITCH * 1.02))
        k += 1
        t += PITCH
    return out


def sdf(cyl, room):
    links = []
    for i, (x, y, yaw, r, ln) in enumerate(cyl):
        g = f'<geometry><cylinder><radius>{r:.3f}</radius><length>{ln:.3f}</length></cylinder></geometry>'
        pose = f'<pose>{x:.3f} {y:.3f} {r:.3f} 0 1.5708 {yaw:.4f}</pose>'
        links.append(f'<collision name="c{i}">{pose}{g}</collision>'
                     f'<visual name="v{i}">{pose}{g}<material><ambient>0.55 0.55 0.57 1</ambient>'
                     f'<diffuse>0.6 0.6 0.62 1</diffuse></material></visual>')
    (x0, x1, y0, y1) = room
    walls = []
    for i, (cx, cy, sx, sy) in enumerate([((x0 + x1) / 2, y0, x1 - x0, 0.1), ((x0 + x1) / 2, y1, x1 - x0, 0.1),
                                          (x0, (y0 + y1) / 2, 0.1, y1 - y0), (x1, (y0 + y1) / 2, 0.1, y1 - y0)]):
        g = f'<geometry><box><size>{sx:.2f} {sy:.2f} 1.0</size></box></geometry>'
        walls.append(f'<collision name="w{i}"><pose>{cx:.2f} {cy:.2f} 0.5 0 0 0</pose>{g}</collision>'
                     f'<visual name="wv{i}"><pose>{cx:.2f} {cy:.2f} 0.5 0 0 0</pose>{g}</visual>')
    return f'''<?xml version="1.0"?>
<!-- make_duct_track.py 로 생성. 직접 고치지 말고 스크립트를 고친 뒤 다시 생성 -->
<sdf version="1.6">
  <world name="duct_track">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    <physics type="ode"><real_time_update_rate>1000</real_time_update_rate><max_step_size>0.001</max_step_size></physics>
    <model name="ducts"><static>true</static><link name="l">{''.join(links)}</link></model>
    <model name="room"><static>true</static><link name="l">{''.join(walls)}</link></model>
  </world>
</sdf>
'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out', nargs='?', default=os.path.join(HERE, 'duct_track.world'))
    ap.add_argument('--png', default=os.path.join(HERE, 'duct_track.png'))
    a = ap.parse_args()
    xs, ys, ws, nx, ny, L = centerline()
    m, i, j = check(xs, ys, ws)
    print(f'중심선 {L:.1f} m, 폭 {ws.min():.2f}~{ws.max():.2f} m, 범위 x {xs.min():.1f}~{xs.max():.1f} y {ys.min():.1f}~{ys.max():.1f}')
    print(f'구간 간 덕트 여유 최소 {m:+.2f} m ({xs[i]:.1f},{ys[i]:.1f})~({xs[j]:.1f},{ys[j]:.1f})' + ('  ← 겹침! CTRL 수정' if m < 0 else ''))
    k0 = int(np.argmin(np.hypot(xs - 0, ys - 1)))
    print(f'출발 (0,1) 근처 중심선 방향 {math.degrees(math.atan2(-nx[k0], ny[k0])):.1f}° (≈0 이어야 함), 폭 {ws[k0]:.2f}')
    cyl = []
    for side in (+1, -1):
        cyl += ducts(*boundary(xs, ys, ws, nx, ny, side))
    pad = 3.0
    allx = [c[0] for c in cyl]
    ally = [c[1] for c in cyl]
    room = (min(allx) - pad, max(allx) + pad, min(ally) - pad, max(ally) + pad)
    with open(a.out, 'w') as f:
        f.write(sdf(cyl, room))
    print(f'덕트 마디 {len(cyl)}개 → {a.out}')
    if a.png:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(12, 5))
        for (x, y, yaw, r, ln) in cyl:
            ax.add_patch(plt.Circle((x, y), r, color='0.6'))
        ax.plot(xs, ys, 'b--', lw=0.6)
        ax.annotate('start', (0, 1), color='g')
        ax.arrow(0, 1, 0.8, 0, width=0.05, color='g')
        ax.set_aspect('equal')
        ax.set_xlim(room[0], room[1])
        ax.set_ylim(room[2], room[3])
        ax.grid(alpha=0.3)
        fig.savefig(a.png, dpi=90, bbox_inches='tight')
        print(f'미리보기 → {a.png}')


if __name__ == '__main__':
    main()
