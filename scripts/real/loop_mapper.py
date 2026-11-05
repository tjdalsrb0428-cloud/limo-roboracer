#!/usr/bin/env python3
"""자체 매퍼: odom(바퀴+IMU) 자세로 스캔을 모으고, 한 바퀴 끝에서 출발 구간과 ICP로 맞춰 루프 오차를 궤적에 분배한 뒤 지도 작성.

  python3 loop_mapper.py maps/real/NAME --robot limo_pro [--own-odom] [--sim]
  1) 트랙 한 바퀴를 천천히(0.3~0.5 m/s) 돌고 출발 위치·방향 근처(1~2 m 안)에 세운다.
  2) Ctrl+C → 루프 보정 → NAME.pgm/.yaml + NAME_traj.csv (마지막 행 = 출발 pose). 다음: raceline.py NAME.yaml

시뮬 덕트 트랙에서 slam_toolbox·Cartographer가 odom(한 바퀴 0.25 m·3°)보다 나빠서(2026-11-05) 만든 방식.
odom→base_link TF가 필요: 로봇 EKF TF 또는 --own-odom(odom_tf.py).
"""
import argparse
import math
import os
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from robot_profile import add_args, load_profile, scan_valid
from rt_common import yaw_of

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.normpath(os.path.join(HERE, '..', '..'))
RES = 0.025
KF_DIST, KF_ANG = 0.10, math.radians(5)          # 키프레임 간격
L_FREE, L_OCC, L_MIN, L_MAX = -0.4, 0.9, -2.0, 3.5
MISS_LEN = 3.0                                    # 반사 없는 레이는 이만큼 빈칸


def se2(x, y, t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])


def icp(src, ref_tree, ref, iters=40, max_d=0.3):
    """2D 점-점 ICP. src를 ref에 맞추는 변환 T(3×3)와 (인라이어 비율, 평균 오차)."""
    T = np.eye(3)
    P = src.copy()
    for it in range(iters):
        d, j = ref_tree.query(P, distance_upper_bound=max_d)
        ok = np.isfinite(d)
        if ok.sum() < 30:
            return T, 0.0, 1e9
        A, B = P[ok], ref[j[ok]]
        ca, cb = A.mean(0), B.mean(0)
        H = (A - ca).T @ (B - cb)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[1] *= -1
            R = Vt.T @ U.T
        t = cb - R @ ca
        P = P @ R.T + t
        Ti = np.eye(3)
        Ti[:2, :2], Ti[:2, 2] = R, t
        T = Ti @ T
        if it > 5 and np.abs(t).max() < 1e-4:
            break
        max_d = max(0.08, max_d * 0.85)            # 점점 엄격하게
    d, _ = ref_tree.query(P, distance_upper_bound=0.1)
    ok = np.isfinite(d)
    return T, ok.mean(), (d[ok].mean() if ok.any() else 1e9)


def voxel(P, v=0.02):
    k = np.floor(P / v).astype(np.int64)
    _, i = np.unique(k[:, 0] * 1000003 + k[:, 1], return_index=True)
    return P[i]


class LoopMapper(Node):
    def __init__(self, prof, sim):
        super().__init__('loop_mapper', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=sim)])
        self.prof = prof
        self.fo, self.fb = prof['frames']['odom'], prof['frames']['base']
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self, spin_thread=False)
        self.kf = []            # (x, y, yaw, 레이저 원점(base), 히트점(base) Nx2, 미스 방향 각(base) )
        self.last = None
        self.nscan = 0
        self.lx = None          # base→laser 고정 변환
        self.pending = []       # TF(스캔 시각)가 올 때까지 보관
        self.ndrop = 0
        self.create_subscription(LaserScan, prof['topics']['scan'], self.on_scan, qos_profile_sensor_data)

    def on_scan(self, m):
        self.nscan += 1
        self.pending.append(m)

    def process(self):
        """보관 스캔 중 스캔 시각의 odom→base TF가 준비된 것 처리. 0.5 s 넘게 안 되면 버림."""
        keep = []
        for m in self.pending:
            st = Time.from_msg(m.header.stamp)
            if self.lx is None:
                try:
                    t = self.buf.lookup_transform(self.fb, m.header.frame_id, Time())
                    self.lx = (t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation))
                except Exception:
                    keep.append(m)
                    continue
            if self.buf.can_transform(self.fo, self.fb, st):
                self.add(m, self.buf.lookup_transform(self.fo, self.fb, st))
            elif (self.get_clock().now() - st).nanoseconds > 5e8:
                self.ndrop += 1
            else:
                keep.append(m)
        self.pending = keep

    def add(self, m, t):
        x, y, th = t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)
        if self.last is not None:
            dx, dy = x - self.last[0], y - self.last[1]
            dth = abs(math.atan2(math.sin(th - self.last[2]), math.cos(th - self.last[2])))
            if math.hypot(dx, dy) < KF_DIST and dth < KF_ANG:
                return
        a, r = scan_valid(m, self.prof)
        lx, ly, lt = self.lx
        hits = np.stack([lx + r * np.cos(a + lt), ly + r * np.sin(a + lt)], 1)
        # 반사 없는 방향(빈칸 레이용): 원래 메시지에서 inf/0/범위 밖
        rr = np.asarray(m.ranges, float)
        aa = m.angle_min + m.angle_increment * np.arange(rr.size)
        miss = aa[~(np.isfinite(rr) & (rr > float(self.prof['lidar']['range_min'])))] + lt
        self.kf.append((x, y, th, (lx, ly), hits, miss))
        self.last = (x, y, th)

    # ---------- 저장 ----------
    def poses(self):
        return np.array([(k[0], k[1], k[2]) for k in self.kf])

    def loop_close(self, P):
        """끝 구간 스캔을 출발 구간 스캔에 맞춰 보정 변환(world, 3×3) 반환. 실패 시 None."""
        s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(P[:, :2], axis=0).T))])
        L = s[-1]
        if L < 8.0:
            print(f'  주행 {L:.1f} m — 너무 짧아 루프 보정 생략', flush=True)
            return None, s
        gap = math.hypot(P[-1, 0] - P[0, 0], P[-1, 1] - P[0, 1])
        if gap > 2.5:
            print(f'  끝이 출발점에서 {gap:.1f} m 떨어짐 — 루프 보정 생략(출발점 근처에서 멈출 것)', flush=True)
            return None, s
        world = lambda i: self.kf[i][4] @ se2(*P[i])[:2, :2].T + P[i, :2]   # noqa: E731
        ref_idx = [i for i in range(len(P)) if s[i] < 4.0]
        src_idx = [i for i in range(len(P)) if s[i] > L - 2.0]
        ref = voxel(np.concatenate([world(i) for i in ref_idx]))
        src = voxel(np.concatenate([world(i) for i in src_idx]))
        T, fit, err = icp(src, cKDTree(ref), ref)
        dth = math.atan2(T[1, 0], T[0, 0])
        print(f'  루프 ICP: 인라이어 {fit:.2f}, 평균 오차 {err * 100:.1f} cm, 보정 ({T[0, 2]:+.3f}, {T[1, 2]:+.3f}) m, '
              f'{math.degrees(dth):+.2f}°', flush=True)
        if fit < 0.5 or err > 0.05 or abs(dth) > math.radians(20) or np.hypot(T[0, 2], T[1, 2]) > 1.5:
            print('  ICP 신뢰 낮음 — 루프 보정 생략', flush=True)
            return None, s
        return T, s

    def correct(self, P, T, s):
        """보정 T를 궤적에 나눠 적용: p' = R(fr·θ)·p + fs·t, yaw' = yaw + fr·θ.
        fr = 누적 회전각 비율(자이로 배율 오차는 돈 만큼 쌓임), fs = 진행 거리 비율. 끝에서 둘 다 1 → T."""
        th, t = math.atan2(T[1, 0], T[0, 0]), T[:2, 2]
        fs = s / s[-1]
        dy = np.abs(np.diff(np.unwrap(P[:, 2])))
        cr = np.concatenate([[0], np.cumsum(dy)])
        fr = cr / cr[-1] if cr[-1] > 1e-6 else fs
        out = P.copy()
        for i in range(len(P)):
            c, sn = math.cos(fr[i] * th), math.sin(fr[i] * th)
            out[i, 0] = c * P[i, 0] - sn * P[i, 1] + fs[i] * t[0]
            out[i, 1] = sn * P[i, 0] + c * P[i, 1] + fs[i] * t[1]
            out[i, 2] = P[i, 2] + fr[i] * th
        return out

    def raster(self, P):
        allpts = [self.kf[i][4] @ se2(*P[i])[:2, :2].T + P[i, :2] for i in range(len(P))]
        Q = np.concatenate(allpts + [P[:, :2]])
        x0, y0 = Q.min(0) - MISS_LEN - 0.5
        x1, y1 = Q.max(0) + MISS_LEN + 0.5
        nx, ny = int((x1 - x0) / RES) + 1, int((y1 - y0) / RES) + 1
        Lg = np.zeros((ny, nx), np.float32)
        for i in range(len(P)):
            R = se2(*P[i])
            ox, oy = R[:2, :2] @ np.array(self.kf[i][3]) + P[i, :2]
            hp = allpts[i]
            mdir = self.kf[i][5] + P[i, 2]
            mp = np.stack([ox + MISS_LEN * np.cos(mdir), oy + MISS_LEN * np.sin(mdir)], 1)
            ends = np.concatenate([hp, mp])
            d = np.hypot(ends[:, 0] - ox, ends[:, 1] - oy)
            n = np.maximum((d / (RES * 0.7)).astype(int), 1)
            idx = np.repeat(np.arange(len(ends)), n)
            frac = np.concatenate([np.arange(k) / k for k in n])
            fx = ((ox + frac * (ends[idx, 0] - ox) - x0) / RES).astype(int)
            fy = ((oy + frac * (ends[idx, 1] - oy) - y0) / RES).astype(int)
            ok = (fx >= 0) & (fx < nx) & (fy >= 0) & (fy < ny)
            Lg.flat[np.unique(fy[ok] * nx + fx[ok])] += L_FREE
            hx = ((hp[:, 0] - x0) / RES).astype(int)
            hy = ((hp[:, 1] - y0) / RES).astype(int)
            ok = (hx >= 0) & (hx < nx) & (hy >= 0) & (hy < ny)
            Lg.flat[np.unique(hy[ok] * nx + hx[ok])] += L_OCC - L_FREE
            np.clip(Lg, L_MIN, L_MAX, out=Lg)
        return Lg, x0, y0

    def save(self, base):
        if len(self.kf) < 5:
            print(f'키프레임 {len(self.kf)}개 — 저장 안 함 (스캔 {self.nscan}개 수신, TF odom→base_link 확인)', flush=True)
            return
        P = self.poses()
        T, s = self.loop_close(P)
        if T is not None:
            P = self.correct(P, T, s)
        Lg, x0, y0 = self.raster(P)
        p = 1 - 1 / (1 + np.exp(Lg))
        img = np.full(Lg.shape, 205, np.uint8)
        img[(Lg != 0) & (p < 0.35)] = 254
        img[p > 0.65] = 0
        img = img[::-1]
        known = np.where(img != 205)
        r0, r1 = max(known[0].min() - 10, 0), min(known[0].max() + 10, img.shape[0])
        c0, c1 = max(known[1].min() - 10, 0), min(known[1].max() + 10, img.shape[1])
        img = img[r0:r1, c0:c1]
        ox, oy = x0 + c0 * RES, y0 + (Lg.shape[0] - r1) * RES
        os.makedirs(os.path.dirname(base) or '.', exist_ok=True)
        with open(base + '.pgm', 'wb') as f:
            f.write(f'P5\n{img.shape[1]} {img.shape[0]}\n255\n'.encode())
            f.write(img.tobytes())
        with open(base + '.yaml', 'w') as f:
            f.write(f'image: {os.path.basename(base)}.pgm\nmode: trinary\nresolution: {RES}\n'
                    f'origin: [{ox:.3f}, {oy:.3f}, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
        with open(base + '_traj.csv', 'w') as f:
            f.write('x,y,yaw\n')
            for x, y, t in P:
                f.write(f'{x:.4f},{y:.4f},{math.atan2(math.sin(t), math.cos(t)):.4f}\n')
        x, y, t = P[-1]
        print(f'지도 저장: {base}.yaml ({img.shape[1]}×{img.shape[0]} 칸), 키프레임 {len(P)}개, '
              f'출발 pose=({x:.2f},{y:.2f},{t:.2f})', flush=True)


def main():
    ap = add_args(argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter))
    ap.add_argument('out', nargs='?', default=os.path.join(WS, 'maps', 'real', 'map'))
    ap.add_argument('--own-odom', action='store_true', help='odom_tf.py 같이 실행(로봇에 odom TF 없을 때)')
    ap.add_argument('--sim', action='store_true')
    a = ap.parse_args()
    base = os.path.abspath(a.out[:-5] if a.out.endswith('.yaml') else a.out)
    prof = load_profile(a.robot, a.calib)
    procs = []
    if a.own_odom:
        procs.append(subprocess.Popen([sys.executable, os.path.join(HERE, 'odom_tf.py'), '--robot', a.robot] +
                                      (['--calib', a.calib] if a.calib else []) + (['--sim'] if a.sim else []),
                                      start_new_session=True))
        time.sleep(2.5)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = LoopMapper(prof, a.sim)
    stop = {'f': False}
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__('f', True))
    print(f'[loop_mapper] robot={prof["name"]} → {base}.*  (한 바퀴 돌고 출발점 근처에 세운 뒤 Ctrl+C)', flush=True)
    t_log = time.time()
    try:
        while rclpy.ok() and not stop['f']:
            rclpy.spin_once(node, timeout_sec=0.02)
            node.process()
            if time.time() - t_log > 5.0:
                t_log = time.time()
                print(f'  키프레임 {len(node.kf)}개 / 스캔 {node.nscan}개(버림 {node.ndrop}), pose={tuple(round(v, 2) for v in node.last) if node.last else "없음(TF 대기)"}',
                      flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.02)
            node.process()
        print('저장 중...', flush=True)
        node.save(base)
        for p in procs:
            try:
                os.killpg(p.pid, signal.SIGINT)
                p.wait(timeout=5)
            except Exception:
                pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
