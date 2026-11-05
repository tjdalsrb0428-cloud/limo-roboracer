#!/usr/bin/env python3
"""위치추정: 파티클 필터(likelihood field) → TF map→odom.

  python3 localizer.py --map maps/real/NAME.yaml --robot limo_pro [--init x y yaw] [--sim]
- 지도: loop_mapper/raceline 형식(pgm/yaml). 초기 자세: --init 없으면 NAME_traj.csv 마지막 줄(= 출발 pose).
- 움직임: TF odom→base_link 변화량(odom_tf.py 또는 로봇 EKF) + 잡음.
- 관측: 스캔 빔(차체·무효 점 제외)을 고르게 N_BEAMS개 골라 지도 벽까지 거리로 우도 계산(점 수·각도 범위는 메시지 그대로).
- 재초기화: RViz "2D Pose Estimate"(/initialpose) — 대회 규정상 initial guess 허용.
- 출력: TF map→odom, /pf/pose(PoseWithCovarianceStamped), 상태 줄(점수 = 빔이 벽에 맞는 비율).
"""
import argparse
import math
import os
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.ndimage import distance_transform_edt
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from robot_profile import add_args, load_profile, scan_valid
from rt_common import yaw_of

N_PART = 600
N_BEAMS = 60
SIGMA = 0.08            # 빔 끝점-벽 거리 표준편차(m): 지도 오차·덕트 주름 포함
Z_RAND = 0.15           # 지도에 없는 점(사람·덕트 틈 너머·잡음) 몫. 작으면 그런 점이 자세를 끌어당김
D_MAX = 1.0             # 거리장 상한(m)
A_TRANS, A_ROT = 0.03, 0.03     # 움직임 잡음: 이동량·회전량에 비례. odom(바퀴+IMU)이 1~3 %로 정확 →
                                 # 10 %면 U턴(원호 통로, 원 중심 회전 방향은 라이다로 구분 불가)에서 진행 방향으로 0.4 m·11° 흘러감(시뮬)
A_FLOOR = (0.005, 0.005, 0.005)  # 매 갱신 최소 잡음 (m, m, rad)
UPD_DIST, UPD_ANG = 0.03, math.radians(2)


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def load_map(path):
    m = yaml.safe_load(open(path))
    img_path = os.path.join(os.path.dirname(path), m['image'])
    with open(img_path, 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()
        img = np.frombuffer(f.read(), np.uint8).reshape(h, w)
    img = img[::-1]                                   # 행 0 = 아래(y 최소)
    occ = img < 50
    res = float(m['resolution'])
    dist = distance_transform_edt(~occ) * res
    return np.minimum(dist, D_MAX).astype(np.float32), res, float(m['origin'][0]), float(m['origin'][1]), img


class Localizer(Node):
    def __init__(self, prof, mp, init, sim):
        super().__init__('localizer', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=sim)])
        self.prof = prof
        self.fm, self.fo, self.fb = 'map', prof['frames']['odom'], prof['frames']['base']
        self.dist, self.res, self.ox, self.oy, img = load_map(mp)
        self.H, self.W = self.dist.shape
        self.free_idx = np.argwhere(img > 250)            # 전역 재초기화용
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self, spin_thread=False)
        self.tfb = TransformBroadcaster(self)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/pf/pose', 10)
        self.create_subscription(LaserScan, prof['topics']['scan'], lambda m: self.pending.append(m), qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_init, 10)
        self.pending = []
        self.lx = None
        self.rng = np.random.default_rng(0)
        self.reset(*init, 0.10, math.radians(5))
        self.last_odom = None           # 마지막 갱신 때 odom 자세
        self.est = np.array(init, float)
        self.m2o = None                 # (x, y, yaw) map→odom
        self.score = 0.0
        self.nupd = 0
        self.t_proc = []

    def reset(self, x, y, th, sxy, sth):
        self.p = np.column_stack([x + sxy * self.rng.standard_normal(N_PART),
                                  y + sxy * self.rng.standard_normal(N_PART),
                                  th + sth * self.rng.standard_normal(N_PART)])
        self.w = np.full(N_PART, 1.0 / N_PART)
        self.get_logger().info(f'초기화 ({x:.2f}, {y:.2f}, {math.degrees(th):.1f}°)')

    def on_init(self, m):
        p = m.pose.pose
        self.reset(p.position.x, p.position.y, yaw_of(p.orientation), 0.15, math.radians(8))
        self.last_odom = None

    # ---------- 필터 ----------
    def motion(self, d):
        dx, dy, dth = d
        n = N_PART
        tr = math.hypot(dx, dy)
        sx = A_TRANS * tr + A_FLOOR[0]
        st = A_ROT * abs(dth) + 0.3 * A_TRANS * tr + A_FLOOR[2]
        ndx = dx + sx * self.rng.standard_normal(n)
        ndy = dy + (A_TRANS * tr * 0.5 + A_FLOOR[1]) * self.rng.standard_normal(n)
        ndth = dth + st * self.rng.standard_normal(n)
        c, s = np.cos(self.p[:, 2]), np.sin(self.p[:, 2])
        self.p[:, 0] += c * ndx - s * ndy
        self.p[:, 1] += s * ndx + c * ndy
        self.p[:, 2] = wrap(self.p[:, 2] + ndth)

    def measure(self, pts):
        """pts: 빔 끝점(base 좌표) B×2 → 파티클별 로그 우도."""
        c, s = np.cos(self.p[:, 2])[:, None], np.sin(self.p[:, 2])[:, None]
        X = self.p[:, 0:1] + c * pts[:, 0] - s * pts[:, 1]
        Y = self.p[:, 1:2] + s * pts[:, 0] + c * pts[:, 1]
        ix = ((X - self.ox) / self.res).astype(np.int32)
        iy = ((Y - self.oy) / self.res).astype(np.int32)
        inside = (ix >= 0) & (ix < self.W) & (iy >= 0) & (iy < self.H)
        d = np.full(X.shape, D_MAX, np.float32)
        d[inside] = self.dist[iy[inside], ix[inside]]
        lik = (1 - Z_RAND) * np.exp(-0.5 * (d / SIGMA) ** 2) + Z_RAND
        return np.log(lik).sum(1), (d < 2 * SIGMA).mean(1)

    def resample(self):
        n = N_PART
        pos = (self.rng.random() + np.arange(n)) / n
        idx = np.searchsorted(np.cumsum(self.w), pos)
        self.p = self.p[np.minimum(idx, n - 1)]
        self.w = np.full(n, 1.0 / n)

    def estimate(self):
        x, y = np.average(self.p[:, 0], weights=self.w), np.average(self.p[:, 1], weights=self.w)
        th = math.atan2(np.average(np.sin(self.p[:, 2]), weights=self.w), np.average(np.cos(self.p[:, 2]), weights=self.w))
        return np.array([x, y, th])

    # ---------- 스캔 처리 ----------
    def process(self):
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
                t = self.buf.lookup_transform(self.fo, self.fb, st)
                self.step(m, (t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)))
            elif (self.get_clock().now() - st).nanoseconds < 5e8:
                keep.append(m)
        self.pending = keep[-3:]

    def step(self, m, od):
        t0 = time.perf_counter()
        od = np.array(od)
        if self.last_odom is None:
            self.last_odom = od
        lo = self.last_odom
        c, s = math.cos(lo[2]), math.sin(lo[2])
        dxw, dyw = od[0] - lo[0], od[1] - lo[1]
        d = (c * dxw + s * dyw, -s * dxw + c * dyw, math.atan2(math.sin(od[2] - lo[2]), math.cos(od[2] - lo[2])))
        if math.hypot(d[0], d[1]) > UPD_DIST or abs(d[2]) > UPD_ANG or self.nupd == 0:
            self.motion(d)
            a, r = scan_valid(m, self.prof)
            if r.size >= 10:
                sel = np.linspace(0, r.size - 1, min(N_BEAMS, r.size)).astype(int)
                lx, ly, lt = self.lx
                pts = np.stack([lx + r[sel] * np.cos(a[sel] + lt), ly + r[sel] * np.sin(a[sel] + lt)], 1)
                ll, hit = self.measure(pts)
                lw = np.log(self.w + 1e-300) + ll
                lw -= lw.max()
                self.w = np.exp(lw)
                self.w /= self.w.sum()
                self.score = float((hit * self.w).sum())
                if 1.0 / (self.w ** 2).sum() < N_PART / 2:
                    self.resample()
            self.last_odom = od
            self.nupd += 1
            self.est = self.estimate()
            self.t_proc.append(time.perf_counter() - t0)
            # map→odom = est ∘ odom⁻¹
            e = self.est
            yo = e[2] - od[2]
            cy, sy = math.cos(yo), math.sin(yo)
            self.m2o = (e[0] - (cy * od[0] - sy * od[1]), e[1] - (sy * od[0] + cy * od[1]), yo)
        self.publish(m.header.stamp)

    def publish(self, stamp):
        if self.m2o is None:
            return
        x, y, th = self.m2o
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id, t.child_frame_id = self.fm, self.fo
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.z, t.transform.rotation.w = math.sin(th / 2), math.cos(th / 2)
        self.tfb.sendTransform(t)
        p = PoseWithCovarianceStamped()
        p.header.stamp, p.header.frame_id = stamp, self.fm
        e = self.est
        p.pose.pose.position.x, p.pose.pose.position.y = float(e[0]), float(e[1])
        p.pose.pose.orientation.z, p.pose.pose.orientation.w = math.sin(e[2] / 2), math.cos(e[2] / 2)
        cov = np.cov(self.p[:, :2].T, aweights=self.w)
        p.pose.covariance[0], p.pose.covariance[1], p.pose.covariance[6], p.pose.covariance[7] = cov[0, 0], cov[0, 1], cov[1, 0], cov[1, 1]
        p.pose.covariance[35] = float(np.average(wrap(self.p[:, 2] - e[2]) ** 2, weights=self.w))
        self.pub.publish(p)


def main():
    ap = add_args(argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter))
    ap.add_argument('--map', required=True)
    ap.add_argument('--init', nargs=3, type=float, default=None, metavar=('X', 'Y', 'YAW'))
    ap.add_argument('--sim', action='store_true')
    ap.add_argument('--a-trans', type=float, default=None, help='움직임 잡음 비율(이동)')
    ap.add_argument('--a-rot', type=float, default=None, help='움직임 잡음 비율(회전)')
    a = ap.parse_args()
    global A_TRANS, A_ROT
    A_TRANS = a.a_trans if a.a_trans is not None else A_TRANS
    A_ROT = a.a_rot if a.a_rot is not None else A_ROT
    if a.init is None:
        tr = a.map[:-5] + '_traj.csv'
        a.init = list(np.loadtxt(tr, delimiter=',', skiprows=1, ndmin=2)[-1]) if os.path.exists(tr) else [0, 0, 0]
    rclpy.init()
    n = Localizer(load_profile(a.robot, a.calib), a.map, a.init, a.sim)
    print(f'[localizer] 지도 {a.map}, 초기 {tuple(round(v, 2) for v in a.init)}, 파티클 {N_PART}, 빔 {N_BEAMS}', flush=True)
    t_log = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.01)
            n.process()
            if time.time() - t_log > 2.0:
                t_log = time.time()
                sd = float(np.sqrt(np.average((n.p[:, 0] - n.est[0]) ** 2 + (n.p[:, 1] - n.est[1]) ** 2, weights=n.w)))
                tp = 1000 * np.mean(n.t_proc[-20:]) if n.t_proc else 0
                print(f'  pose=({n.est[0]:.2f},{n.est[1]:.2f},{math.degrees(n.est[2]):.0f}°) 퍼짐 {sd:.3f} m '
                      f'점수 {n.score:.2f} 갱신 {n.nupd} 처리 {tp:.1f} ms', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
