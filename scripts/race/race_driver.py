#!/usr/bin/env python3
"""레이스라인 추종 + 장애물 회피 드라이버 (Nav2 불필요, /cmd_vel 직접 발행)

사용: python3 scripts/race/race_driver.py --map MAP.yaml --line RACELINE.csv [--init X Y YAW] [--go]
  --init : AMCL 초기 pose 자동 발행(map 좌표)
  --go   : 엔터 대기 없이 바로 출발(테스트용)

구조
  로컬 플래너(10Hz): 레이스라인 기준 Frenet lattice(정거장 0.4m x 횡오프셋 0.08m) DP
    비용 = 레이스라인 이탈^2 + 횡변화^2 + 벽/장애물 근접 + 이전 계획과의 차이(좌우 떨림 방지)
    장애물 = /scan 점 중 정적맵 벽에서 떨어진 점(최근 3초 기억)
  제어(40Hz): pure pursuit(후륜축 기준) + 곡률/여유거리 기반 속도 프로파일
"""
import sys
import math
import time
import argparse
import threading
import numpy as np
from scipy import ndimage
from scipy.interpolate import CubicSpline

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
import tf2_ros

sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
from raceline import GridMap, curvature  # noqa: E402

# 차량
WB = 0.24               # 휠베이스
REAR = 0.12             # base_link -> 후륜축 거리
K_MAX = 1.0 / 0.56      # 최대 곡률(조향 0.42rad)
TRACK_RATIO = 0.213 / (2 * 0.24)
# 속도
V_MAX = 0.9
A_LAT = 2.2             # 횡가속 한계(m/s^2)
A_ACC = 2.0
A_DEC = 1.8
V_MIN_CLR = 0.35        # 좁은 틈 통과 속도
# 플래너
DS_ST = 0.4             # 정거장 간격
N_ST = 18               # 정거장 수(7.2m)
OFFS = np.round(np.arange(-1.6, 1.6001, 0.08), 3)
DO_MAX = 0.36           # 정거장당 최대 횡변화
C_HARD = 0.165          # 경로점-장애물 최소거리(차폭/2=0.11 + 여유)
C_SOFT = 0.32
W_DEV = 0.6
W_DO = 3.0
W_CLR = 40.0
W_HYST = 1.5
W_HEAD = 1.0
# 장애물 인식
OBS_RANGE = 5.5
WALL_TOL = 0.22         # 정적맵 벽에서 이 이상 떨어진 스캔점 = 장애물
OBS_MEM = 3.0
LOC_HALF = 6.0          # 로컬 장애물 격자 반경(m)
DEBUG = bool(__import__('os').environ.get('RACE_DEBUG'))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def speed_profile(kap, ds, vlim, v0, v_end=None):
    v = np.minimum(vlim, np.sqrt(A_LAT / np.maximum(np.abs(kap), 1e-3)))
    # 좌회전 시 adapter가 우후륜 속도로 클램프 -> 중심속도 상한
    left = kap > 0
    v[left] = np.minimum(v[left], V_MAX / (1 + TRACK_RATIO * WB * kap[left]))
    if v_end is not None:
        v[-1] = min(v[-1], v_end)
    for i in range(len(v) - 2, -1, -1):
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * A_DEC * ds))
    v[0] = min(v[0], max(v0, 0.0) + 0.5)
    for i in range(1, len(v)):
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2 * A_ACC * ds))
    return v


class Race(Node):
    def __init__(self, a):
        super().__init__('race_driver', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.a = a
        self.m = GridMap(a.map)
        R = np.loadtxt(a.line, delimiter=',', skiprows=1)
        Q = np.vstack([R, R[:1]])
        s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(Q, axis=0).T))])
        self.L = s[-1]
        self.cs = CubicSpline(s, Q, bc_type='periodic')
        self.ds = 0.05
        self.s = np.arange(0, self.L, self.ds)
        self.R = self.cs(self.s)
        d1 = self.cs(self.s, 1)
        self.T = d1 / np.hypot(d1[:, 0], d1[:, 1])[:, None]
        self.N = np.stack([-self.T[:, 1], self.T[:, 0]], 1)
        self.kap = curvature(self.R)
        self.n = len(self.s)
        self.get_logger().info(f'레이스라인 {self.L:.2f}m, 최대곡률 {np.abs(self.kap).max():.2f}')

        self.tfb = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfb, self)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.pub_path = self.create_publisher(Path, '/race/local_path', 1)
        qtl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pub_line = self.create_publisher(Path, '/race/raceline', qtl)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.pub_line.publish(self.to_path(self.R[::4]))

        self.v_meas = 0.0
        self.obs = []              # (t, Nx2 points in map)
        self.idx = None            # 레이스라인 진행 인덱스
        self.plan = None           # dict(P, v, s_abs, o)
        self.prev_o = None         # (s_abs array, o array)
        self.go = a.go
        self.t_go = None
        self.lap_t0 = None
        self.laps = 0
        self.last_idx_for_lap = None
        self.stuck_t = None
        self.recover_until = 0.0
        self.recover_steer = 0.0
        self.stat = {}
        self.t_log = 0.0
        self.create_timer(0.1, self.plan_step)
        self.create_timer(0.025, self.control_step)

    # ---------- 유틸 ----------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def to_path(self, P):
        msg = Path()
        msg.header.frame_id = 'map'
        for x, y in P:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        return msg

    def pose(self):
        try:
            t = self.tfb.lookup_transform('map', 'base_link', Time())
        except Exception:
            return None
        tr = t.transform
        return tr.translation.x, tr.translation.y, yaw_of(tr.rotation)

    def send_initialpose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        c = [0.0] * 36
        c[0] = c[7] = 0.02 ** 2
        c[35] = 0.03 ** 2
        msg.pose.covariance = c
        self.pub_init.publish(msg)

    # ---------- 센서 ----------
    def on_odom(self, m):
        self.v_meas = m.twist.twist.linear.x

    def on_scan(self, sc):
        try:
            t = self.tfb.lookup_transform('map', sc.header.frame_id, Time.from_msg(sc.header.stamp),
                                          timeout=Duration(seconds=0.05))
        except Exception:
            try:
                t = self.tfb.lookup_transform('map', sc.header.frame_id, Time())
            except Exception:
                return
        tr = t.transform
        lx, ly, lyaw = tr.translation.x, tr.translation.y, yaw_of(tr.rotation)
        r = np.asarray(sc.ranges, dtype=float)
        ang = sc.angle_min + np.arange(len(r)) * sc.angle_increment
        ok = np.isfinite(r) & (r > max(sc.range_min, 0.12)) & (r < OBS_RANGE)
        r, ang = r[ok], ang[ok]
        P = np.stack([lx + r * np.cos(ang + lyaw), ly + r * np.sin(ang + lyaw)], 1)
        if len(P) == 0:
            return
        c = self.m.clear(P)
        P = P[c > WALL_TOL]
        if len(P) >= 3:
            # 고립 잡음점 제거: 주변 0.15m 내 이웃 2개 이상
            D = np.hypot(P[:, None, 0] - P[None, :, 0], P[:, None, 1] - P[None, :, 1])
            P = P[(D < 0.15).sum(1) >= 3]
        else:
            P = P[:0]
        tnow = self.now()
        if DEBUG and int(tnow) % 3 == 0 and len(P):
            np.save(f'.sim_tmp/obsd_{int(tnow)}.npy', np.vstack([[lx, ly], P]))
        self.obs.append((tnow, P))
        self.obs = [o for o in self.obs if tnow - o[0] < OBS_MEM]

    # ---------- 레이스라인 투영 ----------
    def project(self, x, y):
        if self.idx is None:
            cand = np.arange(self.n)
        else:
            cand = (self.idx + np.arange(-40, 120)) % self.n
        d = np.hypot(self.R[cand, 0] - x, self.R[cand, 1] - y)
        i = int(cand[np.argmin(d)])
        dx, dy = x - self.R[i, 0], y - self.R[i, 1]
        o = dx * self.N[i, 0] + dy * self.N[i, 1]
        return i, o

    # ---------- 로컬 장애물 clearance ----------
    def build_dyn(self, x, y):
        pts = [p for _, p in self.obs if len(p)]
        res = 0.05
        n = int(2 * LOC_HALF / res)
        self.dyn_origin = (x - LOC_HALF, y - LOC_HALF)
        if not pts:
            self.dyn = None
            return 0
        P = np.vstack(pts)
        ix = ((P[:, 0] - self.dyn_origin[0]) / res).astype(int)
        iy = ((P[:, 1] - self.dyn_origin[1]) / res).astype(int)
        k = (ix >= 0) & (ix < n) & (iy >= 0) & (iy < n)
        if not k.any():
            self.dyn = None
            return 0
        g = np.ones((n, n), bool)
        g[iy[k], ix[k]] = False
        self.dyn = ndimage.distance_transform_edt(g) * res
        return int(k.sum())

    def clear(self, P):
        c = self.m.clear(P)
        if self.dyn is not None:
            res = 0.05
            ox, oy = self.dyn_origin
            d = ndimage.map_coordinates(self.dyn, [(P[:, 1] - oy) / res, (P[:, 0] - ox) / res],
                                        order=1, mode='constant', cval=LOC_HALF)
            c = np.minimum(c, d)
        return c

    # ---------- 플래너 ----------
    def plan_step(self):
        t0 = time.perf_counter()
        p = self.pose()
        if p is None:
            return
        x, y, yaw = p
        i0, o0 = self.project(x, y)
        self.idx = i0
        nobs = self.build_dyn(x, y)
        # 차량 앞쪽 기준 시작(후륜축-레이스라인 오프셋 대신 중심 사용)
        th_ref = math.atan2(self.T[i0, 1], self.T[i0, 0])
        he = math.atan2(math.sin(yaw - th_ref), math.cos(yaw - th_ref))
        step = int(round(DS_ST / self.ds))
        ist = (i0 + step * np.arange(1, N_ST + 1)) % self.n
        J = len(OFFS)
        Rk = self.R[ist]
        Nk = self.N[ist]
        nodes = Rk[:, None, :] + Nk[:, None, :] * OFFS[None, :, None]       # (K,J,2)
        # 이전 계획 오프셋 (hysteresis)
        if self.prev_o is not None:
            ps, po = self.prev_o
            rel = ((ist - ps[0]) % self.n)
            prev_at = np.interp(rel, (ps - ps[0]) % self.n, po, right=np.nan)
        else:
            prev_at = np.full(N_ST, np.nan)
        INF = 1e9
        cost = np.full(J, INF)
        back = np.zeros((N_ST, J), int)
        # 첫 구간: 시작점(현재 위치) -> 정거장1
        start = np.array([x, y])
        lam = np.linspace(0.25, 1.0, 4)
        segP = start[None, None, :] * (1 - lam)[None, :, None] + nodes[0][:, None, :] * lam[None, :, None]
        c = self.clear(segP.reshape(-1, 2)).reshape(J, -1)
        cmin0 = c.min(1)
        do = OFFS - o0
        slope_err = np.arctan2(do, DS_ST) - he
        cost = (W_DEV * OFFS ** 2 + W_DO * do ** 2 + W_HEAD * slope_err ** 2
                + W_CLR * (np.maximum(0, C_SOFT - c) ** 2).mean(1))
        cost[cmin0 < C_HARD] = INF
        if not np.isnan(prev_at[0]):
            cost += W_HYST * (OFFS - prev_at[0]) ** 2
        node_ok = [cmin0 >= C_HARD]
        reach = [cost < INF / 2]
        costs = [cost]
        lam3 = np.linspace(0.0, 1.0, 5)[1:]
        dO = OFFS[None, :] - OFFS[:, None]                          # (from, to)
        feas_do = np.abs(dO) <= DO_MAX + 1e-9
        for k in range(1, N_ST):
            A = nodes[k - 1]
            B = nodes[k]
            seg = A[:, None, None, :] * (1 - lam3)[None, None, :, None] + B[None, :, None, :] * lam3[None, None, :, None]
            cseg = self.clear(seg.reshape(-1, 2)).reshape(J, J, -1)
            ok = (cseg.min(2) >= C_HARD) & feas_do
            ec = (W_DO * dO ** 2 + W_CLR * (np.maximum(0, C_SOFT - cseg) ** 2).mean(2))
            tot = costs[-1][:, None] + ec + (W_DEV * OFFS ** 2)[None, :]
            if not np.isnan(prev_at[k]):
                tot = tot + (W_HYST * (OFFS - prev_at[k]) ** 2)[None, :]
            tot[~ok] = INF
            back[k] = np.argmin(tot, 0)
            cost = tot[back[k], np.arange(J)]
            costs.append(cost)
        # 가장 멀리 도달 가능한 정거장에서 최소비용 노드
        kk = N_ST - 1
        while kk >= 0 and costs[kk].min() >= INF / 2:
            kk -= 1
        blocked = kk < N_ST - 1
        if kk < 0:
            self.plan = None
            self.stat.update(blocked=True, kk=-1, nobs=nobs, pt=(time.perf_counter() - t0) * 1000)
            return
        j = int(np.argmin(costs[kk]))
        seq = [j]
        for k in range(kk, 0, -1):
            j = back[k][j]
            seq.append(j)
        seq = seq[::-1]
        o_seq = OFFS[seq]
        Pn = nodes[np.arange(kk + 1), seq]
        # 매끄럽게: 현재 위치부터 스플라인
        pts = np.vstack([start, Pn])
        P = self.smooth_path(pts)
        cP = self.clear(P)
        if cP.min() < C_HARD - 0.02:
            P = self.densify(pts)
            cP = self.clear(P)
        dsp = np.hypot(*np.diff(P, axis=0).T)
        dsm = float(np.mean(dsp)) if len(dsp) else 0.05
        kap = curvature_open(P)
        vlim = np.full(len(P), V_MAX)
        tight = np.clip((cP - C_HARD) / (0.30 - C_HARD), 0, 1)
        vlim = np.minimum(vlim, V_MIN_CLR + (V_MAX - V_MIN_CLR) * tight)
        v = speed_profile(kap, dsm, vlim, abs(self.v_meas), v_end=0.0 if blocked else None)
        self.plan = dict(P=P, v=v, t=self.now())
        self.prev_o = (ist[:kk + 1], o_seq)
        self.pub_path.publish(self.to_path(P))
        self.stat.update(blocked=blocked, kk=kk, nobs=nobs, o0=o0, cmin=float(cP.min()),
                         pt=(time.perf_counter() - t0) * 1000)

    def smooth_path(self, pts):
        d = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])
        keep = np.concatenate([[True], np.diff(d) > 1e-4])
        pts, d = pts[keep], d[keep]
        if len(pts) < 3:
            return self.densify(pts)
        cs = CubicSpline(d, pts, bc_type='natural')
        return cs(np.arange(0, d[-1], 0.05))

    @staticmethod
    def densify(pts):
        out = [pts[0]]
        for a, b in zip(pts[:-1], pts[1:]):
            L = math.hypot(*(b - a))
            n = max(1, int(L / 0.05))
            for t in np.linspace(0, 1, n + 1)[1:]:
                out.append(a + (b - a) * t)
        return np.array(out)

    # ---------- 제어 ----------
    def control_step(self):
        tnow = self.now()
        p = self.pose()
        cmd = Twist()
        if p is None or not self.go:
            self.pub.publish(cmd)
            return
        if self.t_go is None:
            self.t_go = tnow
            self.lap_t0 = tnow
            self.get_logger().info('출발!')
        x, y, yaw = p
        self.lap_check(tnow)
        if tnow < self.recover_until:
            cmd.linear.x = -0.3
            cmd.angular.z = -0.3 * self.recover_steer
            self.pub.publish(cmd)
            return
        pl = self.plan
        if pl is None or len(pl['P']) < 3:
            self.pub.publish(cmd)
            self.check_stuck(tnow, 0.5)
            return
        P, v = pl['P'], pl['v']
        rx, ry = x - REAR * math.cos(yaw), y - REAR * math.sin(yaw)
        d = np.hypot(P[:, 0] - rx, P[:, 1] - ry)
        ic = int(np.argmin(d))
        vref = float(v[min(ic + 2, len(v) - 1)])
        ld = float(np.clip(0.35 + 0.45 * max(abs(self.v_meas), vref * 0.8), 0.45, 0.85))
        # 후륜축에서 ld 만큼 떨어진 경로점
        far = np.where((d >= ld) & (np.arange(len(P)) > ic))[0]
        it = int(far[0]) if len(far) else len(P) - 1
        tx, ty = P[it]
        dx, dy = tx - rx, ty - ry
        al = math.atan2(dy, dx) - yaw
        Ld = max(math.hypot(dx, dy), 0.2)
        kap = 2 * math.sin(al) / Ld
        kap = max(-K_MAX, min(K_MAX, kap))
        vcmd = min(vref, math.sqrt(A_LAT / max(abs(kap), 1e-3)))
        if abs(math.atan2(math.sin(al), math.cos(al))) > 1.4:
            vcmd = min(vcmd, 0.3)
        cmd.linear.x = vcmd
        cmd.angular.z = vcmd * kap
        self.pub.publish(cmd)
        self.check_stuck(tnow, vcmd, kap)
        if tnow - self.t_log > 0.5:
            self.t_log = tnow
            st = self.stat
            print(f'[{tnow - self.t_go:6.1f}s] lap{self.laps + 1} s={self.s[self.idx] if self.idx is not None else 0:5.1f} '
                  f'v={self.v_meas:4.2f}/{vcmd:4.2f} o={st.get("o0", 0):+.2f} clr={st.get("cmin", 0):.2f} '
                  f'obs={st.get("nobs", 0)} {"BLOCK" if st.get("blocked") else ""} k={st.get("kk")} '
                  f'plan={st.get("pt", 0):.0f}ms', flush=True)

    def check_stuck(self, tnow, vcmd, kap=0.0):
        if vcmd > 0.15 and abs(self.v_meas) < 0.04:
            if self.stuck_t is None:
                self.stuck_t = tnow
            elif tnow - self.stuck_t > 1.2:
                self.get_logger().warn('정지 감지 -> 짧은 후진')
                self.recover_until = tnow + 0.9
                self.recover_steer = math.copysign(1.0, kap) if abs(kap) > 0.1 else 0.0
                self.stuck_t = None
        else:
            self.stuck_t = None

    def lap_check(self, tnow):
        if self.idx is None:
            return
        if self.last_idx_for_lap is not None:
            if self.last_idx_for_lap > self.n * 0.9 and self.idx < self.n * 0.1:
                self.laps += 1
                lt = tnow - self.lap_t0
                self.lap_t0 = tnow
                print(f'=== LAP {self.laps}: {lt:.2f}s ===', flush=True)
        self.last_idx_for_lap = self.idx


def curvature_open(P):
    if len(P) < 3:
        return np.zeros(len(P))
    k = np.zeros(len(P))
    a, b, c = P[:-2], P[1:-1], P[2:]
    d1 = (c - a) / 2
    d2 = c - 2 * b + a
    num = d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]
    k[1:-1] = num / np.maximum(np.hypot(d1[:, 0], d1[:, 1]) ** 3, 1e-9)
    k[0], k[-1] = k[1], k[-2]
    return ndimage.uniform_filter1d(k, 5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True)
    ap.add_argument('--line', required=True)
    ap.add_argument('--init', nargs=3, type=float)
    ap.add_argument('--go', action='store_true')
    a = ap.parse_args()
    rclpy.init()
    node = Race(a)
    if a.init:
        t_end = time.time() + 1.5
        while time.time() < t_end:
            node.send_initialpose(*a.init)
            rclpy.spin_once(node, timeout_sec=0.3)
    # TF/스캔/첫 계획 준비
    t0 = time.time()
    while rclpy.ok() and (node.plan is None) and time.time() - t0 < 30:
        rclpy.spin_once(node, timeout_sec=0.05)
    print('READY' + ('' if a.go else ' - 엔터를 누르면 즉시 출발'), flush=True)
    if not a.go:
        def wait_enter():
            try:
                input()
            except EOFError:
                return
            node.go = True
        threading.Thread(target=wait_enter, daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
