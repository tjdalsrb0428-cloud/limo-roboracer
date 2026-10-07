#!/usr/bin/env python3
"""레이스라인 추종 + 장애물 회피 드라이버 (Nav2 불필요, /cmd_vel 직접 발행)

사용: python3 scripts/race/race_driver.py [--map MAP.yaml] [--line RACELINE.csv] [--init X Y YAW] [--go]
  인자 생략 시 maps/exam/map.yaml, map_raceline.csv, map_traj.csv (odom_mapper/raceline 기본 출력)
  --init : AMCL 모드(초기 pose 자동 발행). 없으면 odom 모드(map=odom)
  Ctrl+C : 정지 명령(속도 0) 발행 후 종료
  --go   : 엔터 대기 없이 바로 출발(테스트용)

구조
  로컬 플래너(10Hz): 레이스라인 기준 Frenet lattice(정거장 0.4m x 횡오프셋 0.08m) DP
    비용 = 레이스라인 이탈^2 + 횡변화^2 + 벽/장애물 근접 + 이전 계획과의 차이(좌우 떨림 방지)
    장애물 = /scan 점 중 정적맵 벽에서 떨어진 점(최근 3초 기억)
  제어(40Hz): pure pursuit(후륜축 기준) + 곡률/여유거리 기반 속도 프로파일
"""
import os
import sys
import math
import time
import argparse
import threading
import numpy as np
from scipy import ndimage
from scipy.interpolate import CubicSpline
from scipy.spatial import cKDTree

os.environ['ROS_LOCALHOST_ONLY'] = '1'     # 오프라인(Wi-Fi 없음)에서도 통신
import rclpy  # noqa: E402
from rclpy.signals import SignalHandlerOptions  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from std_msgs.msg import Empty
from sensor_msgs.msg import LaserScan
import tf2_ros

sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
from raceline import GridMap, curvature  # noqa: E402
import hybrid_astar  # noqa: E402

# 차량
WB = 0.24               # 휠베이스
REAR = 0.12             # base_link -> 후륜축 거리
K_MAX = 1.0 / 0.56      # 최대 곡률(조향 0.42rad)
TRACK_RATIO = 0.213 / (2 * 0.24)
# 속도
V_MAX = 1.0
A_LAT = 2.2             # 횡가속 한계(m/s^2)
A_ACC = 2.0
A_DEC = 1.8
V_MIN_CLR = 0.35        # 좁은 틈 통과 속도
# 플래너
DS_ST = 0.4             # 정거장 간격
N_ST = 18               # 정거장 수(7.2m)
OFFS = np.round(np.arange(-1.6, 1.6001, 0.08), 3)
DO_MAX = 0.28           # 정거장당 최대 횡변화
N_RIDGE = 6             # 정거장별 추가 노드(clearance 능선=틈 중앙) 수
# 차체(바퀴 포함 0.33x0.26)를 진행방향 5개 원(반경 0.1325)으로 근사
BODY_OFF = (-0.12, -0.06, 0.0, 0.06, 0.12)
C_HARD = 0.165          # 원 중심-장애물 최소거리(반경 0.1325 + 여유 3cm)
C_TIGHT = 0.142         # 막혔을 때만: 0.35m 틈 통과용 빠듯한 기준(여유 1cm)
V_TIGHT = 0.2           # 빠듯한 기준 경로 속도 상한
TIGHT_PAD = 1.6         # 빠듯한 구간 앞뒤 저속 여유(m)
V_BLOCKED = 0.5         # 앞이 막혀 보일 때 속도 상한
NO_SLOW = True          # 기본: 여유 기반 감속·TIGHT 저속 끔(곡률 제한·막힘 정지는 유지). --slow 로 켬
C_START = 0.12          # 현재 위치 근처(첫 구간 앞부분)는 완화: 이미 가까워도 빠져나갈 경로 허용
DYN_RES = 0.025
STATIC_BIAS = 0.03      # 점유격자 셀중심 거리 -> 실제 벽면 거리 보정
RAW_MEM = 1.0           # 빠듯한 기준용 라이다 원점(벽 포함) 기억 시간(s)
RAW_RES = 0.01          # 라이다 실측 거리표 해상도(m)
RAW_HALF = 3.5          # 거리표 창 반폭(m): 계획 범위(7.2m) 중 빠듯한 구간 판정에 충분
RAW_UP = 0.02           # 빠듯한 기준: 벽 여유 = 라이다 실측거리, 단 [지도-0.03, 지도+0.02] 범위로 제한
C_SOFT = 0.30           # 벽 여유(레이스라인 자체가 0.30)
C_SOFT_OBS = 0.45       # 장애물 여유
W_OBS = 150.0
SHADOW = ()               # 그림자 확장 비활성(비스듬한 면의 그림자가 틈을 막음). 측면은 이동하며 누적 관측
W_DEV = 0.6
W_DO = 3.0
W_CLR = 40.0
W_HYST = 1.5
W_HEAD = 1.0
W_DO_TIGHT = 80.0
# 장애물 인식
OBS_RANGE = 5.5
WALL_TOL = 0.22         # 정적맵 벽에서 이 이상 떨어진 스캔점 = 장애물
OBS_MEM = 3.0
LOC_HALF = 6.0          # 로컬 장애물 격자 반경(m)
USE_BLOB_PTS = False    # 지도 박스 셀을 장애물 점으로 추가(래스터 오차로 틈이 좁아져 기본 비활성, 회피는 라이다)
M2O_TAU = 2.0           # map->odom 평활 시정수(s)
STOP_MARGIN = 0.35      # 막혔을 때 정지 여유(m)
REV_T = 0.9             # 복구 후진 시간(s)
MAN_TIME = 3.0          # Hybrid A* 탐색 시간 한도(s)
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
        # 지도의 독립 장애물(박스)은 벽이 아니라 '장애물 점'으로 취급: 정밀거리/장애물 여유 적용
        self.m = GridMap(a.map, np.loadtxt(a.traj, delimiter=',', skiprows=1)[:, :2] if a.traj else None,
                         drop_blobs=True, unknown_free=True)
        bm = self.m.blobs & ~ndimage.binary_erosion(self.m.blobs)
        rr, cc = np.nonzero(bm)
        self.blob_pts = self.m.xy(rr, cc) if len(rr) else np.zeros((0, 2))
        if len(self.blob_pts):
            self.get_logger().info(f'지도 장애물 셀 {len(self.blob_pts)}개(장애물로 처리)')
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
        self.tfl = tf2_ros.TransformListener(self.tfb, self, spin_thread=True)   # TF 는 전용 스레드
        if a.odom:
            # odom 모드: map = odom 항등 static TF (RViz 의 map 고정 프레임에서 경로/차량 화살표 표시용)
            self.stf = tf2_ros.StaticTransformBroadcaster(self)
            st = TransformStamped()
            st.header.frame_id = 'map'
            st.child_frame_id = 'odom'
            st.transform.rotation.w = 1.0
            self.stf.sendTransform(st)
        cg_ctrl = MutuallyExclusiveCallbackGroup()
        cg_sens = MutuallyExclusiveCallbackGroup()
        cg_plan = MutuallyExclusiveCallbackGroup()
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.pub_path = self.create_publisher(Path, '/race/local_path', 1)
        qtl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pub_line = self.create_publisher(Path, '/race/raceline', qtl)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data, callback_group=cg_sens)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10, callback_group=cg_ctrl)
        # 출발 신호(/race/go): python3 scripts/race/go.py 또는 ros2 topic pub --once /race/go std_msgs/msg/Empty
        self.create_subscription(Empty, '/race/go', self.on_go, 10, callback_group=cg_ctrl)
        self.pub_line.publish(self.to_path(self.R[::4]))
        # 불러온 지도를 /map 으로 발행(저장된 지도로 시작할 때 RViz 에 지도 표시. 매퍼가 없을 때만 의미)
        self.pub_map = self.create_publisher(OccupancyGrid, '/map', qtl)
        self.pub_map.publish(self.map_msg(a.map))

        self.v_meas = 0.0
        self.m2o = None
        self.t_m2o = 0.0
        self.man = None            # Hybrid A* 탈출 기동 상태
        self.blocked_t = None
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
        self.raw = []              # (t, Nx2) 벽 포함 라이다 점
        self.raw_dt = None
        self.tight_static = False
        self.t_log = 0.0
        # 플래너(느릴 수 있음)와 제어(40Hz)를 다른 스레드에서 -> 계획 중에도 조향 갱신 유지
        self.create_timer(0.1, self.plan_step, callback_group=cg_plan)
        self.create_timer(0.025, self.control_step, callback_group=cg_ctrl)

    # ---------- 유틸 ----------
    @staticmethod
    def map_msg(yml):
        import yaml
        from PIL import Image
        y = yaml.safe_load(open(yml))
        p = y['image'] if os.path.isabs(y['image']) else os.path.join(os.path.dirname(os.path.abspath(yml)), y['image'])
        img = np.array(Image.open(p).convert('L'))[::-1]          # 아래쪽 행이 원점
        g = np.full(img.shape, -1, np.int8)
        g[img >= 250] = 0
        g[img <= 50] = 100
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.info.resolution = float(y['resolution'])
        m.info.height, m.info.width = img.shape
        m.info.origin.position.x, m.info.origin.position.y = float(y['origin'][0]), float(y['origin'][1])
        m.info.origin.orientation.w = 1.0
        m.data = g.ravel().tolist()
        return m

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

    # map->odom(AMCL 보정)을 저역통과 필터로 평활: AMCL 순간 떨림 제거, 느린 드리프트는 추종
    def update_m2o(self):
        if self.a.odom:                 # odom 지도 모드: map = odom (AMCL 없음)
            self.m2o = np.zeros(3)
            return
        try:
            t = self.tfb.lookup_transform('map', 'odom', Time())
        except Exception:
            return
        tr = t.transform
        z = np.array([tr.translation.x, tr.translation.y, yaw_of(tr.rotation)])
        tn = self.now()
        if self.m2o is None or np.hypot(*(z[:2] - self.m2o[:2])) > 0.5:
            self.m2o = z
        else:
            tau = M2O_TAU if self.go else 0.3
            a = 1.0 - math.exp(-max(tn - self.t_m2o, 0.0) / tau)
            dth = math.atan2(math.sin(z[2] - self.m2o[2]), math.cos(z[2] - self.m2o[2]))
            self.m2o = self.m2o + a * np.array([z[0] - self.m2o[0], z[1] - self.m2o[1], dth])
        self.t_m2o = tn

    def to_map(self, x, y, th):
        mx, my, mth = self.m2o
        c, s_ = math.cos(mth), math.sin(mth)
        return mx + c * x - s_ * y, my + s_ * x + c * y, th + mth

    def pose(self):
        if self.m2o is None:
            return None
        try:
            t = self.tfb.lookup_transform('odom', 'base_link', Time())
        except Exception:
            return None
        tr = t.transform
        return self.to_map(tr.translation.x, tr.translation.y, yaw_of(tr.rotation))

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
    def on_go(self, _):
        if not self.go:
            self.go = True

    def on_odom(self, m):
        self.v_meas = m.twist.twist.linear.x

    def on_scan(self, sc):
        if self.m2o is None:
            return
        try:
            # 스캔 시점의 정확한 pose 만 사용(없으면 버림: 오래된 pose 로 변환하면 장애물이 번짐)
            t = self.tfb.lookup_transform('odom', sc.header.frame_id, Time.from_msg(sc.header.stamp),
                                          timeout=Duration(seconds=0.05))
        except Exception:
            return
        tr = t.transform
        lx, ly, lyaw = self.to_map(tr.translation.x, tr.translation.y, yaw_of(tr.rotation))
        r = np.asarray(sc.ranges, dtype=float)
        ang = sc.angle_min + np.arange(len(r)) * sc.angle_increment
        ok = np.isfinite(r) & (r > max(sc.range_min, 0.12)) & (r < OBS_RANGE)
        r, ang = r[ok], ang[ok]
        P = np.stack([lx + r * np.cos(ang + lyaw), ly + r * np.sin(ang + lyaw)], 1)
        if len(P) == 0:
            return
        tnow = self.now()
        self.raw.append((tnow, P))
        self.raw = [o for o in self.raw if tnow - o[0] < RAW_MEM]
        c = self.m.clear(P)
        keep = c > WALL_TOL
        P, r, ang = P[keep], r[keep], ang[keep]
        if len(P) >= 3:
            # 고립 잡음점 제거: 주변 0.15m 내 이웃 2개 이상
            D = np.hypot(P[:, None, 0] - P[None, :, 0], P[:, None, 1] - P[None, :, 1])
            k = (D < 0.15).sum(1) >= 3
            P, r, ang = P[k], r[k], ang[k]
            # 그림자 확장: 레이 방향으로 뒤쪽 점 추가(정적맵 free 인 곳만)
            # 클러스터 가장자리 레이(각도 끊김/거리 점프 주변 3개)는 제외 -> 틈을 메우지 않음
            brk = np.concatenate([[True], (np.diff(ang) > 2.5 * sc.angle_increment) | (np.abs(np.diff(r)) > 0.08), [True]])
            bi = np.where(brk)[0]
            inner = np.ones(len(P), bool)
            for b in bi:
                inner[max(b - 3, 0):b + 3] = False
            ext = [P]
            for e in SHADOW:
                Q = np.stack([lx + (r + e) * np.cos(ang + lyaw), ly + (r + e) * np.sin(ang + lyaw)], 1)[inner]
                ext.append(Q[self.m.clear(Q) > 0.05])
            P = np.vstack(ext)
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
    def build_raw(self, x, y):
        """벽 포함 라이다 점 -> 1cm 격자 거리표(창 ±RAW_HALF). KD-tree 반복 질의보다 훨씬 빠름"""
        pts = [p for _, p in self.raw if len(p)]
        self.raw_dt = None
        if not pts:
            return
        P = np.vstack(pts)
        n = int(2 * RAW_HALF / RAW_RES)
        self.raw_origin = (x - RAW_HALF, y - RAW_HALF)
        ix = ((P[:, 0] - self.raw_origin[0]) / RAW_RES).astype(int)
        iy = ((P[:, 1] - self.raw_origin[1]) / RAW_RES).astype(int)
        k = (ix >= 0) & (ix < n) & (iy >= 0) & (iy < n)
        if not k.any():
            return
        g = np.ones((n, n), bool)
        g[iy[k], ix[k]] = False
        self.raw_dt = ndimage.distance_transform_edt(g) * RAW_RES

    def build_dyn(self, x, y):
        pts = [p for _, p in self.obs if len(p)]
        if USE_BLOB_PTS and len(self.blob_pts):
            bp = self.blob_pts
            near = (np.abs(bp[:, 0] - x) < LOC_HALF) & (np.abs(bp[:, 1] - y) < LOC_HALF)
            if near.any():
                pts.append(bp[near])
        res = DYN_RES
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
        # 가까운 거리는 실제 점까지 정확히(KD-tree), 1cm 격자로 중복 제거
        Pk = np.unique(np.round(P[k] / 0.01).astype(np.int64), axis=0) * 0.01
        self.kdt = cKDTree(Pk)
        return int(k.sum())

    def clear2(self, P):
        """(정적맵 clearance, 장애물 clearance)"""
        cm = self.m.clear(P)
        c = cm - STATIC_BIAS
        if self.tight_static and self.raw_dt is not None:
            # 빠듯한 기준: 지도 래스터 오차(박스-벽 게이트에서 ~0.02 좁게 잡힘) 대신 라이다 실측 거리
            nr = cm < 0.30
            if nr.any():
                Q = P[nr]
                ox, oy = self.raw_origin
                dr = ndimage.map_coordinates(self.raw_dt, [(Q[:, 1] - oy) / RAW_RES, (Q[:, 0] - ox) / RAW_RES],
                                             order=1, mode='constant', cval=9.0)
                seen = dr < cm[nr] + 0.05          # 라이다가 그 벽면을 실제로 본 경우만(안 보이면 지도값 유지)
                c[nr] = np.where(seen, np.clip(dr, cm[nr] - STATIC_BIAS, cm[nr] + RAW_UP), cm[nr] - STATIC_BIAS)
        if self.dyn is None:
            return c, np.full(len(P), LOC_HALF)
        res = DYN_RES
        ox, oy = self.dyn_origin
        d = ndimage.map_coordinates(self.dyn, [(P[:, 1] - oy) / res, (P[:, 0] - ox) / res],
                                    order=1, mode='constant', cval=LOC_HALF)
        near = d < 0.3
        if near.any():
            d[near] = self.kdt.query(P[near], k=1, workers=-1)[0]
        return c, d

    def clear(self, P):
        c, d = self.clear2(P)
        return np.minimum(c, d)

    @staticmethod
    def penalty(cs, cd):
        return (W_CLR * np.maximum(0, C_SOFT - cs) ** 2 + W_OBS * np.maximum(0, C_SOFT_OBS - cd) ** 2)

    def body_clear(self, P, H):
        """P(...,2) 위치, H(...) 진행방향 -> 차체 3원 중 최소 clearance"""
        sh = P.shape[:-1]
        P = P.reshape(-1, 2)
        H = np.broadcast_to(H, sh).reshape(-1)
        cs = cd = None
        for d in BODY_OFF:
            Q = P + d * np.stack([np.cos(H), np.sin(H)], 1)
            a, b = self.clear2(Q)
            cs = a if cs is None else np.minimum(cs, a)
            cd = b if cd is None else np.minimum(cd, b)
        return cs.reshape(sh), cd.reshape(sh)

    # ---------- 플래너 ----------
    def lattice(self, x, y, yaw, i0, o0, c_hard):
        """레이스라인 Frenet lattice DP. 반환 (kk, o_seq, Pn, ist) kk=-1 이면 첫 정거장도 불가"""
        th_ref = math.atan2(self.T[i0, 1], self.T[i0, 0])
        he = math.atan2(math.sin(yaw - th_ref), math.cos(yaw - th_ref))
        step = int(round(DS_ST / self.ds))
        ist = (i0 + step * np.arange(1, N_ST + 1)) % self.n
        Rk = self.R[ist]
        Nk = self.N[ist]
        OFFk = self.station_offsets(Rk, Nk, c_hard)                          # (K,J)
        J = OFFk.shape[1]
        nodes = Rk[:, None, :] + Nk[:, None, :] * OFFk[:, :, None]          # (K,J,2)
        if self.prev_o is not None:                                          # 이전 계획(hysteresis)
            ps, po = self.prev_o
            rel = ((ist - ps[0]) % self.n)
            prev_at = np.interp(rel, (ps - ps[0]) % self.n, po, right=np.nan)
        else:
            prev_at = np.full(N_ST, np.nan)
        INF = 1e9
        back = np.zeros((N_ST, J), int)
        # 첫 구간: 현재 위치 -> 정거장1 (출발 직후 구간은 기준 완화: 이미 가까워도 빠져나가는 경로 허용)
        start = np.array([x, y])
        lam = np.linspace(0.125, 1.0, 8)
        segP = start[None, None, :] * (1 - lam)[None, :, None] + nodes[0][:, None, :] * lam[None, :, None]
        hd0 = np.arctan2(nodes[0][:, 1] - y, nodes[0][:, 0] - x)
        cs, cd = self.body_clear(segP, np.broadcast_to(hd0[:, None], segP.shape[:2]))
        cc = np.minimum(cs, cd)
        cmin0 = np.minimum(cc[:, 4:].min(1), cc[:, :4].min(1) + (c_hard - C_START))
        O0 = OFFk[0]
        do = O0 - o0
        slope_err = np.arctan2(do, DS_ST) - he
        cost = (W_DEV * O0 ** 2 + W_DO * do ** 2 + W_HEAD * slope_err ** 2
                + self.penalty(cs, cd).mean(1))
        cost[cmin0 < c_hard] = INF
        if not np.isnan(prev_at[0]):
            cost += W_HYST * (O0 - prev_at[0]) ** 2
        costs = [cost]
        lam3 = np.linspace(0.0, 1.0, 5)[1:]
        for k in range(1, N_ST):
            dO = OFFk[k][None, :] - OFFk[k - 1][:, None]           # (from, to)
            A = nodes[k - 1]
            B = nodes[k]
            seg = A[:, None, None, :] * (1 - lam3)[None, None, :, None] + B[None, :, None, :] * lam3[None, None, :, None]
            hd = np.arctan2(B[None, :, 1] - A[:, None, 1], B[None, :, 0] - A[:, None, 0])
            cs, cd = self.body_clear(seg, np.broadcast_to(hd[:, :, None], seg.shape[:3]))
            ok = (np.minimum(cs, cd).min(2) >= c_hard) & (np.abs(dO) <= DO_MAX + 1e-9)
            # 좁은 곳(차체 여유 < 0.25)에서의 횡이동은 크게 벌점 -> 틈 앞에서 미리 정렬 후 직진 진입
            cme = np.minimum(cs, cd).min(2)
            f_t = np.clip((0.25 - cme) / 0.08, 0, 1)
            tot = (costs[-1][:, None] + (W_DO + W_DO_TIGHT * f_t) * dO ** 2 + self.penalty(cs, cd).mean(2)
                   + (W_DEV * OFFk[k] ** 2)[None, :])
            if not np.isnan(prev_at[k]):
                tot = tot + (W_HYST * (OFFk[k] - prev_at[k]) ** 2)[None, :]
            tot[~ok] = INF
            back[k] = np.argmin(tot, 0)
            costs.append(tot[back[k], np.arange(J)])
        kk = N_ST - 1
        while kk >= 0 and costs[kk].min() >= INF / 2:
            kk -= 1
        if kk < 0:
            return -1, None, None, ist
        j = int(np.argmin(costs[kk]))
        seq = [j]
        for k in range(kk, 0, -1):
            j = back[k][j]
            seq.append(j)
        seq = seq[::-1]
        return kk, OFFk[np.arange(kk + 1), seq], nodes[np.arange(kk + 1), seq], ist

    def plan_step(self):
        t0 = time.perf_counter()
        p = self.pose()
        if p is None or self.man is not None:
            return
        x, y, yaw = p
        i0, o0 = self.project(x, y)
        self.idx = i0
        nobs = self.build_dyn(x, y)
        self.tight_static = False
        c_hard = C_HARD
        kk, o_seq, Pn, ist = self.lattice(x, y, yaw, i0, o0, C_HARD)
        tight_mode = False
        if kk < N_ST - 1:
            # 여유 기준으로 막힘 -> 빠듯한 기준(좁은 틈 통과)으로 재탐색, 성공 시 저속
            self.build_raw(x, y)
            self.tight_static = True
            kk2, o2, P2, ist2 = self.lattice(x, y, yaw, i0, o0, C_TIGHT)
            if kk2 > kk:
                kk, o_seq, Pn, ist = kk2, o2, P2, ist2
                tight_mode = True
                c_hard = C_TIGHT
            else:
                self.tight_static = False
        blocked = kk < N_ST - 1
        if DEBUG and blocked and self.now() - getattr(self, 't_dump', 0) > 2.0:
            self.t_dump = self.now()
            pts = [q for _, q in self.obs if len(q)]
            np.savez(f'.sim_tmp/block_{int(self.now())}.npz', pose=np.array([x, y, yaw]),
                     obs=np.vstack(pts) if pts else np.zeros((0, 2)), kk=kk)
        # 막힌 채 거의 정지 -> Hybrid A* 탈출 시도
        if blocked and kk <= 3 and self.go and abs(self.v_meas) < 0.05:
            if self.blocked_t is None:
                self.blocked_t = self.now()
            elif self.now() - self.blocked_t > 0.6:
                self.blocked_t = None
                if self.try_maneuver(x, y, yaw, i0, max(kk, 0)):
                    return
        else:
            self.blocked_t = None
        if kk < 0:
            self.plan = None
            self.stat.update(blocked=True, kk=-1, nobs=nobs, pt=(time.perf_counter() - t0) * 1000)
            return
        pts = np.vstack([np.array([x, y]), Pn])
        P = self.smooth_path(pts)
        cs, cd = self.path_clear(P)
        if np.minimum(cs, cd).min() < c_hard - 0.02:
            P = self.densify(pts)
            cs, cd = self.path_clear(P)
        cP = np.minimum(cs, cd)
        dsp = np.hypot(*np.diff(P, axis=0).T)
        dsm = float(np.mean(dsp)) if len(dsp) else 0.05
        kap = curvature_open(P)
        vlim = np.full(len(P), V_MAX)
        tight = np.minimum(np.clip((cs - c_hard) / (0.28 - c_hard), 0, 1),
                           np.clip((cd - c_hard) / (0.40 - c_hard), 0, 1))
        if not NO_SLOW:
            vlim = np.minimum(vlim, V_MIN_CLR + (V_MAX - V_MIN_CLR) * tight)
        if tight_mode:
            # 빠듯한 구간(여유 < C_HARD)과 그 앞뒤 TIGHT_PAD 만 저속, 나머지는 정상 속도
            narrow = cP < C_HARD
            if narrow.any() and not NO_SLOW:
                pad = max(1, int(TIGHT_PAD / dsm))
                narrow = ndimage.binary_dilation(narrow, iterations=pad)
                vlim[narrow] = np.minimum(vlim[narrow], V_TIGHT)
        elif blocked:
            vlim = np.minimum(vlim, V_BLOCKED)   # 앞이 막힘: 미리 감속해 재계획 여유 확보
        if blocked:
            # 막힘: 마지막 가능 지점보다 STOP_MARGIN 앞에서 정지
            sacc = np.concatenate([[0], np.cumsum(dsp)])
            vlim[sacc > sacc[-1] - STOP_MARGIN] = 0.0
        v = speed_profile(kap, dsm, vlim, abs(self.v_meas), v_end=0.0 if blocked else None)
        self.plan = dict(P=P, v=v, tight=tight, t=self.now())
        self.prev_o = (ist[:kk + 1], o_seq)
        self.pub_path.publish(self.to_path(P))
        self.stat.update(blocked=blocked, kk=kk, nobs=nobs, o0=o0, cmin=float(cP.min()), tight=tight_mode,
                         pt=(time.perf_counter() - t0) * 1000)

    def path_clear(self, P):
        if len(P) < 2:
            return self.clear2(P)
        d = np.gradient(P, axis=0)
        return self.body_clear(P, np.arctan2(d[:, 1], d[:, 0]))

    def station_offsets(self, Rk, Nk, c_hard):
        """정거장별 횡오프셋 후보 = 균일격자 + clearance 능선(틈 중앙) 최대 N_RIDGE개"""
        K = len(Rk)
        fine = np.arange(OFFS[0], OFFS[-1] + 1e-9, 0.01)
        Q = Rk[:, None, :] + Nk[:, None, :] * fine[None, :, None]
        c = self.clear(Q.reshape(-1, 2)).reshape(K, len(fine))
        peak = (c[:, 1:-1] >= c[:, :-2]) & (c[:, 1:-1] > c[:, 2:]) & (c[:, 1:-1] >= c_hard) & (c[:, 1:-1] < 0.6)
        out = np.full((K, len(OFFS) + N_RIDGE), 99.0)
        out[:, :len(OFFS)] = OFFS
        for k in range(K):
            idx = np.where(peak[k])[0] + 1
            if len(idx):
                idx = idx[np.argsort(np.abs(fine[idx]))][:N_RIDGE]
                out[k, len(OFFS):len(OFFS) + len(idx)] = fine[idx]
        return out

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
        self.update_m2o()
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
            # 후진하며 차 앞머리를 가려던 방향으로 돌림 (yaw rate 부호 = 목표 회전 방향)
            cmd.linear.x = -0.3
            cmd.angular.z = 0.3 * 0.8 * K_MAX * self.recover_steer
            self.pub.publish(cmd)
            return
        if self.man is not None:
            self.follow_maneuver(tnow, x, y, yaw)
            return
        pl = self.plan
        if pl is None or len(pl['P']) < 3:
            self.pub.publish(cmd)
            return
        P, v = pl['P'], pl['v']
        rx, ry = x - REAR * math.cos(yaw), y - REAR * math.sin(yaw)
        d = np.hypot(P[:, 0] - rx, P[:, 1] - ry)
        ic = int(np.argmin(d))
        vref = float(v[min(ic + 2, len(v) - 1)])
        ld = float(np.clip(0.25 + 0.55 * max(abs(self.v_meas), vref * 0.8), 0.35, 0.85))
        # 좁은 곳에서는 lookahead 축소(코너 깎기 감소)
        tmin = float(pl['tight'][ic:ic + 25].min()) if ic < len(P) - 1 else 1.0
        ld = max(0.22 if self.stat.get('tight') else 0.28, ld * (0.6 + 0.4 * tmin))
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
                  f'obs={st.get("nobs", 0)} {"BLOCK" if st.get("blocked") else ""}{" TIGHT" if st.get("tight") else ""} k={st.get("kk")} '
                  f'plan={st.get("pt", 0):.0f}ms', flush=True)

    def check_stuck(self, tnow, vcmd, kap=0.0):
        if vcmd > 0.15 and abs(self.v_meas) < 0.04:
            if self.stuck_t is None:
                self.stuck_t = tnow
            elif tnow - self.stuck_t > 1.2:
                self.stuck_t = None
                p = self.pose()
                if p is not None and self.idx is not None and self.try_maneuver(*p, self.idx, 0):
                    return
                pref = math.copysign(1.0, kap) if abs(kap) > 0.1 else 0.0
                st = self.pick_reverse(pref)
                if st is None:
                    self.get_logger().warn('정지 감지 -> 후진 공간 없음, 대기')
                    return
                self.get_logger().warn(f'정지 감지 -> 짧은 후진(조향 {st:+.0f})')
                self.recover_until = tnow + REV_T
                self.recover_steer = st
        else:
            self.stuck_t = None

    # ---------- Hybrid A* 탈출 ----------
    def try_maneuver(self, x, y, yaw, i0, kk):
        self.pub.publish(Twist())
        t0 = time.time()
        step = int(round(DS_ST / self.ds))
        d0 = max(1.6, (kk + 2) * DS_ST + 0.6)
        goal = None
        for dist in np.arange(d0, 6.0, 0.25):
            j = (i0 + int(dist / self.ds)) % self.n
            th = math.atan2(self.T[j, 1], self.T[j, 0])
            if np.minimum(*self.body_clear(self.R[j][None], np.array([th]))).min() >= 0.25:
                goal = (self.R[j, 0], self.R[j, 1], th)
                break
        if goal is None:
            self.get_logger().warn('탈출 목표점 없음')
            return False
        # 로컬 clearance 격자 미리 계산 -> 탐색 중 충돌검사는 배열 인덱싱만
        res = 0.025
        half = 0.5 * math.hypot(goal[0] - x, goal[1] - y) + 1.5
        cx, cy = (x + goal[0]) / 2, (y + goal[1]) / 2
        n = int(2 * half / res)
        gx = cx - half + (np.arange(n) + 0.5) * res
        gy = cy - half + (np.arange(n) + 0.5) * res
        XX, YY = np.meshgrid(gx, gy)
        G = self.clear(np.stack([XX.ravel(), YY.ravel()], 1)).reshape(n, n)
        ox, oy = cx - half, cy - half

        def lookup(Q):
            ix = ((Q[:, 0] - ox) / res).astype(int)
            iy = ((Q[:, 1] - oy) / res).astype(int)
            inside = (ix >= 0) & (ix < n) & (iy >= 0) & (iy < n)
            out = np.zeros(len(Q))
            out[inside] = G[iy[inside], ix[inside]]
            return out

        def body_c(P, H):
            c = np.full(len(P), 9.0)
            u = np.stack([np.cos(H), np.sin(H)], 1)
            for off in BODY_OFF:
                c = np.minimum(c, lookup(P + off * u))
            return c

        thr = C_TIGHT + 0.005
        c_start = float(body_c(np.array([[x, y]]), np.array([yaw]))[0])

        def ok_body(P, H):
            # 시작점 0.35m 이내는 현재 clearance 까지 허용(이미 붙어 있어도 빠져나오는 동작 탐색)
            near = np.hypot(P[:, 0] - x, P[:, 1] - y) < 0.35
            t = np.where(near, min(thr, c_start - 0.01), thr)
            return body_c(P, H) >= t

        path = hybrid_astar.plan((x, y, yaw), goal, ok_body, lambda P: lookup(P) >= 0.13, max_time=MAN_TIME)
        dt = time.time() - t0
        if path is None or len(path) < 2:
            self.get_logger().warn(f'Hybrid A* 실패 ({dt:.2f}s)')
            return False
        A = np.array(path)
        A[0, 3] = A[1, 3]
        nrev = int((A[:, 3] < 0).sum())
        self.get_logger().warn(f'Hybrid A* 탈출 경로: {len(A)}점, 후진 {nrev}점 ({dt:.2f}s)')
        segs = []
        cur = [A[0]]
        for a in A[1:]:
            if a[3] != cur[-1][3] and len(cur) > 1:
                segs.append(np.array(cur))
                cur = [cur[-1].copy()]
                cur[0][3] = a[3]
            cur.append(a)
        segs.append(np.array(cur))
        self.man = dict(segs=segs, k=0, t0=self.now())
        self.pub_path.publish(self.to_path(A[:, :2]))
        return True

    def follow_maneuver(self, tnow, x, y, yaw):
        cmd = Twist()
        mn = self.man
        if abs(self.v_meas) > 0.05:
            mn['t_move'] = tnow
        if tnow - mn['t0'] > 20.0 or tnow - mn.get('t_move', mn['t0']) > 1.5:
            self.get_logger().warn('탈출 기동 정지/시간초과 -> 해제')
            self.man = None
            self.pub.publish(cmd)
            return
        S = mn['segs'][mn['k']]
        d = S[-1, 3] if S[-1, 3] != 0 else 1.0
        # 후륜축 기준 경로
        Rr = S[:, :2] - REAR * np.stack([np.cos(S[:, 2]), np.sin(S[:, 2])], 1)
        rx, ry = x - REAR * math.cos(yaw), y - REAR * math.sin(yaw)
        dist = np.hypot(Rr[:, 0] - rx, Rr[:, 1] - ry)
        ic = int(np.argmin(dist))
        end = Rr[-1]
        hd = np.array([math.cos(yaw), math.sin(yaw)]) * d
        if math.hypot(*(end - [rx, ry])) < 0.08 or (ic == len(Rr) - 1 and np.dot(end - [rx, ry], hd) < 0.02):
            mn['k'] += 1
            if mn['k'] >= len(mn['segs']):
                self.get_logger().info('탈출 기동 완료')
                self.man = None
                self.prev_o = None
                self.obs_keep_t = tnow
            self.pub.publish(cmd)
            return
        ld = 0.3
        far = np.where((dist >= ld) & (np.arange(len(Rr)) > ic))[0]
        tx, ty = Rr[far[0]] if len(far) else end
        hdg = yaw if d > 0 else yaw + math.pi
        al = math.atan2(ty - ry, tx - rx) - hdg
        Ld = max(math.hypot(tx - rx, ty - ry), 0.1)
        kap = max(-K_MAX, min(K_MAX, 2 * math.sin(al) / Ld))
        u = 0.25
        cmd.linear.x = u * d
        cmd.angular.z = u * kap
        self.pub.publish(cmd)
        if tnow - self.t_log > 0.5:
            self.t_log = tnow
            print(f'  [기동] seg {mn["k"] + 1}/{len(mn["segs"])} dir={d:+.0f} ic={ic}/{len(Rr)} '
                  f'end_d={math.hypot(*(end - [rx, ry])):.2f} pos=({x:.2f},{y:.2f},{yaw:+.2f}) kap={kap:+.2f} '
                  f'v={self.v_meas:+.2f}', flush=True)

    def pick_reverse(self, pref):
        """후진 궤적(0.3m/s x REV_T) 충돌검사: 선호 조향 -> 직진 -> 반대 순"""
        p = self.pose()
        if p is None:
            return None
        x0, y0, th0 = p
        for st in dict.fromkeys([pref, 0.0, -pref, 1.0, -1.0]):
            w = 0.3 * 0.8 * K_MAX * st
            x, y, th = x0, y0, th0
            P, H = [], []
            for _ in range(int(REV_T / 0.05)):
                th += w * 0.05
                x -= 0.3 * 0.05 * math.cos(th)
                y -= 0.3 * 0.05 * math.sin(th)
                P.append((x, y))
                H.append(th)
            cs, cd = self.body_clear(np.array(P), np.array(H))
            if np.minimum(cs, cd).min() >= C_HARD - 0.03:
                return st
        return None

    def lap_check(self, tnow):
        if self.idx is None:
            return
        if self.last_idx_for_lap is not None:
            # 출발 직후(출발점이 레이스라인 시작점 살짝 뒤) 통과는 랩으로 세지 않음: 랩1 시간은 엔터 시점부터
            if self.last_idx_for_lap > self.n * 0.9 and self.idx < self.n * 0.1 and tnow - self.lap_t0 > 5.0:
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


def stop_car(node, n=10):
    """정지 명령(속도 0) 반복 발행: Gazebo 플러그인은 마지막 명령을 유지하므로 종료 전에 반드시 필요"""
    node.go = False
    for _ in range(n):
        try:
            node.pub.publish(Twist())
        except Exception:
            return
        time.sleep(0.03)


def main():
    ws = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    base = os.path.join(ws, 'maps', 'exam', 'map')
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default=base + '.yaml')
    ap.add_argument('--line', default=None, help='기본: MAP_raceline.csv')
    ap.add_argument('--init', nargs=3, type=float)
    ap.add_argument('--traj', default=None, help='매핑 주행 궤적(지도 미관측 보정). 기본: MAP_traj.csv')
    ap.add_argument('--go', action='store_true', help='대기 없이 바로 출발(테스트용)')
    ap.add_argument('--enter', action='store_true', help='엔터로 출발(exam.py 통합 실행용)')
    ap.add_argument('--odom', action='store_true', help='(호환용) --init 없으면 항상 odom 모드')
    ap.add_argument('--slow', action='store_true', help='좁은 곳 감속 켬(예전 기본 동작)')
    a = ap.parse_args()
    global NO_SLOW
    NO_SLOW = not a.slow
    stem = os.path.splitext(a.map)[0]
    if a.line is None:
        a.line = stem + '_raceline.csv'
    if a.traj is None and os.path.exists(stem + '_traj.csv'):
        a.traj = stem + '_traj.csv'
    a.odom = a.init is None
    # Ctrl+C 를 rclpy 가 가로채 통신을 먼저 끊지 않게: KeyboardInterrupt 로 받아 정지 명령을 보낸 뒤 종료
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Race(a)
    try:
        run(node, a)
    except KeyboardInterrupt:
        pass
    finally:
        stop_car(node)
        print('\n정지 명령 발행 후 종료', flush=True)
        if rclpy.ok():
            rclpy.shutdown()


def run(node, a):
    if a.init:
        # AMCL 이 구독 시작 + map->odom TF 가 나올 때까지 반복 발행
        t0 = time.time()
        sent = 0
        while rclpy.ok() and time.time() - t0 < 60:
            if node.count_subscribers('/initialpose') > 0:
                node.send_initialpose(*a.init)
                sent += 1
            t1 = time.time()
            while time.time() - t1 < 0.5:
                rclpy.spin_once(node, timeout_sec=0.05)
            if sent >= 2 and node.tfb.can_transform('map', 'odom', Time()):
                break
        print(f'초기 pose 설정 완료 ({sent}회 발행)', flush=True)
    # TF/스캔/첫 계획 준비
    t0 = time.time()
    while rclpy.ok() and (node.plan is None) and time.time() - t0 < 30:
        node.update_m2o()
        rclpy.spin_once(node, timeout_sec=0.05)
    if a.go:
        print('READY', flush=True)
    elif a.enter:                      # 통합 실행(exam.py): 엔터로 출발
        print('READY - 엔터를 누르면 즉시 출발', flush=True)

        def wait_enter():
            try:
                input()
            except EOFError:
                return
            node.go = True
        threading.Thread(target=wait_enter, daemon=True).start()
    else:                              # 단계별 실행: 다른 터미널에서 출발 명령 실행
        print('READY - 다른 터미널에서 출발 명령:  python3 scripts/race/go.py', flush=True)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    finally:
        node.go = False
        ex.shutdown(timeout_sec=0.5)


if __name__ == '__main__':
    main()
