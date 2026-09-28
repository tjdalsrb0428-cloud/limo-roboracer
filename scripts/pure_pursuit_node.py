#!/usr/bin/env python3
"""Pure pursuit + Frenet 횡오프셋 궤적 샘플링 회피 (/scan 기반, nav2 costmap 불필요).

필요한 것: gazebo + amcl(/amcl_pose, map->laser_link tf) + /scan
파라미터: --ros-args -p v_max:=0.8 -p a_lat:=0.8 -p r_safe:=0.20
"""
import csv
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
import tf2_ros

CSV_PATH = "/home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble/maps/spline_path.csv"

DS = 0.05                                   # 기준선 재샘플 간격 [m]
K_LIMIT = 1.6                               # 후보 궤적 허용 곡률 [1/m]
K_STEER_MAX = 1.861                         # LIMO 최대 조향 곡률
OFFSETS = np.arange(-0.60, 0.601, 0.05)     # 횡오프셋 후보 [m]

V_MIN = 0.25
V_CREEP = 0.12
V_REV = -0.15
A_DEC = 0.8                                 # 감속 한도 [m/s^2]
A_UP = 0.6                                  # 가속 한도 [m/s^2]
C_PREF = 0.40                               # 이 이상 떨어지면 여유 충분 [m]
OBS_RANGE = 4.0                             # 장애물로 쓸 라이다 최대 거리 [m]

W_DEV = 1.0                                 # 기준선 이탈 비용
W_CHG = 1.5                                 # 이전 선택과의 차이 비용
W_CLR = 4.0                                 # 여유거리 부족 비용

STUCK_T = 1.5                               # 후보 없음 지속 시 후진 시작 [s]
REV_T = 0.8                                 # 후진 시간 [s]
DT = 0.05


def load_path(path):
    pts = []
    with open(path) as f:
        r = csv.reader(f)
        next(r)
        for row in r:
            pts.append((float(row[0]), float(row[1])))
    a = np.array(pts)
    if math.hypot(*(a[0] - a[-1])) < 1e-3:
        a = a[:-1]
    return a


def resample_closed(a, ds):
    closed = np.vstack([a, a[0]])
    seg = np.hypot(np.diff(closed[:, 0]), np.diff(closed[:, 1]))
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = s[-1]
    n = int(round(total / ds))
    ss = np.linspace(0.0, total, n, endpoint=False)
    x = np.interp(ss, s, closed[:, 0])
    y = np.interp(ss, s, closed[:, 1])
    return np.stack([x, y], 1), total / n


def yaw_from_quat(z, w):
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


class AvoidPurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.declare_parameter('v_max', 0.8)
        self.declare_parameter('a_lat', 0.8)
        self.declare_parameter('r_safe', 0.20)
        self.v_max = self.get_parameter('v_max').value
        self.a_lat = self.get_parameter('a_lat').value
        self.r_safe = self.get_parameter('r_safe').value

        raw = load_path(CSV_PATH)
        self.P, self.ds = resample_closed(raw, DS)
        self.n = len(self.P)
        T = np.roll(self.P, -2, 0) - np.roll(self.P, 2, 0)
        T /= np.maximum(np.linalg.norm(T, axis=1, keepdims=True), 1e-9)
        self.N = np.stack([-T[:, 1], T[:, 0]], 1)   # 좌측 법선

        self.pose = None
        self.obs = None
        self.last_idx = None
        self.prev_off = 0.0
        self.v_cmd = V_MIN
        self.stuck_since = None
        self.rev_until = 0.0

        pose_qos = QoSProfile(depth=10)
        pose_qos.reliability = ReliabilityPolicy.RELIABLE
        pose_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_cb, pose_qos)
        self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(DT, self.control_loop)

        self.get_logger().info(
            f"회피 pure pursuit 시작: {self.n}점(ds={self.ds:.3f}), "
            f"v_max={self.v_max}, a_lat={self.a_lat}, r_safe={self.r_safe}")

    # ---------------- 콜백 ----------------
    def pose_cb(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y,
                     yaw_from_quat(p.orientation.z, p.orientation.w))

    def scan_cb(self, msg):
        try:
            tf = self.tf_buffer.lookup_transform('map', msg.header.frame_id, Time())
        except Exception as e:
            self.get_logger().warn(f"tf 실패: {e}", throttle_duration_sec=2.0)
            return
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = yaw_from_quat(q.z, q.w)

        r = np.asarray(msg.ranges, dtype=np.float64)
        ang = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        ok = np.isfinite(r) & (r >= msg.range_min) & (r < OBS_RANGE)
        r, ang = r[ok], ang[ok]
        lx, ly = r * np.cos(ang), r * np.sin(ang)
        c, s = math.cos(yaw), math.sin(yaw)
        self.obs = np.stack([tx + c * lx - s * ly, ty + s * lx + c * ly], 1)

    # ---------------- 유틸 ----------------
    def publish(self, v, yaw_rate):
        self.v_cmd = v
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(yaw_rate)
        self.pub.publish(msg)

    # ---------------- 제어 루프 ----------------
    def control_loop(self):
        if self.pose is None:
            return
        x, y, yaw = self.pose
        P, N, n, ds = self.P, self.N, self.n, self.ds
        now = time.monotonic()

        if now < self.rev_until:            # 후진 복구 중
            self.publish(V_REV, 0.0)
            return

        # 기준선 위 진행 인덱스
        if self.last_idx is None:
            self.last_idx = int(np.argmin(np.hypot(P[:, 0] - x, P[:, 1] - y)))
        else:
            idxs = (self.last_idx + np.arange(-10, 60)) % n
            d = np.hypot(P[idxs, 0] - x, P[idxs, 1] - y)
            self.last_idx = int(idxs[np.argmin(d)])
        i0 = self.last_idx
        d0 = (x - P[i0, 0]) * N[i0, 0] + (y - P[i0, 1]) * N[i0, 1]

        # 후보 궤적 생성 (현재 횡위치 d0 -> 목표 오프셋을 smoothstep으로 이동)
        H = min(3.0, max(1.5, 1.2 + 1.2 * self.v_cmd))
        M = int(H / ds)
        m = np.arange(1, M + 1)
        idx = (i0 + m) % n
        s = m * ds
        ramp = 1.0 + 0.8 * self.v_cmd
        t = np.clip(s / ramp, 0.0, 1.0)
        sm = t * t * (3.0 - 2.0 * t)
        off = OFFSETS[:, None]
        lat = d0 + (off - d0) * sm[None, :]
        cx = P[idx, 0][None, :] + N[idx, 0][None, :] * lat      # (K, M)
        cy = P[idx, 1][None, :] + N[idx, 1][None, :] * lat
        K = len(OFFSETS)

        # 충돌 여유거리
        clr = np.full(K, 9.9)
        if self.obs is not None and len(self.obs) > 0:
            near = self.obs[np.hypot(self.obs[:, 0] - x, self.obs[:, 1] - y) < H + 1.0]
            chk = s >= 0.3
            if len(near) > 0 and chk.any():
                dx = cx[:, chk][:, :, None] - near[None, None, :, 0]
                dy = cy[:, chk][:, :, None] - near[None, None, :, 1]
                clr = np.sqrt(dx * dx + dy * dy).min(axis=2).min(axis=1)

        # 곡률 (3스텝 chord 방향 변화)
        th = np.arctan2(cy[:, 3:] - cy[:, :-3], cx[:, 3:] - cx[:, :-3])
        dth = np.diff(th, axis=1)
        dth = (dth + math.pi) % (2 * math.pi) - math.pi
        kap = np.abs(dth) / ds                                  # (K, M-4)
        kmax = kap.max(axis=1)

        feas = (clr >= self.r_safe) & (kmax <= K_LIMIT)

        if not feas.any():
            # 후보 없음: 여유가 가장 큰 후보로 기어가고, 오래 막히면 후진
            if self.stuck_since is None:
                self.stuck_since = now
            if now - self.stuck_since > STUCK_T:
                self.rev_until = now + REV_T
                self.stuck_since = None
                self.prev_off = 0.0
                return
            k = int(np.argmax(clr))
            v_tgt = V_CREEP if clr[k] > 0.13 else 0.0
            status = 'stuck'
        else:
            self.stuck_since = None
            cost = (W_DEV * np.abs(OFFSETS)
                    + W_CHG * np.abs(OFFSETS - self.prev_off)
                    + W_CLR * np.clip(C_PREF - clr, 0.0, None))
            cost[~feas] = np.inf
            k = int(np.argmin(cost))
            self.prev_off = float(OFFSETS[k])

            # 속도: 곡률 기반 + 감속거리 + 여유거리
            kc = kap[k]
            s_k = (np.arange(len(kc)) + 2) * ds
            v_curve = np.sqrt(self.a_lat / (kc + 1e-3))
            v_brk = float(np.sqrt(v_curve ** 2 + 2.0 * A_DEC * s_k).min())
            v_cl = V_MIN + (self.v_max - V_MIN) * float(
                np.clip((clr[k] - self.r_safe) / 0.30, 0.0, 1.0))
            v_tgt = max(V_MIN, min(self.v_max, v_brk, v_cl))
            status = 'clear' if abs(OFFSETS[k]) < 1e-6 else 'avoid'

        v = min(v_tgt, self.v_cmd + A_UP * DT) if v_tgt > self.v_cmd else v_tgt

        # pure pursuit: 선택된 후보 위 lookahead 점 추종
        Ld = min(1.4, max(0.6, 0.5 + 0.5 * self.v_cmd))
        dist = np.hypot(cx[k] - x, cy[k] - y)
        j = int(np.argmax(dist >= Ld)) if (dist >= Ld).any() else M - 1
        dxw, dyw = cx[k, j] - x, cy[k, j] - y
        lx = math.cos(yaw) * dxw + math.sin(yaw) * dyw
        ly = -math.sin(yaw) * dxw + math.cos(yaw) * dyw
        ld = max(math.hypot(lx, ly), 1e-3)
        kappa = max(-K_STEER_MAX, min(K_STEER_MAX, 2.0 * ly / (ld * ld)))

        self.publish(v, v * kappa)
        self.get_logger().info(
            f"{status} off={OFFSETS[k]:+.2f} d0={d0:+.2f} clr={clr[k]:.2f} "
            f"kmax={kmax[k]:.2f} v={v:.2f}", throttle_duration_sec=0.5)


def main():
    rclpy.init()
    node = AvoidPurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
