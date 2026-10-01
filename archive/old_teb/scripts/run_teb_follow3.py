#!/usr/bin/env python3
import sys, math, time
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from nav2_simple_commander.robot_navigator import BasicNavigator

SEG_DIST = 4.0     # 평상시 spline 구간(m)
OBS_RANGE = 6.0    # 장애물 인식 거리(m)
WALL_TOL = 0.30    # 정적맵 벽에서 이 이상 떨어진 라이다 점 = 장애물(m)
CLEAR = 0.30       # spline이 장애물 가장자리에서 이보다 가까우면 회피(m)
RAMP_IN = 2.2      # 회피 시작 전환 거리(m), 클수록 미리 튼다
RAMP_OUT = 1.8
HOLD_IN = 0.6
HOLD_OUT = 0.6
MIN_RAMP = 2.0     # 현재 위치 오프셋 -> 목표 오프셋 최소 전환 거리(m)
LIDAR_DX = 0.0     # 라이다가 base_link 앞쪽으로 떨어진 거리(m)
PERIOD = 0.25
WALL_CLR = 0.13    # 회피 경로가 벽에서 이보다 가까우면 위반(m)

def load(p):
    rows = []
    for line in open(p):
        t = line.strip().split(',')
        try:
            rows.append([float(t[0]), float(t[1])])
        except (ValueError, IndexError):
            continue
    return np.array(rows)

def mk_pose(x, y, yaw):
    ps = PoseStamped()
    ps.header.frame_id = 'map'
    ps.header.stamp = rclpy.time.Time().to_msg()
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.orientation.z = math.sin(yaw / 2)
    ps.pose.orientation.w = math.cos(yaw / 2)
    return ps

def smooth(t):
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3 - 2 * t)

def profile(rel):
    up = smooth((rel + HOLD_IN + RAMP_IN) / RAMP_IN)
    down = 1.0 - smooth((rel - HOLD_OUT) / RAMP_OUT)
    return np.minimum(up, down)

def main():
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    nav = BasicNavigator()
    pts = load(sys.argv[1])
    if math.hypot(*(pts[-1] - pts[0])) < 0.03:
        pts = pts[:-1]   # 끝점=시작점 중복이면 seg=0 -> NaN
    n = len(pts)
    dv = np.roll(pts, -1, 0) - pts
    seg = np.hypot(dv[:, 0], dv[:, 1])
    ds = float(seg.mean())
    tang = dv / seg[:, None]
    nrm = np.stack([-tang[:, 1], tang[:, 0]], 1)
    k_seg = int(SEG_DIST / ds)
    k_win = int(OBS_RANGE / ds) + 5

    pose = {}
    st = {'scan': None}
    def pose_cb(m):
        q = m.pose.pose.orientation
        pose['x'] = m.pose.pose.position.x
        pose['y'] = m.pose.pose.position.y
        pose['yaw'] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    def scan_cb(m):
        st['scan'] = m
    def map_cb(m):
        st['grid'] = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        st['res'] = m.info.resolution
        st['ox'] = m.info.origin.position.x
        st['oy'] = m.info.origin.position.y
    qos_tl = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    nav.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', pose_cb, qos_tl)
    nav.create_subscription(LaserScan, '/scan', scan_cb, qos_profile_sensor_data)
    nav.create_subscription(OccupancyGrid, '/map', map_cb, qos_tl)
    pub = nav.create_publisher(Path, '/avoid_path', 1)
    while 'x' not in pose or 'grid' not in st:
        rclpy.spin_once(nav, timeout_sec=0.1)

    def wall_near(x, y, rad):
        g = st['grid']; r = st['res']
        ix = int((x - st['ox']) / r); iy = int((y - st['oy']) / r)
        if ix < 0 or iy < 0 or ix >= g.shape[1] or iy >= g.shape[0]:
            return True
        c = int(rad / r)
        w = g[max(iy - c, 0):iy + c + 1, max(ix - c, 0):ix + c + 1]
        return bool(((w >= 50) | (w < 0)).any())

    def wall_lat(jabs, sign):
        c = pts[jabs]; v = nrm[jabs] * sign
        for l in np.arange(0.05, 1.8, 0.05):
            if wall_near(c[0] + v[0] * l, c[1] + v[1] * l, 0.0):
                return float(l)
        return 1.8

    def detect(last):
        sc = st['scan']
        if sc is None:
            return []
        r = np.array(sc.ranges, dtype=float)
        a = sc.angle_min + np.arange(len(r)) * sc.angle_increment
        ok = np.isfinite(r) & (r > 0.15) & (r < OBS_RANGE)
        r, a = r[ok], a[ok]
        yaw = pose['yaw']
        wa = a + yaw
        px = pose['x'] + LIDAR_DX * math.cos(yaw) + r * np.cos(wa)
        py = pose['y'] + LIDAR_DX * math.sin(yaw) + r * np.sin(wa)
        keep = np.array([not wall_near(x, y, WALL_TOL) for x, y in zip(px, py)], dtype=bool)
        if keep.sum() < 3:
            return []
        P = np.stack([px[keep], py[keep]], 1)
        dP = np.diff(P, axis=0)
        gaps = np.hypot(dP[:, 0], dP[:, 1]) > 0.3
        cid = np.concatenate([[0], np.cumsum(gaps)])
        W = (last + np.arange(-5, k_win)) % n
        out = []
        for c in np.unique(cid):
            Q = P[cid == c]
            if len(Q) < 3:
                continue
            D = np.hypot(Q[:, None, 0] - pts[W, 0][None, :], Q[:, None, 1] - pts[W, 1][None, :])
            wp = D.argmin(axis=1)
            j = int(np.median(wp))
            if j >= len(W) - 2:
                continue
            dq = Q - pts[W[wp]]
            lat = tang[W[wp], 0] * dq[:, 1] - tang[W[wp], 1] * dq[:, 0]
            if lat.min() > 1.4 or lat.max() < -1.4:
                continue
            out.append((j - 5, int(W[j]), float(lat.min()), float(lat.max())))
        return out

    memo = {}
    def plan_obstacles(last, obs):
        for k in list(memo):
            if ((k - last + n // 2) % n) - n // 2 < -15:
                del memo[k]
        out = []
        for rel, jabs, omin, omax in obs:
            if rel < -10:
                continue
            if not (omin - CLEAR < 0 < omax + CLEAR):
                continue
            latL = wall_lat(jabs, +1)
            latR = -wall_lat(jabs, -1)
            gapL = latL - omax
            gapR = omin - latR
            side = 1 if gapL >= gapR else -1
            key = next((k for k in memo if min((k - jabs) % n, (jabs - k) % n) <= 10), None)
            if key is not None:
                s0 = memo[key]
                g0 = gapL if s0 > 0 else gapR
                if g0 >= 0.28:
                    side = s0
                del memo[key]
            memo[jabs] = side
            out.append((rel, jabs, side, (omax + latL) / 2, (omin + latR) / 2))
        return out

    def build(last, plan, flip=False):
        n_pts = max(k_seg, max(o[0] for o in plan) + int((HOLD_OUT + RAMP_OUT + 1.0) / ds))
        n_pts = min(n_pts, int(10.0 / ds))
        ii = np.arange(1, n_pts + 1)
        idxs = (last + ii) % n
        d = np.zeros(n_pts)
        for rel, jabs, side, tL, tR in plan:
            sd = -side if flip else side
            tgt = tL if sd > 0 else tR
            pr = profile((ii - rel) * ds) * tgt
            m = np.abs(pr) > np.abs(d)
            d[m] = pr[m]
        dx = pose['x'] - pts[last, 0]
        dy = pose['y'] - pts[last, 1]
        d_r = tang[last, 0] * dy - tang[last, 1] * dx
        w = smooth(ii * ds / MIN_RAMP)
        d = d * w + d_r * (1 - w)
        P = pts[idxs] + nrm[idxs] * d[:, None]
        skip = int(1.0 / ds)
        viol = sum(1 for x, y in P[skip:] if wall_near(x, y, WALL_CLR))
        return P, viol

    def to_path(P):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = rclpy.time.Time().to_msg()
        yaw = 0.0
        for k in range(len(P)):
            if k < len(P) - 1:
                yaw = math.atan2(P[k + 1, 1] - P[k, 1], P[k + 1, 0] - P[k, 0])
            path.poses.append(mk_pose(P[k, 0], P[k, 1], yaw))
        return path

    last = int(np.argmin(np.hypot(pts[:, 0] - pose['x'], pts[:, 1] - pose['y'])))
    t_last = 0.0
    t_start = time.monotonic()
    hist = []
    try:
        while rclpy.ok():
            rclpy.spin_once(nav, timeout_sec=0.02)
            now = time.monotonic()
            if now - t_last < PERIOD:
                continue
            t_last = now
            x, y = pose['x'], pose['y']

            win = (last + np.arange(-5, 41)) % n
            dd = np.hypot(pts[win, 0] - x, pts[win, 1] - y)
            last = int(win[np.argmin(dd)])

            hist.append((now, x, y))
            hist = [h for h in hist if now - h[0] < 3.0]
            if now - t_start > 6.0 and now - hist[0][0] > 1.8 \
                    and math.hypot(x - hist[0][1], y - hist[0][2]) < 0.05:
                print('정지 감지 -> 후진', flush=True)
                nav.cancelTask()
                nav.backup(backup_dist=0.4, backup_speed=0.3, time_allowance=10)
                while not nav.isTaskComplete():
                    time.sleep(0.05)
                hist.clear()
                continue

            t0 = time.monotonic()
            plan = plan_obstacles(last, detect(last))
            P = None
            mode = 'spline'
            if plan:
                P1, v1 = build(last, plan)
                P2, v2 = build(last, plan, flip=True)
                if v1 <= v2:
                    P, mode, viol = P1, 'avoid', v1
                else:
                    P, mode, viol = P2, 'avoid-flip', v2
                if viol > 0:
                    mode += f'(벽근접{viol})'
            if P is None:
                P = pts[(last + 1 + np.arange(k_seg)) % n]
            t1 = time.monotonic()
            pm = to_path(P)
            nav.followPath(pm, 'FollowPath', 'general_goal_checker')
            pub.publish(pm)
            t2 = time.monotonic()
            print(f'[{mode}] idx={last}/{n} pos=({x:.2f},{y:.2f}) obs={len(plan)}{[(o[0], o[1]) for o in plan]} '
                  f'계산={(t1 - t0) * 1000:.0f}ms 발행={(t2 - t1) * 1000:.0f}ms', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            nav.cancelTask()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
