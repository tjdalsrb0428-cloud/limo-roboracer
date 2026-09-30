#!/usr/bin/env python3
import sys, math, time
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator

SEG_DIST = 4.0     # 평상시 spline 구간(m)
CHECK_DIST = 6.0   # 장애물 검사 거리(m)
COST_TH = 100      # 이 이상이면 막힘(장애물 0.3m 이내)
PAST_DIST = 1.5    # 장애물 지난 뒤 복귀 지점(m)
TAIL_DIST = 2.0    # 복귀 지점 뒤로 붙일 spline(m)
PERIOD = 1.0

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

def main():
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    nav = BasicNavigator()
    pts = load(sys.argv[1])
    n = len(pts)
    ds = np.mean(np.hypot(*(np.roll(pts, -1, 0) - pts).T))
    k_seg = int(SEG_DIST / ds)
    k_chk = int(CHECK_DIST / ds)
    k_past = int(PAST_DIST / ds)
    k_tail = int(TAIL_DIST / ds)

    def yaw_at(i):
        j = (i + 1) % n
        return math.atan2(pts[j, 1] - pts[i, 1], pts[j, 0] - pts[i, 0])

    pose = {}
    def cb(m):
        q = m.pose.pose.orientation
        pose['x'] = m.pose.pose.position.x
        pose['y'] = m.pose.pose.position.y
        pose['yaw'] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    nav.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', cb, qos)
    while 'x' not in pose:
        rclpy.spin_once(nav, timeout_sec=0.1)

    warned = {'cm': False, 'path': False}
    def get_cost_fn():
        try:
            cm = nav.getGlobalCostmap()
            md = cm.metadata
            arr = np.array(cm.data, dtype=np.uint8).reshape(md.size_y, md.size_x)
            ox, oy, res = md.origin.position.x, md.origin.position.y, md.resolution
            def cost(x, y):
                ix = int((x - ox) / res)
                iy = int((y - oy) / res)
                if 0 <= ix < md.size_x and 0 <= iy < md.size_y:
                    return int(arr[iy, ix])
                return 255
            return cost
        except Exception as e:
            if not warned['cm']:
                print('costmap 가져오기 실패:', repr(e))
                warned['cm'] = True
            return None

    last = int(np.argmin(np.hypot(pts[:, 0] - pose['x'], pts[:, 1] - pose['y'])))
    t_last = 0.0
    t_start = time.monotonic()
    hist = []
    try:
        while rclpy.ok():
            rclpy.spin_once(nav, timeout_sec=0.05)
            now = time.monotonic()
            if now - t_last < PERIOD:
                continue
            t_last = now
            x, y = pose['x'], pose['y']

            win = (last + np.arange(-5, 41)) % n
            d = np.hypot(pts[win, 0] - x, pts[win, 1] - y)
            last = int(win[np.argmin(d)])

            # 정지 감지 -> 후진
            hist.append((now, x, y))
            hist = [h for h in hist if now - h[0] < 4.0]
            if now - t_start > 6.0 and now - hist[0][0] > 2.5 \
                    and math.hypot(x - hist[0][1], y - hist[0][2]) < 0.05:
                print('정지 감지 -> 후진')
                nav.cancelTask()
                nav.backup(backup_dist=0.4, backup_speed=0.3, time_allowance=10)
                while not nav.isTaskComplete():
                    time.sleep(0.05)
                hist.clear()
                continue

            # 앞 구간 장애물 검사
            cost = get_cost_fn()
            first_blocked = None
            if cost is not None:
                for j in range(1, k_chk + 1):
                    i = (last + j) % n
                    if cost(pts[i, 0], pts[i, 1]) >= COST_TH:
                        first_blocked = j
                        break

            path = None
            mode = 'spline'
            if first_blocked is not None:
                j = first_blocked + k_past
                for _ in range(3 * k_past):
                    if cost(pts[(last + j) % n, 0], pts[(last + j) % n, 1]) < 30:
                        break
                    j += 1
                gi = (last + j) % n
                start = mk_pose(x, y, pose['yaw'])
                goal = mk_pose(pts[gi, 0], pts[gi, 1], yaw_at(gi))
                sp = None
                try:
                    try:
                        sp = nav.getPath(start, goal, 'GridBased', use_start=True)
                    except TypeError:
                        sp = nav.getPath(start, goal, 'GridBased')
                except Exception as e:
                    if not warned['path']:
                        print('getPath 실패:', repr(e))
                        warned['path'] = True
                if sp is not None and len(sp.poses) > 1:
                    path = sp
                    path.header.stamp = rclpy.time.Time().to_msg()
                    for t in range(1, k_tail + 1):
                        i = (gi + t) % n
                        path.poses.append(mk_pose(pts[i, 0], pts[i, 1], yaw_at(i)))
                    mode = 'avoid'

            if path is None:
                path = Path()
                path.header.frame_id = 'map'
                path.header.stamp = rclpy.time.Time().to_msg()
                for t in range(1, k_seg + 1):
                    i = (last + t) % n
                    path.poses.append(mk_pose(pts[i, 0], pts[i, 1], yaw_at(i)))

            nav.followPath(path, 'FollowPath', 'general_goal_checker')
            print(f'[{mode}] idx={last}/{n} pos=({x:.2f},{y:.2f})')
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
