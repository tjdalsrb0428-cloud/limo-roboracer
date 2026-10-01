#!/usr/bin/env python3
import sys, math, time
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator

SEG_DIST = 4.0
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

def main():
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    nav = BasicNavigator()
    pts = load(sys.argv[1])
    n = len(pts)
    ds = np.mean(np.hypot(*(np.roll(pts, -1, 0) - pts).T))
    k = int(SEG_DIST / ds)

    pose = {}
    def cb(m):
        pose['x'] = m.pose.pose.position.x
        pose['y'] = m.pose.pose.position.y
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    nav.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', cb, qos)
    while 'x' not in pose:
        rclpy.spin_once(nav, timeout_sec=0.1)

    last = int(np.argmin(np.hypot(pts[:, 0] - pose['x'], pts[:, 1] - pose['y'])))
    t_last = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(nav, timeout_sec=0.05)
            if time.monotonic() - t_last < PERIOD:
                continue
            t_last = time.monotonic()
            win = (last + np.arange(-5, 41)) % n
            d = np.hypot(pts[win, 0] - pose['x'], pts[win, 1] - pose['y'])
            last = int(win[np.argmin(d)])

            idxs = (last + 1 + np.arange(k)) % n
            path = Path()
            path.header.frame_id = 'map'
            path.header.stamp = rclpy.time.Time().to_msg()
            for i in idxs:
                j = (i + 1) % n
                yaw = math.atan2(pts[j, 1] - pts[i, 1], pts[j, 0] - pts[i, 0])
                ps = PoseStamped()
                ps.header = path.header
                ps.pose.position.x = float(pts[i, 0])
                ps.pose.position.y = float(pts[i, 1])
                ps.pose.orientation.z = math.sin(yaw / 2)
                ps.pose.orientation.w = math.cos(yaw / 2)
                path.poses.append(ps)
            nav.followPath(path, 'FollowPath', 'general_goal_checker')
            fb = nav.getFeedback()
            print(f'idx={last}/{n} pos=({pose["x"]:.2f},{pose["y"]:.2f}) '
                  f'speed={getattr(fb, "speed", -1):.2f} dist={getattr(fb, "distance_to_goal", -1):.2f}')
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
