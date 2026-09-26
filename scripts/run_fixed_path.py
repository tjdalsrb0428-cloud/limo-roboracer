#!/usr/bin/env python3
import csv
import math
import sys
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

CSV_PATH = "/home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble/maps/spline_path.csv"
MAX_RETRY = 20

def load_path_points(path):
    pts = []
    with open(path) as f:
        r = csv.reader(f)
        next(r)
        for row in r:
            pts.append((float(row[0]), float(row[1]), float(row[2])))
    return pts

def build_path_msg(nav, pts, laps=1, trim_last=30):
    path = Path()
    path.header.frame_id = "map"
    path.header.stamp = rclpy.time.Time().to_msg()
    for lap_i in range(laps):
        lap_pts = pts
        if lap_i == laps - 1:
            lap_pts = pts[:-trim_last] if trim_last < len(pts) else pts
        for x, y, yaw in lap_pts:
            p = PoseStamped()
            p.header.frame_id = "map"
            p.header.stamp = path.header.stamp
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.orientation.z = math.sin(yaw / 2.0)
            p.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(p)
    return path

def main():
    laps = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    nav = BasicNavigator()
    pts = load_path_points(CSV_PATH)
    print(f"spline path {len(pts)}개 점 로드, {laps}랩으로 이어붙임 (총 {len(pts)*laps}개 pose)")

    path_msg = build_path_msg(nav, pts, laps=laps)

    retry_count = 0
    while retry_count <= MAX_RETRY:
        nav.followPath(path_msg)
        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb:
                print(f"남은 거리: {fb.distance_to_goal:.2f}m, 속도: {fb.speed:.2f}m/s")

        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            print("최종 결과: SUCCEEDED")
            break
        else:
            retry_count += 1
            print(f"결과: {result} -> 후진 후 재시도 ({retry_count}/{MAX_RETRY})")
            nav.backup(backup_dist=0.4, backup_speed=0.3)
            while not nav.isTaskComplete():
                pass
    else:
        print("최대 재시도 횟수 초과, 종료")

    rclpy.shutdown()

if __name__ == "__main__":
    main()
