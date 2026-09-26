#!/usr/bin/env python3
import csv
import math
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

CSV_PATH = "/home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble/maps/waypoints_manual.csv"

def make_pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p

def main():
    rclpy.init()
    nav = BasicNavigator()

    pts = []
    with open(CSV_PATH) as f:
        r = csv.reader(f)
        next(r)
        for row in r:
            pts.append((float(row[0]), float(row[1])))

    goals = []
    for i in range(0, len(pts)):
        x, y = pts[i]
        nx, ny = pts[i + 1] if i + 1 < len(pts) else pts[0]
        yaw = math.atan2(ny - y, nx - x)
        goals.append(make_pose(nav, x, y, yaw))

    # nav2가 이미 켜진 상태라 active 체크 생략, 바로 전송
    nav.goThroughPoses(goals)

    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        if fb:
            print(f"남은 거리: {fb.distance_remaining:.2f}m")

    result = nav.getResult()
    print("최종 결과:", result)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
