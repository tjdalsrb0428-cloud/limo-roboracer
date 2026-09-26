#!/usr/bin/env python3
import csv
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

CSV_PATH = "/home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble/maps/spline_path.csv"

LOOKAHEAD = 0.6       # m
MAX_SPEED = 0.6       # m/s (adapter 한계 0.9보다 낮게 여유)
MIN_SPEED = 0.3        # m/s
MAX_YAW_RATE = 1.5     # rad/s (adapter가 max_steer=0.42rad로 다시 클램프함)


def load_path_points(path):
    pts = []
    with open(path) as f:
        r = csv.reader(f)
        next(r)
        for row in r:
            pts.append((float(row[0]), float(row[1])))
    return pts


def yaw_from_quat(z, w):
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.pts = load_path_points(CSV_PATH)
        self.n = len(self.pts)
        self.last_idx = 0
        self.pose = None

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_cb, qos)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f"pure pursuit 시작: {self.n}개 점, lookahead={LOOKAHEAD}m, 목표점 없음(Ctrl+C로 종료)")

    def pose_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        yaw = yaw_from_quat(z, w)
        self.pose = (x, y, yaw)

    def find_target_idx(self, x, y):
        n = self.n
        search_range = 80
        best_idx = self.last_idx
        for k in range(search_range):
            idx = (self.last_idx + k) % n
            px, py = self.pts[idx]
            dist = math.hypot(px - x, py - y)
            if dist >= LOOKAHEAD:
                best_idx = idx
                break
            best_idx = idx
        return best_idx

    def control_loop(self):
        if self.pose is None:
            self.get_logger().info("waiting for pose...", throttle_duration_sec=1.0)
            return
        self.get_logger().info(f"pose ok: {self.pose}", throttle_duration_sec=1.0)
        x, y, yaw = self.pose

        n = self.n
        near_range = 40
        best_d = None
        best_i = self.last_idx
        for k in range(near_range):
            idx = (self.last_idx + k) % n
            px, py = self.pts[idx]
            d = math.hypot(px - x, py - y)
            if best_d is None or d < best_d:
                best_d = d
                best_i = idx
        self.last_idx = best_i

        target_idx = self.find_target_idx(x, y)
        tx, ty = self.pts[target_idx]

        dx = tx - x
        dy = ty - y
        local_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        local_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy

        ld = math.hypot(local_x, local_y)
        if ld < 1e-3:
            return

        curvature = 2.0 * local_y / (ld * ld)

        speed = MAX_SPEED
        if abs(curvature) > 2.0:
            speed = MIN_SPEED

        yaw_rate = speed * curvature
        yaw_rate = max(-MAX_YAW_RATE, min(MAX_YAW_RATE, yaw_rate))

        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = yaw_rate
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        stop = Twist()
        node.pub.publish(stop)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
