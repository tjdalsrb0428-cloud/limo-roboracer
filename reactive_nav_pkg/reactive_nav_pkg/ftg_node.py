import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np


class FollowTheGap(Node):
    def __init__(self):
        super().__init__('ftg_node')

        self.declare_parameter('bubble_radius_m', 0.3)
        self.declare_parameter('max_range', 5.0)
        self.declare_parameter('min_gap_size', 5)
        self.declare_parameter('max_speed', 1.2)
        self.declare_parameter('min_speed', 0.4)
        self.declare_parameter('max_steer_angle', 0.6)
        self.declare_parameter('fov_deg', 180.0)

        self.bubble_radius = self.get_parameter('bubble_radius_m').value
        self.max_range = self.get_parameter('max_range').value
        self.min_gap_size = self.get_parameter('min_gap_size').value
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.max_steer = self.get_parameter('max_steer_angle').value
        self.fov_deg = self.get_parameter('fov_deg').value

        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('FTG reactive navigation node started')

    def preprocess(self, ranges):
        ranges = np.array(ranges, dtype=np.float32)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=self.max_range, neginf=0.0)
        ranges = np.clip(ranges, 0.0, self.max_range)
        return ranges

    def apply_bubble(self, ranges, angle_increment):
        closest_idx = int(np.argmin(ranges))
        closest_dist = ranges[closest_idx]
        if closest_dist <= 0.01:
            return ranges
        bubble_pts = int(self.bubble_radius / max(closest_dist * angle_increment, 1e-4))
        start = max(0, closest_idx - bubble_pts)
        end = min(len(ranges), closest_idx + bubble_pts)
        ranges[start:end] = 0.0
        return ranges

    def find_max_gap(self, ranges):
        masked = ranges > 0.1
        best_start, best_len = 0, 0
        cur_start, cur_len = None, 0
        for i, v in enumerate(masked):
            if v:
                if cur_start is None:
                    cur_start = i
                cur_len += 1
            else:
                if cur_len > best_len:
                    best_len, best_start = cur_len, cur_start
                cur_start, cur_len = None, 0
        if cur_len > best_len:
            best_len, best_start = cur_len, cur_start
        if best_len < self.min_gap_size:
            return None
        return best_start, best_start + best_len - 1

    def best_point_in_gap(self, ranges, start, end):
        segment = ranges[start:end + 1]
        return start + int(np.argmax(segment))

    def scan_cb(self, msg: LaserScan):
        ranges = self.preprocess(msg.ranges)

        total_angle = msg.angle_max - msg.angle_min
        fov_rad = np.deg2rad(self.fov_deg)
        n = len(ranges)
        center = n // 2
        half_window = int((fov_rad / total_angle) * n / 2) if total_angle > 0 else n // 2
        lo = max(0, center - half_window)
        hi = min(n, center + half_window)
        ranges = ranges[lo:hi]

        ranges = self.apply_bubble(ranges, msg.angle_increment)

        gap = self.find_max_gap(ranges)
        cmd = Twist()

        if gap is None:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub.publish(cmd)
            return

        start, end = gap
        best_idx = self.best_point_in_gap(ranges, start, end)

        idx_in_full = lo + best_idx
        angle = msg.angle_min + idx_in_full * msg.angle_increment
        steer = float(np.clip(angle, -self.max_steer, self.max_steer))

        speed_ratio = 1.0 - (abs(steer) / self.max_steer)
        speed = self.min_speed + (self.max_speed - self.min_speed) * speed_ratio

        cmd.linear.x = float(speed)
        cmd.angular.z = steer
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
