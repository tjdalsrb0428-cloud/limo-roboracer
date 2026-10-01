#!/usr/bin/env python3
import csv
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PointStamped
from nav2_msgs.msg import Costmap
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (PointStamped 변환 등록용)

CSV_PATH = "/home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble/maps/spline_path.csv"

LOOKAHEAD = 0.6
MAX_SPEED = 0.6
MIN_SPEED = 0.3
STOP_SPEED = 0.0
MAX_YAW_RATE = 1.5

LATERAL_OFFSETS = [0.0, 0.1, -0.1, 0.2, -0.2, 0.3, -0.3, 0.4, -0.4]
COST_THRESHOLD = 80
UNKNOWN_AS_FREE = True


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

        self.costmap = None
        self.prev_offset = 0.0

        pose_qos = QoSProfile(depth=10)
        pose_qos.reliability = ReliabilityPolicy.RELIABLE
        pose_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.sub_pose = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_cb, pose_qos)

        costmap_qos = QoSProfile(depth=1)
        costmap_qos.reliability = ReliabilityPolicy.RELIABLE
        costmap_qos.durability = DurabilityPolicy.VOLATILE
        self.sub_costmap = self.create_subscription(
            Costmap, '/costmap/costmap_raw', self.costmap_cb, costmap_qos)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f"pure pursuit(+costmap 회피) 시작: {self.n}개 점, lookahead={LOOKAHEAD}m (Ctrl+C로 종료)")

    def pose_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        yaw = yaw_from_quat(z, w)
        self.pose = (x, y, yaw)

    def costmap_cb(self, msg):
        self.costmap = {
            'data': msg.data,
            'res': msg.metadata.resolution,
            'ox': msg.metadata.origin.position.x,
            'oy': msg.metadata.origin.position.y,
            'size_x': msg.metadata.size_x,
            'size_y': msg.metadata.size_y,
            'frame_id': msg.header.frame_id,
        }

    def get_cost_at_map_point(self, mx, my):
        if self.costmap is None:
            return None
        cm = self.costmap
        try:
            pt = PointStamped()
            pt.header.frame_id = 'map'
            pt.header.stamp = rclpy.time.Time().to_msg()
            pt.point.x = mx
            pt.point.y = my
            transformed = self.tf_buffer.transform(
                pt, cm['frame_id'], timeout=rclpy.duration.Duration(seconds=0.05))
            cx, cy = transformed.point.x, transformed.point.y
        except Exception as e:
            self.get_logger().warn(f"tf transform 실패: {e}", throttle_duration_sec=2.0)
            return None

        col = int((cx - cm['ox']) / cm['res'])
        row = int((cy - cm['oy']) / cm['res'])
        if col < 0 or col >= cm['size_x'] or row < 0 or row >= cm['size_y']:
            return 0 if UNKNOWN_AS_FREE else 100
        idx = row * cm['size_x'] + col
        if idx < 0 or idx >= len(cm['data']):
            return 0 if UNKNOWN_AS_FREE else 100
        val = cm['data'][idx]
        self.get_logger().info(f"cost_check mx={mx:.2f} my={my:.2f} cx={cx:.2f} cy={cy:.2f} col={col} row={row} val={val}", throttle_duration_sec=0.3)
        if val < 0:
            return 0 if UNKNOWN_AS_FREE else 100
        return val

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

    def path_tangent(self, idx):
        n = self.n
        i0 = (idx - 2) % n
        i1 = (idx + 2) % n
        x0, y0 = self.pts[i0]
        x1, y1 = self.pts[i1]
        dx, dy = x1 - x0, y1 - y0
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return 1.0, 0.0
        return dx / norm, dy / norm

    def pick_avoidance_point(self, target_idx):
        HORIZON = 15
        W_DEVIATION = 1.0
        W_CONTINUITY = 2.0

        horizon_idx = [(target_idx + k) % self.n for k in range(HORIZON)]

        best_score = None
        best_off = None
        for off in LATERAL_OFFSETS:
            safe = True
            costs_none = False
            for idx in horizon_idx:
                px, py = self.pts[idx]
                tdx, tdy = self.path_tangent(idx)
                nx, ny = -tdy, tdx
                cx = px + nx * off
                cy = py + ny * off
                cost = self.get_cost_at_map_point(cx, cy)
                if cost is None:
                    costs_none = True
                    break
                if cost >= COST_THRESHOLD:
                    safe = False
                    break
            if costs_none:
                self.prev_offset = 0.0
                tx, ty = self.pts[target_idx]
                return tx, ty, 'clear'
            if not safe:
                continue
            score = W_DEVIATION * abs(off) + W_CONTINUITY * abs(off - self.prev_offset)
            if best_score is None or score < best_score:
                best_score = score
                best_off = off

        if best_off is None:
            tx, ty = self.pts[target_idx]
            return tx, ty, 'stuck'

        self.prev_offset = best_off
        tx, ty = self.pts[target_idx]
        tdx, tdy = self.path_tangent(target_idx)
        nx, ny = -tdy, tdx
        cx = tx + nx * best_off
        cy = ty + ny * best_off
        status = 'clear' if best_off == 0.0 else 'avoided'
        return cx, cy, status

    def control_loop(self):
        if self.pose is None:
            return
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
        base_x, base_y = self.pts[target_idx]
        raw_cost = self.get_cost_at_map_point(base_x, base_y)
        tx, ty, status = self.pick_avoidance_point(target_idx)
        self.get_logger().info(
            f"status={status} robot=({x:.2f},{y:.2f}) target=({base_x:.2f},{base_y:.2f}) raw_cost={raw_cost}",
            throttle_duration_sec=0.5)

        dx = tx - x
        dy = ty - y
        local_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        local_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy

        ld = math.hypot(local_x, local_y)
        if ld < 1e-3:
            return

        curvature = 2.0 * local_y / (ld * ld)

        if status == 'stuck':
            speed = STOP_SPEED
        else:
            speed = MAX_SPEED
            if abs(curvature) > 2.0 or status == 'avoided':
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
        pass
    finally:
        try:
            stop = Twist()
            node.pub.publish(stop)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
