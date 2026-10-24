#!/usr/bin/env python3
"""실차 바퀴 오도메트리 흉내 (roboracer_sim.launch.py real:=true)

Gazebo 플러그인 정답 위치 /odom_gt 의 매 스텝 이동량을 차체 기준으로 바꾸고
속도 배율·회전 배율 오차, 회전 편향, 잡음을 넣어 적분한다 → /odom + TF odom→base_link.
오차는 누적되므로 map 프레임 위치는 위치추정(파티클 필터 등)으로 보정해야 한다.
정답 비교는 /odom_gt (같은 odom 프레임 원점에서 시작).
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class SimWheelOdom(Node):
    def __init__(self):
        super().__init__('sim_wheel_odom')
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        self.v_scale = p('v_scale', 1.03)      # 바퀴 반경 오차: 실제보다 3 % 더 갔다고 봄
        self.w_scale = p('w_scale', 0.96)      # 조향·미끄러짐: 회전을 4 % 덜 봄
        self.w_bias = p('w_bias', 0.004)       # rad/s, 정지 중에는 적용 안 함
        self.v_noise = p('v_noise', 0.02)      # 이동량 상대 잡음(표준편차)
        self.w_noise = p('w_noise', 0.02)      # rad/s 잡음(표준편차)
        self.rate = p('rate', 50.0)            # /odom 발행 Hz
        self.rng = np.random.default_rng(p('seed', 0))

        self.pub = self.create_publisher(Odometry, '/odom', 50)
        self.tfb = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom_gt', self.on_gt, 100)
        self.prev = None          # (t, x, y, yaw) 직전 정답
        self.pose = None          # [x, y, yaw] 오도메트리
        self.last_pub = -1.0
        self.v = self.w = 0.0

    def on_gt(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        x, y = m.pose.pose.position.x, m.pose.pose.position.y
        yaw = yaw_of(m.pose.pose.orientation)
        if self.prev is not None:
            pt, px, py, pyaw = self.prev
            dt = t - pt
            # Gazebo 리셋(시간 역행·순간이동) → 오도메트리를 정답 위치로 다시 시작
            if dt < 0 or math.hypot(x - px, y - py) > 0.3:
                self.prev, self.pose = None, None
        if self.prev is None:
            self.prev = (t, x, y, yaw)
            if self.pose is None:
                self.pose = [x, y, yaw]
            return
        pt, px, py, pyaw = self.prev
        dt = t - pt
        if dt <= 0:
            return
        # 정답 이동량을 직전 차체 기준으로
        c, s = math.cos(pyaw), math.sin(pyaw)
        dfx = c * (x - px) + s * (y - py)
        dlat = -s * (x - px) + c * (y - py)
        dyaw = math.atan2(math.sin(yaw - pyaw), math.cos(yaw - pyaw))
        moving = abs(dfx) > 1e-5
        k = self.v_scale * (1 + self.v_noise * self.rng.standard_normal()) if moving else 1.0
        dfx, dlat = dfx * k, dlat * k
        dyaw = dyaw * self.w_scale
        if moving:
            dyaw += (self.w_bias + self.w_noise * self.rng.standard_normal()) * dt
        ox, oy, oyaw = self.pose
        mid = oyaw + dyaw / 2
        ox += math.cos(mid) * dfx - math.sin(mid) * dlat
        oy += math.sin(mid) * dfx + math.cos(mid) * dlat
        oyaw = math.atan2(math.sin(oyaw + dyaw), math.cos(oyaw + dyaw))
        self.pose = [ox, oy, oyaw]
        self.v = 0.8 * self.v + 0.2 * dfx / dt
        self.w = 0.8 * self.w + 0.2 * dyaw / dt
        self.prev = (t, x, y, yaw)
        if t - self.last_pub >= 1.0 / self.rate - 1e-4:
            self.last_pub = t
            self.publish(m.header.stamp)

    def publish(self, stamp):
        x, y, yaw = self.pose
        qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y = x, y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        o.twist.twist.linear.x, o.twist.twist.angular.z = self.v, self.w
        self.pub.publish(o)
        tf = TransformStamped()
        tf.header = o.header
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
        self.tfb.sendTransform(tf)


def main():
    rclpy.init()
    n = SimWheelOdom()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
