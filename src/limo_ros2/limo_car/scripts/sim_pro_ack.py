#!/usr/bin/env python3
"""LIMO Pro 명령 흉내 (roboracer_sim.launch.py real:=true robot:=limo_pro)

실차 Pro 드라이버(limo_ackerman_driver.cpp ackermanCmdCallback)처럼 /limo/ack_cmd(AckermannDrive)를 받는다:
  보드값 = steering_angle / 1.215, 보드값 ±0.42로 제한.
보드값 → 실제 중심 조향각 관계는 실차에서 아직 모름 → 여기서는 실제 = 보드값 × board_to_center(기본 1.215, 즉 명령 그대로)로 가정.
결과를 Gazebo 플러그인 입력(/ackermann_controller/cmd_vel)으로 바꿔 보낸다(Twist 어댑터와 같은 변환).
명령이 cmd_timeout 동안 안 오면 정지(실차 보드는 마지막 명령 유지 여부 미확인 → 안전 쪽).
"""
import math
import os
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ackermann_twist_adapter import convert_twist  # noqa: E402


class SimProAck(Node):
    def __init__(self):
        super().__init__('sim_pro_ack')
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        self.L = p('wheelbase', 0.20)
        self.max_steer = p('max_steer', 0.41)          # Gazebo 플러그인 한계(중심각)
        self.div = p('cmd_div', 1.215)                 # 드라이버: 보드값 = 명령 / 1.215
        self.board_max = p('board_max', 0.42)          # 드라이버: 보드값 제한
        self.b2c = p('board_to_center', 1.215)         # 가정: 실제 중심각 = 보드값 × 1.215
        self.timeout = p('cmd_timeout', 0.5)
        self.pub = self.create_publisher(Twist, '/ackermann_controller/cmd_vel', 1)
        self.create_subscription(AckermannDrive, '/limo/ack_cmd', self.on_cmd, 10)
        self.last = self.get_clock().now()
        self.create_timer(0.1, self.watchdog)
        self.stopped = True

    def on_cmd(self, m):
        board = max(-self.board_max, min(self.board_max, m.steering_angle / self.div))
        delta = board * self.b2c
        v = m.speed
        yaw_rate = v * math.tan(delta) / self.L if abs(v) > 1e-6 else 0.0
        out = Twist()
        out.linear.x, out.angular.z = convert_twist(v, yaw_rate, self.L, self.max_steer)
        self.pub.publish(out)
        self.last = self.get_clock().now()
        self.stopped = False

    def watchdog(self):
        if not self.stopped and (self.get_clock().now() - self.last).nanoseconds * 1e-9 > self.timeout:
            self.pub.publish(Twist())
            self.stopped = True


def main():
    rclpy.init()
    n = SimProAck()
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
