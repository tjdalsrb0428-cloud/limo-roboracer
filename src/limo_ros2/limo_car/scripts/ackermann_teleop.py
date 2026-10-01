#!/usr/bin/env python3

import os
import sys
import termios
import tty

os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')   # Gazebo 런치와 같은 localhost 전용 통신
import rclpy  # noqa: E402
from rclpy.node import Node
from geometry_msgs.msg import Twist


class AckermannTeleop(Node):

    def __init__(self):
        super().__init__('ackermann_teleop')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # teleop_twist_keyboard와 비슷한 초기값
        self.speed = 0.5
        self.turn = 1.0

        self.speed_step = 0.1
        self.turn_step = 0.1

        self.get_logger().info(
            '\n'
            'Ackermann Teleop\n'
            '-----------------\n'
            'Moving around:\n'
            '   u    i    o\n'
            '   j    k    l\n'
            '   m    ,    .\n'
            '\n'
            'For Ackermann:\n'
            '   i : forward\n'
            '   , : backward\n'
            '   j : forward + left\n'
            '   l : forward + right\n'
            '   k : stop\n'
            '\n'
            'Speed:\n'
            '   q/z : increase/decrease speed\n'
            '   w/x : increase/decrease turn\n'
            '\n'
            'CTRL-C or q to quit\n'
            f'\nSpeed: {self.speed:.2f} m/s'
            f'\nTurn:  {self.turn:.2f}\n'
        )

    def publish(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.pub.publish(msg)

    def print_status(self):
        self.get_logger().info(
            f'Speed: {self.speed:.2f} m/s | '
            f'Turn: {self.turn:.2f}'
        )


def main():
    rclpy.init()
    node = AckermannTeleop()

    old_settings = termios.tcgetattr(sys.stdin)

    try:
        tty.setcbreak(sys.stdin.fileno())

        while rclpy.ok():

            key = sys.stdin.read(1)

            # 전진
            if key == 'i':
                node.publish(node.speed, 0.0)

            # 후진
            elif key == ',':
                node.publish(-node.speed, 0.0)

            # Ackermann 좌회전
            elif key == 'j':
                node.publish(node.speed, node.turn)

            # 정지
            elif key == 'k':
                node.publish(0.0, 0.0)

            # Ackermann 우회전
            elif key == 'l':
                node.publish(node.speed, -node.turn)

            # 속도 증가
            elif key == 'q':
                node.speed = min(node.speed + node.speed_step, 0.5)
                node.print_status()

            # 속도 감소
            elif key == 'z':
                node.speed = max(node.speed - node.speed_step, 0.1)
                node.print_status()

            # 조향 증가
            elif key == 'w':
                node.turn = min(node.turn + node.turn_step, 1.0)
                node.print_status()

            # 조향 감소
            elif key == 'x':
                node.turn = max(node.turn - node.turn_step, 0.1)
                node.print_status()

            # 종료
            elif key == '\x03':
                break

            rclpy.spin_once(node, timeout_sec=0.01)

    finally:
        node.publish(0.0, 0.0)

        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            old_settings
        )

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
