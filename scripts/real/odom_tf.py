#!/usr/bin/env python3
"""자체 오도메트리: 바퀴 속도(/odom twist, Pro 버그 역산) + IMU 자이로 z → TF odom→base_link (+ /odom_race).

드라이버 odom의 위치(pose)는 쓰지 않는다(Pro는 계산 버그). 로봇에서 EKF가 이미 odom→base_link TF를 내면
TF 트리 충돌(base_link 부모 2개) → 그때는 이걸 켜지 말고 EKF TF를 쓴다(check: tf2_echo odom base_link).
  python3 odom_tf.py --robot limo_pro [--sim]
시작 직후 2초 정지 상태로 자이로 편향을 잰다(그동안 움직이지 말 것).
"""
import argparse
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from robot_profile import add_args, load_profile
from rt_common import odom_speed


class OdomTf(Node):
    def __init__(self, prof, sim):
        super().__init__('odom_tf', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=sim)])
        self.prof = prof
        self.fo, self.fb = prof['frames']['odom'], prof['frames']['base']
        self.oscale = float(prof['calib'].get('odom_scale', 1.0))
        self.v = 0.0
        self.x = self.y = self.yaw = 0.0
        self.bias, self.bias_n, self.bias_sum = None, 0, 0.0
        self.t_imu = None
        self.t0 = None
        self.tfb = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, '/odom_race', 20)
        self.create_subscription(Odometry, prof['topics']['odom'], self.on_odom, 50)
        self.create_subscription(Imu, prof['topics']['imu'], self.on_imu, qos_profile_sensor_data)
        self.n = 0

    def on_odom(self, m):
        self.v = odom_speed(m) * self.oscale

    def on_imu(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        gz = m.angular_velocity.z
        if self.t0 is None:
            self.t0 = t
        if self.bias is None:                    # 처음 2 s: 정지 편향 측정
            self.bias_sum += gz
            self.bias_n += 1
            if t - self.t0 >= 2.0:
                self.bias = self.bias_sum / max(1, self.bias_n)
                self.get_logger().info(f'자이로 편향 {self.bias:+.5f} rad/s → 적분 시작')
            self.t_imu = t
            self.publish(m.header.stamp, 0.0)
            return
        dt = t - self.t_imu
        self.t_imu = t
        if dt <= 0 or dt > 0.2:
            return
        w = gz - self.bias
        mid = self.yaw + w * dt / 2
        self.x += self.v * math.cos(mid) * dt
        self.y += self.v * math.sin(mid) * dt
        self.yaw = math.atan2(math.sin(self.yaw + w * dt), math.cos(self.yaw + w * dt))
        self.n += 1
        if self.n % 2 == 0:                      # IMU 100 Hz → TF 50 Hz
            self.publish(m.header.stamp, w)

    def publish(self, stamp, w):
        qz, qw = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.fo
        tf.child_frame_id = self.fb
        tf.transform.translation.x, tf.transform.translation.y = self.x, self.y
        tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
        self.tfb.sendTransform(tf)
        o = Odometry()
        o.header = tf.header
        o.child_frame_id = self.fb
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        o.twist.twist.linear.x, o.twist.twist.angular.z = self.v, w
        self.pub.publish(o)


def main():
    ap = add_args(argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter))
    ap.add_argument('--sim', action='store_true', help='시뮬(use_sim_time)')
    a = ap.parse_args()
    rclpy.init()
    n = OdomTf(load_profile(a.robot, a.calib), a.sim)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
