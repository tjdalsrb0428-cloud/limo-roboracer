#!/usr/bin/env python3
"""매핑 중 로봇 궤적(map->base_link) 기록: python3 record_traj.py OUT.csv  (Ctrl+C/SIGINT 로 종료)"""
import sys
import math
import rclpy
from rclpy.time import Time
import tf2_ros


def main():
    rclpy.init()
    n = rclpy.create_node('record_traj', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    b = tf2_ros.Buffer()
    tf2_ros.TransformListener(b, n)
    f = open(sys.argv[1], 'w')
    f.write('x,y,yaw\n')
    last = [None]

    def tick():
        try:
            t = b.lookup_transform('map', 'base_link', Time())
        except Exception:
            return
        x, y = t.transform.translation.x, t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if last[0] is None or math.hypot(x - last[0][0], y - last[0][1]) > 0.03:
            f.write(f'{x:.4f},{y:.4f},{yaw:.4f}\n')
            f.flush()
            last[0] = (x, y)
    n.create_timer(0.05, tick)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    f.close()


if __name__ == '__main__':
    main()
