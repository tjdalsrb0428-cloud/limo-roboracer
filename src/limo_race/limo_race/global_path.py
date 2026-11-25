"""global_path: 저장한 레이스라인을 불러와 global path 로 발행하는 노드.

  ros2 run limo_race global_path --ros-args -p map:=NAME [-p line:=다른.csv]

maps/real/NAME_raceline.csv → /race/raceline (nav_msgs/Path, transient local, RViz 파란 선).
local_planner·controller 가 이 토픽을 받아 기준선으로 쓴다.
"""
import os

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Path
from rclpy.node import Node

from limo_race import common

PARAMS = dict(map='map', line='')


class GlobalPath(Node):
    def __init__(self, prm):
        super().__init__('global_path')
        common.declare(self, prm)
        line = prm['line'] or common.map_base(prm['map']) + '_raceline.csv'
        if not os.path.exists(line):
            raise SystemExit(f'[global_path] 레이스라인 없음: {line} (raceline_generator 먼저)')
        R = np.loadtxt(line, delimiter=',', skiprows=1, ndmin=2)[:, :2]
        self.pub = self.create_publisher(Path, '/race/raceline', common.qos_latched())
        self.pub.publish(common.path_msg(R, stamp=self.get_clock().now().to_msg()))
        L = float(np.hypot(*np.diff(np.vstack([R, R[:1]]), axis=0).T).sum())
        print(f'[global_path] {line}: {len(R)}점, 길이 {L:.2f} m → /race/raceline', flush=True)


def main():
    rclpy.init()
    node = GlobalPath(common.read_params(PARAMS))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        common.quiet_exit()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
