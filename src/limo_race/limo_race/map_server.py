"""map_server: 저장한 지도를 불러와 발행하는 노드.

  ros2 run limo_race map_server --ros-args -p map:=NAME

maps/real/NAME.yaml(+pgm) → /map (OccupancyGrid), NAME_traj.csv → /map_traj (Path, 마지막 점 = 출발 pose).
둘 다 transient local(늦게 켠 노드·RViz 도 받음). raceline_generator, localizer, local_planner 가 구독한다.
"""
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node

from limo_race import common

PARAMS = dict(map='map')


class MapServer(Node):
    def __init__(self, prm):
        super().__init__('map_server')
        common.declare(self, prm)
        base = common.map_base(prm['map'])
        yml = base + '.yaml'
        if not os.path.exists(yml):
            raise SystemExit(f'[map_server] 지도 없음: {yml}')
        self.pub_map = self.create_publisher(OccupancyGrid, '/map', common.qos_latched())
        self.pub_traj = self.create_publisher(Path, '/map_traj', common.qos_latched())
        g = common.grid_msg_from_file(yml)
        g.header.stamp = self.get_clock().now().to_msg()
        self.pub_map.publish(g)
        msg = f'[map_server] {yml} ({g.info.width}×{g.info.height} 칸, {g.info.resolution:.3f} m)'
        tr = base + '_traj.csv'
        if os.path.exists(tr):
            T = common.read_traj(tr)
            self.pub_traj.publish(common.path_msg(T[:, :2], T[:, 2], stamp=g.header.stamp))
            msg += f', 궤적 {len(T)}점, 출발 pose=({T[-1, 0]:.2f}, {T[-1, 1]:.2f}, {T[-1, 2]:.2f})'
        else:                       # 빈 궤적이라도 발행(구독 노드가 기다리지 않게)
            self.pub_traj.publish(Path(header=g.header))
            self.get_logger().warn(f'{tr} 없음 → 출발 pose 를 모름(localizer 는 -p init:=[x,y,yaw] 필요)')
        print(msg, flush=True)


def main():
    rclpy.init()
    prm = common.read_params(PARAMS)
    node = MapServer(prm)
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
