"""raceline_generator: 지도 → 레이스라인(global path) 생성 노드.

  ros2 run limo_race raceline_generator --ros-args -p map:=NAME [-p clear:=0.30 -p kmax:=1.25 -p stay:=true]

/map + /map_traj(map_server 또는 mapper 최종 지도)를 받아 raceline.py 알고리즘으로 계산
→ maps/real/NAME_raceline.csv(+ .png 오버레이) 저장, /race/raceline(Path, 파란 선) 발행.
출발 pose = /map_traj 마지막 점(매핑 끝 자세), -p start:=[x,y,yaw] 로 바꿀 수 있다.
stay:=true 면 RViz 확인용으로 계속 발행(Ctrl+C 로 종료), false 면 저장 후 종료.
"""
import os
import shutil
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node

from limo_race import common

import raceline  # noqa: E402

PARAMS = dict(map='map', clear=0.30, kmax=1.25, astar=False, stay=True, start=[0.0])


class RacelineGenerator(Node):
    def __init__(self, prm):
        super().__init__('raceline_generator')
        common.declare(self, prm)
        self.pub = self.create_publisher(Path, '/race/raceline', common.qos_latched())

    def publish(self, R):
        self.pub.publish(common.path_msg(R, stamp=self.get_clock().now().to_msg()))


def main():
    rclpy.init()
    prm = common.read_params(PARAMS)
    base = common.map_base(prm['map'])
    print('[raceline_generator] /map, /map_traj 대기...', flush=True)
    got = common.wait_topics({'map': ('/map', OccupancyGrid), 'traj': ('/map_traj', Path)},
                             what='[raceline_generator] ')
    if got is None:
        return
    tmp = common.tmp_dir('rl')
    try:
        T = common.path_array(got['traj'], with_yaw=True)
        yml = common.write_map(got['map'], os.path.join(tmp, 'map'), T if len(T) else None)
        traj = os.path.join(tmp, 'map_traj.csv') if len(T) else None
        st = prm['start']
        if len(st) >= 3:
            x0, y0, yaw0 = map(float, st[:3])
        elif len(T):
            x0, y0, yaw0 = T[-1]
        else:
            x0 = y0 = yaw0 = 0.0
            print('  출발 pose 없음(/map_traj 비어 있음) → (0, 0, 0). -p start:=[x,y,yaw] 로 지정', flush=True)
        print(f'  출발 pose = ({x0:.2f}, {y0:.2f}, {yaw0:.2f}) → {base}_raceline.csv', flush=True)
        os.makedirs(os.path.dirname(base), exist_ok=True)
        t0 = time.time()
        R, _ = raceline.make(yml, x0, y0, yaw0, prm['clear'], prm['kmax'], base + '_raceline.csv',
                             traj=traj, use_astar=prm['astar'])
        print(f'[raceline_generator] 저장: {base}_raceline.csv, {base}_raceline.png ({time.time() - t0:.1f} s)',
              flush=True)
    except Exception as e:
        print(f'[raceline_generator] 실패: {e}', flush=True)
        rclpy.try_shutdown()
        raise SystemExit(1)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    node = RacelineGenerator(prm)
    node.publish(R)
    try:
        if prm['stay']:
            print('  /race/raceline 발행 중(RViz 파란 선). Ctrl+C 로 종료', flush=True)
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        common.quiet_exit()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
