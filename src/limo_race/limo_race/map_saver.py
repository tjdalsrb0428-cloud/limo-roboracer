"""map_saver: 지도 저장 노드.

  ros2 run limo_race map_saver --ros-args -p map:=NAME
  저장:  ros2 service call /map_saver/save std_srvs/srv/Trigger

/map_saver/save 를 받으면 mapper 에 /mapper/finalize(루프 닫힘 보정)를 요청하고,
최종 /map + /map_traj 를 받아 maps/real/NAME.pgm / NAME.yaml / NAME_traj.csv 로 저장한다(이름 = 파라미터 map).
mapper 없이 실행하면 지금 발행 중인 /map + /map_traj 를 그대로 저장한다(map_server 지도 복사 등).
"""
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from limo_race import common

PARAMS = dict(map='map')


class MapSaver(Node):
    def __init__(self, prm):
        super().__init__('map_saver')
        common.declare(self, prm)
        self.base = common.map_base(prm['map'])
        self.cg = ReentrantCallbackGroup()
        self.cli = self.create_client(Trigger, '/mapper/finalize', callback_group=self.cg)
        self.create_service(Trigger, '/map_saver/save', self.on_save, callback_group=self.cg)
        self.busy = False

    def on_save(self, req, res):
        if self.busy:
            res.success, res.message = False, '저장 중'
            return res
        self.busy = True
        try:
            res.success, res.message = self.save()
        except Exception as e:
            res.success, res.message = False, f'오류: {e}'
        self.busy = False
        self.get_logger().info(res.message) if res.success else self.get_logger().error(res.message)
        return res

    def save(self):
        warn = ''
        if self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('mapper 에 루프 닫힘 보정 요청 (/mapper/finalize)...')
            fut = self.cli.call_async(Trigger.Request())
            t0 = time.time()
            while not fut.done() and time.time() - t0 < 120:
                time.sleep(0.05)
            if not fut.done():
                return False, 'mapper 응답 없음(120 s)'
            r = fut.result()
            if not r.success:
                return False, f'mapper 실패: {r.message}'
            if r.message.startswith('경고'):
                warn = '경고: 루프 보정 없이 저장됨 → 매핑 계속 중(출발점 근처로 가서 다시 저장하면 덮어씀) | '
        else:
            self.get_logger().warn('mapper 없음 → 지금 발행 중인 /map, /map_traj 저장')
        # 최종 지도·궤적은 transient local → 새로 구독하면 마지막(최종) 것을 받는다
        got = {}
        subs = [self.create_subscription(OccupancyGrid, '/map', lambda m: got.__setitem__('map', m),
                                         common.qos_latched(), callback_group=self.cg),
                self.create_subscription(Path, '/map_traj', lambda m: got.__setitem__('traj', m),
                                         common.qos_latched(), callback_group=self.cg)]
        t0 = time.time()
        while len(got) < 2 and time.time() - t0 < 10:
            time.sleep(0.05)
        for s in subs:
            self.destroy_subscription(s)
        if 'map' not in got:
            return False, '/map 을 못 받음'
        T = common.path_array(got['traj'], with_yaw=True) if 'traj' in got else None
        yml = common.write_map(got['map'], self.base, T)
        msg = f'저장: {yml} ({got["map"].info.width}×{got["map"].info.height} 칸)'
        if T is not None and len(T):
            msg += f', 궤적 {len(T)}점, 출발 pose=({T[-1, 0]:.2f}, {T[-1, 1]:.2f}, {T[-1, 2]:.2f})'
        else:
            msg += ', 궤적 없음(/map_traj)'
        return True, warn + msg


def main():
    rclpy.init()
    prm = common.read_params(PARAMS)
    node = MapSaver(prm)
    print(f'[map_saver] {node.base}.* 로 저장. 저장 명령: ros2 service call /map_saver/save std_srvs/srv/Trigger',
          flush=True)
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        common.quiet_exit()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
