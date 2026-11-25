"""localizer: 위치추정 노드 (파티클 필터, scripts/real/localizer.py 알고리즘).

  ros2 run limo_race localizer --ros-args -p robot:=limo_pro [-p init:=[x,y,yaw]] [-p sim:=true]

/map + /map_traj(map_server) 를 받아 시작. 초기 자세 = /map_traj 마지막 점(매핑 끝 = 출발 pose) 또는 init.
출력: TF map→odom, /pf/pose. RViz "2D Pose Estimate"(/initialpose) 로 재초기화.
"""
import math
import os
import shutil
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import OccupancyGrid, Path

from limo_race import common

import localizer as lz  # noqa: E402  (scripts/real/localizer.py)

PARAMS = dict(robot='limo_pro', calib='auto', sim=False, init=[0.0], a_trans=0.0, a_rot=0.0)


def main():
    rclpy.init()
    prm = common.read_params(PARAMS)
    prof, _ = common.load_robot(prm['robot'], prm['calib'], log=lambda s: print('[localizer] ' + s, flush=True))
    if prm['a_trans'] > 0:
        lz.A_TRANS = prm['a_trans']
    if prm['a_rot'] > 0:
        lz.A_ROT = prm['a_rot']
    print('[localizer] /map, /map_traj 대기...', flush=True)
    got = common.wait_topics({'map': ('/map', OccupancyGrid), 'traj': ('/map_traj', Path)}, what='[localizer] ')
    if got is None:
        return
    T = common.path_array(got['traj'], with_yaw=True)
    if len(prm['init']) >= 3:
        init = [float(v) for v in prm['init'][:3]]
    elif len(T):
        init = list(T[-1])
    else:
        init = [0.0, 0.0, 0.0]
        print('[localizer] 출발 pose 모름 → (0,0,0). RViz 2D Pose Estimate 로 지정할 것', flush=True)
    tmp = common.tmp_dir('loc')
    yml = common.write_map(got['map'], os.path.join(tmp, 'map'))
    n = lz.Localizer(prof, yml, init, prm['sim'])
    shutil.rmtree(tmp, ignore_errors=True)
    common.declare(n, prm)
    print(f'[localizer] 초기 ({init[0]:.2f}, {init[1]:.2f}, {math.degrees(init[2]):.0f}°), '
          f'파티클 {lz.N_PART}, 빔 {lz.N_BEAMS}', flush=True)
    t_log = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.01)
            n.process()
            if time.time() - t_log > 2.0:
                t_log = time.time()
                sd = float(np.sqrt(np.average((n.p[:, 0] - n.est[0]) ** 2 + (n.p[:, 1] - n.est[1]) ** 2, weights=n.w)))
                tp = 1000 * np.mean(n.t_proc[-20:]) if n.t_proc else 0
                print(f'  pose=({n.est[0]:.2f},{n.est[1]:.2f},{math.degrees(n.est[2]):.0f}°) 퍼짐 {sd:.3f} m '
                      f'점수 {n.score:.2f} 갱신 {n.nupd} 처리 {tp:.1f} ms', flush=True)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        common.quiet_exit()
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
