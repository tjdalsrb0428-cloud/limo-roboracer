"""local_planner / controller 공용: race_driver.Race 의 메서드(알고리즘)를 그대로 쓰되, 노드 구성만 새로 한다."""
import math
import os
import shutil
from types import SimpleNamespace

import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from scipy.interpolate import CubicSpline

from limo_race import common

import race_driver as rd  # noqa: E402
from raceline import curvature  # noqa: E402


class RaceBase(rd.Race):
    """rd.Race 를 상속하지만 __init__ 은 Node 만 초기화(타이머·구독은 각 노드가 만든다)."""

    def init_node(self, name, prm):
        Node.__init__(self, name, parameter_overrides=[
            __import__('rclpy').parameter.Parameter('use_sim_time', value=prm['sim'])])
        common.declare(self, prm)
        self.a = SimpleNamespace(odom=False, global_only=prm.get('global_only', False), go=prm.get('go', False))
        self.v_meas = 0.0
        self.m2o = None
        self.t_m2o = 0.0
        self.go = self.a.go
        self.idx = None
        self.stat = {}
        self.t_log = 0.0

    def set_line(self, R):
        """레이스라인 점(N×2) → 주기 스플라인, 0.05 m 간격 (race_driver.Race.__init__ 과 같다)."""
        Q = np.vstack([R, R[:1]])
        s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(Q, axis=0).T))])
        self.L = s[-1]
        self.cs = CubicSpline(s, Q, bc_type='periodic')
        self.ds = 0.05
        self.s = np.arange(0, self.L, self.ds)
        self.R = self.cs(self.s)
        d1 = self.cs(self.s, 1)
        self.T = d1 / np.hypot(d1[:, 0], d1[:, 1])[:, None]
        self.N = np.stack([-self.T[:, 1], self.T[:, 0]], 1)
        self.kap = curvature(self.R)
        self.n = len(self.s)


def wait_map_line(tag, need_map=True):
    """/map, /map_traj, /race/raceline 을 받아 (지도 yaml 임시 경로, 궤적 csv 또는 None, 레이스라인 N×2, tmp)."""
    specs = {'line': ('/race/raceline', Path)}
    if need_map:
        specs.update(map=('/map', OccupancyGrid), traj=('/map_traj', Path))
    print(f'[{tag}] {", ".join(v[0] for v in specs.values())} 대기...', flush=True)
    got = common.wait_topics(specs, what=f'[{tag}] ')
    if got is None:
        return None
    R = common.path_array(got['line'])
    if not need_map:
        return None, None, R, None
    tmp = common.tmp_dir(tag)
    T = common.path_array(got['traj'], with_yaw=True)
    yml = common.write_map(got['map'], os.path.join(tmp, 'map'), T if len(T) else None)
    traj = os.path.join(tmp, 'map_traj.csv') if len(T) else None
    return yml, traj, R, tmp


def cleanup(tmp):
    if tmp:
        shutil.rmtree(tmp, ignore_errors=True)


def stop_tf(node):
    """TF 전용 스레드 종료(안 하면 rclpy 종료 후에도 대기 상태로 남아 프로세스가 안 끝남 → launch 가 SIGTERM)."""
    tfl = getattr(node, 'tfl', None)
    if tfl is not None and hasattr(tfl, 'executor'):
        try:
            tfl.executor.shutdown(timeout_sec=1.0)
            tfl.dedicated_listener_thread.join(timeout=1.0)
        except Exception:
            pass


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))
