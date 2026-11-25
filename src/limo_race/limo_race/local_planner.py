"""local_planner: 장애물 인식 + local path 노드 (race_driver 의 Frenet lattice 알고리즘).

  ros2 run limo_race local_planner --ros-args -p robot:=limo_pro [-p vmax:=0.5 -p lite:=false -p slow:=false -p global_only:=false -p sim:=true]

입력: /map·/map_traj(map_server), /race/raceline(global_path), /scan, /odom, TF map→odom(localizer)·odom→base_link,
      /race/go(출발), /race/control_event(controller 가 알리는 막힘·탈출 결과)
출력: /race/plan (limo_race_msgs/RacePlan: 경로 + 속도 + 상태 → controller), /race/local_path (RViz 초록 선)
막힘 대처(탈출 경로 Hybrid A*, 조향 후진)도 여기서 정하고 RacePlan 의 mode 로 controller 에 보낸다.
"""
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
import tf2_ros
from limo_race_msgs.msg import ControlEvent, RacePlan
from nav_msgs.msg import Odometry, Path
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from scipy import ndimage
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty

from limo_race import common
from limo_race.planner_base import RaceBase, cleanup, rd, stop_tf, wait_map_line

from raceline import GridMap  # noqa: E402

PARAMS = dict(robot='limo_pro', calib='auto', sim=False, vmax=1.0, lite=False, slow=False, global_only=False)
MAN_TIMEOUT = 30.0      # controller 가 탈출 결과를 안 알려 오면 이 시간 뒤 해제


class _StopPub:
    """race_driver 코드의 self.pub.publish(Twist()) (탈출 탐색 직전 정지) → RacePlan STOP 발행."""

    def __init__(self, node):
        self.node = node

    def publish(self, _msg):
        self.node.send_plan(RacePlan.STOP)


class LocalPlanner(RaceBase):
    def __init__(self, prm, yml, traj, R):
        self.init_node('local_planner', prm)
        self.m = GridMap(yml, np.loadtxt(traj, delimiter=',', skiprows=1)[:, :2] if traj else None,
                         drop_blobs=True, unknown_free=True)
        bm = self.m.blobs & ~ndimage.binary_erosion(self.m.blobs)
        rr, cc = np.nonzero(bm)
        self.blob_pts = self.m.xy(rr, cc) if len(rr) else np.zeros((0, 2))
        self.set_line(R)
        self.get_logger().info(f'레이스라인 {self.L:.2f}m, 최대곡률 {np.abs(self.kap).max():.2f}')
        cg_sens = MutuallyExclusiveCallbackGroup()
        cg_ctrl = MutuallyExclusiveCallbackGroup()
        cg_plan = MutuallyExclusiveCallbackGroup()       # 계획 타이머와 이벤트 처리는 한 줄로(서로 섞이지 않게)
        self.pub = _StopPub(self)
        self.pub_plan = self.create_publisher(RacePlan, '/race/plan', 5)
        self.pub_path = self.create_publisher(Path, '/race/local_path', 1)
        T = rd.PROF['topics']
        self.create_subscription(LaserScan, T['scan'], self.on_scan, qos_profile_sensor_data, callback_group=cg_sens)
        self.create_subscription(Odometry, T['odom'], self.on_odom, 10, callback_group=cg_ctrl)
        self.create_subscription(Empty, '/race/go', self.on_go, 10, callback_group=cg_ctrl)
        self.create_subscription(ControlEvent, '/race/control_event', self.on_event, 10, callback_group=cg_plan)
        self.man = None
        self.t_man = 0.0
        self.blocked_t = None
        self.obs = []
        self.virt = []
        self.plan = None
        self.prev_o = None
        self.sides = []
        self.obs_P = np.zeros((0, 2))
        self.stuck_t = None
        self.recover_until = 0.0
        self.recover_steer = 0.0
        self.raw = []
        self.raw_dt = None
        self.tight_static = False
        self.create_timer(0.1, self.plan_step, callback_group=cg_plan)
        self.create_timer(0.1, self.on_update_m2o, callback_group=cg_ctrl)
        self.tfb = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfb, self, spin_thread=True)

    def on_update_m2o(self):
        self.update_m2o()

    # ---------- 발행 ----------
    def send_plan(self, mode, P=None, v=None, tight=None, A=None):
        m = RacePlan()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.mode = mode
        if P is not None:
            m.x, m.y = P[:, 0].astype(float).tolist(), P[:, 1].astype(float).tolist()
            m.speed = np.asarray(v, float).tolist()
            m.tight = np.asarray(tight, float).tolist()
        if A is not None:          # 탈출 경로 (x, y, heading, direction)
            m.x, m.y = A[:, 0].tolist(), A[:, 1].tolist()
            m.heading, m.direction = A[:, 2].tolist(), A[:, 3].tolist()
        if mode == RacePlan.REVERSE:
            m.reverse_steer = float(self.recover_steer)
            m.reverse_time = float(max(self.recover_until - self.now(), 0.0))
        st = self.stat
        m.blocked = bool(st.get('blocked', False))
        m.tight_mode = bool(st.get('tight', False))
        m.reached = int(st.get('kk') if st.get('kk') is not None else -1)
        m.n_obstacle = int(st.get('nobs', 0))
        m.offset = float(st.get('o0', 0.0))
        m.clearance = float(st.get('cmin', 0.0))
        m.plan_ms = float(st.get('pt') or 0.0)
        m.line_index = int(self.idx) if self.idx is not None else -1
        self.pub_plan.publish(m)

    # ---------- 계획 ----------
    def plan_step(self):
        if self.man is not None and self.now() - self.t_man > MAN_TIMEOUT:
            self.get_logger().warn('탈출 결과 응답 없음 → 해제')
            self.man = None
        self.stat['pt'] = None
        man0 = self.man
        super().plan_step()
        if self.man is not None or man0 is not None:
            return
        if self.stat.get('pt') is None:          # 자세 없음 등으로 계획 안 함
            return
        if self.plan is None:
            self.send_plan(RacePlan.STOP)
        else:
            pl = self.plan
            self.send_plan(RacePlan.NORMAL, pl['P'], pl['v'], pl['tight'])

    def try_maneuver(self, x, y, yaw, i0, kk):
        ok = super().try_maneuver(x, y, yaw, i0, kk)
        if ok:
            self.t_man = self.now()
            A = np.vstack(self.man['segs'])      # 구간 경계 점은 방향만 바뀐 복사본 → controller 가 같은 구간으로 다시 나눔
            self.send_plan(RacePlan.MANEUVER, A=A)
        return ok

    def start_reverse(self, tnow, pref):
        super().start_reverse(tnow, pref)
        self.send_plan(RacePlan.REVERSE)

    # ---------- controller 이벤트 ----------
    def on_event(self, e):
        tnow = self.now()
        if e.type == ControlEvent.STUCK:
            self.get_logger().warn('controller: 정지 감지(명령은 가는데 안 움직임)')
            self.mark_contact(1.0)
            p = self.pose()
            if (p is not None and self.idx is not None and not self.a.global_only
                    and self.try_maneuver(*p, self.idx, 0)):
                return
            kap = float(e.value)
            self.start_reverse(tnow, math.copysign(1.0, kap) if abs(kap) > 0.1 else 0.0)
        elif e.type == ControlEvent.MANEUVER_STALLED:
            self.mark_contact(float(e.value) or 1.0)
            self.man = None
            self.start_reverse(tnow, 0.0)
        elif e.type == ControlEvent.MANEUVER_DONE:
            self.get_logger().info('탈출 기동 완료')
            self.man = None
            self.prev_o = None


def main():
    rclpy.init()
    prm = common.read_params(PARAMS)
    prof, _ = common.load_robot(prm['robot'], prm['calib'], log=lambda s: print('[local_planner] ' + s, flush=True))
    common.configure_driver(rd, prof, prm['vmax'], prm['lite'], prm['slow'])
    r = wait_map_line('local_planner')
    if r is None:
        return
    yml, traj, R, tmp = r
    try:
        node = LocalPlanner(prm, yml, traj, R)
    finally:
        cleanup(tmp)
    print(f'[local_planner] 시작: V_MAX {rd.V_MAX}, 정거장 {rd.N_ST}, 횡후보 {len(rd.OFFS)}'
          f'{", global_only(회피 없음)" if prm["global_only"] else ""}{", slow" if prm["slow"] else ""}', flush=True)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        common.quiet_exit()
        ex.shutdown(timeout_sec=0.5)
        stop_tf(node)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
