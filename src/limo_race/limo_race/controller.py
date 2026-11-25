"""controller: 주행 노드 (pure pursuit, race_driver 의 제어 부분). 모터 명령을 내는 유일한 노드.

  ros2 run limo_race controller --ros-args -p robot:=limo_pro [-p vmax:=0.5 -p sim:=true -p go:=false]

입력: /race/plan(local_planner), /race/raceline(랩 계산), TF map→odom·odom→base_link, /odom(속도), /race/go(출발)
출력: 모터 명령(LIMO1 /cmd_vel Twist, Pro /limo/ack_cmd AckermannDrive), /race/control_event(막힘 → planner),
      /race/car_pose(RViz 차 위치)
출발 전에는 정지 명령만 보낸다. 출발: ros2 run limo_race go  또는  ros2 topic pub --once /race/go std_msgs/msg/Empty
Ctrl+C: 정지 명령(속도 0) 10회 발행 후 종료.
"""
import math
import signal
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from limo_race_msgs.msg import ControlEvent, RacePlan
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Empty

from limo_race import common
from limo_race.planner_base import RaceBase, rd, stop_tf, wait_map_line

from robot_profile import CmdSender  # noqa: E402

PARAMS = dict(robot='limo_pro', calib='auto', sim=False, vmax=1.0, go=False)
HOLD_T = 5.0            # 막힘을 알린 뒤 planner 응답(후진/탈출 경로)을 기다리며 정지하는 최대 시간


class Controller(RaceBase):
    def __init__(self, prm, R):
        self.init_node('controller', prm)
        self.set_line(R)
        cg_ctrl = MutuallyExclusiveCallbackGroup()
        cg_plan = MutuallyExclusiveCallbackGroup()
        self.pub = rd.CmdAdapter(CmdSender(self, rd.PROF))
        self.pub_ev = self.create_publisher(ControlEvent, '/race/control_event', 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/race/car_pose', 1)
        self.create_subscription(Odometry, rd.PROF['topics']['odom'], self.on_odom, 10, callback_group=cg_ctrl)
        self.create_subscription(Empty, '/race/go', self.on_go, 10, callback_group=cg_ctrl)
        self.create_subscription(RacePlan, '/race/plan', self.on_plan, 5, callback_group=cg_plan)
        self.man = None
        self.plan = None
        self.t_go = None
        self.lap_t0 = None
        self.laps = 0
        self.last_idx_for_lap = None
        self.stuck_t = None
        self.recover_until = 0.0
        self.recover_steer = 0.0
        self.hold_until = 0.0
        self.t_event = -1.0
        self.stalled = False
        self.ready = False
        self.t_pose = 0.0
        self.create_timer(0.025, self.control_step, callback_group=cg_ctrl)
        self.tfb = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfb, self, spin_thread=True)

    # ---------- planner 에서 온 경로 ----------
    def on_plan(self, m):
        tnow = self.now()
        newer = Time.from_msg(m.header.stamp).nanoseconds * 1e-9 > self.t_event
        self.stat = dict(blocked=m.blocked, tight=m.tight_mode, kk=m.reached, nobs=m.n_obstacle, o0=m.offset,
                         cmin=m.clearance, pt=m.plan_ms)
        if m.line_index >= 0:
            self.idx = m.line_index
        if m.mode == RacePlan.NORMAL:
            if len(m.x) >= 2:
                self.plan = dict(P=np.stack([m.x, m.y], 1).astype(float), v=np.asarray(m.speed, float),
                                 tight=np.asarray(m.tight, float), t=tnow)
            if newer:
                self.hold_until = 0.0
        elif m.mode == RacePlan.STOP:
            self.plan = None
        elif m.mode == RacePlan.MANEUVER:
            A = np.stack([m.x, m.y, m.heading, m.direction], 1).astype(float)
            cut = np.where(A[1:, 3] != A[:-1, 3])[0] + 1
            self.man = dict(segs=[s for s in np.split(A, cut) if len(s)], k=0, t0=tnow)
            self.plan = None
            self.hold_until = 0.0
        elif m.mode == RacePlan.REVERSE:
            self.recover_until = tnow + m.reverse_time
            self.recover_steer = m.reverse_steer
            self.hold_until = 0.0

    def event(self, typ, value=0.0):
        e = ControlEvent()
        e.header.stamp = self.get_clock().now().to_msg()
        e.type, e.value = typ, float(value)
        self.t_event = self.now()
        self.hold_until = self.t_event + HOLD_T
        self.pub_ev.publish(e)

    # ---------- 제어 ----------
    def control_step(self):
        tnow = self.now()
        if not self.ready and self.plan is not None:
            self.ready = True
            print('READY - 출발:  ros2 run limo_race go   (또는 ros2 topic pub --once /race/go std_msgs/msg/Empty)',
                  flush=True)
        if tnow < self.hold_until and self.man is None and tnow >= self.recover_until:
            self.update_m2o()               # planner 응답 대기: 정지
            self.pub.publish(Twist())
        else:
            super().control_step()
        if tnow - self.t_pose > 0.1:
            self.t_pose = tnow
            p = self.pose()
            if p is not None:
                ps = PoseStamped()
                ps.header.frame_id = 'map'
                ps.header.stamp = self.get_clock().now().to_msg()
                ps.pose.position.x, ps.pose.position.y = float(p[0]), float(p[1])
                ps.pose.orientation.z, ps.pose.orientation.w = math.sin(p[2] / 2), math.cos(p[2] / 2)
                self.pub_pose.publish(ps)

    def check_stuck(self, tnow, vcmd, kap=0.0):
        if vcmd > 0.15 and abs(self.v_meas) < 0.04:
            if self.stuck_t is None:
                self.stuck_t = tnow
            elif tnow - self.stuck_t > 1.2:
                self.stuck_t = None
                self.pub.publish(Twist())
                self.event(ControlEvent.STUCK, kap)
        else:
            self.stuck_t = None

    def follow_maneuver(self, tnow, x, y, yaw):
        self.stalled = False
        super().follow_maneuver(tnow, x, y, yaw)
        if self.man is None and not self.stalled:
            self.event(ControlEvent.MANEUVER_DONE)
            self.hold_until = 0.0

    def mark_contact(self, d):          # 탈출 기동이 멈춤 → planner 에 알림(가상 장애물·후진은 planner 가 정함)
        self.stalled = True
        self.event(ControlEvent.MANEUVER_STALLED, d)

    def start_reverse(self, tnow, pref):
        pass                            # planner 의 REVERSE 명령을 기다린다


def _interrupt(*_):
    raise KeyboardInterrupt


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # Ctrl+C → 정지 명령 보낸 뒤 종료
    signal.signal(signal.SIGINT, _interrupt)        # 부모가 SIGINT 를 무시하게 띄웠어도(백그라운드 셸) 받게
    signal.signal(signal.SIGTERM, _interrupt)       # launch 가 SIGTERM 으로 올려 보내도 정지 명령은 보낸다
    prm = common.read_params(PARAMS)
    prof, _ = common.load_robot(prm['robot'], prm['calib'], log=lambda s: print('[controller] ' + s, flush=True))
    common.configure_driver(rd, prof, prm['vmax'])
    node = None
    try:
        r = wait_map_line('controller', need_map=False)
        if r is None:
            return
        node = Controller(prm, r[2])
        print(f'[controller] 시작: WB {rd.WB}, K_MAX {rd.K_MAX:.2f}, V_MAX {rd.V_MAX}, 명령 {prof["topics"]["cmd"]}',
              flush=True)
        ex = MultiThreadedExecutor(num_threads=3)
        ex.add_node(node)
        try:
            ex.spin()
        finally:
            node.go = False
            ex.shutdown(timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            signal.signal(signal.SIGINT, signal.SIG_IGN)    # 정지 명령 보내는 동안 또 눌러도 끊기지 않게
            rd.stop_car(node)
            print('\n[controller] 정지 명령 발행 후 종료', flush=True)
            time.sleep(0.1)
            stop_tf(node)
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
