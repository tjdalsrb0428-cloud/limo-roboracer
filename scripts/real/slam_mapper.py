#!/usr/bin/env python3
"""SLAM 매핑(Cartographer 기본 / slam_toolbox) → raceline.py용 지도·궤적 저장. odom_mapper.py의 실차판.

  python3 slam_mapper.py maps/real/NAME --robot limo_pro [--keys] [--own-odom] [--sim]
  1) 조종기(SWB 가운데)나 --keys(키보드)로 트랙 한 바퀴를 천천히 돈다(0.3~0.5 m/s).
  2) 출발 위치·방향에 차를 세운다(마지막 자세 = 출발 pose).
  3) Ctrl+C → (Cartographer는 궤적 마무리·최적화 후) NAME.pgm/.yaml + NAME_traj.csv 저장, SLAM 종료.
  다음: python3 ../race/raceline.py maps/real/NAME.yaml

--own-odom: 로봇에 odom→base_link TF가 없을 때 odom_tf.py(바퀴 속도+IMU)를 같이 띄운다(EKF TF가 있으면 쓰지 말 것).
저장 형식은 odom_mapper.py와 같다(trinary 0/205/254, 0.025 m). 궤적·지도는 map 프레임(= SLAM 시작 위치 원점).
"""
import argparse
import math
import os
import select
import signal
import subprocess
import sys
import termios
import time
import tty

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from tf2_ros import Buffer, TransformListener

from robot_profile import CmdSender, add_args, load_profile
from rt_common import yaw_of

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.normpath(os.path.join(HERE, '..', '..'))
SLAM_CFG = os.path.join(WS, 'config', 'slam', 'slam_toolbox.yaml')
CARTO_DIR = os.path.join(WS, 'config', 'cartographer')
HELP = ('키보드: i 전진, u/o 좌/우 전진, , 후진(m/. 좌/우 후진), k 정지, q/z 속도 ±, w/x 조향 ±   '
        'Ctrl+C = 정지 + 저장')


class SlamMapper(Node):
    def __init__(self, prof, sim, keys):
        super().__init__('slam_mapper', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=sim)])
        self.prof = prof
        self.fm, self.fb = 'map', prof['frames']['base']
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self, spin_thread=False)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.map = None
        self.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self, 'map', m), qos)
        self.traj = []
        self.occ = 65
        self.cmd = CmdSender(self, prof) if keys else None
        self.create_timer(0.1, self.track)

    def pose(self):
        try:
            t = self.buf.lookup_transform(self.fm, self.fb, rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        return t.transform.translation.x, t.transform.translation.y, yaw_of(q)

    def track(self):
        p = self.pose()
        if p is None:
            return
        if not self.traj or math.hypot(p[0] - self.traj[-1][0], p[1] - self.traj[-1][1]) > 0.02:
            self.traj.append(p)

    def save(self, base):
        m = self.map
        if m is None:
            print('/map 수신 없음 - 저장 안 함 (SLAM 로그 확인)', flush=True)
            return False
        w, h, res = m.info.width, m.info.height, m.info.resolution
        g = np.asarray(m.data, dtype=np.int16).reshape(h, w)[::-1]      # 위쪽이 +y
        img = np.full(g.shape, 205, np.uint8)
        img[(g >= 0) & (g < 35)] = 254                                  # 점유확률 %: 빈칸 < 35
        img[g >= self.occ] = 0                                          # 벽(Cartographer는 벽 칸 확률이 55~70 % 정도라 55)
        known = np.where(img != 205)
        r0, r1 = max(known[0].min() - 10, 0), min(known[0].max() + 10, img.shape[0])
        c0, c1 = max(known[1].min() - 10, 0), min(known[1].max() + 10, img.shape[1])
        img = img[r0:r1, c0:c1]
        ox = m.info.origin.position.x + c0 * res
        oy = m.info.origin.position.y + (h - r1) * res
        os.makedirs(os.path.dirname(base) or '.', exist_ok=True)
        name = os.path.basename(base)
        with open(base + '.pgm', 'wb') as f:                            # PIL 없이 P5 저장
            f.write(f'P5\n{img.shape[1]} {img.shape[0]}\n255\n'.encode())
            f.write(img.tobytes())
        with open(base + '.yaml', 'w') as f:
            f.write(f'image: {name}.pgm\nmode: trinary\nresolution: {res}\norigin: [{ox:.3f}, {oy:.3f}, 0]\n'
                    f'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
        traj = list(self.traj)
        p = self.pose()
        if p is not None:
            traj.append(p)                                              # 마지막 행 = 출발 pose
        with open(base + '_traj.csv', 'w') as f:
            f.write('x,y,yaw\n')
            for x, y, yw in traj:
                f.write(f'{x:.4f},{y:.4f},{yw:.4f}\n')
        x, y, yw = traj[-1] if traj else (0, 0, 0)
        print(f'지도 저장: {base}.yaml ({img.shape[1]}×{img.shape[0]} 칸, {res} m), 궤적 {len(traj)}점, '
              f'출발 pose=({x:.2f},{y:.2f},{yw:.2f})', flush=True)
        return True


def main():
    ap = add_args(argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter))
    ap.add_argument('out', nargs='?', default=os.path.join(WS, 'maps', 'real', 'map'))
    ap.add_argument('--keys', action='store_true', help='키보드 조종(터미널)')
    ap.add_argument('--own-odom', action='store_true', help='odom_tf.py 같이 실행(로봇에 odom TF 없을 때)')
    ap.add_argument('--sim', action='store_true', help='시뮬(use_sim_time)')
    ap.add_argument('--backend', choices=['cartographer', 'slam_toolbox'], default='cartographer')
    ap.add_argument('--slam-cfg', default=None, help='설정 파일(cartographer: lua, slam_toolbox: yaml)')
    ap.add_argument('--odom-topic', default='/odom_race',
                    help='Cartographer odom 입력(기본 /odom_race = odom_tf.py; Pro 드라이버 /odom 은 위치 버그)')
    a = ap.parse_args()
    base = os.path.abspath(a.out[:-5] if a.out.endswith('.yaml') else a.out)
    prof = load_profile(a.robot, a.calib)

    procs = []
    st = 'true' if a.sim else 'false'
    if a.own_odom:
        cmd = [sys.executable, os.path.join(HERE, 'odom_tf.py'), '--robot', a.robot] + \
              (['--calib', a.calib] if a.calib else []) + (['--sim'] if a.sim else [])
        procs.append(subprocess.Popen(cmd, start_new_session=True))
        time.sleep(2.5)                                                # 자이로 편향 측정 동안 정지
    if a.backend == 'slam_toolbox':
        procs.append(subprocess.Popen(
            ['ros2', 'run', 'slam_toolbox', 'async_slam_toolbox_node', '--ros-args',
             '--params-file', a.slam_cfg or SLAM_CFG, '-p', f'use_sim_time:={st}', '--log-level', 'warn'],
            start_new_session=True))
    else:
        lua = a.slam_cfg or os.path.join(CARTO_DIR, 'limo_race_2d.lua')
        procs.append(subprocess.Popen(
            ['ros2', 'run', 'cartographer_ros', 'cartographer_node',
             '-configuration_directory', os.path.dirname(os.path.abspath(lua)),
             '-configuration_basename', os.path.basename(lua), '--ros-args',
             '-p', f'use_sim_time:={st}', '-r', f'odom:={a.odom_topic}', '-r', 'scan:=' + prof['topics']['scan'],
             '-r', 'imu:=' + prof['topics']['imu'], '--log-level', 'warn'],
            start_new_session=True))
        procs.append(subprocess.Popen(
            ['ros2', 'run', 'cartographer_ros', 'cartographer_occupancy_grid_node',
             '-resolution', '0.025', '-publish_period_sec', '1.0', '--ros-args', '-p', f'use_sim_time:={st}'],
            start_new_session=True))

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = SlamMapper(prof, a.sim, a.keys)
    node.occ = 55 if a.backend == 'cartographer' else 65
    tele = a.keys and sys.stdin.isatty()
    old = None
    v = steer = 0.0
    speed, sdeg = 0.4, 0.35
    keys = {'i': (1, 0), 'u': (1, 1), 'o': (1, -1), 'j': (1, 1), 'l': (1, -1), 'k': (0, 0),
            ',': (-1, 0), 'm': (-1, -1), '.': (-1, 1)}
    if tele:
        print(HELP, flush=True)
        old = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
    print(f'[slam_mapper] robot={prof["name"]} 저장 위치 {base}.*  (Ctrl+C = 저장)', flush=True)
    t_pub, t_log = 0.0, time.time()
    stop = {'f': False}
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__('f', True))
    try:
        while rclpy.ok() and not stop['f']:
            rclpy.spin_once(node, timeout_sec=0.02)
            if tele and select.select([sys.stdin], [], [], 0.0)[0]:
                c = sys.stdin.read(1)
                if c in keys:
                    s1, s2 = keys[c]
                    v, steer = s1 * speed, s2 * sdeg * (1 if s1 >= 0 else -1)
                elif c in 'qz':
                    speed = min(0.8, speed + 0.1) if c == 'q' else max(0.1, speed - 0.1)
                elif c in 'wx':
                    sdeg = min(0.5, sdeg + 0.05) if c == 'w' else max(0.1, sdeg - 0.05)
                if c in 'qzwx':
                    print(f'  속도 {speed:.1f} m/s, 조향 {sdeg:.2f} rad', flush=True)
            if node.cmd and time.time() - t_pub > 0.1:
                node.cmd.send(v, steer)
                t_pub = time.time()
            if time.time() - t_log > 5.0:
                t_log = time.time()
                dead = [p.args[2] if len(p.args) > 2 else p.args for p in procs if p.poll() is not None]
                if dead:
                    print(f'  ! 종료된 프로세스: {dead} — 위 오류 확인', flush=True)
                p = node.pose()
                mi = node.map.info if node.map else None
                print(f'  pose={tuple(round(x, 2) for x in p) if p else "없음(TF map→base_link 대기)"} '
                      f'map={f"{mi.width}×{mi.height}" if mi else "없음"} 궤적 {len(node.traj)}점', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        if old:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        if node.cmd:
            node.cmd.stop(10)
        wait = 1.5
        if a.backend == 'cartographer':                               # 궤적 마무리 → 전체 최적화(루프 닫힘)
            print('Cartographer 최종 최적화 중...', flush=True)
            try:
                subprocess.run(['ros2', 'service', 'call', '/finish_trajectory',
                                'cartographer_ros_msgs/srv/FinishTrajectory', '{trajectory_id: 0}'],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=30)
                wait = 4.0
            except subprocess.TimeoutExpired:
                print('  최종 최적화 응답 없음(Cartographer가 죽었을 수 있음) → 현재 지도로 저장', flush=True)
        # 최신 지도 받기(1 s 주기 발행)
        t0 = time.time()
        while time.time() - t0 < wait:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.save(base)
        for p in procs:
            try:
                os.killpg(p.pid, signal.SIGINT)
                p.wait(timeout=5)
            except Exception:
                try:
                    os.killpg(p.pid, signal.SIGKILL)
                except Exception:
                    pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
