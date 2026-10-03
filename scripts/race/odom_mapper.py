#!/usr/bin/env python3
"""odom 기반 점유격자 매핑 (위치를 아는 매핑: 시뮬 odom = 정확한 위치)

  python3 scripts/race/odom_mapper.py [OUT_BASE]     (기본 OUT_BASE = maps/exam/map)
    - 터미널에서 직접 실행하면 키보드 조종 가능(u/i/o, ,), Ctrl+C = 정지 + 저장 + 종료
    - Ctrl+C(SIGINT) 도 저장 후 종료
  저장: OUT_BASE.pgm/.yaml + OUT_BASE_traj.csv (마지막 행 = 종료 시점 /odom pose = 출발 pose)
map 프레임 = odom 프레임 (static TF map->odom 항등 발행, /map 1Hz 발행 -> RViz 확인용)
Gazebo 리셋(Ctrl+R) 대비: 스캔을 PENDING 개 늦게 반영하고, 시간 역행/순간이동 감지 시 대기 스캔 폐기
"""
import os
import sys
import math
import time
import signal
import numpy as np

os.environ['ROS_LOCALHOST_ONLY'] = '1'     # 오프라인(Wi-Fi 없음)에서도 통신
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.time import Time  # noqa: E402
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry  # noqa: E402
from geometry_msgs.msg import TransformStamped, Twist  # noqa: E402
from PIL import Image  # noqa: E402
import tf2_ros  # noqa: E402

WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RES = 0.025
SIZE_X, SIZE_Y = 60.0, 60.0      # 출발점 기준 ±30 m (큰 트랙도 잘리지 않게; 저장/발행은 그린 영역만)
L_FREE, L_OCC, L_MIN, L_MAX = -0.4, 0.9, -2.0, 3.5
MISS_LEN = 5.0
PENDING = 5             # 반영 지연 스캔 수(약 0.5s): 리셋 순간의 잘못된 스캔이 지도에 찍히기 전에 버림
JUMP = 0.3              # 연속 스캔 사이 위치 변화가 이 이상이면 순간이동(리셋)으로 판단
SKIP_AFTER = 1.0        # 리셋 감지 후 스캔 무시 시간(벽시계 s)

TELEOP_HELP = '''
  [매핑 조종]  u i o  : 좌전진 / 전진 / 우전진
               m , .  : 좌후진 / 후진 / 우후진
               k : 정지   q/z 속도 +/-   w/x 조향 +/-   Ctrl+C = 매핑 끝(정지 후 저장)
'''


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Mapper(Node):
    def __init__(self, base):
        super().__init__('odom_mapper', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.base = base
        self.tfb = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfb, self)
        self.stf = tf2_ros.StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.rotation.w = 1.0
        self.stf.sendTransform(t)
        self.nx, self.ny = int(SIZE_X / RES), int(SIZE_Y / RES)
        self.L = np.zeros((self.ny, self.nx), np.float32)
        self.origin = None
        self.traj = []
        self.pending = []           # (scan, (lx, ly, lyaw), (bx, by, byaw))
        self.last_stamp = None
        self.last_pose = None
        self.skip_until = 0.0
        self.odom = None
        self.nreset = 0
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), 10)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 1)
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, '/map', q)
        self.create_timer(1.0, self.publish)
        self.nscan = 0

    def reset_detected(self):
        self.pending.clear()
        self.tfb.clear()
        self.skip_until = time.time() + SKIP_AFTER
        self.last_stamp = None
        self.last_pose = None
        self.nreset += 1
        print('  Gazebo 리셋 감지: 직전 스캔 폐기', flush=True)

    def on_scan(self, sc):
        st = Time.from_msg(sc.header.stamp).nanoseconds * 1e-9
        if self.last_stamp is not None and st < self.last_stamp - 0.2:      # 시뮬 시간 역행 = 리셋
            self.reset_detected()
            return
        if time.time() < self.skip_until:
            return
        try:
            t = self.tfb.lookup_transform('odom', sc.header.frame_id, Time.from_msg(sc.header.stamp))
            tb = self.tfb.lookup_transform('odom', 'base_link', Time.from_msg(sc.header.stamp))
        except Exception:
            return
        tr = t.transform
        lp = (tr.translation.x, tr.translation.y, yaw_of(tr.rotation))
        bp = (tb.transform.translation.x, tb.transform.translation.y, yaw_of(tb.transform.rotation))
        if self.last_pose is not None and math.hypot(bp[0] - self.last_pose[0], bp[1] - self.last_pose[1]) > JUMP:
            self.reset_detected()                                            # 순간이동 = 리셋
            return
        self.last_stamp = st
        self.last_pose = bp
        self.pending.append((sc, lp, bp))
        if len(self.pending) > PENDING:
            self.integrate(*self.pending.pop(0))

    def integrate(self, sc, lp, bp):
        lx, ly, lyaw = lp
        self.traj.append(bp)
        if self.origin is None:
            self.origin = (round(lx - SIZE_X / 2, 1), round(ly - SIZE_Y / 2, 1))
        ox, oy = self.origin
        r = np.asarray(sc.ranges, dtype=float)
        a = sc.angle_min + np.arange(len(r)) * sc.angle_increment + lyaw
        hit = np.isfinite(r) & (r < sc.range_max - 0.05) & (r > sc.range_min)
        rr = np.where(hit, r, MISS_LEN)
        rr = np.where(np.isfinite(rr), rr, MISS_LEN)
        # free: 레이 위 샘플(끝점 직전까지)
        n = np.maximum((rr / (RES * 0.7)).astype(int), 1)
        idx = np.repeat(np.arange(len(rr)), n)
        frac = np.concatenate([np.arange(k) / k for k in n])
        d = frac * rr[idx]
        fx = ((lx + d * np.cos(a[idx]) - ox) / RES).astype(int)
        fy = ((ly + d * np.sin(a[idx]) - oy) / RES).astype(int)
        ok = (fx >= 0) & (fx < self.nx) & (fy >= 0) & (fy < self.ny)
        cells = np.unique(fy[ok] * self.nx + fx[ok])
        self.L.flat[cells] += L_FREE
        hx = ((lx + r[hit] * np.cos(a[hit]) - ox) / RES).astype(int)
        hy = ((ly + r[hit] * np.sin(a[hit]) - oy) / RES).astype(int)
        ok = (hx >= 0) & (hx < self.nx) & (hy >= 0) & (hy < self.ny)
        hc = np.unique(hy[ok] * self.nx + hx[ok])
        self.L.flat[hc] += L_OCC - L_FREE        # 같은 스캔에서 free 로 깎인 몫 보상
        np.clip(self.L, L_MIN, L_MAX, out=self.L)
        self.nscan += 1

    def grid(self):
        p = 1 - 1 / (1 + np.exp(self.L))
        g = np.full(self.L.shape, -1, np.int8)
        g[p < 0.35] = 0
        g[p > 0.65] = 100
        return g

    def publish(self):
        if self.origin is None:
            return
        # 그린 영역(관측된 칸)만 잘라 발행: 60 m 격자 전체를 매초 보내면 느림
        rows = np.nonzero(np.any(self.L != 0, axis=1))[0]
        cols = np.nonzero(np.any(self.L != 0, axis=0))[0]
        if len(rows) == 0:
            return
        r0, r1 = max(rows[0] - 10, 0), min(rows[-1] + 11, self.ny)
        c0, c1 = max(cols[0] - 10, 0), min(cols[-1] + 11, self.nx)
        p = 1 - 1 / (1 + np.exp(self.L[r0:r1, c0:c1]))
        g = np.full(p.shape, -1, np.int8)
        g[p < 0.35] = 0
        g[p > 0.65] = 100
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = RES
        m.info.height, m.info.width = g.shape
        m.info.origin.position.x = self.origin[0] + c0 * RES
        m.info.origin.position.y = self.origin[1] + r0 * RES
        m.info.origin.orientation.w = 1.0
        m.data = g.ravel().tolist()
        self.pub.publish(m)

    def save(self):
        if time.time() >= self.skip_until:       # 리셋 직후가 아니면 대기 중 스캔도 반영
            for p in self.pending:
                self.integrate(*p)
        self.pending.clear()
        if self.origin is None:
            print('스캔 없음 - 저장 안 함', flush=True)
            return
        g = self.grid()[::-1]
        img = np.full(g.shape, 205, np.uint8)
        img[g == 0] = 254
        img[g == 100] = 0
        # 사용 영역만 잘라 저장
        known = np.where(img != 205)
        r0, r1 = max(known[0].min() - 10, 0), min(known[0].max() + 10, img.shape[0])
        c0, c1 = max(known[1].min() - 10, 0), min(known[1].max() + 10, img.shape[1])
        img = img[r0:r1, c0:c1]
        ox = self.origin[0] + c0 * RES
        oy = self.origin[1] + (self.ny - r1) * RES
        name = self.base.split('/')[-1]
        os.makedirs(os.path.dirname(self.base) or '.', exist_ok=True)
        Image.fromarray(img).save(self.base + '.pgm')
        open(self.base + '.yaml', 'w').write(
            f'image: {name}.pgm\nmode: trinary\nresolution: {RES}\norigin: [{ox:.3f}, {oy:.3f}, 0]\n'
            f'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
        traj = list(self.traj)
        if self.odom is not None:                # 마지막 행 = 현재(종료 시점) pose = 출발 pose
            p = self.odom.pose.pose
            traj.append((p.position.x, p.position.y, yaw_of(p.orientation)))
        with open(self.base + '_traj.csv', 'w') as f:
            f.write('x,y,yaw\n')
            for x, y, yw in traj:
                f.write(f'{x:.4f},{y:.4f},{yw:.4f}\n')
        x, y, yw = traj[-1]
        print(f'지도 저장: {self.base}.yaml (스캔 {self.nscan}개, 리셋 {self.nreset}회), '
              f'출발 pose=({x:.2f},{y:.2f},{yw:.2f})', flush=True)


def main():
    base = sys.argv[1] if len(sys.argv) > 1 else os.path.join(WS, 'maps', 'exam', 'map')
    rclpy.init()
    m = Mapper(os.path.abspath(base))
    stop = {'f': False}
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__('f', True))
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__('f', True))
    tele = sys.stdin.isatty()
    old = None
    v = w = 0.0
    speed, turn = 0.5, 1.0
    keys = {'i': (1, 0), 'u': (1, 1), 'o': (1, -1), 'j': (1, 1), 'l': (1, -1), 'k': (0, 0),
            ',': (-1, 0), 'm': (-1, -1), '.': (-1, 1)}
    if tele:
        import select
        import termios
        import tty
        print(TELEOP_HELP, flush=True)
        old = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
    t_pub = 0.0
    try:
        while rclpy.ok() and not stop['f']:
            rclpy.spin_once(m, timeout_sec=0.02)
            if not tele:
                continue
            r, _, _ = select.select([sys.stdin], [], [], 0.0)
            if r:
                c = sys.stdin.read(1)
                if c in keys:
                    a, b = keys[c]
                    v, w = a * speed, b * turn
                elif c == 'q':
                    speed = min(speed + 0.1, 0.8)
                elif c == 'z':
                    speed = max(speed - 0.1, 0.1)
                elif c == 'x':
                    turn = max(turn - 0.1, 0.3)
                elif c == 'w':
                    turn = min(turn + 0.1, 2.0)
                if c in 'qzwx':
                    print(f'\r  속도 {speed:.1f} m/s  조향 {turn:.1f}    ', end='', flush=True)
                    if v != 0.0:
                        v = math.copysign(speed, v)
                        w = math.copysign(turn, w) if w else 0.0
            if time.time() - t_pub > 0.1:
                cmd = Twist()
                cmd.linear.x, cmd.angular.z = v, w
                m.cmd.publish(cmd)
                t_pub = time.time()
    finally:
        if tele:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
            for _ in range(5):
                m.cmd.publish(Twist())
                time.sleep(0.05)
            print(flush=True)
        # 최신 /odom 확보 후 저장
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < 0.3:
            rclpy.spin_once(m, timeout_sec=0.05)
        m.save()
        m.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
