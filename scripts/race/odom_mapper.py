#!/usr/bin/env python3
"""odom 기반 점유격자 매핑 (위치를 아는 매핑: 시뮬 odom = 정확한 위치)

  python3 odom_mapper.py OUT_BASE        -> SIGINT(Ctrl+C) 시 OUT_BASE.pgm/.yaml + OUT_BASE_traj.csv 저장
map 프레임 = odom 프레임 (static TF map->odom 항등 발행, /map 1Hz 발행 -> RViz 확인용)
"""
import sys
import math
import signal
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from PIL import Image
import tf2_ros

RES = 0.025
SIZE_X, SIZE_Y = 36.0, 24.0
L_FREE, L_OCC, L_MIN, L_MAX = -0.4, 0.9, -2.0, 3.5
MISS_LEN = 5.0


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
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, '/map', q)
        self.create_timer(1.0, self.publish)
        self.nscan = 0

    def on_scan(self, sc):
        try:
            t = self.tfb.lookup_transform('odom', sc.header.frame_id, Time.from_msg(sc.header.stamp))
            tb = self.tfb.lookup_transform('odom', 'base_link', Time.from_msg(sc.header.stamp))
        except Exception:
            return
        tr = t.transform
        q = tr.rotation
        lyaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        lx, ly = tr.translation.x, tr.translation.y
        qb = tb.transform.rotation
        self.traj.append((tb.transform.translation.x, tb.transform.translation.y,
                          math.atan2(2 * (qb.w * qb.z + qb.x * qb.y), 1 - 2 * (qb.y * qb.y + qb.z * qb.z))))
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
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = RES
        m.info.width, m.info.height = self.nx, self.ny
        m.info.origin.position.x, m.info.origin.position.y = self.origin
        m.info.origin.orientation.w = 1.0
        m.data = self.grid().ravel().tolist()
        self.pub.publish(m)

    def save(self):
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
        Image.fromarray(img).save(self.base + '.pgm')
        open(self.base + '.yaml', 'w').write(
            f'image: {name}.pgm\nmode: trinary\nresolution: {RES}\norigin: [{ox:.3f}, {oy:.3f}, 0]\n'
            f'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
        with open(self.base + '_traj.csv', 'w') as f:
            f.write('x,y,yaw\n')
            for x, y, yw in self.traj:
                f.write(f'{x:.4f},{y:.4f},{yw:.4f}\n')
        print(f'지도 저장: {self.base}.yaml (스캔 {self.nscan}개, 궤적 {len(self.traj)}점)', flush=True)


def main():
    rclpy.init()
    m = Mapper(sys.argv[1])
    stop = {'f': False}
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__('f', True))
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__('f', True))
    while rclpy.ok() and not stop['f']:
        rclpy.spin_once(m, timeout_sec=0.1)
    m.save()
    m.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
