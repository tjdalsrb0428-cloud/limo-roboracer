"""mapper: 매핑 노드 (loop_mapper 알고리즘).

  ros2 run limo_race mapper --ros-args -p robot:=limo_pro [-p sim:=true]

- 주행하는 동안: /map(그리는 중인 지도, 1 Hz), /mapping/trajectory(지나온 길), /race/car_pose(차 위치), TF map→odom(항등)
- 서비스 /mapper/finalize (std_srvs/Trigger): 루프 닫힘 보정 → 최종 지도를 /map, 최종 궤적을 /map_traj 로 발행(이후 매핑 멈춤).
  파일 저장은 map_saver 가 한다(ros2 service call /map_saver/save std_srvs/srv/Trigger).
- Ctrl+C: 저장하지 않고 종료.
"""
import contextlib
import io
import math
import os
import shutil
import threading
import time

import numpy as np

from limo_race import common

import rclpy  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from geometry_msgs.msg import PoseStamped, TransformStamped  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Path  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from rclpy.time import Time  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from tf2_ros import TransformBroadcaster  # noqa: E402

import loop_mapper as lm  # noqa: E402

LIVE_RES = 0.05          # 실시간 표시용 지도 해상도(최종 지도는 0.025)
LIVE_HALF = 30.0         # 첫 위치 중심 ±30 m
L_OCC_T = math.log(0.65 / 0.35)

PARAMS = dict(robot='limo_pro', calib='auto', sim=False)


class Mapper(lm.LoopMapper):
    def __init__(self, prof, prm):
        super().__init__(prof, prm['sim'])
        common.declare(self, prm)
        self.lock = threading.Lock()
        self.finalized = False
        self.hold_live_until = 0.0
        self.live = None            # (L, x0, y0, n)
        self.bbox = None            # 그린 칸 범위 [r0, r1, c0, c1]
        self.pub_map = self.create_publisher(OccupancyGrid, '/map', common.qos_latched())
        self.pub_traj = self.create_publisher(Path, '/map_traj', common.qos_latched())
        self.pub_live_traj = self.create_publisher(Path, '/mapping/trajectory', 1)
        self.pub_pose = self.create_publisher(PoseStamped, '/race/car_pose', 1)
        self.tfb = TransformBroadcaster(self)
        self.create_service(Trigger, '/mapper/finalize', self.on_finalize)

    # ---------- 실시간 지도 ----------
    def add(self, m, t):
        if self.finalized:
            return
        n0 = len(self.kf)
        super().add(m, t)
        if len(self.kf) > n0:
            self.live_add(self.kf[-1])

    def live_add(self, k):
        x, y, th, (lx, ly), hits, miss = k
        if self.live is None:
            n = int(2 * LIVE_HALF / LIVE_RES)
            self.live = (np.zeros((n, n), np.float32), float(x - LIVE_HALF), float(y - LIVE_HALF), n)
        L, x0, y0, n = self.live
        R = lm.se2(x, y, th)
        ox, oy = R[:2, :2] @ np.array([lx, ly]) + [x, y]
        hp = hits @ R[:2, :2].T + [x, y]
        mdir = miss + th
        mp = np.stack([ox + lm.MISS_LEN * np.cos(mdir), oy + lm.MISS_LEN * np.sin(mdir)], 1)
        ends = np.concatenate([hp, mp])
        if not len(ends):
            return
        d = np.hypot(ends[:, 0] - ox, ends[:, 1] - oy)
        cnt = np.maximum((d / (LIVE_RES * 0.7)).astype(int), 1)
        idx = np.repeat(np.arange(len(ends)), cnt)
        frac = np.concatenate([np.arange(c) / c for c in cnt])
        fx = ((ox + frac * (ends[idx, 0] - ox) - x0) / LIVE_RES).astype(int)
        fy = ((oy + frac * (ends[idx, 1] - oy) - y0) / LIVE_RES).astype(int)
        ok = (fx >= 0) & (fx < n) & (fy >= 0) & (fy < n)
        if not ok.any():
            return
        L.flat[np.unique(fy[ok] * n + fx[ok])] += lm.L_FREE
        hx = ((hp[:, 0] - x0) / LIVE_RES).astype(int)
        hy = ((hp[:, 1] - y0) / LIVE_RES).astype(int)
        okh = (hx >= 0) & (hx < n) & (hy >= 0) & (hy < n)
        L.flat[np.unique(hy[okh] * n + hx[okh])] += lm.L_OCC - lm.L_FREE
        np.clip(L, lm.L_MIN, lm.L_MAX, out=L)
        b = [int(fy[ok].min()), int(fy[ok].max()) + 1, int(fx[ok].min()), int(fx[ok].max()) + 1]
        self.bbox = b if self.bbox is None else [min(self.bbox[0], b[0]), max(self.bbox[1], b[1]),
                                                  min(self.bbox[2], b[2]), max(self.bbox[3], b[3])]

    def publish_live(self):
        if self.live is None or self.bbox is None:
            return
        L, x0, y0, n = self.live
        r0, r1, c0, c1 = self.bbox
        r0, c0 = max(r0 - 4, 0), max(c0 - 4, 0)
        r1, c1 = min(r1 + 4, n), min(c1 + 4, n)
        S = L[r0:r1, c0:c1]
        g = np.full(S.shape, -1, np.int8)
        g[S < -L_OCC_T] = 0
        g[S > L_OCC_T] = 100
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = LIVE_RES
        m.info.width, m.info.height = c1 - c0, r1 - r0
        m.info.origin.position.x, m.info.origin.position.y = float(x0 + c0 * LIVE_RES), float(y0 + r0 * LIVE_RES)
        m.info.origin.orientation.w = 1.0
        m.data = g.ravel().tolist()
        self.pub_map.publish(m)
        if self.kf:
            P = self.poses()
            self.pub_live_traj.publish(common.path_msg(P[:, :2], P[:, 2], stamp=m.header.stamp))

    def publish_tf_pose(self):
        """매핑 중에는 map = odom (항등). 위치추정이 없으므로 짧은 미래 시각으로 발행(조회 지연 대비)."""
        now = self.get_clock().now()
        t = TransformStamped()
        t.header.stamp = (now + Duration(seconds=0.2)).to_msg()
        t.header.frame_id, t.child_frame_id = 'map', self.fo
        t.transform.rotation.w = 1.0
        self.tfb.sendTransform(t)
        try:
            tr = self.buf.lookup_transform(self.fo, self.fb, Time())
        except Exception:
            return
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = tr.header.stamp
        p.pose.position.x, p.pose.position.y = tr.transform.translation.x, tr.transform.translation.y
        p.pose.orientation = tr.transform.rotation
        self.pub_pose.publish(p)

    # ---------- 최종 지도 ----------
    def on_finalize(self, req, res):
        with self.lock:
            if self.finalized:
                res.success, res.message = True, '이미 완료됨(최종 지도 발행 중)'
                return res
            for _ in range(10):
                self.process()
            tmp = common.tmp_dir('final')
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                self.save(os.path.join(tmp, 'map'))
            out = buf.getvalue().strip().replace(os.path.join(tmp, 'map') + '.yaml', '최종 지도')
            print(out, flush=True)
            yml = os.path.join(tmp, 'map.yaml')
            if not os.path.exists(yml):
                res.success, res.message = False, out or '저장 실패'
                shutil.rmtree(tmp, ignore_errors=True)
                return res
            grid = common.grid_msg_from_file(yml)
            grid.header.stamp = self.get_clock().now().to_msg()
            T = common.read_traj(os.path.join(tmp, 'map_traj.csv'))
            skipped = '보정 생략' in out
            self.finalized = not skipped
            self.hold_live_until = time.time() + 15.0      # map_saver 가 이 지도를 받아 가는 동안 실시간 지도로 덮지 않음
            self.pub_map.publish(grid)
            self.pub_traj.publish(common.path_msg(T[:, :2], T[:, 2], stamp=grid.header.stamp))
            shutil.rmtree(tmp, ignore_errors=True)
            res.success, res.message = True, out.replace('\n', ' | ')
            if skipped:
                res.message = ('경고: 루프 보정 없이 저장됨 → 매핑 계속 중. 출발점 근처(진행 방향 같게)로 가서 다시 저장하면 덮어씀 | '
                               + res.message)
                self.get_logger().warn('루프 보정 생략 → 매핑 계속(다시 저장 가능)')
            else:
                self.get_logger().info('최종 지도 발행(/map, /map_traj). 매핑 멈춤.')
            return res


def main():
    rclpy.init()
    prm = common.read_params(PARAMS)
    prof, _ = common.load_robot(prm['robot'], prm['calib'])
    node = Mapper(prof, prm)
    print('[mapper] 한 바퀴 돌고 출발점 근처(진행 방향 같게)에 세운 뒤 → '
          'ros2 service call /map_saver/save std_srvs/srv/Trigger', flush=True)
    t_log = t_map = t_tf = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            with node.lock:
                if not node.finalized:
                    node.process()
            now = time.time()
            if not node.finalized:
                if now - t_tf > 0.1:
                    t_tf = now
                    node.publish_tf_pose()
                if now - t_map > 1.0 and now > node.hold_live_until:
                    t_map = now
                    node.publish_live()
            else:
                if now - t_tf > 0.1:
                    t_tf = now
                    node.publish_tf_pose()
            if now - t_log > 5.0 and not node.finalized:
                t_log = now
                print(f'  키프레임 {len(node.kf)}개 / 스캔 {node.nscan}개(버림 {node.ndrop}), '
                      f'pose={tuple(round(v, 2) for v in node.last) if node.last else "없음(TF odom→base_link 대기)"}',
                      flush=True)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        common.quiet_exit()
        if not node.finalized:
            print('[mapper] 저장하지 않고 종료', flush=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
