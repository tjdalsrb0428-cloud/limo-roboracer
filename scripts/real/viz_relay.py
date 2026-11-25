#!/usr/bin/env python3
"""노트북 RViz용 중계: 로봇 안(도메인 0, LOCALHOST_ONLY=1) 토픽 → Wi-Fi(도메인 42) 로 다시 발행.

조교 드라이버·우리 노드 설정은 그대로 두고, 이 프로세스만 두 쪽에 붙는다.
  - 수신 프로세스: 현재 환경(도메인 0, localhost) 에서 구독 → 직렬화 바이트를 파이프로 전달
  - 송신 자식 프로세스: ROS_LOCALHOST_ONLY=0, ROS_DOMAIN_ID=42 로 그대로 발행
Wi-Fi 가 느리므로 /scan 은 5 Hz 로 줄이고, /map 은 바뀔 때만 보낸다.

로봇 컨테이너:  python3 viz_relay.py [--domain 42] [--scan-hz 5]
노트북:         ROS_LOCALHOST_ONLY=0 ROS_DOMAIN_ID=42 rviz2 -d config/rviz/real.rviz
"""
import argparse
import os
import struct
import subprocess
import sys
import time

# (토픽, 타입 모듈, 타입 이름, transient_local 여부)
TOPICS = [
    ('/tf', 'tf2_msgs.msg', 'TFMessage', False),
    ('/tf_static', 'tf2_msgs.msg', 'TFMessage', True),
    ('/scan', 'sensor_msgs.msg', 'LaserScan', False),
    ('/map', 'nav_msgs.msg', 'OccupancyGrid', True),
    ('/race/raceline', 'nav_msgs.msg', 'Path', True),
    ('/race/local_path', 'nav_msgs.msg', 'Path', False),
    ('/pf/pose', 'geometry_msgs.msg', 'PoseWithCovarianceStamped', False),
]


def msg_type(mod, name):
    import importlib
    return getattr(importlib.import_module(mod), name)


def qos(latched):
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
    if latched:
        return QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    return QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE)


def sender():
    """자식: stdin 으로 받은 (토픽 번호, 바이트) 를 도메인 42 에 발행."""
    import rclpy
    from rclpy.serialization import deserialize_message
    rclpy.init()
    n = rclpy.create_node('viz_relay_out')
    types = [msg_type(m, t) for _, m, t, _ in TOPICS]
    pubs = [n.create_publisher(ty, tp, qos(l)) for ty, (tp, _, _, l) in zip(types, TOPICS)]
    inp = sys.stdin.buffer
    while True:
        h = inp.read(5)
        if len(h) < 5:
            break
        i, ln = struct.unpack('<BI', h)
        data = inp.read(ln)
        try:
            pubs[i].publish(deserialize_message(data, types[i]))
        except Exception as e:
            print(f'[relay] {TOPICS[i][0]} 발행 실패: {e}', file=sys.stderr, flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--domain', type=int, default=42)
    ap.add_argument('--scan-hz', type=float, default=5.0)
    ap.add_argument('--child', action='store_true', help=argparse.SUPPRESS)
    a = ap.parse_args()
    if a.child:
        sender()
        return

    env = dict(os.environ, ROS_LOCALHOST_ONLY='0', ROS_DOMAIN_ID=str(a.domain))
    env.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)       # 로컬 전용 UDP 설정 제거 → Wi-Fi 로 나감
    child = subprocess.Popen([sys.executable, os.path.abspath(__file__), '--child'], stdin=subprocess.PIPE, env=env)

    import rclpy
    from rclpy.serialization import serialize_message
    rclpy.init()
    n = rclpy.create_node('viz_relay_in')
    last = {}
    static = {}
    cnt = [0] * len(TOPICS)

    def make_cb(i):
        tp = TOPICS[i][0]

        def cb(m):
            now = time.time()
            if tp == '/scan' and now - last.get(tp, 0) < 1.0 / a.scan_hz:
                return
            last[tp] = now
            if tp == '/tf_static':      # 발행자가 여럿 → 모아서 한 메시지로(transient_local depth 1 이라 마지막 것만 남는 문제)
                for t in m.transforms:
                    static[t.child_frame_id] = t
                m.transforms = list(static.values())
            data = serialize_message(m)
            try:
                child.stdin.write(struct.pack('<BI', i, len(data)) + data)
                child.stdin.flush()
                cnt[i] += 1
            except BrokenPipeError:
                raise SystemExit('[relay] 송신 프로세스 종료')
        return cb

    for i, (tp, mod, name, latched) in enumerate(TOPICS):
        n.create_subscription(msg_type(mod, name), tp, make_cb(i), qos(latched))
    print(f'[relay] 도메인 0(로봇 안) → 도메인 {a.domain}(Wi-Fi). 노트북: '
          f'ROS_LOCALHOST_ONLY=0 ROS_DOMAIN_ID={a.domain} rviz2 -d config/rviz/real.rviz', flush=True)
    t_log = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.1)
            if time.time() - t_log > 5.0:
                t_log = time.time()
                print('  ' + ' '.join(f'{TOPICS[i][0]}={c}' for i, c in enumerate(cnt)), flush=True)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        child.terminate()


if __name__ == '__main__':
    main()
