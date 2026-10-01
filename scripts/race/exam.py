#!/usr/bin/env python3
"""시험용 통합 실행기 (컨테이너 안에서 실행)

  python3 scripts/race/exam.py [--world WORLD] [--no-gui] [--skip-mapping MAP.yaml --odom]

순서
  1) Gazebo 실행(world 선택)                       - 이미 떠 있으면 --no-sim
  2) odom 매핑 시작 -> 같은 터미널 키보드(u/i/o, ,) 로 한 바퀴 -> 출발 위치/방향에 세우고 엔터
  3) 지도 저장(maps/exam/), 마지막 pose = 출발 pose
  4) 레이스라인 생성(출발 pose 기준, 2~10초)
  5) race_driver 실행 -> READY 뜨면 "시작"과 함께 엔터 (무한 랩, Ctrl+C 로 정지)
종료: Ctrl+C 후  pkill -f scripts/race; killros
"""
import os
import sys
import time
import math
import signal
import argparse
import subprocess

WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
LOG = os.path.join(WS, 'logs', 'exam')
OUT = os.path.join(WS, 'maps', 'exam')
PROCS = []
# 시험장 Wi-Fi 없음/끊김 대비: ROS2(DDS)와 Gazebo 통신을 이 PC 내부(localhost)로 한정.
# 하위 프로세스(Gazebo, 매퍼, 드라이버)는 이 환경을 그대로 물려받는다.
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['GAZEBO_IP'] = '127.0.0.1'


def run_bg(name, cmd):
    os.makedirs(LOG, exist_ok=True)
    f = open(os.path.join(LOG, name + '.log'), 'w')
    p = subprocess.Popen(cmd, shell=True, stdout=f, stderr=subprocess.STDOUT, preexec_fn=os.setsid, cwd=WS)
    PROCS.append((name, p))
    return p


def kill(name):
    for n, p in PROCS:
        if n == name and p.poll() is None:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
            try:
                p.wait(5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)


def say(msg):
    print(f'\033[1;36m[exam {time.strftime("%H:%M:%S")}]\033[0m {msg}', flush=True)


def wait_msg(topic, proc=None, timeout=60):
    """토픽 메시지를 실제로 1개 받을 때까지 대기 (daemon 캐시에 속지 않음)"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        if proc is not None and proc.poll() is not None:
            return False
        r = subprocess.run(['timeout', '4', 'ros2', 'topic', 'echo', '--once', '--no-daemon', topic],
                           capture_output=True, text=True)
        if r.returncode == 0 and r.stdout.strip():
            return True
    return False


def wait_topic(topic, timeout=60):
    t0 = time.time()
    while time.time() - t0 < timeout:
        out = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True).stdout.split()
        if topic in out:
            return True
        time.sleep(1.0)
    return False


TELEOP_HELP = '''
  [매핑 조종]  u i o  : 좌전진 / 전진 / 우전진
               j k l  : 좌전진 / 정지 / 우전진
               m , .  : 좌후진 / 후진 / 우후진
               q/z 속도 +/-   w/x 조향 +/-   엔터 = 매핑 끝(정지 후 저장)
'''


def teleop_until_enter():
    """같은 터미널 키보드 조종(10Hz 연속 발행). 엔터 입력 시 정지하고 반환"""
    if not sys.stdin.isatty():          # 자동 테스트(파이프 입력): 엔터만 대기
        input()
        return
    import select
    import termios
    import tty
    import rclpy
    from geometry_msgs.msg import Twist
    rclpy.init()
    n = rclpy.create_node('exam_teleop')
    pub = n.create_publisher(Twist, '/cmd_vel', 1)
    speed, turn = 0.5, 1.0
    v = w = 0.0
    keys = {'i': (1, 0), 'u': (1, 1), 'o': (1, -1), 'j': (1, 1), 'l': (1, -1), 'k': (0, 0),
            ',': (-1, 0), 'm': (-1, -1), '.': (-1, 1)}
    print(TELEOP_HELP, flush=True)
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        t_pub = 0.0
        while True:
            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            if r:
                c = sys.stdin.read(1)
                if c in ('\n', '\r'):
                    break
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
                m = Twist()
                m.linear.x, m.angular.z = v, w
                pub.publish(m)
                t_pub = time.time()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        for _ in range(5):
            pub.publish(Twist())
            time.sleep(0.05)
        n.destroy_node()
        rclpy.shutdown()
    print(flush=True)


def odom_pose(timeout=5.0):
    """/odom 1개를 받아 현재 pose 반환 (Gazebo Ctrl+R 리셋 후에도 정확: 플러그인 odom = 월드 좌표)"""
    import rclpy
    from nav_msgs.msg import Odometry
    rclpy.init()
    n = rclpy.create_node('exam_odom')
    got = {}
    n.create_subscription(Odometry, '/odom', lambda m: got.__setitem__('m', m), 10)
    t0 = time.time()
    while 'm' not in got and time.time() - t0 < timeout:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.destroy_node()
    rclpy.shutdown()
    if 'm' not in got:
        return None
    p = got['m'].pose.pose
    q = p.orientation
    return (p.position.x, p.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def _unused_get_pose(timeout=10.0):
    import rclpy
    from rclpy.time import Time
    import tf2_ros
    rclpy.init()
    n = rclpy.create_node('exam_pose', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    b = tf2_ros.Buffer()
    tf2_ros.TransformListener(b, n, spin_thread=False)
    t0 = time.time()
    pose = None
    while time.time() - t0 < timeout:
        rclpy.spin_once(n, timeout_sec=0.1)
        try:
            t = b.lookup_transform('map', 'base_link', Time())
            q = t.transform.rotation
            pose = (t.transform.translation.x, t.transform.translation.y,
                     math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
            break
        except Exception:
            pass
    n.destroy_node()
    rclpy.shutdown()
    return pose


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', default='')
    ap.add_argument('--no-gui', action='store_true')
    ap.add_argument('--no-sim', action='store_true', help='Gazebo 가 이미 실행 중')
    ap.add_argument('--skip-mapping', default='', help='기존 지도 사용(MAP.yaml), 매핑 생략')
    ap.add_argument('--init', nargs=3, type=float, help='--skip-mapping 시 초기 pose')
    ap.add_argument('--carto', action='store_true', help='cartographer 매핑 + AMCL 위치추정(기존 방식)')
    ap.add_argument('--odom', action='store_true', help='--skip-mapping 지도가 odom_mapper 지도일 때')
    a = ap.parse_args()
    t_all = time.time()
    try:
        if not a.no_sim:
            w = f' world:={os.path.abspath(a.world)}' if a.world else ''
            g = ' gui:=false rviz:=false' if a.no_gui else ''
            say(f'1) Gazebo 시작{w}')
            sim = run_bg('sim', f'ros2 launch limo_car roboracer_sim.launch.py{w}{g}')
            if not wait_msg('/scan', sim, 90):
                say('Gazebo 기동 실패(/scan 수신 안 됨) - logs/exam/sim.log 확인')
                return
            time.sleep(2.0)
        traj = ''
        if a.skip_mapping:
            map_yaml = os.path.abspath(a.skip_mapping)
            pose = tuple(a.init) if a.init else (0.0, 0.0, 0.0)
        elif not a.carto:
            say('2) 매핑 시작 (odom 기반 점유격자)')
            os.makedirs(OUT, exist_ok=True)
            base = os.path.join(OUT, 'map_' + time.strftime('%m%d_%H%M%S'))
            run_bg('mapper', f'{sys.executable} -u {WS}/scripts/race/odom_mapper.py {base}')
            say('   키보드로 한 바퀴 돈 뒤, 출발 위치/방향에 세우고 엔터')
            teleop_until_enter()
            pose_now = odom_pose()
            t_save = time.time()
            kill('mapper')
            map_yaml = base + '.yaml'
            traj = base + '_traj.csv'
            if not os.path.exists(map_yaml):
                say('지도 저장 실패: logs/exam/mapper.log 확인')
                return
            if pose_now is not None:
                pose = pose_now
            else:
                rows = [l.strip().split(',') for l in open(traj).readlines()[1:] if l.count(',') == 2]
                pose = tuple(float(v) for v in rows[-1])
            say(f'3) 지도 저장 {map_yaml} ({time.time() - t_save:.1f}s), 현재 pose=({pose[0]:.2f},{pose[1]:.2f},{pose[2]:.2f})')
        else:
            say('2) 매핑 시작 (cartographer)')
            cfg = os.path.join(WS, 'config', 'cartographer')
            run_bg('carto', f'ros2 run cartographer_ros cartographer_node -configuration_directory {cfg} '
                            f'-configuration_basename limo_2d.lua --ros-args -p use_sim_time:=true')
            run_bg('carto_grid', 'ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.05 '
                                 '-publish_period_sec 1.0 --ros-args -p use_sim_time:=true')
            wait_topic('/map', 30)
            os.makedirs(OUT, exist_ok=True)
            base = os.path.join(OUT, 'map_' + time.strftime('%m%d_%H%M%S'))
            traj = base + '_traj.csv'
            run_bg('traj', f'{sys.executable} {WS}/scripts/race/record_traj.py {traj}')
            say('   다른 터미널에서 teleop 으로 한 바퀴 돌고, 출발 위치/방향에 세운 뒤 여기서 엔터')
            input()
            t_save = time.time()
            kill('traj')
            rows = [l.strip().split(',') for l in open(traj).readlines()[1:] if l.count(',') == 2]
            if len(rows) < 10:
                say('궤적 기록 실패(map->base_link TF 없음): logs/exam/carto.log 확인')
                return
            pose = tuple(float(v) for v in rows[-1])
            say(f'3) 지도 저장 {base}.yaml, 현재 pose=({pose[0]:.2f},{pose[1]:.2f},{pose[2]:.2f})')
            r = subprocess.run(f'ros2 run nav2_map_server map_saver_cli -f {base} --ros-args '
                               f'-p save_map_timeout:=10.0 -p use_sim_time:=true',
                               shell=True, capture_output=True, text=True, cwd=WS)
            if not os.path.exists(base + '.yaml'):
                say('지도 저장 실패:\n' + r.stdout[-800:] + r.stderr[-800:])
                return
            kill('carto')
            kill('carto_grid')
            map_yaml = base + '.yaml'
            say(f'   저장 완료 ({time.time() - t_save:.1f}s)')
        use_amcl = a.carto or (a.skip_mapping and not a.odom)
        say('4) 레이스라인 생성' + (' + 위치추정(AMCL) 시작' if use_amcl else ' (위치: odom)'))
        if use_amcl:
            run_bg('loc', f'ros2 launch {WS}/launch/localization.launch.py use_sim_time:=true map:={map_yaml}')
        elif a.skip_mapping:
            run_bg('mapper_tf', 'ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom')
        line = os.path.splitext(map_yaml)[0] + '_raceline.csv'
        t_rl = time.time()
        targ = ['--traj', traj] if traj else []
        r = subprocess.run([sys.executable, os.path.join(WS, 'scripts/race/raceline.py'), map_yaml, line,
                            f"{pose[0]:.4f}", f"{pose[1]:.4f}", f"{pose[2]:.4f}"] + targ, cwd=WS)
        if r.returncode != 0 or not os.path.exists(line):
            say('레이스라인 생성 실패')
            return
        say(f'   레이스라인 {line} ({time.time() - t_rl:.1f}s), 확인용 그림: {os.path.splitext(line)[0]}.png')
        say(f'5) 드라이버 실행 (준비 총 {time.time() - t_all:.0f}s). READY 후 엔터 = 즉시 출발')
        sys.stdout.flush()
        os.execvp(sys.executable, [sys.executable, '-u', os.path.join(WS, 'scripts/race/race_driver.py'),
                                   '--map', map_yaml, '--line', line] + targ +
                  (['--init', f"{pose[0]:.4f}", f"{pose[1]:.4f}", f"{pose[2]:.4f}"] if use_amcl else ['--odom']))
    except KeyboardInterrupt:
        say('중단 - 백그라운드 프로세스 정리')
        for n, p in PROCS:
            if p.poll() is None:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)


if __name__ == '__main__':
    main()
