"""limo_race 공용: 워크스페이스 찾기, 기존 스크립트 import 경로, ROS 파라미터, 로봇 프로파일, 지도/경로 메시지 ↔ 파일.

알고리즘은 scripts/race, scripts/real 의 기존 코드(python3 버전과 같은 것)를 그대로 불러 쓴다.
"""
import math
import os
import sys
import tempfile

import numpy as np


def _find_ws():
    env = os.environ.get('LIMO_RACE_WS')
    if env and os.path.isfile(os.path.join(env, 'scripts', 'race', 'race_driver.py')):
        return env
    for start in (os.path.dirname(os.path.realpath(__file__)), os.getcwd()):
        d = start
        while True:
            if os.path.isfile(os.path.join(d, 'scripts', 'race', 'race_driver.py')):
                return d
            nd = os.path.dirname(d)
            if nd == d:
                break
            d = nd
    raise RuntimeError('워크스페이스(scripts/race/race_driver.py)를 못 찾음 → '
                       'colcon build --symlink-install 로 빌드했는지 확인, 또는 export LIMO_RACE_WS=코드폴더')


WS = _find_ws()
for _p in (os.path.join(WS, 'scripts', 'race'), os.path.join(WS, 'scripts', 'real')):
    if _p not in sys.path:
        sys.path.insert(0, _p)
os.environ['RACE_REAL'] = '1'      # race_driver/go: ROS_LOCALHOST_ONLY 강제하지 않음(셸 환경 그대로)

MAP_DIR = os.path.join(WS, 'maps', 'real')


def quiet_exit():
    """종료 정리 중에는 Ctrl+C 를 더 받지 않는다(터미널 Ctrl+C 는 launch 와 노드가 둘 다 받아 두 번 들어옴 → 정리 중 traceback)."""
    import signal
    try:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    except Exception:
        pass


# ---------------- 파라미터 ----------------
def read_params(defaults, name='limo_race_params'):
    """--ros-args -p 로 받은 값을 읽는다(rclpy.init 이후). 타입은 기본값 타입으로 맞춘다
    (launch 에서 vmax:=1 이 정수로 오는 등 → 자유 타입으로 선언 후 변환)."""
    from rcl_interfaces.msg import ParameterDescriptor
    import rclpy
    n = rclpy.create_node(name)
    out = {}
    for k, v in defaults.items():
        n.declare_parameter(k, v, ParameterDescriptor(dynamic_typing=True))
        out[k] = cast(n.get_parameter(k).value, v)
    n.destroy_node()
    return out


def cast(val, like):
    if val is None:
        return like
    if isinstance(like, bool):
        return val if isinstance(val, bool) else str(val).lower() in ('1', 'true', 'yes', 'on')
    if isinstance(like, float):
        return float(val)
    if isinstance(like, int):
        return int(val)
    if isinstance(like, str):
        return str(val)
    if isinstance(like, list):
        return list(val) if val is not None else like
    return val


def declare(node, params):
    """노드에 파라미터를 선언해 ros2 param list/get 으로 보이게 한다(값은 이미 읽은 것)."""
    from rcl_interfaces.msg import ParameterDescriptor
    for k, v in params.items():
        if not node.has_parameter(k):
            try:
                node.declare_parameter(k, v, ParameterDescriptor(dynamic_typing=True, read_only=True))
            except Exception:
                pass


# ---------------- 로봇 프로파일 ----------------
CALIB_PREFIX = {'limo_pro': 'calib_pro_', 'limo1': 'calib_limo1_'}


def resolve_calib(robot, calib):
    """calib: '' 또는 'auto' = 그 로봇의 가장 최근 보정 파일, 'none' = 보정 없음, 그 외 = 파일 이름/경로."""
    if calib.lower() == 'none':
        return None
    if calib and calib.lower() != 'auto':
        return calib
    d = os.path.join(WS, 'config', 'robots')
    pre = CALIB_PREFIX.get(robot, f'calib_{robot}_')
    c = sorted(f for f in os.listdir(d) if f.startswith(pre) and f.endswith('.yaml'))
    return c[-1] if c else None


def load_robot(robot, calib='auto', log=print):
    from robot_profile import load_profile
    cal = resolve_calib(robot, calib)
    prof = load_profile(robot, cal)
    log(f'robot={prof["name"]} calib={cal or "없음"} 명령={prof["topics"]["cmd"]}')
    return prof, cal


def configure_driver(rd, prof, vmax, lite=False, slow=False):
    """race_driver 모듈 전역값을 실차 프로파일로 설정 (race_driver.py main 의 --robot 처리와 같다)."""
    rd.NO_SLOW = not slow
    if lite:
        rd.N_ST, rd.N_RIDGE = 12, 3
        rd.OFFS = np.round(np.arange(-1.56, 1.5601, 0.12), 3)
    if vmax and vmax > 0:
        rd.V_MAX = vmax
    rd.PROF = prof
    veh = prof['vehicle']
    rd.WB = float(veh['wheelbase'])
    rd.REAR = rd.WB / 2
    rd.K_MAX = math.tan(0.9 * float(veh.get('max_steer_center', 0.41))) / rd.WB
    rd.TRACK_RATIO = float(veh.get('track', 0.172)) / (2 * rd.WB)
    rd.M2O_TAU = 0.5


# ---------------- 지도 이름 ----------------
def map_base(name):
    """'t1202' → <ws>/maps/real/t1202 , 경로(.yaml 포함 가능)면 그대로."""
    name = str(name)
    if name.endswith('.yaml'):
        name = name[:-5]
    if os.sep in name or name.startswith('.'):
        return os.path.abspath(name)
    return os.path.join(MAP_DIR, name)


# ---------------- QoS ----------------
def qos_latched():
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      reliability=ReliabilityPolicy.RELIABLE)


# ---------------- 메시지 ↔ 파일 ----------------
def path_msg(P, yaw=None, frame='map', stamp=None):
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    m = Path()
    m.header.frame_id = frame
    if stamp is not None:
        m.header.stamp = stamp
    for i, (x, y) in enumerate(np.asarray(P)[:, :2]):
        ps = PoseStamped()
        ps.header.frame_id = frame
        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
        th = float(yaw[i]) if yaw is not None else 0.0
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        m.poses.append(ps)
    return m


def path_array(m, with_yaw=False):
    from rt_common import yaw_of
    if with_yaw:
        return np.array([(p.pose.position.x, p.pose.position.y, yaw_of(p.pose.orientation)) for p in m.poses])
    return np.array([(p.pose.position.x, p.pose.position.y) for p in m.poses])


def grid_msg_from_file(yml):
    """pgm/yaml → OccupancyGrid (254 빈칸 → 0, 0 벽 → 100, 205 미관측 → -1)."""
    import race_driver as rd
    return rd.Race.map_msg(yml)


def write_map(grid, base, traj=None):
    """OccupancyGrid(+궤적 x,y,yaw) → base.pgm/.yaml(/_traj.csv). loop_mapper 저장 형식과 같다."""
    w, h = grid.info.width, grid.info.height
    d = np.asarray(grid.data, np.int16).reshape(h, w)
    img = np.full((h, w), 205, np.uint8)
    img[(d >= 0) & (d <= 25)] = 254
    img[d >= 65] = 0
    img = img[::-1]
    os.makedirs(os.path.dirname(base) or '.', exist_ok=True)
    with open(base + '.pgm', 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        f.write(img.tobytes())
    res = round(float(grid.info.resolution), 6)
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    with open(base + '.yaml', 'w') as f:
        f.write(f'image: {os.path.basename(base)}.pgm\nmode: trinary\nresolution: {res}\n'
                f'origin: [{ox:.3f}, {oy:.3f}, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    if traj is not None and len(traj):
        write_traj(traj, base + '_traj.csv')
    return base + '.yaml'


def write_traj(P, path):
    with open(path, 'w') as f:
        f.write('x,y,yaw\n')
        for x, y, t in P:
            f.write(f'{x:.4f},{y:.4f},{math.atan2(math.sin(t), math.cos(t)):.4f}\n')


def read_traj(path):
    return np.loadtxt(path, delimiter=',', skiprows=1, ndmin=2)


def write_line(R, path):
    np.savetxt(path, np.asarray(R)[:, :2], delimiter=',', fmt='%.4f', header='x,y', comments='')


def tmp_dir(tag):
    return tempfile.mkdtemp(prefix=f'limo_race_{tag}_')


def wait_topics(specs, timeout=None, log=print, what=''):
    """transient local 토픽 여러 개를 한 번씩 받을 때까지 기다린다.
    specs = {키: (토픽, 타입)} → {키: 메시지}. timeout 초과 시 None."""
    import time
    import rclpy
    n = rclpy.create_node('limo_race_wait')
    got = {}
    for k, (tp, ty) in specs.items():
        n.create_subscription(ty, tp, lambda m, k=k: got.__setitem__(k, m), qos_latched())
    t0 = t_log = time.time()
    try:
        while rclpy.ok() and len(got) < len(specs):
            rclpy.spin_once(n, timeout_sec=0.1)
            if timeout and time.time() - t0 > timeout:
                return None
            if time.time() - t_log > 5.0:
                t_log = time.time()
                miss = [specs[k][0] for k in specs if k not in got]
                log(f'{what}대기 중: {", ".join(miss)} (map_server / global_path 실행 중인지 확인)')
    finally:
        n.destroy_node()
    return got if len(got) == len(specs) else None
