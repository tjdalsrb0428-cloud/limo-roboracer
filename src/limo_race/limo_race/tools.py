"""측정·보조 도구를 ros2 run 으로 실행 (scripts/real 의 같은 이름 스크립트를 그대로 실행).

ROS 파라미터를 기존 옵션으로 바꿔 넘긴다:  -p robot:=limo_pro -p speed:=0.3 -p latch:=true  →  --robot limo_pro --speed 0.3 --latch
  ros2 run limo_race check_sensors --ros-args -p robot:=limo_pro
  ros2 run limo_race check_hose    --ros-args -p robot:=limo_pro
  ros2 run limo_race calib_speed   --ros-args -p robot:=limo_pro -p speed:=0.3
  ros2 run limo_race calib_steer   --ros-args -p robot:=limo_pro -p mode:=circle
  ros2 run limo_race teleop_key    --ros-args -p robot:=limo_pro
  ros2 run limo_race odom_tf       --ros-args -p robot:=limo_pro      (로봇 EKF TF 가 없을 때만)
  ros2 run limo_race viz_relay     [--ros-args -p domain:=42 -p scan_hz:=5]
calib 는 생략 = 그 로봇 최근 보정 파일, calib:=none = 보정 없음.
"""
import importlib
import sys

from limo_race import common

ROBOT_TOOLS = {'check_sensors', 'check_hose', 'calib_speed', 'calib_steer', 'teleop_key', 'odom_tf'}


def ros_params(argv):
    """argv 에서 '--ros-args ... -p k:=v' 와 맨 'k:=v' 를 뽑는다 → (dict, 나머지 인자)."""
    out, rest, in_ros, i = {}, [], False, 0
    while i < len(argv):
        t = argv[i]
        if t == '--ros-args':
            in_ros = True
        elif t == '--':
            in_ros = False
        elif in_ros and t in ('-p', '--param') and i + 1 < len(argv):
            k, _, v = argv[i + 1].partition(':=')
            out[k] = v
            i += 1
        elif in_ros:
            pass                             # -r 등 다른 ROS 인자는 무시
        elif ':=' in t and not t.startswith('-'):
            k, _, v = t.partition(':=')
            out[k] = v
        else:
            rest.append(t)
        i += 1
    return out, rest


def run(name):
    prm, rest = ros_params(sys.argv[1:])
    args = []
    if name in ROBOT_TOOLS:
        robot = prm.pop('robot', 'limo_pro')
        cal = common.resolve_calib(robot, prm.pop('calib', 'auto'))
        args += ['--robot', robot] + (['--calib', cal] if cal else [])
        print(f'[{name}] robot={robot} calib={cal or "없음"}', flush=True)
    for k, v in prm.items():
        opt = '--' + k.replace('_', '-')
        if v.lower() in ('true', 'false'):
            if v.lower() == 'true':
                args.append(opt)
        else:
            args += [opt, v]
    sys.argv = [name] + args + rest
    importlib.import_module(name).main()


def check_sensors():
    run('check_sensors')


def check_hose():
    run('check_hose')


def calib_speed():
    run('calib_speed')


def calib_steer():
    run('calib_steer')


def teleop_key():
    run('teleop_key')


def odom_tf():
    run('odom_tf')


def viz_relay():
    run('viz_relay')
