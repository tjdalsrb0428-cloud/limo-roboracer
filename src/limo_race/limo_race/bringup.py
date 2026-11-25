"""bringup: 통합본 — 명령 하나로 매핑 → 지도 저장 → global path 생성 → 주행 준비.

  ros2 run limo_race bringup robot:=limo_pro map:=NAME [vmax:=0.5]
  1) 매핑(mapping.launch): 한 바퀴 돌고 출발점 근처에 세운 뒤 Ctrl+C
     → 지도 저장(/map_saver/save) → 매핑 종료
  2) global path(map_server + raceline_generator) 생성·저장
  3) 주행(race.launch) READY → 다른 터미널에서  ros2 run limo_race go
  4) Ctrl+C → 정지 명령 후 전체 종료
인자(launch 와 같다): calib:=auto  sim:=false  own_odom:=false  viz:=true  lite:=false  slow:=false  global_only:=false
      skip_mapping:=true (이미 저장한 NAME 지도로 2)부터)   skip_raceline:=true (NAME_raceline.csv 있으면 3)부터)
"""
import os
import signal
import subprocess
import sys
import time

from limo_race import common
from limo_race.tools import ros_params

DEF = dict(robot='limo_pro', calib='auto', map='map', vmax='1.0', sim='false', own_odom='false', viz='true',
           lite='false', slow='false', global_only='false', skip_mapping='false', skip_raceline='false')
INT = {'n': 0, 't': 0.0}


def on_int(*_):
    INT['n'] += 1
    INT['t'] = time.time()


def say(s):
    print(f'\033[1;33m[bringup] {s}\033[0m', flush=True)


def start(cmd):
    say('실행: ' + ' '.join(cmd))
    return subprocess.Popen(cmd, start_new_session=True)       # Ctrl+C 는 bringup 만 받는다


def stop(p, wait=15.0):
    """launch 는 launch 프로세스에만 SIGINT(노드 정리는 launch 가 한다. 그룹 전체에 보내면 노드가 두 번 받음),
    ros2 run 은 그룹 전체(ros2 run 껍데기는 자식에게 전달하지 않는다)."""
    if p is None or p.poll() is not None:
        return
    try:
        if 'launch' in p.args:
            p.send_signal(signal.SIGINT)
        else:
            os.killpg(p.pid, signal.SIGINT)
        p.wait(timeout=wait)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGTERM)
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def wait_int(procs=()):
    """Ctrl+C 를 기다린다. 감시하는 프로세스가 먼저 죽으면 False."""
    n0 = INT['n']
    while INT['n'] == n0:
        for p in procs:
            if p.poll() is not None:
                return False
        time.sleep(0.1)
    return True


def call_save():
    import rclpy
    from std_srvs.srv import Trigger
    n = rclpy.create_node('bringup')
    cli = n.create_client(Trigger, '/map_saver/save')
    try:
        if not cli.wait_for_service(timeout_sec=5.0):
            return False, '/map_saver/save 서비스 없음'
        fut = cli.call_async(Trigger.Request())
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 150:
            rclpy.spin_once(n, timeout_sec=0.1)
        if not fut.done():
            return False, '저장 응답 없음'
        r = fut.result()
        return r.success, r.message
    finally:
        n.destroy_node()


def main():
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    a = dict(DEF)
    a.update(ros_params(sys.argv[1:])[0])
    yes = lambda k: str(a[k]).lower() in ('true', '1', 'yes')     # noqa: E731
    signal.signal(signal.SIGINT, on_int)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    base = common.map_base(a['map'])
    arg = lambda *ks: [f'{k}:={a[k]}' for k in ks]                  # noqa: E731
    relay = start(['ros2', 'run', 'limo_race', 'viz_relay']) if yes('viz') else None
    procs = []
    try:
        # ---------- 1) 매핑 ----------
        if not yes('skip_mapping'):
            mp = start(['ros2', 'launch', 'limo_race', 'mapping.launch.py', 'viz:=false']
                       + arg('robot', 'calib', 'map', 'sim', 'own_odom'))
            procs = [mp]
            say('매핑: 한 바퀴 돌고 출발점 근처(진행 방향 같게)에 세운 뒤 Ctrl+C → 지도 저장')
            need_wait = True
            while True:
                if need_wait and not wait_int(procs):
                    say('매핑이 먼저 종료됨 → 중단')
                    return
                need_wait = True
                say('지도 저장 중 (루프 닫힘 보정)...')
                ok, msg = call_save()
                say(msg)
                warn = ok and msg.startswith('경고')
                if ok and not warn:
                    break
                if warn:
                    say('루프 보정 없이 저장됨 → 매핑 계속. 출발점 근처로 가서 Ctrl+C = 다시 저장, '
                        '3초 안에 Ctrl+C 두 번 = 지금 지도로 진행')
                else:
                    say('저장 실패 → 매핑 계속. 다시 Ctrl+C = 재시도, 3초 안에 Ctrl+C 두 번 = 전체 종료')
                if not wait_int(procs):
                    return
                t1, n1 = time.time(), INT['n']
                while time.time() - t1 < 3.0 and INT['n'] == n1:
                    time.sleep(0.1)
                if INT['n'] != n1:              # 두 번 눌림
                    if warn:
                        break
                    return
                need_wait = False               # 한 번 → 바로 다시 저장
            stop(mp)
            procs = []
        elif not os.path.exists(base + '.yaml'):
            say(f'지도 없음: {base}.yaml')
            return
        # ---------- 2) global path ----------
        if not (yes('skip_raceline') and os.path.exists(base + '_raceline.csv')):
            ms = start(['ros2', 'run', 'limo_race', 'map_server', '--ros-args', '-p', f'map:={a["map"]}'])
            rg = start(['ros2', 'run', 'limo_race', 'raceline_generator', '--ros-args', '-p', f'map:={a["map"]}',
                        '-p', 'stay:=false'])
            procs = [ms, rg]
            n0 = INT['n']
            while rg.poll() is None:
                if INT['n'] != n0:
                    say('중단')
                    return
                time.sleep(0.1)
            stop(ms)
            procs = []
            if rg.returncode != 0 or not os.path.exists(base + '_raceline.csv'):
                say('레이스라인 생성 실패 → 종료 (지도는 저장됨: ' + base + '.yaml)')
                return
        # ---------- 3) 주행 ----------
        INT['n'] = 0
        rc = start(['ros2', 'launch', 'limo_race', 'race.launch.py', 'viz:=false']
                   + arg('robot', 'calib', 'map', 'vmax', 'sim', 'own_odom', 'lite', 'slow', 'global_only'))
        procs = [rc]
        say('주행 준비: READY 뜨면 다른 터미널에서  ros2 run limo_race go   / 정지: Ctrl+C')
        wait_int(procs)
        say('정지')
    finally:
        for p in reversed(procs):
            stop(p)
        stop(relay)
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
