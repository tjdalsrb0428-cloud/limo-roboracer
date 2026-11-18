#!/usr/bin/env python3
"""키보드 조종(조종기 대신, 매핑용). 로봇 종류에 맞는 명령(LIMO1 /cmd_vel, Pro /limo/ack_cmd)으로 보낸다.

  python3 teleop_key.py --robot limo_pro [--speed 0.3] [--latch]

키: i 전진 / u 좌회전 전진 / o 우회전 전진 / , 후진 / m 좌후진 / . 우후진 / k 정지
    q / z 속도 +0.05 / -0.05 (최대 0.6),  w / x 조향 +0.05 / -0.05 rad (최대 0.42),  Ctrl+C 종료(정지 명령 후)
안전: 키를 '누르고 있는 동안만' 움직인다(마지막 키 후 0.6 s 지나면 정지). --latch면 k 누를 때까지 유지.
      전진 중 정면 ±20° 0.30 m 안에 물체가 보이면 전진 막음(후진은 가능).
"""
import argparse
import math
import select
import sys
import termios
import time
import tty

import numpy as np

import rt_common as rc
from robot_profile import add_args, scan_valid

MOVES = {'i': (1, 0), 'u': (1, 1), 'o': (1, -1), ',': (-1, 0), 'm': (-1, 1), '.': (-1, -1), 'k': (0, 0)}


def front_blocked(node, dist=0.30, half_deg=20.0):
    m = node.msgs.get('scan')
    if m is None:
        return False
    a, r = scan_valid(m, node.prof)
    sel = np.abs(a) <= math.radians(half_deg)
    return bool(sel.any() and r[sel].min() < dist)


def main():
    ap = add_args(argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter))
    ap.add_argument('--speed', type=float, default=0.3)
    ap.add_argument('--steer', type=float, default=0.35)
    ap.add_argument('--latch', action='store_true', help='키를 떼도 k 누를 때까지 계속 움직임')
    a = ap.parse_args()
    if not sys.stdin.isatty():
        print('터미널(tty)에서 실행해야 한다 (docker exec -it ...)')
        return
    node = rc.start('teleop_key', a, want=('scan', 'status'), cmd=True)
    print(__doc__.split('\n\n', 1)[1], flush=True)
    print(rc.control_mode_note(node), flush=True)
    spd, steer = a.speed, a.steer
    mv = (0, 0)
    last_key = 0.0
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        nxt = time.monotonic()
        while True:
            rclpy_spin(node)
            while select.select([sys.stdin], [], [], 0)[0]:
                c = sys.stdin.read(1)
                if c in MOVES:
                    mv = MOVES[c]
                    last_key = time.monotonic()
                elif c == 'q':
                    spd = min(0.6, spd + 0.05)
                elif c == 'z':
                    spd = max(0.1, spd - 0.05)
                elif c == 'w':
                    steer = min(0.42, steer + 0.05)
                elif c == 'x':
                    steer = max(0.05, steer - 0.05)
                else:
                    continue
                print(f'\r속도 {spd:.2f} m/s  조향 {steer:.2f} rad  키 {c!r}      ', end='', flush=True)
            now = time.monotonic()
            if now < nxt:
                time.sleep(0.005)
                continue
            nxt = now + 0.05   # 20 Hz
            if not a.latch and now - last_key > 0.6:
                mv = (0, 0)
            v = mv[0] * spd
            if v > 0 and front_blocked(node):
                v = 0.0
                print('\r정면 0.30 m 안 물체 → 전진 막음                ', end='', flush=True)
            node.cmd.send(v, mv[1] * steer if v != 0 else 0.0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        print('\n정지')
        rc.finish(node)


def rclpy_spin(node):
    import rclpy
    rclpy.spin_once(node, timeout_sec=0.0)


if __name__ == '__main__':
    main()
