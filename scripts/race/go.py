#!/usr/bin/env python3
"""출발 명령: 대기(READY) 중인 race_driver 에 출발 신호(/race/go) 발행

  python3 scripts/race/go.py
"""
import os
import time
if os.environ.get('RACE_REAL') != '1':     # 실차는 RACE_REAL=1 → 네트워크 사용
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy  # noqa: E402
from std_msgs.msg import Empty  # noqa: E402

rclpy.init()
n = rclpy.create_node('race_go')
pub = n.create_publisher(Empty, '/race/go', 10)
t0 = time.time()
while pub.get_subscription_count() == 0 and time.time() - t0 < 5.0:   # 드라이버와 연결될 때까지
    rclpy.spin_once(n, timeout_sec=0.01)
if pub.get_subscription_count() == 0:
    print('드라이버(race_driver.py)가 실행 중이 아님 - READY 확인 후 다시 실행')
else:
    for _ in range(3):
        pub.publish(Empty())
        time.sleep(0.02)
    print('출발!')
n.destroy_node()
rclpy.shutdown()
