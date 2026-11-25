"""go: 출발 신호(/race/go, std_msgs/Empty) 발행.

  ros2 run limo_race go
  (같은 일: ros2 topic pub --once /race/go std_msgs/msg/Empty — 연결 대기 때문에 약 0.7 s 늦음)
"""
import time

import rclpy
from std_msgs.msg import Empty


def main():
    rclpy.init()
    n = rclpy.create_node('race_go')
    pub = n.create_publisher(Empty, '/race/go', 10)
    t0 = time.time()
    # 구독자 = controller + local_planner (2개). 하나만 연결된 채 보내면 나머지가 못 받을 수 있다
    while pub.get_subscription_count() < 2 and time.time() - t0 < 3.0:
        rclpy.spin_once(n, timeout_sec=0.01)
    if pub.get_subscription_count() == 0:
        print('controller 가 실행 중이 아님 - READY 확인 후 다시 실행')
    else:
        t1 = time.time()                    # 연결 직후 몇 개는 유실될 수 있다(실차 망) → 0.5 s 동안 반복
        while time.time() - t1 < 0.5:
            pub.publish(Empty())
            rclpy.spin_once(n, timeout_sec=0.03)
        print(f'출발! (받는 노드 {pub.get_subscription_count()}개)')
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
