# controller → local_planner: 주행 중 생긴 일 (막힘 대처는 planner 가 결정)
uint8 STUCK=1              # 명령은 가는데 차가 1.2 s 넘게 안 움직임. value = 명령 곡률(돌리려던 방향)
uint8 MANEUVER_DONE=2      # 탈출 경로 끝까지 감
uint8 MANEUVER_STALLED=3   # 탈출 경로 따라가다 멈춤/시간초과. value = 막힌 쪽(+1 앞, -1 뒤)

std_msgs/Header header
uint8 type
float32 value
