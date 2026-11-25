# local_planner → controller: 따라갈 경로와 속도 (map 좌표)
uint8 NORMAL=0      # 경로(x, y) + 점별 목표 속도(speed) + 여유비(tight) 추종
uint8 STOP=1        # 정지 (갈 수 있는 경로 없음 / 탈출 경로 탐색 중)
uint8 MANEUVER=2    # 막힘 탈출 경로(Hybrid A*): x, y, heading, direction(+1 전진, -1 후진)
uint8 REVERSE=3     # 조향 후진: reverse_steer(-1~+1, 앞머리를 돌릴 방향) 로 reverse_time 초

std_msgs/Header header
uint8 mode
float32[] x
float32[] y
float32[] speed
float32[] tight
float32[] heading
float32[] direction
float32 reverse_steer
float32 reverse_time

# 상태 (controller 로그·랩 계산용)
bool blocked        # 계획 범위 끝까지 못 감
bool tight_mode     # 빠듯한 기준(C_TIGHT)으로 찾은 경로
int32 reached       # 도달 정거장 (끝까지 = 정거장 수 - 1)
int32 n_obstacle    # 장애물 점 수
float32 offset      # 레이스라인에서 벗어난 거리 (m, + 왼쪽)
float32 clearance   # 경로 최소 여유 (m)
float32 plan_ms     # 계획 시간
int32 line_index    # 레이스라인 진행 인덱스 (0.05 m 간격)
