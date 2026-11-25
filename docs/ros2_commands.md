# ros2 명령 버전 (limo_race 패키지)

python3 버전(`scripts/race`, `scripts/real`)과 **같은 알고리즘**을 ROS2 노드로 나눠 `ros2 run` / `ros2 launch` 로 실행한다.
python3 버전은 그대로 남아 있고 둘 중 아무거나 써도 된다(결과 동일: 시뮬 덕트 트랙 Pro 1.0 m/s 47.8/47.3/47.4 s vs python3 48.2/47.4/47.4 s).

## 1. 준비 (로봇 컨테이너)
setup.sh 가 자동으로 빌드한다. 직접 할 때(코드 폴더에서):
```bash
colcon build --symlink-install --base-paths src/limo_race src/limo_race_msgs --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash          # 컨테이너 새 셸은 자동
```
- 예전 clone 이라 `src/limo_race` 가 없으면: `git sparse-checkout add src/limo_race src/limo_race_msgs`
- symlink 빌드라 python 코드를 고쳐도 다시 빌드할 필요 없음(노드·launch 추가 때만 다시).

## 2. 단계별 실행
지도 이름 = `map:=NAME` → `maps/real/NAME.*`. 로봇 = `robot:=limo_pro | limo1`.
```bash
# ① 매핑 (터미널 A). 조종기로 한 바퀴 → 출발 위치·방향에 세움
ros2 launch limo_race mapping.launch.py robot:=limo_pro map:=NAME
#    저장 (터미널 B): 루프 닫힘 보정 → NAME.pgm/.yaml/_traj.csv
ros2 service call /map_saver/save std_srvs/srv/Trigger
#    확인 후 A 에서 Ctrl+C (Ctrl+C 는 저장하지 않는다)

# ② global path (A): RViz 에 지도 + 파란 선 → 확인 후 Ctrl+C
ros2 launch limo_race raceline.launch.py map:=NAME

# ③ 주행 (A): READY 뜰 때까지
ros2 launch limo_race race.launch.py robot:=limo_pro map:=NAME vmax:=0.5
#    출발 (B)
ros2 run limo_race go                        # 또는 ros2 topic pub --once /race/go std_msgs/msg/Empty
#    정지: A 에서 Ctrl+C (controller 가 속도 0 보내고 종료). 위험하면 조종기 SWB 가운데
```

## 3. 통합본 (터미널 하나)
```bash
ros2 run limo_race bringup robot:=limo_pro map:=NAME vmax:=0.5
```
1. 매핑 시작 → 한 바퀴 돌고 세운 뒤 **Ctrl+C** → 지도 저장 → 레이스라인 생성 → 주행 준비(READY)
2. 다른 터미널에서 `ros2 run limo_race go`
3. **Ctrl+C** → 정지 명령 후 전부 종료
- 저장 실패(ICP 신뢰 낮음 등)면 매핑을 계속한다: 다시 Ctrl+C = 재시도, 3초 안에 Ctrl+C 두 번 = 종료.
- `skip_mapping:=true`(저장된 NAME 지도로 시작), `skip_raceline:=true`(NAME_raceline.csv 있으면 바로 주행).

## 4. launch 인자 (공통)
| 인자 | 기본 | 뜻 |
|---|---|---|
| robot | limo_pro | 로봇 프로파일(config/robots) |
| calib | auto | auto = 그 로봇 최근 `calib_*.yaml`, none = 보정 없음, 파일 이름 지정 가능 |
| map | map | 지도 이름(maps/real/NAME) |
| vmax | 1.0 | 최고 속도 m/s |
| own_odom | false | true = odom_tf 노드 실행(로봇 EKF 가 odom→base_link TF 를 안 낼 때만) |
| viz | true | 노트북 RViz 중계(viz_relay) |
| lite / slow / global_only | false | 계산 줄이기 / 좁은 곳 감속 / 회피 없이 레이스라인만 |
| sim | false | 시뮬(use_sim_time) |

노드 하나만: `ros2 run limo_race <노드> --ros-args -p robot:=limo_pro -p map:=NAME ...`

## 5. 노드와 토픽
| 노드 | 하는 일 | 입력 → 출력 |
|---|---|---|
| mapper | 매핑(loop_mapper) | /scan, TF odom→base_link → /map(그리는 중, 1 Hz), /mapping/trajectory, /race/car_pose, 서비스 /mapper/finalize |
| map_saver | 지도 저장 | 서비스 /map_saver/save → mapper 보정 요청 → 파일 |
| map_server | 지도 불러오기 | NAME.yaml/_traj.csv → /map, /map_traj |
| raceline_generator | global path 생성 | /map, /map_traj → NAME_raceline.csv(.png), /race/raceline |
| global_path | global path 불러오기 | NAME_raceline.csv → /race/raceline |
| localizer | 위치추정(파티클 필터) | /map, /map_traj, /scan, TF → TF map→odom, /pf/pose |
| local_planner | 장애물 인식 + local path + 막힘 대처 | /map, /race/raceline, /scan, TF → /race/plan(limo_race_msgs/RacePlan), /race/local_path |
| controller | pure pursuit, 모터 명령 | /race/plan, /race/go → /limo/ack_cmd(Pro) 또는 /cmd_vel(LIMO1), /race/control_event, /race/car_pose |
| go | 출발 신호 | → /race/go |
| bringup | 통합본 | 위 launch 들을 순서대로 |

도구: `check_sensors`, `check_hose`, `calib_speed`, `calib_steer`, `teleop_key`, `odom_tf`, `viz_relay`
(`ros2 run limo_race calib_steer --ros-args -p robot:=limo_pro -p mode:=circle`, 옵션 = python3 버전 옵션 이름).

막힘 대처 흐름: controller 가 "명령은 가는데 안 움직임"을 /race/control_event(STUCK)로 알림 → local_planner 가 가상 장애물 + Hybrid A* 탈출 경로(MANEUVER)
또는 조향 후진(REVERSE)을 /race/plan 으로 보냄 → controller 가 수행 후 완료/멈춤을 다시 알림.

## 6. RViz (노트북)
로봇에서 viz_relay 가 켜져 있으면(launch 기본) 노트북 컨테이너에서:
```bash
ROS_LOCALHOST_ONLY=0 ROS_DOMAIN_ID=42 rviz2 -d config/rviz/real.rviz
```
| 단계 | 보이는 것 |
|---|---|
| 매핑 | 그려지는 지도(회색, 1 Hz) + 노란 선(지나온 길) + 주황 화살표(차) + 빨간 점(라이다). 저장하면 보정된 최종 지도로 바뀜 |
| global path | 지도 + 파란 선(레이스라인) |
| 주행 | 지도 + 파란 선(global) + 초록 선(local) + 주황 화살표(차) + PF 자세(공분산) + 라이다 |
- 위치가 틀어지면 RViz "2D Pose Estimate" 로 다시 잡는다(/initialpose → localizer). 단, 노트북→로봇 방향은 중계가 없어 지금은 로봇 안에서만 동작.

## 7. 문제 대처
| 증상 | 대처 |
|---|---|
| `Package 'limo_race' not found` | 코드 폴더에서 `source install/setup.bash`, 없으면 1. 빌드 |
| 노드가 `... 대기 중: /map` 에서 멈춤 | map_server(또는 raceline.launch/race.launch) 가 같은 지도 이름으로 떠 있는지 |
| READY 안 뜸 | localizer 상태 줄(점수 0.8↑) 확인, TF odom→base_link(로봇 드라이버) 있는지 |
| go 가 "controller 가 실행 중이 아님" | race.launch 의 READY 뒤에 실행 |
| 매핑 저장 실패 | `ICP 신뢰 낮음`/`출발점에서 멀다` → 출발점 근처로 가서 다시 저장 서비스 호출(매핑은 계속 진행 중) |
