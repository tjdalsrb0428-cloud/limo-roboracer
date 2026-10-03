# 레이싱 시스템 (scripts/race)

Nav2/TEB 없이 **odom 매핑 → 자동 레이스라인 → 자체 드라이버**로 달린다.
Q1, Q2 모두 같은 절차를 따르고, 터미널은 1개만 쓴다.

## 실행 방법

두 가지 방법이 있고, 결과는 같다.

### A. 통합 실행 (터미널 1개)

컨테이너 안에서:
```bash
cd /home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble
python3 scripts/race/exam.py                      # Q1 (기본 트랙)
python3 scripts/race/exam.py --world 파일.world    # world 파일을 ws 안에 복사해 둔 경우
```
1. Gazebo가 뜨면 같은 터미널에서 키보드로 트랙을 **한 바퀴** 돈다.
   - `i` 전진, `u`/`o` 좌/우 전진, `,` 후진, `k` 정지, `q`/`z` 속도 ±(최대 0.8)
2. 출발 위치/방향에 세운다(Gazebo Ctrl+R로 초기 위치 복귀해도 된다) → **엔터**.
3. 지도 저장 → 레이스라인 생성(2~15초) → `READY`가 뜨면 "시작"과 함께 **엔터**를 친다. 무한 랩을 돈다.

### B. 단계별 실행 (터미널 3개, 시험에서 사용)

**터미널 1** (Gazebo, 끝날 때까지 그대로 둔다):
```bash
cd /home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble
ros2 launch limo_car roboracer_sim.launch.py world:=$PWD/FILE.world
```
Q1 기본 트랙이면 `world:=...`를 생략한다.

**터미널 2** (아래 3개를 차례로 실행):
```bash
cd /home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble
python3 scripts/race/odom_mapper.py
python3 scripts/race/raceline.py
python3 scripts/race/race_driver.py
```
1. `odom_mapper.py`: 키보드로 한 바퀴 돌고 출발 위치에 세운 뒤(Gazebo Ctrl+R 가능) **Ctrl+C** → 정지 + 저장(`maps/exam/map.*`).
   이름 지정: `odom_mapper.py maps/exam/이름` → `raceline.py maps/exam/이름.yaml` → `race_driver.py --map maps/exam/이름.yaml`.
2. `raceline.py`: 레이스라인을 생성한다(인자 없이 위 지도와 출발 위치를 자동으로 쓴다).
3. `race_driver.py`: `READY`(RViz에 지도 + 파란 선)로 대기한다.

**터미널 3** (출발):
```bash
cd /home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble
python3 scripts/race/go.py      # "시작"과 동시에 실행 → 약 0.1초 안에 출발
```

## 종료

- 주행 중 **Ctrl+C**를 누르면 정지 명령(속도 0)을 보낸 뒤 종료한다. 차가 그 자리에 선다.
- Gazebo 종료: 터미널 1에서 Ctrl+C를 누른다. 통합 실행(A)이었거나 Gazebo가 남아 있으면 아래 명령을 친다.
  ```bash
  pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f rviz2
  ```

## 네트워크

ROS2와 Gazebo 통신은 이 PC 내부(localhost)로만 한다. 런치 파일과 각 스크립트에 `ROS_LOCALHOST_ONLY=1`, `GAZEBO_IP=127.0.0.1`을 설정해 두었다.
- Wi-Fi가 없거나 학교망처럼 다른 노트북이 같은 망에 있어도 서로 섞이지 않는다.
- 다른 터미널에서 `ros2 topic list` 등으로 확인하려면 먼저 `export ROS_LOCALHOST_ONLY=1`을 친다.

준비 시간은 Gazebo 기동부터 READY까지 약 90~140초다. 그중 매핑 한 바퀴가 60~90초다.
레이스라인 확인 그림은 `maps/exam/map*_raceline.png`에 저장된다(파랑=루프, 빨강=레이스라인).
RViz 표시: 지도(회색), 라이다(점), 레이스라인(파랑), 실시간 계획 경로(초록), 차량 위치·방향(주황 화살표).

## 파일

| 파일 | 역할 |
|---|---|
| `exam.py` | 통합 실행기: Gazebo → 매핑(내장 키보드 조종) → 저장 → 레이스라인 → 드라이버(exec, 엔터 대기) |
| `odom_mapper.py` | odom(시뮬에서는 정확한 위치) 기반 레이캐스팅 점유격자 0.025m + 키보드 조종. 궤적·출발 pose 저장, Gazebo 리셋 감지 |
| `raceline.py` | 지도와 출발 pose로 폐루프 레이스라인 생성 + 지도 장애물 전역 회피 DP |
| `race_driver.py` | 주행: 로컬 lattice 회피(10Hz) + pure pursuit(40Hz) + 속도 프로파일 |
| `hybrid_astar.py` | 막혔을 때 쓰는 탈출 기동(전진/후진) + 옵션용 전역 탐색 |
| `make_obs_world.py` | 테스트용 0.65m 박스 world 생성 |
| `record_traj.py` | (`--carto` 모드 전용) 매핑 궤적 기록 |

옵션:
- `--no-gui`: Gazebo/RViz 창 없이 실행
- `--skip-mapping MAP.yaml --odom`: 매핑 생략(이미 만든 odom 지도 사용)
- `--carto`: 예전 방식(cartographer 매핑 + AMCL 위치추정)

## 작동 방식 요약

1. **매핑**
   - `/scan`의 각 레이를 odom pose로 지도에 투영한다.
   - 레이가 지나간 칸은 free, 끝점은 occupied로 기록한다(log-odds).
   - map 프레임은 odom 프레임과 같다(static TF).
2. **레이스라인**
   - 지도에서 독립된 작은 덩어리(박스 장애물)를 빼고 트랙만 남긴다.
   - 출발선 수직 절단 → 최소비용 루프 추출 → 벽 여유 0.30m 최단선(elastic band) → 법선 오프셋 QP(곡률 ≤1.25) 순으로 만든다.
   - 지도 장애물을 반영한 **전역 회피 DP**로 한 바퀴 회피 경로를 확정한다.
     - 0.3m 정거장 × 0.04m 횡간격
     - 박스 여유 0.45m
     - 좁은 틈 앞에서 미리 정렬
3. **드라이버**
   - **경로 계획**: 레이스라인 기준 Frenet lattice DP(0.4m × 18정거장)로 매 0.1초 경로를 다시 계획한다.
     - 비용: 이탈², 횡변화², 벽/장애물 근접 벌점, 이전 계획과의 차이(떨림 방지)
   - **장애물 인식**
     - 라이다 점 중 지도 벽이 아닌 점을 장애물로 본다(3초 기억).
     - 가까운 거리는 KD-tree로 정밀 계산한다.
     - 차체는 바퀴 포함 0.33×0.26m를 5원으로 근사한다.
   - **좁은 틈(0.35m)**: 정상 기준(여유 3cm)으로 막히면 빠듯한 기준(여유 1cm)으로 재탐색한다.
     - 이때 벽 거리는 지도 대신 **라이다 실측점**(벽 포함, 1초 기억)으로 잰다. 지도 래스터 오차로 박스-박스 게이트가 실제보다 좁게 잡히는 문제를 막기 위해서다.
     - 실측값은 지도값 −0.03 ~ +0.02 범위로 제한한다.
     - 빠듯한 구간과 그 앞뒤 1.6m만 0.2m/s로 지나고, 나머지는 정상 속도로 달린다. 진입 전에 자세를 바로잡을 거리를 확보하기 위해서다.
     - 그래도 막히면 Hybrid A\* 탈출 기동을 한다.
   - **속도와 조향**
     - 속도는 min(1.0, √(2.2/곡률), 여유 기반 감속)에 가감속 한계를 적용한다.
     - 조향은 후륜축 기준 pure pursuit다.
   - **스레드**: 계획과 제어는 별도 스레드에서 돈다. 계획이 느려도 40Hz 조향은 끊기지 않는다.
