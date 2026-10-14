## 응답 규칙
- 반말체(~이다), 간결하게, 필요한 말만. 코드/명령은 복붙 가능하게.(사용자가 직접 해야 할 경우가 생긴다면)

# LIMO 자율주행 레이싱 (완성: 2026-10-03)

Gazebo 시뮬레이션의 Ackermann 조향 LIMO로 **처음 보는 트랙을 현장 매핑 → 최단·최고속 경로 자동 생성 → 장애물 회피하며 무한 랩 주행**하는 시스템.
Nav2/AMCL/cartographer 없이 `scripts/race/`의 자체 구현으로 동작한다. 사용자가 여러 번 직접 검증하고 완성을 선언했다.

## 1. 환경
- Ubuntu 22.04 호스트 + Docker 컨테이너 `limo_gazebo_humble`(ROS2 Humble, Gazebo Classic 11). docker.sh 재실행 금지.
- 워크스페이스(호스트=컨테이너 동일 경로): `/home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble` → 항상 절대경로, `~` 금지.
  컨테이너는 이 워크스페이스만 마운트한다(USB `/media`, `~/Downloads` 다른 폴더는 컨테이너에서 안 보임 → 호스트에서 ws로 복사).
- 호스트에도 ROS가 있고 `~/ros2_ws`를 source 한다 → `ros2`, `python3 scripts/race/...`는 반드시 컨테이너(`root@` 프롬프트) 안에서.
- 컨테이너 접속: 첫 터미널 `docker start -ai limo_gazebo_humble`(꺼져 있으면 켜고 접속, 여기서 exit 하면 컨테이너 종료), 나머지 `docker exec -it limo_gazebo_humble bash`.
  (alias `limo-enter`, `gazebo`, `mapping`, `teleop` 등은 남아 있지만 시험에서는 alias 금지)
- 로봇 limo_ackermann: 제자리회전 불가, 휠베이스 0.24 m, 최대 조향 0.42 rad(최소 회전반경 약 0.56 m), 차체(바퀴 포함) 약 0.33×0.26 m, 최대속도 1.0 m/s.
- GitHub: tjdalsrb0428-cloud/limo-roboracer (private, main). 의미 있는 변경마다 구체적 메시지로 커밋.
- 노트북: LG gram 16Z90TP. 내장 Wi-Fi(Intel BE201, 8086:7740)는 커널 6.8용 펌웨어가 없어 미작동 → USB 동글(RTL8192EU) 사용. 해결은 커널 6.11+ 필요(시험 전에는 건드리지 않음).

## 2. 시험 규정 (중간고사 최종 공지, 2026-10-14)
- 순서: **Q1 준비 7분 → Q1 주행 3분 → Q2 준비 5분 → Q2 주행 5분**. 준비 초과 시 실패. 본인 시작 5분 전 도착해 노트북 부팅 등 준비 완료.
- **맵(트랙)은 바뀌지 않지만 기존 매핑 파일은 전부 지우고 매핑부터 다시** → 시험 전 `maps/exam/` 비우기(root 소유라 컨테이너에서
  `rm -rf maps/exam/*`; odom_mapper가 폴더 없으면 자동 생성). 저장 지도(q1 등) 사용 불가.
- Q1: 장애물 없는 맵에서 매핑 → global path 생성 → 주행.
- Q2: USB의 obstacle map .world 3개 중 학생이 각 그룹 시작 시 직접 골라 배포. **Q1 맵과 똑같고 장애물만 추가 → 추가 매핑 X, Q1 global path 그대로 사용**,
  local planner(라이다 실시간 장애물 인식)로 회피 주행.
- **Q2 랩타임 = 한 바퀴 완주 시간.** 학생이 "시작"을 외치는 순간부터 완주까지. 본인 폰 촬영으로 시간 측정 허용(화면 녹화는 리소스 때문에 비추천).
- 인터넷 연결 금지, 메모장·노션·단축키 금지, 모든 명령 직접 타이핑. **Tab 자동완성 가능, 윗 방향키(이전 명령) 금지.** 시험 중 질의응답 금지.
- 알고리즘 자유(오픈소스/Claude 허용). alias 금지.
- 시험은 본인 노트북. "컨테이너에 있던 것 다 지운다"는 최종 공지 기준 = 기존 매핑 파일 삭제로 해석(컨테이너 재생성 언급 없음).
  컨테이너는 조교 이미지 `yspark98/limo:gazebo_humble` 기반이고, 원본 이미지에는 scikit-image 없음(raceline.py 필요), numpy 1.21/scipy 1.8, pip 없음.
  지금 컨테이너에 추가 설치돼 있음(numpy 1.26, scipy 1.15, skimage 0.25). 컨테이너를 원본으로 재생성해야 하면 그 전에 `docker commit`으로 이미지 저장 필요.
  ws(호스트 마운트)는 컨테이너 삭제와 무관.
- "시작"과 동시에 랩타임 측정(심사위원 스톱워치, Gazebo 화면 보면서). 무한 랩으로 돌리다 학생이 Ctrl+C로 정지.
  출발선/완주 인지·자동정지 로직은 넣지 않음(심사위원마다 측정 방식이 다름).

## 3. 실행 방법 (단계별, 터미널 A~D)
| 터미널 | 용도 | 위치 |
|---|---|---|
| A | Gazebo + RViz (켜두기) | 컨테이너 |
| B | 매핑 → 레이스라인 → 드라이버 | 컨테이너 |
| C | 출발 명령 go.py | 컨테이너 |
| D | USB → ws 복사 | 호스트 |

```bash
# ===== Q1: 장애물 없는 맵 (매핑 + 주행) =====
# D (호스트): USB world 복사 (Q1, Q2 같이)
ls /media/mksung0428/ ; ls /media/mksung0428/USB이름/
cp /media/mksung0428/USB이름/Q1파일.world /home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble/
cp /media/mksung0428/USB이름/Q2파일.world /home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble/
# A
xhost +local:docker
docker start -ai limo_gazebo_humble
cd /home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble
ros2 launch limo_car roboracer_sim.launch.py world:=$PWD/Q1파일.world   # 기본 트랙이면 world:= 생략
# B
docker exec -it limo_gazebo_humble bash
cd /home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble
python3 scripts/race/odom_mapper.py maps/exam/q          # 키보드로 한 바퀴 → Gazebo Ctrl+R(출발선 복귀) → 터미널 B에서 Ctrl+C(정지+저장)
python3 scripts/race/raceline.py maps/exam/q.yaml        # global path 생성
python3 scripts/race/race_driver.py --map maps/exam/q.yaml   # READY 대기(RViz에 지도+파란 선)
# C
docker exec -it limo_gazebo_humble bash
cd /home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble
python3 scripts/race/go.py                 # "시작"과 동시에 엔터 → 약 0.1초 안에 출발
# 정지: B Ctrl+C(속도 0 발행 후 종료) → A Ctrl+C. 남으면: pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f rviz2

# ===== Q2: 장애물 있는 맵 (매핑 없음, Q1 지도 사용) =====
# A
ros2 launch limo_car roboracer_sim.launch.py world:=$PWD/Q2파일.world
# B  (odom_mapper·raceline 치지 않음)
python3 scripts/race/race_driver.py --map maps/exam/q.yaml
# C
python3 scripts/race/go.py
```
- 매핑은 반드시 장애물 없는 world에서. 장애물 world에서 매핑하면 장애물이 지도에 벽으로 찍힘(2026-10-14 `ifac1` 지도가 이 상태였음 → RViz에 장애물 표시).
- Q2 RViz: 회색 지도엔 장애물 없음이 정상. 파란 선(global)은 장애물을 관통, 초록 선(local)이 비켜 감.
- 드라이버는 `--map X.yaml`이면 `X_raceline.csv`(global path), `X_traj.csv`를 자동 로드(`--line`으로 지정 가능).
- 지도 이름 지정: `odom_mapper.py maps/exam/이름` → `raceline.py maps/exam/이름.yaml` → `race_driver.py --map maps/exam/이름.yaml`.
  이름 생략 시 `maps/exam/map.*`(매번 덮어씀). 저장된 지도로 시작하면 매핑 생략 가능(테스트용, 시험에서는 저장 지도 사용 불가).
- 저장해 둔 지도: `maps/exam/q1.yaml`(기본 트랙), `maps/exam/obs6.yaml`(obs6). `maps/exam/`는 gitignore, 컨테이너(root) 소유.
- 통합 실행(유지): `python3 scripts/race/exam.py [--world W.world]` → Gazebo~드라이버를 한 터미널에서, 매핑 끝·출발 모두 엔터(드라이버에 `--enter`).
  사용자 요청으로 통합본은 남겨두되 주로 단계별 실행을 쓴다.
- 준비 시간: Q1 Gazebo 기동~READY 약 90~140초(매핑 한 바퀴가 대부분, 제한 7분), Q2 1분 이내(제한 5분).
- 키보드(매퍼): i 전진, u/o 좌/우 전진, , 후진(m/. 좌/우 후진), k 정지, q/z 속도 ±(기본 0.5, 최대 0.8), w/x 조향 ±.

## 4. 파일 구조 (scripts/race)
| 파일 | 역할 |
|---|---|
| `odom_mapper.py` | odom 기반 점유격자 매핑 + 내장 키보드 조종, Ctrl+C 시 `.pgm/.yaml/_traj.csv` 저장 |
| `raceline.py` | 지도+출발 pose → 레이스라인(global path) `_raceline.csv/.png`. ROS 불필요 |
| `race_driver.py` | 주행 노드: 장애물 인식 + Frenet lattice local planner + pure pursuit, 지도/경로 RViz 발행 |
| `go.py` | /race/go(std_msgs/Empty) 발행 → 드라이버 출발 |
| `hybrid_astar.py` | 막힘 탈출용 Hybrid A*(전진/후진) + 옵션 전역 탐색 |
| `exam.py` | 통합 실행기(Gazebo→매핑→레이스라인→드라이버) |
| `make_obs_world.py` | 테스트용 0.65 m 박스 world 생성 |
| `record_traj.py` | (`exam.py --carto` 전용) cartographer 매핑 궤적 기록 |

수정한 패키지 파일:
- `src/limo_ros2/limo_car/gazebo/ackermann.xacro`: ackermann 플러그인 max_speed 0.5→1.0, 차체 충돌박스는 수업 배포본(2026-10-07) 값 0.19×0.31 사용(예전엔 0.31×0.19로 고쳐 썼음, 배포본으로도 ifac_roboracer_obstacles_1 무충돌)
- `ackermann_twist_adapter.py`: max_speed 1.0 (Twist 회전속도 → 조향각 변환)
- `launch/roboracer_sim.launch.py`: `gui` 인자, `ROS_LOCALHOST_ONLY=1`, `GAZEBO_IP=127.0.0.1` 설정(오프라인/학교망 혼선 대비). 스폰 (0, 1, yaw 0) = 출발선
- `rviz/gazebo.rviz`: Map, LaserScan, Raceline(/race/raceline 파랑), Local Path(/race/local_path 초록), Car Pose(/odom 주황 화살표)
- `scripts/ackermann_teleop.py`: ROS_LOCALHOST_ONLY=1
- 테스트 world: `src/limo_ros2/limo_car/worlds/test/obs1~6.world` (obs4 = 0.35 m 강제 게이트 2곳, obs5 = 박스-벽 0.35 m, obs6 = 가운데3/양옆3+3 혼합 9개)

## 5. 데이터 흐름
```
Gazebo ─/scan, /odom, TF(odom→base_link)─▶ odom_mapper ─▶ map.pgm/.yaml + map_traj.csv ─▶ raceline ─▶ map_raceline.csv
Gazebo ─/scan, /odom, TF─▶ race_driver ─/cmd_vel─▶ ackermann_twist_adapter ─▶ LIMO
go.py ─/race/go─▶ race_driver ─/map, /race/raceline, /race/local_path, TF(map→odom 항등)─▶ RViz
```
모든 스크립트와 런치는 `ROS_LOCALHOST_ONLY=1`을 **강제** 설정한다(컨테이너 대화형 셸 기본값이 0이라 setdefault로는 노드끼리 어긋났던 문제).

## 6. 알고리즘 상세

### 6-1. 매핑 (odom_mapper.py) — odom 기반 점유격자, log-odds
- 위치: Gazebo ackermann 플러그인 /odom = 시뮬레이터의 실제 위치(오차 0). → 위치를 아는 매핑(mapping with known poses), SLAM 불필요.
- 격자 0.025 m, 크기 60×60 m(첫 스캔 위치 중심 ±30 m; 36×24 m였을 때 큰 리허설 트랙에서 지도가 잘려 레이스라인 실패 → 확대).
- 각 레이: 지나간 칸 log-odds −0.4, 끝점 칸 +0.9(같은 스캔에서 깎인 몫 보상), 범위 [−2.0, 3.5]. 반사 없는 레이는 5 m까지 빈칸.
- 판정: p<0.35 빈칸(254), p>0.65 벽(0), 나머지 미관측(205). 저장은 관측 영역+10칸만 잘라 `.pgm` + `.yaml`(trinary, resolution, origin).
- 궤적 `_traj.csv`(x,y,yaw) 기록, 마지막 줄 = 종료 시점 /odom = **출발 pose**.
- Gazebo 리셋(Ctrl+R) 대비: 스캔 5개(약 0.5 s) 지연 반영. 시뮬 시간 역행(>0.2 s) 또는 위치 순간이동(>0.3 m) 감지 시 대기 스캔 폐기 + TF 버퍼 초기화 + 1 s 무시(유령 벽 방지).
- /map은 1 Hz, 그린 영역만 잘라 발행. map→odom 항등 static TF 발행. 터미널(tty)이면 키보드 조종(10 Hz /cmd_vel), Ctrl+C = 정지 5회 + 저장.

### 6-2. 위치추정 (Localization)
- 별도 알고리즘 없음. map 프레임 = odom 프레임(항등 static TF), odom = 실제 위치를 그대로 사용. 드라이버는 TF `odom→base_link`로 자세를 읽는다.
- RViz에서 라이다 점이 지도 벽과 겹치는 것은 위치가 정답이기 때문(추정 결과 아님).
- AMCL(cartographer 지도 + 파티클 필터)은 비교 후 미채택: 고속/긴 직선에서 위치가 튀어 0.35 m 틈에서 충돌, cartographer는 긴 직선에서 지도 늘어짐.
  `exam.py --carto`로 예전 방식 사용 가능(최근 미검증). 실제 로봇이면 휠 오도메트리 오차 때문에 위치추정이 필요하다.

### 6-3. Global path (raceline.py)
목표: 벽 여유 0.30 m를 지키며 차가 돌 수 있는(곡률 ≤ 1.25, 반경 0.8 m) 가장 짧은 한 바퀴.
1. **지도 정리(GridMap)**: 점유 칸을 1칸 팽창 후 8-연결 라벨링, 바운딩박스 대각선 1.4 m 미만인 독립 덩어리 = 박스 장애물(blobs)로 분리.
   빈칸 = 흰색만(occ_p<0.1), closing 2회로 작은 미관측 구멍 메움, 경계 대부분이 빈칸인 미관측 섬은 빈칸, 매핑 궤적 주변 0.6 m(밝은 칸) 빈칸 확정.
   궤적에서 0.3 m 이상 순간이동한 구간(리셋)은 잇지 않음(벽을 가로지르는 띠 방지). 거리변환으로 벽까지 거리 `dist`.
2. **루프 추출(extract_loop)**: 출발 pose에서 진행방향 수직선으로 트랙을 자르고(통과 불가 처리), 비용 1 + 0.5/max(dist, 0.05)(dist ≥ c_min만 통과)로
   출발점 앞 seed → 뒤 seed 최소비용 경로(skimage MCP_Geometric). c_min 0.25 → 0.20 → 0.16 → 0.13 순으로 시도.
3. **deseam**: 추출 루프가 출발점을 강제로 지나 생기는 S자 꺾임 제거 — 출발점 앞뒤 1.2 m를 지우고 주기 스플라인으로 재연결(여유 위반 시 원본).
4. **taut(elastic band)**: 가우시안 평활 후 0.4 → 0.2 → 0.1 m 간격으로, 라플라시안(0.3) 당김 + 벽 0.30 m 이내 점을 기울기 방향으로 최대 0.04 m 밀어냄 반복 → 최단선.
5. **곡률 제한 QP(optimize/solve_offsets)**: 기준선 r_i, 법선 n_i, p_i = r_i + a_i n_i, lb ≤ a ≤ ub(벽 여유 0.30 m 구간).
   min Σ|p_{i+1}−p_i|² + Σ w_i|p_{i+1}−2p_i+p_{i−1}|², L-BFGS-B. w_i 초기 (0.3/ds)², 곡률 > 1.25×0.92 구간 주변 0.6 m의 w_i ×1.8 반복(최대 25회).
   곡률 ≤ 1.7(K_ACCEPT) & 여유 OK면 채택. QP 결과 여유 < 0.12(0.35 m 틈 등)면 추출 루프를 가우시안(σ 4→2→1) 평활한 중앙 루프 사용.
   (옵션 `--astar`: 곡률 미달 시 전역 Hybrid A* 기준선 — 기본 비활성)
6. **장애물 회피 DP(avoid_dp, 박스가 있을 때)**: 0.3 m 정거장 × 오프셋 −1.6~+1.6 m(0.04 m).
   벽 거리 = 지도 dist − 0.03(STATIC_BIAS), 박스 거리 = blobs 거리변환. 차체 원 5개로 에지(4개 보간점) 검사.
   에지 비용 = (0.6 + 80·f_t)·Δo²/ds + 0.6·ds·o² + ds·[40·max(0, 0.30−벽)² + 150·max(0, 0.45−박스)²], f_t = clip((0.25−여유)/0.08, 0, 1)(좁은 곳 횡이동 억제), |Δo| ≤ 0.21 m.
   **2차 DP**: 상태 (직전 오프셋, 현재 오프셋), 곡률 ≈ k_ref/(1 − k_ref·o) + o″(o″ = 2차 차분/ds²)가 한도 초과면 금지, 곡률 벌점 0.05·ds·o″².
   기준 단계: (c_hard 0.165, 곡률 1.5) → (0.142, 1.5) → (0.165, 1.75) → (0.142, 1.75) → (0.12, 1.75) → (0.12, 무제한).
   고정점(오프셋 0)은 박스에서 가장 먼 정거장(출발선 근처에 두면 게이트 앞 S자 꺾임 발생). 결과는 σ=1 정거장 평활(충돌 구간 제외) 후 0.1 m 리샘플.
7. 출발 pose에 가장 가까운 점이 0번이 되게 회전해 저장. 인자 생략 시 `maps/exam/map.yaml`, 궤적 자동, 출발 pose = 궤적 마지막 줄.

### 6-4. 장애물 인식 (race_driver.py on_scan)
- 라이다 점을 맵 좌표로 변환(스캔 시점의 정확한 TF만 사용, 없으면 버림 — 오래된 자세로 변환하면 장애물이 번짐). 0.12 m < r < 5.5 m.
- 지도 벽에서 0.22 m(WALL_TOL) 이상 떨어진 점 = 장애물. 0.15 m 내 이웃 2개 미만(자기 포함 3개 미만) 외톨이 점 = 잡음 제거.
- 최근 3 s(OBS_MEM) 누적. 그림자 확장은 비활성(비스듬한 면 그림자가 틈을 막음).
- 장애물 거리: 로컬 ±6 m 창의 2.5 cm 거리변환 + 0.3 m 이내는 KD-tree(1 cm 중복 제거)로 실제 점까지 정밀 거리. 로그 `obs=N` = 장애물 점 수.
- 지도 박스 셀을 장애물 점으로 넣는 것은 비활성(USE_BLOB_PTS=False, 래스터 오차로 틈 좁아짐) → 박스 회피는 라이다로.

### 6-5. Local planner (race_driver.py lattice/plan_step, 10 Hz)
- Frenet: 레이스라인(주기 3차 스플라인, 0.05 m) 기준 s(진행), d(횡편차).
- 정거장 0.4 m × 18개(7.2 m), 횡후보 −1.6~+1.6 m(0.08 m, 41개) + 정거장별 "여유 능선" 노드 최대 6개(0.01 m 해상도 여유 극대점 = 좁은 틈 중앙).
- 차체: 원 5개(오프셋 −0.12, −0.06, 0, 0.06, 0.12 m, 반지름 0.1325 m). 원 중심 최소 거리 C_HARD 0.165 m(겉면 여유 약 3 cm).
  정적맵 거리 = 지도 dist − 0.03. 첫 구간 앞부분은 C_START 0.12까지 완화(이미 붙어 있어도 빠져나오는 경로 허용).
- 비용 = 0.6·d² + (3 + 80·f_t)·Δd² + 1.0·진행방향 오차² + 40·(0.30−벽)² + 150·(0.45−장애물)² + 1.5·(직전 계획과의 차이)². |Δd| ≤ 0.28 m/정거장.
- DP로 끝까지(k=17) 가는 최소비용 경로 → 스플라인 평활(여유 위반 시 직선 보간) → 속도 프로파일.
- **막힘 계층**: 18정거장 못 가면 BLOCK → 빠듯한 기준 C_TIGHT 0.142(여유 약 1 cm)로 재탐색(TIGHT). 이때 벽 거리는 라이다 원점(벽 포함, 1 s 기억)의
  1 cm 거리표(±3.5 m)로 계산, 지도값 −0.03 ~ +0.02로 제한(라이다가 그 면을 본 경우만; 박스-벽 게이트가 지도에서 ~2 cm 좁게 잡혀 정지하던 문제 해결).
  TIGHT 구간(여유 < C_HARD)과 앞뒤 1.6 m(TIGHT_PAD)만 0.2 m/s. BLOCK이면 0.5 m/s 이하, 막힌 지점 0.35 m 앞 정지.
  여전히 막혀(k ≤ 3) 0.6 s 이상 정지 → Hybrid A* 탈출(목표 = 레이스라인 앞 1.6~6 m 중 여유 ≥0.25 지점, 3 s 한도, 프리미티브 0.2 m, 곡률 ±1/0.6·±1/1.2·0, 후진 비용 2, 전환 비용 1).
  명령 > 0.15 m/s인데 1.2 s 정지 → 탈출 시도, 실패 시 0.9 s 후진(−0.3 m/s).

### 6-6. Controller (race_driver.py control_step, 40 Hz)
- Pure pursuit(후륜축 기준, base_link 뒤 0.12 m): κ = 2·sinα / L_d, ω = v·κ (어댑터가 δ = atan(L·κ)로 변환), |κ| ≤ 1/0.56.
- L_d = clip(0.25 + 0.55·max(|v|, 0.8·v_ref), 0.35, 0.85) × (0.6 + 0.4·여유비), 최소 0.22(TIGHT)/0.28.
- 속도 프로파일: v ≤ min(1.0, √(2.2/κ), 0.35 + 0.65·여유비) → 뒤→앞 감속 1.8 m/s², 앞→뒤 가속 2.0 m/s². 명령은 min(v_ref, √(2.2/|κ|)), 방향 오차 > 1.4 rad면 0.3.
  **2026-10-07부터 기본 = 감속 없음**: 여유비 항(0.35+0.65·여유비)과 TIGHT 0.2 m/s를 끈다(곡률 제한, BLOCK 0.5·정지는 유지). 예전 동작은 `race_driver.py --slow`. ifac_roboracer_obstacles_1에서 무충돌, 랩타임 단축 확인.
  여유비 = min(clip((벽−c)/(0.28−c)), clip((장애물−c)/(0.40−c))).
- 계획(cg_plan)·제어(cg_ctrl)·센서(cg_sens) 콜백 그룹 분리 + MultiThreadedExecutor(4), TF 리스너 전용 스레드.
  (단일 스레드였을 때 계획이 제어를 막아 지연된 자세로 장애물이 번지던 문제 → 가장 큰 개선)

### 6-7. 출발·정지·랩·표시
- 출발: 단계별 = /race/go 구독(go.py 0.1 s, `ros2 topic pub --once /race/go std_msgs/msg/Empty` 0.7 s). `--enter`(exam.py) = 엔터, `--go` = 즉시(테스트).
- 정지: rclpy 시그널 핸들러 비활성(SignalHandlerOptions.NO) → Ctrl+C를 KeyboardInterrupt로 받아 속도 0을 10회 발행 후 종료(플러그인이 마지막 명령을 유지하므로 필수).
- 랩: 레이스라인 인덱스가 90 % 이후 → 10 % 이전으로 넘어가면 1랩(출발 5 s 이내 제외), `=== LAP n: xx.xs ===`(sim time, GUI 실행 시 RTF ≈ 1.0).
- 드라이버는 시작 시 불러온 지도를 /map(transient local), map→odom 항등 static TF, 레이스라인을 /race/raceline으로 발행 → 저장 지도로도 RViz 표시.
- 상태 줄: `[t] lapN s=진행거리 v=실제/명령 o=횡편차 clr=경로 최소여유 obs=장애물점수 BLOCK/TIGHT k=도달정거장(17=전부) plan=계획ms`.

### 6-8. 주요 파라미터 요약
| 구분 | 값 |
|---|---|
| 매핑 | 0.025 m, 60×60 m, log-odds −0.4/+0.9, [−2, 3.5], 지연 5스캔 |
| 레이스라인 | 벽 여유 0.30, 곡률 1.25(채택 ≤1.7), 박스 분리 1.4 m, deseam 1.2 m |
| 회피 DP | 0.3 m × 0.04 m, 박스 여유 0.45, 곡률 1.5/1.75, c_hard 0.165/0.142/0.12 |
| 장애물 인식 | WALL_TOL 0.22, 기억 3 s, 범위 5.5 m, KD-tree 0.3 m 이내 |
| 로컬 계획 | 10 Hz, 0.4 m × 18, ±1.6 m(0.08) + 능선 6, C_HARD 0.165, C_TIGHT 0.142, DO_MAX 0.28 |
| 비용 가중치 | W_DEV 0.6, W_DO 3, W_DO_TIGHT 80, W_HEAD 1, W_CLR 40(0.30), W_OBS 150(0.45), W_HYST 1.5 |
| 제어 | 40 Hz, L_d 0.22~0.85, V_MAX 1.0, A_LAT 2.2, A_ACC 2.0, A_DEC 1.8, V_MIN_CLR 0.35, V_TIGHT 0.2, TIGHT_PAD 1.6, V_BLOCKED 0.5 |

## 7. 결과 (sim time, 무충돌)
- Q1 기본 트랙 42.4 s/랩(처음 Nav2+TEB 92~99 s). 규정형 랜덤 장애물 맵(r1~r6, 박스 4~5개) 44.5~49.4 s, obs5 46.7 s, obs4(0.35 m 강제 게이트 2곳) 약 67 s.
- **Q2 방식(장애물 없는 지도 + 장애물 world, 매핑 없음, GUI 끔, 2026-10-14)**: 모두 무충돌·탈출기동 0.
  기본 트랙 q1 지도: 박스 r1~r6·obs5 43.2~47.8 s. 배포 `ifac_roboracer_obstacles_1` 6바퀴 42.8~43.8 s(장애물 최소 여유 0.04 m, 가장 빠듯).
  생성 트랙(`.sim_tmp/tk`, 장애물 3개 박스/판 혼합, 5바퀴): 복잡 c1 61.8, c2 55.7, c3 55.8(최소 여유 0.08), c4 60.6 / 단순 s1 57.6, s2 52.9 s.
  새 복잡 트랙 `cx_q1/cx_q2.world`(ws 루트)는 사용자가 직접 확인. Hybrid A* 탈출은 이 테스트들에서 발동 안 해 미검증.
  사용자가 직접 5바퀴 중 장애물 충돌·Hybrid A* 이상 목격(장애물 찍힌 ifac1 지도 사용 시로 추정, GUI 부하 가능성) — 재현 안 됨, 원인 미확정.
- 준비 90~140 s. Wi-Fi 끈 상태 완주, 리허설 트랙(~/Downloads/rehearsal_usb, 큰 트랙)에서 사용자 직접 검증.
- 한계: 위치를 시뮬 정답(odom)에 의존(실차는 위치추정 필요). 0.35 m 틈은 0.2 m/s 통과라 랩 증가. 장애물 9개처럼 촘촘한 배치(obs6)는 가끔 접촉 — 시험은 박스 3개라 대상 외.

## 8. 폐기한 방식과 이유
- Nav2 goThroughPoses/waypoint, followPath N랩 이어붙이기: goal checker가 루프(출발=도착)를 SUCCEEDED로 오판, trim_last도 무효 → "도착" 개념 의존 구조 전부 폐기.
- TEB followPath + spline 회피(archive/old_teb): Ackermann에서 infeasible 리셋 반복, 평균 0.45 m/s.
- AMCL 위치추정, cartographer 매핑: 위 6-2.
- TUM racing line 전환, waypoint 자동추출(skeleton): 반복 실패. 수동 웨이포인트 클릭: 준비 시간 부담 → 자동 레이스라인으로 대체.
- Frenet pure pursuit 회피(후진 반복/충돌), costmap 없는 pure pursuit(회피 불가).
- 장애물 스캔점 그림자 확장(비스듬한 면 그림자가 틈을 막음), 지도 박스셀을 장애물점으로 추가(래스터 오차로 틈 좁아짐).
- 회피 DP 1차(곡률 무제약)·출발선 고정점: 급커브/S자 꺾임 → 2차 DP + 고정점 이동.
- 출발 엔터/`touch go`: 사용자 요청으로 명령 실행(go.py)으로 변경.

## 9. 개발 중 해결한 주요 문제
- 플러그인 max_speed 0.45/0.5 상한 → 1.0. 차체 충돌박스 x/y 뒤바뀜.
- 단일 스레드 executor가 제어를 굶김 → 멀티스레드(가장 큰 개선).
- lattice 이산화로 좁은 틈 놓침 → 능선 노드. 격자 양자화 → KD-tree 정밀 거리.
- 박스-벽 게이트가 지도에서 좁게 잡혀 정지 → TIGHT 시 라이다 실측 1 cm 거리표. TIGHT 감속을 경로 전체 → 빠듯 구간 ±1.6 m로 한정.
- Wi-Fi 순단 시 DDS 탐색 실패/학교망 혼선 → ROS_LOCALHOST_ONLY=1, GAZEBO_IP=127.0.0.1 강제.
- 출발 pose x=−7e−06 지수표기를 argparse가 옵션으로 오인 → 고정소수점 전달. 출발 직후 가짜 LAP → 5 s 이내 무시.
- Ctrl+C 후 계속 직진 → 정지 명령 발행. Ctrl+R 리셋 시 유령 벽 → 지연 반영/폐기.
- exam.py 중단 시 매퍼/Gazebo 고아 프로세스 → 항상 정리(SIGHUP/SIGTERM 포함).
- 저장 지도로 시작 시 RViz 지도 없음 → 드라이버가 /map 발행. 큰 트랙 지도 잘림 → 60×60 m.
- 드라이버 시작 시 `AttributeError: 'Race' object has no attribute 'm2o'`(GUI 부하 때 발생): TF 리스너 스레드가 노드 전체 콜백을 돌려 초기화 중 /scan 이 먼저 처리됨 → TF 리스너를 `__init__` 맨 끝에서 시작(2026-10-09).

## 10. 테스트 하네스 (.sim_tmp/, gitignore)
- `batch.sh TAG world...`(결과 `.sim_tmp/batch_TAG.txt`), `pipeline_test.sh WORLD TAG RUNSEC MAPPATH`(env: RESET=1 리셋 포함, FV=매핑 속도, RUNSEC, GUI=1).
- `gt_follow.py`(GT odom으로 매핑 주행 대역), `collide.py WORLD SEC`(정답 위치 기준 충돌/최소 여유), `gtgrid.py`, `map_path.py`, `rand_world.py`(규정형 랜덤 world), `mix_world.py`(가운데/양옆 혼합), `flowB_test.sh`(단계별 실행 + Ctrl+C 정지 검증), `kill.sh`, `dx.sh`(컨테이너 실행, LOCALHOST 설정).
- Q2 방식: `q2_test.sh WORLD_ABS TAG MAP RUNSEC [드라이버 인자]`(매핑 없이 저장 지도 + 장애물 world, 결과 한 줄).
  `tk/gen.py NAME SEED complex|simple`(랜덤 트랙 Q1/Q2 world + 매핑 경로, 출발 (0,1)+x), `tk/tk_test.sh NAME`(GT 매핑→raceline→Q2 5바퀴, 결과 `tk/results.txt`).
  `plate_world.py SEED OUT`(기본 트랙 판 3개 랜덤), `cx/make_worlds.py`(cx 트랙). collide.py는 `*obstacle_N` 판도 판정.
- 시뮬 동시 실행 금지(같은 localhost 토픽 충돌). 테스트 중 사용자는 Gazebo 켜지 않기.

## 작업 원칙
- 파라미터 수정 시 관련 파일 전부 `git diff` 확인(sed 누락 버그 경험).
- 큰 변경 전 커밋/백업, 한 단계씩 검증 후 진행.
- 시험 준비 시간이 짧다는 것(5분)을 항상 고려: 명령은 짧게, 실패 시 대처가 바로 가능하게.

## 2026-10-15 실전 연습 결과 (화면 켬)
- 사용자 매핑 지도 + 배포 장애물 맵, 감속 없음: 판(−8, 1.15) 앞에서 막판 좌우 뒤집기(o −0.04→−1.10)로 반복 충돌. 화면 꺼도 충돌.
  같은 트랙·장애물이라도 지도(매핑할 때마다 수 cm 차이)에 따라 비킬 방향 결정이 바뀜 → 감속 없음은 지도 운에 좌우됨.
- 계획 시간: q1 화면 끔 99 ms → 사용자 지도 balanced 191 ms, 화면 켬 222 ms, performance 114 ms(발열로 사용자가 balanced 유지).
- `--slow`(화면 켬, 사용자 지도): 배포 맵 5바퀴 + r1~r6·p1 무충돌, 44.4~49.6 s → **Q2는 `--slow` 권장**.
- 충돌 후 탈출 실패: 장애물에 닿은 상태라 시작부터 충돌 판정 → Hybrid A* 즉시 실패, 후진 검사도 "공간 없음". (충돌 시 랩 초기화라 우선순위 낮음)
- 차체: 배포본 충돌박스 0.19(앞뒤)×0.31(좌우) → Gazebo 차 폭 0.31인데 플래너는 0.265(원 반경 0.1325), collide.py도 0.26으로 판정. 미수정.
- 시도 후 보류: 회피 방향 유지(장애물 덩어리별 좌/우 고정, W_SIDE 30) — 화면 끔 배포 맵 3바퀴 무충돌, 화면 켬 배포 맵 2바퀴째 충돌.
  **작업 트리 race_driver.py에 미커밋 상태로 남아 있음**(백업 `.sim_tmp/race_driver_side2.py`). 원래 코드 = `git checkout scripts/race/race_driver.py`.
  사용자가 이 코드로 새 지도 + 감속 없음 주행 시 무충돌 확인(1회).
- 실패한 시도(되돌림): 시험형 하드 맵(gate 유형이 트랙을 거의 막아 비현실적) — 규정 밖 배치 만들지 말 것. 넓은 쪽 우선 방향 선택 — 충돌.
- 연습용 장애물 모델 `models/obstacle_box/{box_040,box_065,plate_08,plate_10,plate_12}.sdf` 미커밋. spawn 은 `export ROS_LOCALHOST_ONLY=1` 필요.

## 다음 할 일
- 시험 직전 `maps/exam/` 비우기(최종 공지: 기존 매핑 파일 삭제 후 재매핑).
- race_driver.py 미커밋 수정(회피 방향 유지) 채택/폐기 결정. 시험 전 반드시 작업 트리 정리.
- 사용자가 본 Q2 충돌·Hybrid A* 이상의 재현 조건 확인(지도, world, 바퀴, 로그 문구). 미확인 상태에선 코드 수정 안 함.
- GitHub push 안 된 커밋 있음(사용자 요청 시 push).
