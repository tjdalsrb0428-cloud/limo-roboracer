# 레이싱 시스템 (scripts/race)

Nav2/TEB 없이 **odom 매핑 → 자동 레이스라인 → 자체 드라이버**로 달린다.
Q1, Q2 모두 같은 절차를 따른다.

## 시험 당일 절차 (터미널 2개)

**터미널 1** (`limo-enter`로 들어간 뒤):
```bash
cd /home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble
python3 scripts/race/exam.py --world /USB경로/심사위원.world      # Q1(기본 트랙)은 --world 생략
```

1. Gazebo가 뜨고, `2) 매핑 시작` 메시지가 나온다.
2. **터미널 2**(`limo-enter`)에서 텔레옵으로 트랙을 **한 바퀴** 돈다.
   ```bash
   python3 /home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble/src/limo_ros2/limo_car/scripts/ackermann_teleop.py
   ```
   - 매핑은 odom 기반이라 지도 왜곡이 없다. 속도는 아무래도 좋고, 벽과 장애물이 한 번씩 보이기만 하면 된다.
   - 마지막에는 **출발할 위치에, 레이스 진행 방향을 바라보게** 세운다(이 위치와 방향이 레이스라인의 기준이 된다).
   - **텔레옵을 Ctrl+C로 종료**한다(출발 후 /cmd_vel 충돌 방지).
3. 터미널 1에서 **엔터**를 친다. 지도 저장(1초 미만) → 레이스라인 생성(2~5초) → `READY`가 표시된다.
4. 심사위원이 "시작"을 외치면 **엔터**를 친다. 즉시 출발하고 무한 랩을 돈다. 랩타임은 `=== LAP n: xx.xs ===`로 표시된다.
5. 끝나면 Ctrl+C를 누른 뒤 `pkill -f scripts/race; killros`를 실행한다.

실측 준비 시간(Gazebo 기동부터 READY까지): 약 90~110초. 그중 매핑 한 바퀴가 60~90초다.
레이스라인 확인 그림은 `maps/exam/map_*_raceline.png`에 저장된다(파랑=루프, 빨강=레이스라인).

## 파일

| 파일 | 역할 |
|---|---|
| `exam.py` | 통합 실행기. Gazebo → 매핑 → 저장 → 레이스라인 → 드라이버(exec, 엔터 대기) |
| `odom_mapper.py` | odom(시뮬에서는 정확한 위치) 기반 레이캐스팅 점유격자 0.025m. 궤적도 저장 |
| `raceline.py` | 지도와 출발 pose로 폐루프 레이스라인 생성 + 지도 장애물 회피 DP |
| `race_driver.py` | 주행: 로컬 lattice 회피(10Hz) + pure pursuit(40Hz) + 속도 프로파일 |
| `hybrid_astar.py` | 막혔을 때 쓰는 탈출 기동(전진/후진) + 옵션용 전역 탐색 |
| `make_obs_world.py` | 테스트용 0.65m 박스 world 생성 |
| `record_traj.py` | (cartographer 모드 전용) 매핑 궤적 기록 |

옵션:
- `--carto`: 예전 방식(cartographer 매핑 + AMCL 위치추정)
- `--skip-mapping MAP.yaml --odom`: 매핑 생략(이미 만든 odom 지도 사용)
- `--no-gui`: Gazebo/RViz 창 없이 실행

## 작동 방식 요약

1. **매핑**: `/scan`의 각 레이를 odom pose로 지도에 투영한다. 지나간 칸은 free, 끝점은 occupied(log-odds)로 기록한다. map 프레임은 odom 프레임과 같다(static TF).
2. **레이스라인**: 지도에서 독립된 작은 덩어리(박스 장애물)를 빼고 트랙만 남긴다.
   - 출발선 수직 절단 → 최소비용 루프 추출 → 벽 여유 0.30m를 지키는 최단선(elastic band)
   - → 법선 오프셋 QP로 곡률 ≤1.25(부족하면 국소 재가중) 경로를 만든다.
   - 그다음 지도 장애물을 반영한 **전역 회피 DP**(0.3m 정거장 × 0.04m 횡간격, 박스 여유 0.45m, 좁은 틈 앞에서는 미리 정렬)로 한 바퀴 회피 경로를 확정한다.
3. **드라이버**:
   - 레이스라인 기준 Frenet lattice DP(0.4m × 18정거장)로 매 0.1초 경로를 다시 계획한다.
   - 비용은 이탈², 횡변화², 벽/장애물 근접 벌점, 이전 계획과의 차이(좌우 떨림 방지)다.
   - 장애물은 라이다 점 중 벽이 아닌 점(3초 기억)으로 잡고, 가까운 거리는 KD-tree로 정밀 계산한다.
   - 차체는 바퀴 포함 0.33×0.26m를 5원으로 근사한다.
   - 막히면 빠듯한 기준(0.35m 틈용)으로 재탐색하고 0.2m/s로 통과한다. 그래도 막히면 Hybrid A\* 탈출 기동을 한다.
   - 속도는 min(0.9, √(2.2/곡률), 여유 기반 감속)에 가감속 한계를 적용한다. 조향은 후륜축 기준 pure pursuit다.
   - 계획과 제어는 별도 스레드에서 돌아서, 계획이 느려도 40Hz 조향은 끊기지 않는다.
