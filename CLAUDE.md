##응답 규칙
- 반말체(~이다), 간결하게, 필요한 말만. 코드/명령은 복붙 가능하게.(사용자가 직접 해야 할 경우가 생긴다면)

## 환경
- Ubuntu 22.04, ROS2 Humble, Gazebo. 컨테이너 접속: `limo-enter` (docker.sh 재실행 금지)
- 워크스페이스(호스트=컨테이너 동일 경로): /home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble → 항상 절대경로, `~` 금지
- alias: gazebo, localization, mapping, teleop (navigation 은 archive 로 옮긴 TEB 설정을 가리켜 더 이상 사용 안 함)
- 로봇 limo_ackermann: 제자리회전 불가, wheelbase 0.24, max_steer 0.42, 최대속도 0.9m/s. Ackermann 모드만 사용
- GitHub: tjdalsrb0428-cloud/limo-roboracer (private, main). 의미 있는 변경마다 구체적 메시지로 커밋

## 시험 규정
- Q1(장애물 없는 맵) 1바퀴, Q2(장애물 있는 맵) 1바퀴(랩타임 측정)
- Q1, Q2 각각 코드 준비~마무리 총 5분, 초과 시 실패
- Q2: 심사위원이 USB로 .world 3개 준비, 학생이 시작 시 직접 선택해 배포
- "시작" 외치는 순간부터 랩타임 기록 → 터미널에 엔터 치면 즉시 출발해야 함
- 트랙은 3개 중 랜덤 하나, 현장 매핑부터. 알고리즘 자유(오픈소스/깃허브 오픈소스/Claude 허용)
- 장애물: 0.65m 정육면체, 틈 0.35m 기준
- 무한랩으로 돌리다 내가 직접 정지(Ctrl+C). 출발선/완주 인지·자동정지 로직은 넣지 않음(심사위원마다 측정 방식이 다름)

## 현재 확정 구조 (scripts/race, 상세: scripts/race/README.md)
- 실행 한 줄: `python3 scripts/race/exam.py [--world W.world]` → 텔레옵 1바퀴 매핑 후 엔터 → READY → "시작"과 함께 엔터
- 매핑: odom_mapper.py (odom=시뮬 정확 위치, 0.025m 점유격자, map=odom 프레임, AMCL 미사용). cartographer+AMCL 은 `--carto` 옵션
- 레이스라인: raceline.py (지도+출발pose 자동, 수동 클릭 없음). 독립 박스 제외 QP 최단선(곡률≤1.25, 벽여유 0.30) + 지도 장애물 전역 회피 DP
- 주행: race_driver.py (Nav2 없음, /cmd_vel 직접). Frenet lattice 10Hz + pure pursuit 40Hz, 멀티스레드, 라이다 장애물 KD-tree 정밀거리, 막힘 시 빠듯기준(0.35m 틈) → Hybrid A* 탈출
- Gazebo 수정: ackermann 플러그인 max_speed 0.5→1.0, 차체 충돌박스 x/y 뒤바뀜 수정(0.31x0.19)
- 결과(sim time, 3a78b6d): Q1 42.4s/랩, 랜덤 장애물맵 r1~r6 44.5~49.4s, obs5 46.7s, obs4(강제게이트 2곳) 61~62s, 전부 무충돌, 준비 90~135s
- exam.py: ROS_LOCALHOST_ONLY=1/GAZEBO_IP=127.0.0.1 (시험장 오프라인 대비), 출발 pose는 엔터 시 /odom 에서 읽음. 사용자가 Wi-Fi 끈 상태로 장애물 없는 맵 완주 확인(2026-10-01)
- 테스트 world: src/limo_ros2/limo_car/worlds/test/obs1~6.world (obs4 = 0.35m 강제 게이트 2곳, obs5 = 박스-벽 0.35m, obs6 = 가운데/양옆 혼합 9개)
- 실행: A) exam.py 통합 / B) 터미널1 ros2 launch limo_car roboracer_sim.launch.py world:=절대경로, 터미널2 odom_mapper.py → raceline.py → race_driver.py (인자 생략 시 maps/exam/map.*)

## 폐기된 방식
- goThroughPoses/waypoint 재계획, Nav2 followPath로 N랩 이어붙이기(goal checker가 SUCCEEDED 오판정하는 구조적 문제, trim_last도 무효)
- Racing line(TUM) 전환, waypoint 자동추출(skeleton 등 반복 실패 → 수동 클릭 확정)
- Frenet pure pursuit 회피(후진 반복/충돌), costmap 없는 pure pursuit(회피 불가)
- Nav2 goal/SUCCEEDED 개념에 의존하는 구조 전부
- TEB followPath + spline 회피(run_teb_follow3.py, 평균 0.45m/s, infeasible 리셋 반복) → archive/old_teb
- 장애물 스캔점 그림자 확장(비스듬한 면 그림자가 틈을 막음), 지도 박스셀을 장애물점으로 추가(래스터 오차로 틈 좁아짐)

## 작업 원칙
- 파라미터 수정 시 관련 파일 전부 `git diff` 확인(sed 누락 버그 경험)
- 큰 변경 전 커밋/백업, 한 단계씩 검증 후 진행
- 리포트용 스크린샷이 필요한 시점은 알려줄 것(중요한 것만)

## 남은 과제
1. 0.35m 강제 게이트(obs4): 해결(TIGHT 시 라이다 실측 벽거리). 게이트 구간 0.2m/s 라 랩 +15s
2. Gazebo GUI 켠 상태 실시간 계수(RTF) 확인: 랩타임은 sim time 기준 출력
3. 속도 상한 1.0(플러그인/adapter/드라이버 V_MAX) 적용 완료
4. USB world 가 별도 모델/heightmap 파일을 참조하면 폴더째 복사 필요(당일 확인)
그리고 반드시 알아야 하는거는 우리가 지금은 주어진 게 많아서 이것저것 해볼 수 있지만 실제 랩타임 측정할 때는 다 전부 바로바로 해야돼. 준비시간이 적잖아. 그거까지 생각해야돼

## 진행 상황 / 다음 할 일 (2026-10-01 갱신)
- 완료: 피드백 (a) Ctrl+C 정지 (b) killall 대신 pkill 안내 (c) RViz 레이스라인(파랑)/로컬경로(초록)/차량 화살표(주황)
  (d) 단계별 실행 추가(exam.py 통합 실행은 유지: 사용자 요청 "통합한 건 분리하지 마") (e) Ctrl+R 리셋 시 유령 벽 방지 — 커밋 959589b
- obs4 2랩째 게이트 접촉 원인 = 회피 DP 고정점/추출 루프가 출발점을 강제 통과해 S자 꺾임 → deseam + 고정점 이동 + TIGHT_PAD 1.6 으로 해결
- 사용자가 obs6(가운데3/왼벽3/오른벽3 장애물) 직접 테스트: 완벽 주행, Hybrid A* 탈출기동도 성공 (2026-10-01)
- 사용자가 Wi-Fi 끈 상태로 Q1 완주 확인
- 남은 것: (1) 959589b 이후 전체 회귀 테스트(Q1, r1~r6, obs5, obs4) 미실시 → 다음에 먼저 돌릴 것
  (2) 사용자가 추가로 고칠 것 요청 예정 (3) 보고서(사용자가 만족 후 요청)
- 보고서: 스크린샷 PNG + Word/PDF, 4쪽 이내
  - 제출 항목: ① 적용 주행 알고리즘 및 선정 근거 ② 동작 원리 및 주요 파라미터 ③ 트랙 주행 결과(완주 여부, 주행 화면)
  - 형식: 1.실습 목적 및 목표 2.실습 관련 이론 3.실습 수행 내용(3.1 절차/방법 3.2 결과 3.3 실행 결과 화면) 4.결과 분석 및 고찰 5.참고 자료
- 노트북: LG gram 16Z90TP, 내장 Wi-Fi Intel BE201(8086:7740)는 커널 6.8 에 맞는 펌웨어(bz-a0-fm-c0-80~86)가 없어 미작동,
  USB 동글(RTL8192EU) 사용 중. 해결하려면 커널 6.11+ 필요 → 시험 전에는 건드리지 않기로 권고
- 테스트 하네스: .sim_tmp/ (gitignore) batch.sh, pipeline_test.sh(RESET=1, FV=, RUNSEC=), collide.py, mix_world.py
