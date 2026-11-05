# 다음 실습 절차서 (2시간, LIMO1 / LIMO Pro 공용)

목표: **셋업 → 측정(보정값) → 정리**를 확실히 끝낸다. 매핑·주행은 해당 코드(할 일 5~7번)가 준비되면 이 문서에 추가한다.
원칙: 로봇에서는 실행·측정만 한다. 로봇 호스트 설정은 건드리지 않는다. 대용량 작업(큰 복사·압축)은 주행·기록 중 금지(라이다가 죽음).

| 시간 | 단계 |
|---|---|
| 0~5분 | 1. 하드웨어 확인 |
| 5~20분 | 2. 네트워크·SSH |
| 20~25분 | 3. 드라이버 실행 |
| 25~35분 | 4. 우리 환경(setup.sh) |
| 35~75분 | 5. 측정 |
| 75~105분 | (매핑·주행 — 준비되면 추가) / 측정 반복·재확인 |
| 105~120분 | 6. 결과 저장, 7. 정리 |

---

## 0. 전날 준비 (노트북)
- [ ] USB 메모리에 이미지 백업 복사: `cp /home/mksung0428/limo-robot_v1.tar /media/mksung0428/USB이름/`
- [ ] 줄자, 노트북 충전기, USB Wi-Fi 동글, 폰(핫스팟 대비) 충전
- [ ] 코드 최신 push 확인: `cd /home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble && git status && git log origin/main -1 --oneline`
- [ ] **GitHub 토큰 만들기** (로봇에서 private 저장소 받을 때 비밀번호 대신 사용, 1주 유효·읽기 전용)
  1. github.com → 오른쪽 위 프로필 → Settings → 맨 아래 Developer settings
  2. Personal access tokens → **Fine-grained tokens** → Generate new token
  3. Expiration: 7 days / Repository access: **Only select repositories → limo-roboracer**
  4. Permissions → Repository permissions → **Contents: Read-only** (나머지 그대로) → Generate
  5. `github_pat_...` 토큰을 노트북 메모에 임시 보관(실습 끝나면 지워도 됨, 만료되면 자동 무효)

## 1. 하드웨어 확인 (5분)
- [ ] 바닥에 놓기, 상자·조종기·로봇 번호 일치 → **로봇 번호 기록**: ______
- [ ] LIMO1인지 Pro인지 기록: ______  (아래에서 `ROBOT=limo1` 또는 `ROBOT=limo_pro`)
- [ ] 충전기 분리, 배터리 커넥터 확인
- [ ] 바퀴 노브 = Ackermann(주황불 아님), 전원 버튼 **살살 길게**
- [ ] 조종기 SWB **가운데**로 전후진·조향 살짝 확인

## 2. 네트워크·SSH (15분)
**학교 Wi-Fi 먼저**, 안 되면 바로 핫스팟.
1. 로봇 화면 오른쪽 위 Wi-Fi → 학교 Wi-Fi 연결
   - 학번 등 **개인 계정 로그인**이 필요한 Wi-Fi면 → 공용 로봇에 계정 저장 금지 → 핫스팟으로
2. 로봇 터미널: `hostname -I` → Wi-Fi IP(172.17.0.1은 docker라 아님) 기록: ______
3. 노트북도 같은 Wi-Fi에 연결 후 노트북 터미널:
```bash
ping -c 3 로봇IP
ssh -o ServerAliveInterval=5 wego@로봇IP        # Pro (비번 wego). LIMO1은 agilex@로봇IP (비번은 조교 확인)
```
   - ping 실패 = 기기 간 통신 차단(클라이언트 격리) → 핫스팟으로
   - 로그인 메뉴가 나오면 `1` (Global)
4. 핫스팟일 때: 폰을 로봇·노트북 가까이, 핫스팟 화면 켜 두기. (지난번 ping 0.3~1.7 s → 화면 반응 느림 정상)

## 3. 드라이버 실행 (5분) — SSH 터미널 1 (계속 켜 둠)
**Pro**
```bash
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash; source /home/wego/wego_ws/install/setup.bash
ros2 launch wego teleop_launch.py
```
**LIMO1** (조교 공지 방식, 조교 컨테이너 안에서)
```bash
sudo docker run -it --name limo_humble --network host --privileged --device=/dev/ttyTHS1:/dev/ttyTHS1 --device=/dev/ttyUSB0:/dev/ttyUSB0 -v /dev/bus/usb:/dev/bus/usb -v /home/agilex/Documents:/home/agilex/Documents yspark98/limo1_humble:latest bash
# (이미 있으면) sudo docker start limo_humble && sudo docker exec -it limo_humble bash
export ROS_LOCALHOST_ONLY=1
cd /home/agilex/Documents/limo_basic_ws && source install/setup.bash && ros2 launch wego teleop_launch.py
```
- `Now lidar is scanning...` 나오면 정상. `Check Sum` 오류가 계속되면 Ctrl+C 후 다시 실행.

## 4. 우리 환경 (10분) — SSH 터미널 2
```bash
export ROS_LOCALHOST_ONLY=1
git -c credential.helper= clone --depth 1 --filter=blob:none --sparse https://github.com/tjdalsrb0428-cloud/limo-roboracer.git ~/Documents/limo-roboracer
#   Username: tjdalsrb0428-cloud   Password: 토큰 붙여넣기 (화면에 안 보임)
cd ~/Documents/limo-roboracer && git sparse-checkout set scripts config docker
bash scripts/real/setup.sh limo_pro                                   # LIMO1이면 limo1
#   인터넷 느리면: bash scripts/real/setup.sh limo_pro --tar /media/계정/USB이름/limo-robot_v1.tar
sudo docker exec -it limo_race bash
```
컨테이너 안(이후 측정은 전부 여기서. 들어가면 코드 폴더에서 시작하고 ROS는 자동 source됨):
```bash
cd scripts/real
ROBOT=limo_pro                                                         # LIMO1이면 limo1
```
- 컨테이너에서 토픽이 안 보이면: `ros2 topic list` → 비어 있으면 ROS_LOCALHOST_ONLY가 드라이버와 같은지(둘 다 1) 확인.
- docker 막힘 대비: 측정 스크립트는 로봇 호스트에서도 돈다(Pro 호스트: `source /opt/ros/humble/setup.bash` 후 같은 명령).

## 5. 측정 (40분) — 결과 숫자는 노트북 메모에 복사
1. **센서 점검** (로봇 정지)
```bash
python3 check_sensors.py --robot $ROBOT
```
   - 라이다 각도·점 수·주기가 프로파일과 다르면 기록. IMU 편향 기록.
   - EKF 확인(우리 TF와 겹치는지): `timeout 3 ros2 topic echo --once /odometry/filtered | grep -A3 position` → NaN인지,
     `timeout 5 ros2 run tf2_ros tf2_echo odom base_link` → TF가 나오는지(나오면 EKF가 TF 발행 중) 기록.
   - 차체가 찍히는지: `check_hose.py --sector 110`에서 0.1~0.2 m 묶음이 ±100° 바깥에 있으면 자기 몸체.
2. **덕트(벽) 감지** (로봇 정지, 덕트 정면 0.5 / 1 / 2 / 3 m에 차례로 놓고 각각 5초)
```bash
python3 check_hose.py --robot $ROBOT
```
   - 기록: 거리별 점 수/기대, 빠짐, 이음매 틈으로 바깥이 보이는지, 덕트보다 가까운 이상한 점(바닥 찍힘). Ctrl+C로 종료.
3. **줄자 측정**: 차 길이·폭(바퀴 포함), 바닥~라이다 중심 높이, 라이다가 차 앞에서 얼마나 뒤에 있는지, 덕트 지름.
4. 이제부터 로봇이 움직인다 → **조종기 SWB 위(코드 제어)**. 안 움직이면 `check_sensors` 상태 줄의 control_mode 확인.
5. **속도 보정** (정면 2.5 m 이상 앞에 벽/상자, 일반 바닥·잔디 각각)
```bash
python3 calib_speed.py --robot $ROBOT --speed 0.3
python3 calib_speed.py --robot $ROBOT --speed 0.6 --dist 1.5
```
   - 기록: speed_scale, odom_scale, 반응 지연.
6. **직진 트림** (앞 2 m 이상 빈 곳)
```bash
python3 calib_steer.py --robot $ROBOT --mode straight
```
7. **조향 배율** (지름 2 m 원 공간)
```bash
python3 calib_steer.py --robot $ROBOT --mode circle --steer 0.3
python3 calib_steer.py --robot $ROBOT --mode circle --steer 0.45 --time 6
```
   - 기록: steer_scale(좌/우), 0점 치우침, 최소 회전반경. 직진 트림은 steer_trim 넣고 다시 돌려 확인.
8. 남는 시간: 드라이버의 계산 속도 확인(LIMO1 Jetson Nano) — 할 일 7번 lite 모드 준비되면 추가.

결과는 노트북에서 `config/robots/calib_로봇번호.yaml`로 저장해 커밋(예):
```yaml
calib:
  steer_scale: 0.95
  steer_trim: -0.012
  speed_scale: 1.02
  odom_scale: 0.98
```

## 6. 결과 저장 (5분)
- 측정 출력은 SSH 화면에서 복사해 노트북 메모로. 파일이 생겼으면(지도 등, 수백 KB) 노트북 터미널에서:
```bash
scp -o ServerAliveInterval=5 -r wego@로봇IP:Documents/limo-roboracer/maps /home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble/maps/real_$(date +%m%d)
```

## 7. 정리 (10분) — 반드시
1. 컨테이너 밖(SSH 터미널 2, `exit`로 나와서):
```bash
bash ~/Documents/limo-roboracer/scripts/real/teardown.sh                  # 핫스팟 썼으면 --wifi 핫스팟이름
```
2. 드라이버 터미널 1: Ctrl+C. (LIMO1: 조교 컨테이너는 공지상 조교가 정리 — 우리가 만든 limo_humble은 `sudo docker rm -f limo_humble`)
3. 로봇 끄기: `sudo shutdown now` (또는 전원 버튼 살살 길게), 불 꺼짐 확인, 번호 맞춰 반납.

---

## 문제 대처
| 증상 | 원인·대처 |
|---|---|
| 키 입력·붙여넣기가 늦게 나옴, Ctrl+C 안 먹는 듯 | 네트워크 지연. 기다리기. 멈추면 엔터 → `~` `.` 로 SSH 끊고 재접속 |
| ssh 연결 끊김 / IP 바뀜 | 로봇에서 `hostname -I`로 새 IP |
| 컨테이너에서 토픽 안 보임 | ROS_LOCALHOST_ONLY 양쪽 1인지, 드라이버 켜져 있는지. `ros2 daemon stop` 후 다시 |
| 로봇이 명령에 안 움직임 | 조종기 SWB 위, Ackermann 모드, `check_sensors`의 control_mode·motion_mode. Pro는 `/limo/ack_cmd`, LIMO1은 `/cmd_vel` |
| 라이다 `Check Sum` 오류·`Device Failed` | 로봇 부하(큰 복사 등) 중단 → 드라이버 재시작 |
| docker pull 매우 느림 | Ctrl+C → `setup.sh ... --tar USB파일` |
| git clone 인증 실패 | Username은 `tjdalsrb0428-cloud`, Password는 토큰(계정 비번 아님), 토큰 권한 Contents Read-only·저장소 선택 확인 |
| 측정 중 위험 | Ctrl+C = 정지 명령 10회 후 종료. 조종기 SWB 가운데 = 수동 |
