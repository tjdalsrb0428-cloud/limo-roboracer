#!/bin/bash
# 로봇에서 우리 실행 환경(docker 컨테이너 limo_race)을 띄운다. 로봇 호스트 설정은 건드리지 않는다.
#
# 0) 코드 받기 (처음 한 번, 로봇 터미널. 저장소 public → 인증 없음)
#   git clone --depth 1 --filter=blob:none --sparse https://github.com/tjdalsrb0428-cloud/limo-roboracer.git ~/Documents/limo-roboracer
#   cd ~/Documents/limo-roboracer && git sparse-checkout set scripts config docker docs src/limo_race src/limo_race_msgs
# 1) 실행
#   bash ~/Documents/limo-roboracer/scripts/real/setup.sh limo_pro            # Docker Hub에서 pull
#   bash ~/Documents/limo-roboracer/scripts/real/setup.sh limo1 --tar /media/USB/limo-robot.tar   # USB에서 load
# 2) 컨테이너 들어가기
#   sudo docker exec -it limo_race bash
set -e
ROBOT=${1:?사용법: setup.sh limo1|limo_pro [--tar 파일] [--image 이름]}
shift
IMAGE=tjdalsrb0428/limo-robot:v1
TAR=""
UID_RUN=""
while [ $# -gt 0 ]; do
  case "$1" in
    --tar) TAR=$2; shift 2;;
    --image) IMAGE=$2; shift 2;;
    --uid) UID_RUN=$2; shift 2;;
    *) echo "모르는 옵션: $1"; exit 1;;
  esac
done
CODE=$(cd "$(dirname "$0")/../.." && pwd)
DOCS=$HOME/Documents
[ -f "$CODE/config/robots/$ROBOT.yaml" ] || { echo "프로파일 없음: $CODE/config/robots/$ROBOT.yaml"; exit 1; }
case "$CODE" in "$DOCS"/*) ;; *) echo "경고: 코드가 ~/Documents 밖에 있다($CODE). 공지 규칙상 ~/Documents 안에 둘 것.";; esac
LO=${ROS_LOCALHOST_ONLY:-1}
DOM=${ROS_DOMAIN_ID:-0}

echo "[setup] robot=$ROBOT image=$IMAGE code=$CODE ROS_LOCALHOST_ONLY=$LO ROS_DOMAIN_ID=$DOM"
if [ -n "$TAR" ]; then
  echo "[setup] USB 이미지 load: $TAR"; sudo docker load -i "$TAR"
elif ! sudo docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "[setup] pull $IMAGE (약 0.5 GB)"; sudo docker pull "$IMAGE"
fi
# LOCALHOST_ONLY=1이면 DDS를 127.0.0.1로 제한(안 하면 토픽 목록만 보이고 데이터가 안 옴, 2026-11-18 Pro)
DDS=""
[ "$LO" = 1 ] && DDS="-e FASTRTPS_DEFAULT_PROFILES_FILE=$CODE/docker/robot/fastdds_udp_local.xml"
# 실행 계정: 드라이버와 같은 uid여야 DDS 데이터가 온다(Pro: 호스트 wego 드라이버 ↔ root 컨테이너면 토픽 목록만 보이고 0 Hz, 2026-11-18).
# LIMO1은 드라이버가 조교 컨테이너(root) → root(0). 바꾸려면 --uid N.
[ -z "$UID_RUN" ] && { [ "$ROBOT" = limo1 ] && UID_RUN=0 || UID_RUN=$(id -u); }
USER_OPT="-v $CODE/docker/robot/bashrc:/etc/bash.bashrc:ro"
[ "$UID_RUN" != 0 ] && USER_OPT="$USER_OPT --user $UID_RUN:$UID_RUN -e HOME=/tmp"
echo "[setup] 컨테이너 실행 uid=$UID_RUN"
sudo docker rm -f limo_race >/dev/null 2>&1 || true
# --network host: 로봇 드라이버 토픽을 그대로 봄. --ipc host: DDS 통신 안정. 장치 접근은 필요 없음(드라이버는 호스트/조교 컨테이너).
sudo docker run -d --name limo_race --network host --ipc host \
  -e ROS_LOCALHOST_ONLY="$LO" -e ROS_DOMAIN_ID="$DOM" -e LIMO_ROBOT="$ROBOT" $DDS $USER_OPT \
  -v "$DOCS":"$DOCS" -w "$CODE" "$IMAGE" >/dev/null
echo "[setup] 컨테이너 limo_race 실행됨"
# ROS2 패키지(limo_race: ros2 run/launch 버전) 빌드. 예전 clone 이라 폴더가 없으면 받아 온다
if [ ! -d "$CODE/src/limo_race" ] && [ -d "$CODE/.git" ]; then
  echo "[setup] src/limo_race 받기 (git sparse-checkout add)"
  git -C "$CODE" sparse-checkout add src/limo_race src/limo_race_msgs || echo "[setup] 경고: 못 받음 → python3 버전만 사용 가능"
fi
if [ -d "$CODE/src/limo_race" ]; then
  echo "[setup] limo_race 빌드 (colcon, 약 1분)"
  sudo docker exec limo_race bash -c "source /opt/ros/humble/setup.bash && source /opt/limo_ws/install/setup.bash && cd $CODE && \
    colcon build --symlink-install --base-paths src/limo_race src/limo_race_msgs --cmake-args -DBUILD_TESTING=OFF 2>&1 | tail -3" \
    || echo "[setup] 경고: limo_race 빌드 실패 → python3 버전 사용"
fi
echo
echo "다음:"
echo "  (드라이버가 안 켜져 있으면 먼저) $(python3 -c "import yaml;print(yaml.safe_load(open('$CODE/config/robots/$ROBOT.yaml'))['host']['driver_launch'])" 2>/dev/null)"
[ "$ROBOT" = limo1 ] && echo "    (LIMO1은 조교 컨테이너 yspark98/limo1_humble 안에서 실행 — 공지 참고)"
echo "  sudo docker exec -it limo_race bash"
echo "  cd $CODE/scripts/real && python3 check_sensors.py --robot $ROBOT"
echo "  (ros2 버전) ros2 run limo_race check_sensors --ros-args -p robot:=$ROBOT"
