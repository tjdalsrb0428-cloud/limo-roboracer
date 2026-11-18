#!/bin/bash
# 로봇에서 우리 실행 환경(docker 컨테이너 limo_race)을 띄운다. 로봇 호스트 설정은 건드리지 않는다.
#
# 0) 코드 받기 (처음 한 번, 로봇 터미널. 저장소 public → 인증 없음)
#   git clone --depth 1 --filter=blob:none --sparse https://github.com/tjdalsrb0428-cloud/limo-roboracer.git ~/Documents/limo-roboracer
#   cd ~/Documents/limo-roboracer && git sparse-checkout set scripts config docker docs
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
while [ $# -gt 0 ]; do
  case "$1" in
    --tar) TAR=$2; shift 2;;
    --image) IMAGE=$2; shift 2;;
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
sudo docker rm -f limo_race >/dev/null 2>&1 || true
# --network host: 로봇 드라이버 토픽을 그대로 봄. --ipc host: DDS 통신 안정. 장치 접근은 필요 없음(드라이버는 호스트/조교 컨테이너).
sudo docker run -d --name limo_race --network host --ipc host \
  -e ROS_LOCALHOST_ONLY="$LO" -e ROS_DOMAIN_ID="$DOM" -e LIMO_ROBOT="$ROBOT" \
  -v "$DOCS":"$DOCS" -w "$CODE" "$IMAGE" >/dev/null
echo "[setup] 컨테이너 limo_race 실행됨"
echo
echo "다음:"
echo "  (드라이버가 안 켜져 있으면 먼저) $(python3 -c "import yaml;print(yaml.safe_load(open('$CODE/config/robots/$ROBOT.yaml'))['host']['driver_launch'])" 2>/dev/null)"
[ "$ROBOT" = limo1 ] && echo "    (LIMO1은 조교 컨테이너 yspark98/limo1_humble 안에서 실행 — 공지 참고)"
echo "  sudo docker exec -it limo_race bash"
echo "  cd $CODE/scripts/real && python3 check_sensors.py --robot $ROBOT"
