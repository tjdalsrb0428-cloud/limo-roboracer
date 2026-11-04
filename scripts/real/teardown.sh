#!/bin/bash
# 실습 끝: 우리가 로봇에 만든 것을 전부 지운다(컨테이너, 이미지, 코드, 선택: Wi-Fi 프로파일, /tmp bag).
#   bash ~/Documents/limo-roboracer/scripts/real/teardown.sh [--wifi 핫스팟이름]
# 지도·결과를 노트북으로 먼저 옮겼는지 확인하고 실행할 것.
IMAGE_REPO=tjdalsrb0428/limo-robot
WIFI=""
[ "$1" = "--wifi" ] && WIFI=$2
CODE=$(cd "$(dirname "$0")/../.." && pwd)

echo "지울 것:"
echo "  - docker 컨테이너 limo_race, 이미지 $IMAGE_REPO:*"
echo "  - 코드 폴더 $CODE"
[ -d "$CODE/maps" ] && echo "    (주의: $CODE/maps 안의 지도도 지워진다)"
[ -n "$WIFI" ] && echo "  - Wi-Fi 프로파일 '$WIFI'"
ls -d /tmp/bag* >/dev/null 2>&1 && echo "  - /tmp/bag*"
read -r -p "진행? (y/N) " ans
[ "$ans" = y ] || { echo "취소"; exit 0; }

sudo docker rm -f limo_race 2>/dev/null
for img in $(sudo docker images --format '{{.Repository}}:{{.Tag}}' | grep "^$IMAGE_REPO:"); do sudo docker rmi "$img"; done
rm -rf /tmp/bag* 2>/dev/null
case "$CODE" in "$HOME"/Documents/*) rm -rf "$CODE";; *) echo "코드 폴더가 ~/Documents 밖이라 지우지 않음: $CODE";; esac
[ -n "$WIFI" ] && sudo nmcli con delete "$WIFI"
echo "정리 끝. 남은 docker:"; sudo docker ps -a; sudo docker images
