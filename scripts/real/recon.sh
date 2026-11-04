#!/bin/bash
# 실차 LIMO 정찰: 로봇 환경·ROS 구성·소스를 읽기만 해서 tar.gz 로 노트북에 가져온다.
# 로봇에는 아무것도 남기지 않는다(임시 폴더는 끝나면 삭제, 결과는 stdout 으로 전송).
#
# 노트북(호스트)에서:
#   ssh 계정@로봇IP 'bash -s' < scripts/real/recon.sh > recon_$(date +%m%d_%H%M).tgz
#   mkdir -p recon && tar xzf recon_*.tgz -C recon
# 진행 상황은 터미널(stderr)에 출력된다. 약 2~4분.
#
# ROS 를 로봇 호스트와 실행 중인 docker 컨테이너 양쪽에서 찾아 각각 수집한다.
# 컨테이너에는 docker exec 로 읽기 명령만 실행한다(ros2 daemon 은 우리가 켰으면 끈다).

set -u
D=$(mktemp -d /tmp/limo_recon.XXXXXX)
trap 'rm -rf "$D"' EXIT
log() { echo "[recon] $*" >&2; }
run() {  # run 파일명 명령...  → $D/파일명
    local f=$1; shift
    { echo "\$ $*"; timeout 20 bash -c "$*" 2>&1; } > "$D/$f" 2>&1
}

# ---------- 1. 시스템 ----------
log "system"
mkdir -p "$D/sys"
run sys/os         'cat /etc/os-release; uname -a; cat /etc/nv_tegra_release 2>/dev/null; tr -d "\0" < /proc/device-tree/model 2>/dev/null; echo'
run sys/hw         'nproc; free -h; df -h; lsusb 2>/dev/null; ls -l /dev/ttyTHS* /dev/ttyUSB* /dev/ttyACM* /dev/video* 2>/dev/null'
run sys/net        'hostname; whoami; id; ip -br a; iwgetid 2>/dev/null; ip route'
run sys/internet   'ping -c1 -W3 8.8.8.8; curl -sS -m5 -o /dev/null -w "github %{http_code}\n" https://github.com; curl -sS -m5 -o /dev/null -w "dockerhub %{http_code}\n" https://registry-1.docker.io/v2/'
run sys/ps         'ps -eo pid,user,etime,args --sort=pid | grep -v "\[" '
run sys/services   'systemctl list-units --type=service --state=running --no-pager 2>/dev/null; ls /etc/xdg/autostart ~/.config/autostart 2>/dev/null; crontab -l 2>/dev/null'
run sys/home       'ls -la ~; ls ~/* 2>/dev/null | head -200'
run sys/bashrc     'cat ~/.bashrc'
run sys/env        'env | grep -E "ROS|RMW|CYCLONE|FASTRTPS|DDS" | sort'
run sys/python     'python3 --version; python3 -c "import numpy, scipy; print(numpy.__version__, scipy.__version__)" 2>&1; python3 -c "import skimage; print(skimage.__version__)" 2>&1'

# ---------- 2. docker ----------
log "docker"
mkdir -p "$D/docker"
CONTAINERS=""
if docker ps >/dev/null 2>&1; then
    run docker/version 'docker version; docker info 2>/dev/null | head -40'
    run docker/images  'docker images --digests'
    run docker/ps      'docker ps -a --no-trunc'
    for c in $(docker ps -a --format '{{.Names}}'); do   # 꺼진 컨테이너(조교 ROS2 컨테이너 등)도 설정 기록
        run "docker/inspect_$c" "docker inspect $c"
    done
    for c in $(docker ps --format '{{.Names}}'); do
        docker exec "$c" bash -c 'ls -d /opt/ros/*' >/dev/null 2>&1 && CONTAINERS="$CONTAINERS $c"
    done
else
    run docker/error 'docker ps'   # 권한 없음/미설치 기록 (sudo 는 쓰지 않는다)
fi

# ---------- 2-1. ROS1 (구형 LIMO: 호스트 18.04 melodic, roscore 가 떠 있을 때만) ----------
if ls /opt/ros/melodic/setup.bash >/dev/null 2>&1; then
    log "ROS1 host"
    mkdir -p "$D/ros1"
    P1='. /opt/ros/melodic/setup.bash; for s in $(find ~ -maxdepth 4 -path "*/devel/setup.bash" 2>/dev/null); do . $s; done;'
    run ros1/nodes  "$P1 rosnode list"
    run ros1/topics "$P1 rostopic list -v"
    run ros1/params "$P1 rosparam list"
    run ros1/ws     'find ~ -maxdepth 4 -path "*/devel/setup.bash" 2>/dev/null; ls ~/*_ws/src 2>/dev/null'
fi

# ---------- 3. ROS2 (호스트 + ROS 있는 컨테이너) ----------
TARGETS=""
ls -d /opt/ros/foxy /opt/ros/humble /opt/ros/jazzy 2>/dev/null | grep -q . && TARGETS="host"
TARGETS="$TARGETS $CONTAINERS"
log "ROS targets: ${TARGETS:-없음}"

SRC_FIND='find / -maxdepth 5 -path "*/install/setup.bash" -not -path "/proc/*" -not -path "/sys/*" 2>/dev/null'
for t in $TARGETS; do
    O="$D/ros_$t"; mkdir -p "$O"
    if [ "$t" = host ]; then X() { timeout 20 bash -c "$PRE $1" 2>&1; }
    else X() { timeout 20 docker exec "$t" bash -c "$PRE $1" 2>&1; }; fi
    PRE='for f in /opt/ros/*/setup.bash; do . "$f"; done;'
    SETUPS=$(X "$SRC_FIND")
    for s in $SETUPS; do PRE="$PRE . $s 2>/dev/null;"; done
    echo "$SETUPS" > "$O/setups"
    X 'env | grep -E "ROS|RMW|CYCLONE|FASTRTPS|DDS" | sort' > "$O/env"
    # 로봇 노드가 쓰는 통신 설정(도메인 ID 등)을 실행 중인 프로세스에서 읽어 맞춘다 → 안 맞으면 토픽이 안 보임
    PENV=$(X 'for p in $(pgrep -f "ros2|_node|launch|python3"); do tr "\0" "\n" < /proc/$p/environ 2>/dev/null | grep -E "^(ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|RMW_IMPLEMENTATION|CYCLONEDDS_URI|FASTRTPS_DEFAULT_PROFILES_FILE|ROS_DISCOVERY_SERVER)="; done | sort | uniq -c | sort -rn')
    echo "$PENV" > "$O/proc_env"
    for v in ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE ROS_DISCOVERY_SERVER; do
        line=$(echo "$PENV" | awk '{print $2}' | grep "^$v=" | head -1)
        [ -n "$line" ] && PRE="$PRE export '$line';"
    done

    had_daemon=0
    X 'ros2 daemon status' | grep -q "is running" && had_daemon=1
    X 'ros2 daemon start' >/dev/null; sleep 3

    log "[$t] nodes/topics"
    X 'ros2 node list'      > "$O/nodes"
    X 'ros2 topic list -t'  > "$O/topics"
    X 'ros2 pkg list'       > "$O/pkgs"
    mkdir -p "$O/node" "$O/param" "$O/topic" "$O/iface" "$O/sample" "$O/hz"
    while read -r n; do
        [ -z "$n" ] && continue; f=$(echo "$n" | tr '/' '_')
        X "ros2 node info $n"  > "$O/node/$f"
        X "ros2 param dump $n" > "$O/param/$f.yaml"
    done < "$O/nodes"
    while read -r tp ty; do
        [ -z "$tp" ] && continue; f=$(echo "$tp" | tr '/' '_')
        X "ros2 topic info -v $tp" > "$O/topic/$f"
    done < "$O/topics"
    tr -d '[]' < "$O/topics" | awk '{print $2}' | tr ',' '\n' | sort -u | while read -r ty; do
        [ -z "$ty" ] && continue
        X "ros2 interface show $ty" > "$O/iface/$(echo "$ty" | tr '/' '_')"
    done

    log "[$t] samples/hz"
    for tp in $(awk '{print $1}' "$O/topics" | grep -E 'scan|odom|imu|cmd_vel|limo_status|battery|joint_states'); do
        f=$(echo "$tp" | tr '/' '_')
        X "timeout 5 ros2 topic echo --once --no-arr $tp" > "$O/sample/$f"
        X "timeout 5 ros2 topic echo --once $tp | head -c 30000" > "$O/sample/${f}_full"
        X "timeout 6 ros2 topic hz $tp | tail -4" > "$O/hz/$f"
    done
    X 'timeout 5 ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable /tf_static' > "$O/sample/tf_static"
    X 'timeout 3 ros2 topic echo /tf | head -400' > "$O/sample/tf"
    X 'timeout 5 ros2 topic echo --once --full-length --qos-durability transient_local --qos-reliability reliable /robot_description' > "$O/sample/robot_description"
    X 'ros2 param get /robot_state_publisher robot_description' > "$O/sample/robot_description_param"

    [ $had_daemon = 0 ] && X 'ros2 daemon stop' >/dev/null

    # ---------- 4. 소스 (워크스페이스 src, 빌드 산출물·대용량 제외) ----------
    log "[$t] source"
    for s in $SETUPS; do
        ws=$(dirname "$(dirname "$s")")
        case "$ws" in /opt/ros*) continue;; esac
        [ "$t" = host ] && ok=$([ -d "$ws/src" ] && echo y) || ok=$(docker exec "$t" test -d "$ws/src" && echo y)
        [ "$ok" = y ] || continue
        name=$(echo "$ws" | tr '/' '_')
        TARCMD="tar czf - -C $ws --exclude=.git --exclude='*.bag' --exclude='*.db3' --exclude='*.pcd' --exclude='*.pbstream' src"
        if [ "$t" = host ]; then bash -c "$TARCMD" > "$O/src$name.tgz" 2>/dev/null
        else docker exec "$t" bash -c "$TARCMD" > "$O/src$name.tgz" 2>/dev/null; fi
        log "  $ws/src → $(du -h "$O/src$name.tgz" | cut -f1)"
    done
done

log "done → 노트북으로 전송"
tar czf - -C "$D" .
