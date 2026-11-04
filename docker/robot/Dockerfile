# 로봇(LIMO1 / LIMO Pro)에서 돌릴 우리 실행 환경. ARM64(Jetson) 대상.
# 코드는 넣지 않는다 → 로봇 ~/Documents 에 git clone 한 코드를 마운트해서 쓴다(이미지는 public 이어도 됨).
#
# 빌드(노트북, ARM64 에뮬레이션 필요):
#   docker buildx build --platform linux/arm64 -t tjdalsrb0428/limo-robot:v1 --load docker/robot
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-slam-toolbox \
        ros-humble-ackermann-msgs \
        ros-humble-tf2-ros \
        ros-humble-tf2-geometry-msgs \
        python3-pip python3-yaml \
    && rm -rf /var/lib/apt/lists/*

# 시뮬 컨테이너와 같은 버전 (raceline.py·race_driver.py 검증 버전)
RUN pip3 install --no-cache-dir numpy==1.26.4 scipy==1.15.3 scikit-image==0.25.2

# limo_msgs (/limo_status) — LIMO1·Pro 동일
COPY limo_msgs /opt/limo_ws/src/limo_msgs
RUN . /opt/ros/humble/setup.sh && cd /opt/limo_ws \
    && colcon build --cmake-args -DBUILD_TESTING=OFF && rm -rf build log

# Fast DDS 공유메모리 끄기: 호스트(wego/agilex)와 컨테이너(root) 사이 /dev/shm 권한 문제로
# 탐색은 되는데 데이터가 안 오는 경우를 막는다(UDP만 사용).
COPY fastdds_udp.xml /opt/fastdds_udp.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/fastdds_udp.xml

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
 && echo 'source /opt/limo_ws/install/setup.bash' >> /root/.bashrc
WORKDIR /root
CMD ["sleep", "infinity"]
