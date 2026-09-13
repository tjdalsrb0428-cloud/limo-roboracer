#!/bin/bash

xhost +SI:localuser:root

docker run -it \
  --name limo_gazebo_humble \
  --network host \
  --ipc host \
  --device /dev/dri:/dev/dri \
  --workdir /home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /etc/localtime:/etc/localtime:ro \
  -v /home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble:/home/mksung0428/Downloads/LIMO_GAZEBO/ws_limo_humble:rw \
  yspark98/limo:gazebo_humble
