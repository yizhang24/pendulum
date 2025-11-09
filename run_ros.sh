#!/usr/bin/env bash
xhost +local:docker
sudo docker run -d --rm \
  --name ros2 \
  --gpus all \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="XDG_RUNTIME_DIR=/tmp/runtime-root" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --device /dev/dri \
  -p 2222:22 \
  -p 8765:8765 \
  --volume=".:/root/project" ros2-ssh:latest
