#!/bin/bash

SlideSlamWs="/home/risha/slideslam_docker_ws"
SlideSlamCodeDir="/home/slideslam_docker_ws/src/semantic-CSLAM"
BAGS_DIR="/home/risha/slideslam_docker_ws/bags"

# Ensure the bags directory actually exists so Docker doesn't create a fake one
mkdir -p "$BAGS_DIR"
xhost +local:root
docker run -it \
   --rm \
   --name="slideslam_ros" \
   --net="host" \
   --privileged \
   --gpus="all" \
   --workdir="/opt/slideslam_docker_ws" \
   --env="DISPLAY=$DISPLAY" \
   --env="QT_X11_NO_MITSHM=1" \
   --volume="$SlideSlamWs:/opt/slideslam_docker_ws" \
   --volume="$SlideSlamCodeDir:$SlideSlamCodeDir" \
   --volume="$BAGS_DIR:/opt/bags" \
   --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
   xurobotics/slide-slam:latest \
   bash
