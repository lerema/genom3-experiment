#!/bin/bash
THIS_DIR=$(
    cd $(dirname $0)
    pwd
)
WORK_DIR=/home/ubuntu/drone-experiment/genom3-experiment

docker run -it --rm -e USER=ubuntu -e PASSWORD=ubuntu -e RESOLUTION=1920x1080 -v /dev/shm:/dev/shm \
    -v ${THIS_DIR}:$WORK_DIR -w $WORK_DIR -p 6080:80 \
    genom3-experiment:latest
