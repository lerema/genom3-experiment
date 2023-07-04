#!/bin/bash
THIS_DIR=$(
    cd $(dirname $0)
    pwd
)
WORK_DIR=/home/ubuntu/drone-experiment/genom3-experiment

# Check if the image is already built
if [ -z "$(docker images -q genom3-experiment:latest)" ]; then
    echo "Building the image..."
    docker build -t genom3-experiment:latest .
fi

docker run -it --rm -e USER=ubuntu -e PASSWORD=ubuntu -e RESOLUTION=1920x1080 -v /dev/shm:/dev/shm \
    -v ${THIS_DIR}:$WORK_DIR -w $WORK_DIR -p 6080:80 \
    -v /dev/:/dev/ --privileged --name genom3-experiment \
    genom3-experiment:latest
