#!/bin/bash

xhost +
docker run -it --rm \
    --network=host \
    --ipc=host \
    --privileged \
    -v "/dev:/dev" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    --name dcist-jackal-`hostname`-service \
    dcist-jackal-`hostname`:service \
    bash
xhost -
