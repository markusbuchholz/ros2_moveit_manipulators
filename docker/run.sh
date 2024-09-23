#!/usr/bin/env bash

XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]; then
    xauth_list=$(xauth nlist $DISPLAY)
    xauth_list=$(sed -e 's/^..../ffff/' <<< "$xauth_list")
    if [ ! -z "$xauth_list" ]; then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

docker run -it \
    --rm \
    --name moveit2_container \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -v "$XAUTH:$XAUTH" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "/home/markus/underwater/ros2_moveit_manipulators/src/:/root/colcon_ws/src" \
    --privileged \
    --security-opt seccomp=unconfined \
    --network host \
    --device /dev/ttyUSB0 \
    ros2_moveit2:latest

