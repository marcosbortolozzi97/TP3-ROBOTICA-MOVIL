#!/bin/bash
# =====================================================
# Script de ejecución del contenedor Docker para RViz
# =====================================================

xhost +local:docker >/dev/null 2>&1

docker run -it --rm \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/datasets:/datasets \
    -v ~/ros2_ws:/ros2_ws \
    stereo_pointcloud_ros2

xhost -local:docker >/dev/null 2>&1
