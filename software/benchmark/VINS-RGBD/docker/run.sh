#!/bin/bash

container_name="vinsrgbd"  
image_name="joriswenyuli/vinsrgbd"

if [ "$(docker ps -a -q -f name=$container_name)" ]; then
  docker stop $container_name
  docker rm $container_name
fi

docker run -it --name $container_name --gpus all \
	--privileged \
	--net=host \
        -v /dev:/dev \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v /home/wenyu/Downloads:/root/downloads \
        -v /media/wenyu:/root/dataset \
        -v $(dirname $(dirname $(readlink -f "$0"))):/root/catkin_ws/src/$(basename $(dirname $(dirname "$(readlink -f "$0")"))) \
        --device-cgroup-rule "c 81:* rmw" \
        --device-cgroup-rule "c 189:* rmw" \
        -e DISPLAY=$DISPLAY $image_name bash
