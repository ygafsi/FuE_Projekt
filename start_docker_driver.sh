#!/bin/sh
uid=$(eval "id -u")
gid=$(eval "id -g")

# Mac user:
# uid=1000
# gid=1000

docker build --network=host --build-arg UID="$uid" --build-arg GID="$gid" -t ros2_bachelor/ros2:jazzy .
echo "Run Container"
xhost + local:root

docker run --name ros2_bachelor --privileged -it -e DISPLAY=$DISPLAY \
--privileged \
-v /dev/bus/usb:/dev/bus/usb \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-p 50001-50003:50001-50003 \
-v ~/.Xauthority:/home/robot/.Xauthority \
-v $(pwd)/ros2_ws:/home/robot/ros2_ws \
-v $(pwd)/scripts:/home/robot/scripts \
--net host --rm --ipc host ros2_bachelor/ros2:jazzy
