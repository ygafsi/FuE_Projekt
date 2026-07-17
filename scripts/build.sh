#!/bin/bash

set -e

WORKSPACE="/home/robot/ros2_ws"

echo "Building ROS2 workspace..."

cd "$WORKSPACE"

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source "$WORKSPACE/install/setup.bash"

echo "Build completed successfully."