#!/bin/bash

set -e

WORKSPACE="/home/robot/ros2_ws"

cd "$WORKSPACE"

source /opt/ros/jazzy/setup.bash
source "$WORKSPACE/install/setup.bash"

echo "Starting KL-TCG driver..."
echo "Serial port: /dev/ttyUSB0"
echo "Press Ctrl+C to stop."
echo

ros2 run screwdriver_driver kl_tcg_driver