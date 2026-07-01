#!/bin/bash

cd /home/robot/ros2_ws
source install/setup.bash
ros2 run screwdriver_driver kl_tcg_driver
