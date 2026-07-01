#!/bin/bash

source /home/robot/ros2_ws/install/setup.bash

SESSION="screwdriver_monitor"
tmux kill-session -t $SESSION 2>/dev/null

tmux new-session -d -s $SESSION "watch -n 0.5 'ros2 topic echo /screwdriver/state --field data --once'"
tmux split-window -h "watch -n 0.5 'ros2 topic echo /screwdriver/result --field data --once'"

tmux select-layout even-horizontal
tmux attach-session -t $SESSION