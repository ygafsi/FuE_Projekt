#!/bin/bash

set -e

WORKSPACE="/home/robot/ros2_ws"
SESSION="screwdriver_monitor"

source /opt/ros/jazzy/setup.bash
source "$WORKSPACE/install/setup.bash"

# Remove a previous monitor session if it still exists.
tmux kill-session -t "$SESSION" 2>/dev/null || true

# Left pane: controller state.
tmux new-session -d -s "$SESSION" \
    "watch -t -n 0.5 \
    'ros2 topic echo --once /screwdriver/state --field data'"

# Right pane: final tightening result.
tmux split-window -h -t "$SESSION" \
    "watch -t -n 0.5 \
    'ros2 topic echo --once /screwdriver/result --field data'"

tmux select-layout -t "$SESSION" even-horizontal

tmux attach-session -t "$SESSION"