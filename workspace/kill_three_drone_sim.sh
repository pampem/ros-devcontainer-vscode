#!/bin/bash

echo "Stopping all tmux sessions and related processes..."

SESSION_NAME="drone_sim"
s
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
    tmux kill-session -t $SESSION_NAME
    echo "Tmux session '$SESSION_NAME' killed."
fi

PROCESSES=("gazebo" "sim_vehicle.py" "mavros" "ardupilot" "roslaunch" "rosout" "console")

for PROC in "${PROCESSES[@]}"; do
    if pgrep -f $PROC > /dev/null; then
        kill $(pgrep -f $PROC)
        echo "Killed process: $PROC"
    else
        echo "No process found: $PROC"
    fi
done

echo "All processes stopped successfully!"
