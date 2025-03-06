#!/bin/bash

# ROS環境のセットアップ
source /opt/ros/noetic/setup.bash
source ~/workspace/devel/setup.bash

echo "Starting simulation using tmux..."

tmux new-session -d -s drone_sim -n main

# Gazebo
tmux send-keys -t drone_sim "gazebo --verbose ~/git-repository/ardupilot_gazebo/worlds/iris_arducopter_runway_three_drone.world" C-m
echo "Gazebo started"

sleep 2s

# SITL1
tmux new-window -t drone_sim -n SITL1
tmux send-keys -t drone_sim:SITL1 "cd ~/git-repository/ardupilot/ArduCopter/ && ../Tools/autotest/sim_vehicle.py -f gazebo-iris --console" C-m
echo "SITL1 started"

sleep 2s

# SITL2
tmux new-window -t drone_sim -n SITL2
tmux send-keys -t drone_sim:SITL2 "cd ~/git-repository/ardupilot/ArduCopter && ../Tools/autotest/sim_vehicle.py -f gazebo-iris --console --instance 1" C-m
echo "SITL2 started"

sleep 2s
# SITL3
tmux new-window -t drone_sim -n SITL3
tmux send-keys -t drone_sim:SITL3 "cd ~/git-repository/ardupilot/ArduCopter && ../Tools/autotest/sim_vehicle.py -f gazebo-iris --console --instance 2" C-m
echo "SITL3 started"
sleep 2s

sleep 10s

# MAVROS1
tmux new-window -t drone_sim -n MAVROS1
tmux send-keys -t drone_sim:MAVROS1 "roslaunch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555 __ns:=/drone1" C-m
echo "MAVROS1 started"

# MAVROS2
tmux new-window -t drone_sim -n MAVROS2
tmux send-keys -t drone_sim:MAVROS2 "roslaunch mavros apm.launch fcu_url:=udp://127.0.0.1:14560@14565 __ns:=/drone2" C-m
echo "MAVROS2 started"

# MAVROS3
tmux new-window -t drone_sim -n MAVROS3
tmux send-keys -t drone_sim:MAVROS3 "roslaunch mavros apm.launch fcu_url:=udp://127.0.0.1:14570@14575 __ns:=/drone3" C-m
echo "MAVROS3 started"

tmux attach -t drone_sim
