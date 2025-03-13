# Three Drone Demo under gazebo

launch Gazebo
```
launch_three_drone_sim.sh
```
これを実行すればいい。

```
gazebo --verbose ~/git-repository/ardupilot_gazebo/worlds/iris_arducopter_runway_three_drones.world 
cd ~/git-repository/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -f gazebo-iris --console
../Tools/autotest/sim_vehicle.py -f gazebo-iris --console --instance 1
../Tools/autotest/sim_vehicle.py -f gazebo-iris --console --instance 2

roslaunch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555 __ns:=/drone1
roslaunch mavros apm.launch fcu_url:=udp://127.0.0.1:14560@14565 __ns:=/drone2
roslaunch mavros apm.launch fcu_url:=udp://127.0.0.1:14570@14575 __ns:=/drone3
```

(on different terminal)
launch joy_node
```
rosparam set joy_node/dev "/dev/input/js1"
rosrun joy joy_node

or

rosrun joy joy_node _dev:=/dev/input/js1s
```

## Takeoff with Loiter mode
Under mavproxy

GUIDED -> arm throttle -> takeoff 5 

Each Drones.

## Control Node
(on ROS 1 Host PC)

```
rosrun drone_demo multi_drone_controller
```

Grab the joystick

That's all!