# Three Drone Demo

## Mocap
launch Mocap system
(on windows mocap pc)
- launch motive
- select calibrated file
- make sure every three drone's rigid body is created.

(on ROS 1 Host PC)
Launch Mocap System
```
roslaunch drone_demo three_drone_mocap.launch
```

Launch Topic conversion node
```
rosrun drone_demo apm_bridge.py 
```

(on different terminal)
launch joy_node
```
rosrun joy joy_node
```

## Mavros
every drone has own IP.
drone1: 192.168.11.110
drone2: 192.168.11.120
drone3: 192.168.11.130

```
sshdrone1 (alias for ssh ubuntu@192.168.11.110)
(hit the password, ubuntu)
roslaunch mavros apm.launch __ns:=/drone1
```

(different terminal)
```
sshdrone2
(hit the password, ubuntu)
roslaunch mavros apm.launch __ns:=/drone2
```
(different terminal)
```
sshdrone3
(hit the password, ubuntu)
roslaunch mavros apm.launch __ns:=/drone3
```

## Takeoff with Loiter mode

Make sure every mavros got the vision pose and no error.

- Grab the RC Controller
- Switch to the Loiter mode
- Slowly bring-up the throttle
- Make sure that drone is holding the ideal position

## Control Node
(on ROS 1 Host PC)

```
rosrun drone_demo multi_drone_controller
```

Grab the joystick

That's all!