ccr
===
Ros node for Co-Creation Robot.

Prerequisites
---
* [ros serial](http://wiki.ros.org/serial)
* [ros joy](http://wiki.ros.org/joy)

Compiling
---
Just clone the repository in the src folder of a catkin workspace. Then run catkin_make.

To generate the model you have to launch in the model folder the dedicated script:
```
source generate_model.sh
```

To simulate the robot in Gazebo you first have to add this lines to your bashrc:

```
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(rospack find rulo)
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:$(rospack find rulo)
```

Usage
---
---
```
roslaunch ccr ccr.launch
```
to run the basic software and have access to the following topics:

- mobile_base/event/battery
- mobile_base/event/bumper
- mobile_base/event/cliff
- mobile_base/event/ultrasonic_sensor
- mobile_base/event/optical_ranging_sensor
- mobile_base/event/ir_character
- mobile_base/event/mode
- mobile_base/event/wheel_drop
- /cmd_vel
- /odom
- /rosout
- /rosout_agg
- /tf

you can read sensors (/battery, /buttons, /bumper, ...) and send commands (/cmd_vel, ...).

---
```
roslaunch ccr ccr_joy.launch
```
to run both the basic software and ros joy to move the robot with a controller.

<!--[Instructions](cad/laser_support/instructions.md)-->
