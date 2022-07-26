# Schunk SVH ROS2 Driver

This is the repository for the Schunk SVH ROS2 driver.
It completes the [standalone
library](https://github.com/fzi-forschungszentrum-informatik/schunk_svh_library)
with everything that you need to operate the five finger hand in a ROS2 context.


## Installation
Inside the `src` folder of your ROS2 workspace, get the relevant packages

```bash
git clone -b ros2-foxy-devel git@github.com:fzi-forschungszentrum-informatik/schunk_svh_ros_driver.git
git clone -b main git@github.com:fzi-forschungszentrum-informatik/schunk_svh_library.git
rosdep install --ignore-src --from-paths ./ -y -r
```

You can then navigate back (`cd ..`) and build everything in the root of your ROS2 workspace with

```bash
colcon build
```

## Connection to the SVH
There are a few minimal steps required before you can connect to the SVH for the first time.
You'll find them [here](https://github.com/fzi-forschungszentrum-informatik/schunk_svh_library).

## Getting started

In a sourced terminal, run
```bash
ros2 launch schunk_svh_driver schunk_svh_driver.launch.py control:=left_hand
```
if you have a *left* Schunk SVH hand, else omit the `control` parameter. The default is `control:=right_hand`.
This will start initializing the SVH.
After calibrating each degree of freedom separately, the SVH is ready to use.

The driver starts with an *active* [joint_trajectory_controller](http://wiki.ros.org/joint_trajectory_controller) for commanding all joints in a synchronized way.
You can send trajectory goals to its action server for controlling groups of joints.

An easy example is provided with the `schunk_svh_driver/scripts/example.py` script.
Call that in a sourced terminal and you can move individual fingers with sliders in a minimalistic GUI.
