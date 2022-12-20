![build badge](https://github.com/fzi-forschungszentrum-informatik/schunk_svh_ros_driver/actions/workflows/industrial_ci_humble_action.yml/badge.svg)
[![License](https://img.shields.io/badge/License-GPLv3-orange)](https://opensource.org/licenses/gpl-license)

# Schunk SVH ROS2 Driver

This is the repository for the Schunk SVH ROS2 driver.
It completes the [standalone
library](https://github.com/fzi-forschungszentrum-informatik/schunk_svh_library)
with everything that you need to operate the five finger hand in a ROS2 context.


## Installation
Inside the `src` folder of your ROS2 workspace, get the relevant packages

```bash
git clone -b ros2-humble https://github.com/fzi-forschungszentrum-informatik/schunk_svh_ros_driver.git
git clone -b main https://github.com/fzi-forschungszentrum-informatik/schunk_svh_library.git
rosdep install --ignore-src --from-paths ./ -y -r
```

You can then navigate back (`cd ..`) and build everything in the root of your ROS2 workspace with

```bash
colcon build
```

## Connection to the SVH
There are a few minimal steps required before you can connect to the SVH for the first time.
You'll find them [here](https://github.com/fzi-forschungszentrum-informatik/schunk_svh_library#connection-to-the-svh).

## Getting started

In every fresh terminal, source the setup scripts in the following order
```
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```
where `install` is the local folder within your ROS2 workspace.
Now, run
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

## Setup on a Raspberry Pi
Here's a [step-by-step tutorial](doc/raspberry_pi.md) of how to setup the Schunk SVH ROS2 driver on a Raspberry Pi 4.
