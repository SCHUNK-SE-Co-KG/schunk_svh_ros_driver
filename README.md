# Schunk SVH ROS Driver

This is the repository for the Schunk SVH ROS1 driver.
It completes the [standalone
library](https://github.com/fzi-forschungszentrum-informatik/schunk_svh_library)
with everything that you need to operate the five finger hand in a ROS1 context.


## Installation
You can either setup a fresh ROS workspace, for instance with
```bash
mkdir -p $HOME/schunk_ws/src && cd "$_"
catkin_init_workspace
```
or use an existing one.
Inside the `src` folder of that ROS workspace, get the relevant ROS packages

```bash
git clone -b main git@github.com:fzi-forschungszentrum-informatik/schunk_svh_ros_driver.git
git clone -b main git@github.com:fzi-forschungszentrum-informatik/schunk_svh_library.git
rosdep install --ignore-src --from-paths ./ -y -r
```

We use `catkin build` for building the workspace. That's more convenient when working with the `schunk_svh_library`, which is a non-catkin package and would else require `catkin_make_isolated`.
`catkin` is part of the `catkin_tools` package. You can install it e.g. with
```bash
sudo apt-get install python3-catkin-tools
```
[Here's](https://catkin-tools.readthedocs.io/en/latest/installing.html) more information about this install.

You can then build everything in the root of the ROS workspace with

```bash
cd ..
catkin build
```
If you used an existing workspace for the new Schunk components, you might need
to remove the `build` folder first so that you don't mix build spaces with
previous calls to `catkin_make`.

## Connection to the SVH
There are a few minimal steps required before you can connect to the SVH for the first time.
You'll find them [here](https://github.com/fzi-forschungszentrum-informatik/schunk_svh_library).

## Getting started
In a sourced terminal, run
```bash
roslaunch schunk_svh_driver svh_ros_control_standalone.launch
```
This will start initializing the SVH.
After calibrating each degree of freedom separately, the SVH is ready to use.

The driver starts with a *running* [joint_trajectory_controller](http://wiki.ros.org/joint_trajectory_controller) for commanding all joints in a synchronized way.
You can send trajectory goals to its action server for controlling groups of joints.

An easy access is provided with `rqt`'s joint trajectory controller plugin.
You might need to install that first with e.g.
```bash
sudo apt install ros-noetic-rqt-joint-trajectory-controller
```
In a sourced terminal, call `rqt` and navigate to `Plugins` -> `Robot Tools` -> `Joint trajectory controller`.
You can then move individual fingers with sliders.
