# Schunk SVH Simulation
This package provides a minimalist simulation environment with Gazebo.
You can use it as an example to get to know the ROS-control based workflow for the SVH and
also as a starting point for your more complex simulation setup.


## Install dependencies
The Schunk SVH has several joints that are coupled to the motion of the primary 9 DOF.
In the URDF specification, these joints basically mimic their reference joints with a special multiplier.
We use [this very handy plugin](https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins) to achieve this in Gazebo.
It's not yet released to the latest ROS distros, so that you need to install that from source:
Inside the `src` folder of your ROS workspace, call

```bash
git clone https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins.git
```
and build your workspace normally.
You'll then have the `libroboticsgroup_upatras_gazebo_mimic_joint_plugin.so` available that's used in the SVH's `xacro` definition.


## Getting started
In a sourced terminal, run

```bash
roslaunch schunk_svh_simulation svh_gazebo_standalone.launch
```
This will bring up an empty Gazebo world with the Schunk SVH (left hand).
You can now use ROS-control and the `rqt_joint_trajectory_controller` to move individual joints:
Inside `rqt`, navigate to `Plugins -> Robot Tools -> Joint trajectory controller` and move the joints with sliders.
You might need to install that plugin first with

```bash
sudo apt install ros-$ROS_DISTRO-rqt-joint-trajectory-controller
```
since that's not part of the default ROS Desktop installation.
