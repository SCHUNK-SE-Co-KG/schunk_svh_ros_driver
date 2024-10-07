# SCHUNK SVH Simulation
This package provides a simulated SVH for controller development and testing.

## Physics engine and rationales
We build the simulator on Todorov's [MuJoCo](https://mujoco.org/) physics engine, which
has been acquired and open-sourced by Google [here](https://github.com/deepmind/mujoco).
This gives us a strong environment to design and test finger control for object manipulation with minimal dependencies.


## Build and install

1. Download MuJoCo's pre-built [library package](https://github.com/deepmind/mujoco/releases/) and extract that somewhere.
It's ready-to-use and we will just point to it during the build.
   ```bash
   cd $HOME
   wget https://github.com/deepmind/mujoco/releases/download/3.2.3/mujoco-3.2.3-linux-x86_64.tar.gz
   tar -xf mujoco-3.2.3-linux-x86_64.tar.gz
   ```

2. We make use of the [GLFW3](https://www.glfw.org/) wrapper around OpenGL. You can install it on Ubuntu with
   ```bash
   sudo apt-get install libglfw3-dev libglew-dev
   ```

3. Switch to the *root* of your ROS2 workspace and build the package (*standalone*) with
   ```bash
   colcon build --cmake-args "-DMUJOCO_DIR=$HOME/mujoco-3.2.3" --packages-select schunk_svh_simulation
   ```


## Getting started
In the *root* of your ROS2 workspace
```bash
source install/setup.bash
ros2 launch schunk_svh_simulation simulation.launch.py
```

This will start a simulated world with the SCHUNK SVH.


## Interaction with the simulator
Supported commands:
- **Rotate**: Left mouse button
- **Zoom**: Middle mouse button
- **Drag**: Right mouse button
- **Restart**: Backspace key


## Hardware interfaces for controllers
Exposed interfaces per joint:

- `command_interfaces`: position
- `state_interfaces`: position, velocity
