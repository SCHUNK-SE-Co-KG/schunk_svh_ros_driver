# Tips and tricks

## How to convert the SVH urdf to a MuJoCo model

1. run xacro to create the urdf, e.g.
   ```bash
   xacro /home/stefan/src/ros2_workspaces/svh/src/schunk_svh_ros_driver/schunk_svh_driver/urdf/schunk_svh_driver.xacro control:=right_hand plugin:=schunk_svh_simulation/Simulator device_file:=unused mujoco_model:=/home/stefan/src/ros2_workspaces/svh/src/schunk_svh_ros_driver/schunk_svh_simulation/etc/svh_mujoco.xml mesh_directory:=/home/stefan/src/ros2_workspaces/svh/src/schunk_svh_ros_driver/schunk_svh_simulation/meshes >> svh.urdf
   ```
2. add a
   ```bash
   <mujoco> <compiler discardvisual="false"/> </mujoco>
   ```
   to the `<robot>` tag.
3. Inside the generated `.urdf`, replace the `.dae` with `.stl` files
4. Convert the actual `.dae` files into binary `.stl` with Blender
5. Put all `.stl` files into MuJoCo's `compile` executable binary
6. Compile with `compile svh.urdf svh.xml`
7. Copy the content inside the `<worldbody>` tag
8. Remove lines with duplicate meshes that don't have `contype` or `coaffinity`. They will lead to constant collisions and destabilize the engine.
