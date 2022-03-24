################################################################################
# Copyright 2022 FZI Research Center for Information Technology
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# -----------------------------------------------------------------------------
# \file    schunk_svh_driver.launch.py
#
# \author  Stefan Scherzinger <scherzin@fzi.de>
# \date    2022/03/02
#
# -----------------------------------------------------------------------------

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "device_file", default_value="/dev/ttyUSB0", description="Device file of the Schunk SVH serial interface."
        )
    )
    device_file = LaunchConfiguration("device_file")

    declared_arguments.append(
        DeclareLaunchArgument(
            "handedness", default_value="right", description="Is the Schunk SVH a right or a left hand?"
        )
    )
    handedness = LaunchConfiguration("handedness")

    # Build the URDF with command line xacro.
    # We also pass parameters for the system_interface here.
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("schunk_svh_driver"), "urdf", "schunk_svh_driver.xacro"]
            ),
            " ",
            "device_file:=",
            device_file,
            " ",
            "handedness:=",
            handedness,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("schunk_svh_driver"), "cfg", "schunk_svh_driver.yaml",
        ]
    )

    # The actual driver is a ROS2-control system interface.
    # Start that with the usual ROS2 controller manager mechanisms.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        #prefix="screen -d -m gdb -command=/home/scherzin/.ros/my_debug_log --ex run --args",
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    # Spawn additional nodes for control and visualization
    joint_state_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_controller", "-c", "/controller_manager"],
    )
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    # List all nodes that we want to start
    nodes = [
        control_node,
        joint_state_controller_spawner,
        joint_trajectory_controller_spawner
    ]

    return LaunchDescription(declared_arguments + nodes)
