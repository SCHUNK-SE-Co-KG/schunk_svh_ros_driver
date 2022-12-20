################################################################################
#
# © Copyright 2022 SCHUNK Mobile Greifsysteme GmbH, Lauffen/Neckar Germany
# © Copyright 2022 FZI Forschungszentrum Informatik, Karlsruhe, Germany
#
# This file is part of the Schunk SVH Driver.
#
# The Schunk SVH Driver is free software: you can redistribute it and/or
# modify it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or (at your
# option) any later version.
#
# The Schunk SVH Driver is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
# Public License for more details.
#
# You should have received a copy of the GNU General Public License along with
# the Schunk SVH Driver. If not, see <https://www.gnu.org/licenses/>.
#
###############################################################################

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
            "control", default_value="right_hand", description="Is the Schunk SVH a right_hand or a left_hand?"
        )
    )
    control = LaunchConfiguration("control")

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
            "control:=",
            control,
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
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    # Spawn additional nodes for control and visualization
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )
    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[control, "-c", "/controller_manager"],
    )

    # List all nodes that we want to start
    nodes = [
        control_node,
        joint_state_broadcaster_spawner,
        hand_controller_spawner
    ]

    return LaunchDescription(declared_arguments + nodes)
