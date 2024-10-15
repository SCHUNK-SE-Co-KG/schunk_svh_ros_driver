# Copyright 2024 SCHUNK SE & Co. KG
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program. If not, see <https://www.gnu.org/licenses/>.
# --------------------------------------------------------------------------------

# -----------------------------------------------------------------------------
# \file    simulation.launch.py
#
# \author  Stefan Scherzinger <stefan.scherzinger@de.schunk.com>
# \date    2024/10/01
#
# -----------------------------------------------------------------------------


from launch import LaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

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
            "control:=right_hand",
            " ",
            "plugin:=schunk_svh_simulation/Simulator",
            " ",
            "device_file:=unused",
            " ",
            "mujoco_model:=",
            PathJoinSubstitution(
                [
                    FindPackageShare("schunk_svh_simulation"),
                    "etc",
                    "svh_mujoco.xml",
                ]
            ),
            " ",
            "mesh_directory:=",
            PathJoinSubstitution(
                [
                    FindPackageShare("schunk_svh_simulation"),
                    "meshes",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("schunk_svh_simulation"),
            "config",
            "controller_manager.yaml",
        ]
    )

    # The actual simulation is a ROS2-control system interface.
    # Start that with the usual ROS2 controller manager mechanisms.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[("~/robot_description", "/robot_description"),],
        #prefix="screen -d -m gdb -command=/home/stefan/.gdb_debug_config --ex run --args",  # noqa E501
    )

    # TF tree
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Convenience function for easy spawner construction
    def controller_spawner(name, *args):
        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[name] + [a for a in args],
        )

    # Active controllers
    active_list = [
        "joint_state_broadcaster",
    ]
    active_spawners = [controller_spawner(controller) for controller in active_list]

    # Inactive controllers
    inactive_list = [
        "joint_trajectory_controller",
    ]
    state = "--inactive"
    inactive_spawners = [
        controller_spawner(controller, state) for controller in inactive_list
    ]

    # Nodes to start
    nodes = (
        [control_node, robot_state_publisher]
        + active_spawners
        + inactive_spawners
    )

    return LaunchDescription(declared_arguments + nodes)
