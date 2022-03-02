# - BEGIN LICENSE BLOCK -------------------------------------------------------
# - END LICENSE BLOCK ---------------------------------------------------------

# -----------------------------------------------------------------------------
# \file    schunk_svh_driver.launch.py
#
# \author  Stefan Scherzinger <scherzin@fzi.de>
# \date    2022/03/02
#
# -----------------------------------------------------------------------------

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

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
        prefix="screen -d -m gdb -command=/home/scherzin/.ros/my_debug_log --ex run --args",
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

    # List all nodes that we want to start
    nodes = [
        control_node,
        joint_state_controller_spawner
    ]

    return LaunchDescription(declared_arguments + nodes)
