
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_pytest
import rclpy
import pytest


# We avoid black's F811, F401 linting warnings
# by using pytest's special conftest.py file.
# See documentation here:
# https://docs.pytest.org/en/7.1.x/reference/fixtures.html#conftest-py-sharing-fixtures-across-multiple-files  # noqa: E501

@pytest.fixture(scope="module")
def isolated():
    rclpy.init()
    yield
    rclpy.shutdown()


@launch_pytest.fixture(scope="module")
def launch_simulator():
    setup = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("schunk_svh_simulation"),
                "launch",
                "simulation.launch.py",
            ]
        ),
    )
    return LaunchDescription([setup, launch_pytest.actions.ReadyToTest()])


@launch_pytest.fixture(scope="module")
def launch_driver():
    setup = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("schunk_svh_driver"), "launch", "schunk_svh_driver.launch.py"]
        )
    )
    until_ready = 10.0 # sec
    return LaunchDescription([setup, TimerAction(period=until_ready, actions=[launch_pytest.actions.ReadyToTest()])])
