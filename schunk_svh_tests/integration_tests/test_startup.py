#!/usr/bin/env python3
import unittest

import launch
import launch.actions
import launch_testing.actions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import os
import time
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchController


def generate_test_description():

    setup = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("schunk_svh_driver"), "launch", "schunk_svh_driver.launch.py"]
        )
    )
    until_ready = 10.0 # sec
    return LaunchDescription([setup, TimerAction(period=until_ready, actions=[launch_testing.actions.ReadyToTest()])])


class IntegrationTest(unittest.TestCase):
    """ An integration test for the basic workflow of ROS2-control

    We test if the hand controller and the joint state controller started as expected.
    """
    def __init__(self, *args):
        super().__init__(*args)

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_startup")
        cls.setup_interfaces(cls)

        cls.our_controllers = [
            'right_hand',
            'joint_state_controller',
        ]


    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setup_interfaces(self):
        """ Setup interfaces for ROS2 services and topics """

        timeout = rclpy.time.Duration(seconds=5)
        self.list_controllers = self.node.create_client(ListControllers, "/controller_manager/list_controllers")
        if not self.list_controllers.wait_for_service(timeout.nanoseconds/1e9):
            self.fail("Service list_controllers not available")

        self.switch_controller = self.node.create_client(SwitchController, '/controller_manager/switch_controller')
        if not self.switch_controller.wait_for_service(timeout.nanoseconds/1e9):
            self.fail("Service switch_controllers not available")

    def test_controller_startup(self):
        """ Test whether every controller started correctly

        We check if the list of all controllers currently managed by the
        controller manager contains our controllers and if they have `state:
        active`.
        """
        for name in self.our_controllers:
            self.assertTrue(self.check_state(name, 'active'), f"{name} is started correctly")

    def check_state(self, controller, state):
        """ Check the controller's state

        Return True if the controller's state is `state`, else False.
        Return False if the controller is not listed.
        """
        req = ListControllers.Request()
        future = self.list_controllers.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        for entry in future.result().controller:
            if entry.name == controller:
                return True if entry.state == state else False
        return False
