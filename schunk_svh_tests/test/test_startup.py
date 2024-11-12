#!/usr/bin/env python3

import pytest
import rclpy
import rclpy.client
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchController
from test.conftest import launch_driver


@pytest.mark.launch(fixture=launch_driver)
def test_driver_launches_correctly(launch_context, isolated):
    our_controllers = [
            'right_hand',
            'joint_state_broadcaster',
        ]

    node = Node("test_startup")

    # Setup interfaces
    timeout = rclpy.time.Duration(seconds=5)
    list_controllers = node.create_client(ListControllers, "/controller_manager/list_controllers")
    if not list_controllers.wait_for_service(timeout.nanoseconds/1e9):
        assert False, "Service list_controllers not available"

    switch_controller = node.create_client(SwitchController, '/controller_manager/switch_controller')
    if not switch_controller.wait_for_service(timeout.nanoseconds/1e9):
        assert False, "Service switch_controllers not available"

    def check_controller_state(controller: str, state: str) -> bool:
        req = ListControllers.Request()
        future = list_controllers.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        for entry in future.result().controller:
            if entry.name == controller:
                return True if entry.state == state else False
        return False

    for controller in our_controllers:
        assert check_controller_state(controller, 'active')
