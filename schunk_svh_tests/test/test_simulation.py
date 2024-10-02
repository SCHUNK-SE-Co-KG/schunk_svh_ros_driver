#!/usr/bin/env python3
import pytest
from rclpy.node import Node
import time
from test.conftest import launch_simulator


@pytest.mark.launch(fixture=launch_simulator)
def test_simulator_startup_works(launch_context, isolated):
    print("#################  Starting a ros2 node")
    node = Node("test_simulator")
    print("#################  Started the node")
    until_ready = 2.0  # sec
    time.sleep(until_ready)
    print("#################  Getting node names")
    nodes = node.get_node_names()
    print("#################  Got node names")
    print(nodes)
    assert "svh" in nodes
