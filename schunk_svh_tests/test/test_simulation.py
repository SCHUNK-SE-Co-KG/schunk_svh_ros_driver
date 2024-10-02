#!/usr/bin/env python3
import pytest
from rclpy.node import Node
import time
from test.conftest import launch_simulator


@pytest.mark.launch(fixture=launch_simulator)
def test_simulator_startup_works(launch_context, isolated):
    node = Node("test_simulator")
    until_ready = 2.0  # sec
    time.sleep(until_ready)
    nodes = node.get_node_names()
    print(nodes)
    assert "svh" in nodes
