# Schunk SVH Tests
Integration tests and high-level concept validation for the Schunk SVH ROS2 driver.

Useful information on integration tests in ROS2 is available [here][1]

## Run tests manually
In a sourced terminal, call
```bash
colcon test --packages-select schunk_svh_tests && colcon test-result --verbose
```
to run and inspect the integration tests locally.

[1]: https://github.com/ros2/launch/tree/master/launch_testing#quick-start-example
