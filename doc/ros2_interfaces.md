# ROS2 Interfaces Documentation

This document describes the ROS2 topics, services, and actions provided by the Schunk SVH ROS2 driver.

## Overview

The Schunk SVH ROS2 driver uses standard ROS2-control controllers and broadcasters to expose the hand's functionality.

---

## Topics

### Published Topics

| Topic Name | Message Type | Description |
|------------|--------------|-------------|
| `/svh/initialized` | `std_msgs/msg/Bool` | Indicates whether the SVH has been successfully initialized and homed. Published once after startup with `true` on success or `false` on failure. |
| `/joint_states` | `sensor_msgs/msg/JointState` | Current state of all joints (position, velocity, effort). Published by the `joint_state_broadcaster`. |
| `/robot_description` | `std_msgs/msg/String` | The URDF robot description. Published by the `robot_state_publisher`. |
| `/tf` | `tf2_msgs/msg/TFMessage` | Transform tree for all links. Published by the `robot_state_publisher`. |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Static transforms. Published by the `robot_state_publisher`. |

### Subscribed Topics

| Topic Name | Message Type | Description |
|------------|--------------|-------------|
| `/<controller_name>/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | Accepts trajectory commands for the hand joints. The controller name is either `right_hand` or `left_hand` depending on the launch configuration. |

---

## Actions

### Joint Trajectory Controller Action

| Action Name | Action Type | Description |
|-------------|-------------|-------------|
| `/<controller_name>/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Action server for executing joint trajectories. Provides feedback during execution and a result upon completion. The controller name is either `right_hand` or `left_hand`. |

#### Goal Fields
- `trajectory` (`trajectory_msgs/msg/JointTrajectory`): The trajectory to execute
- `goal_time_tolerance` (`builtin_interfaces/msg/Duration`): Tolerance for goal time
- `goal_tolerance` (`control_msgs/msg/JointTolerance[]`): Per-joint goal tolerances
- `path_tolerance` (`control_msgs/msg/JointTolerance[]`): Per-joint path tolerances

#### Feedback Fields
- `header` (`std_msgs/msg/Header`): Timestamp
- `joint_names` (`string[]`): Names of joints being controlled
- `desired` (`trajectory_msgs/msg/JointTrajectoryPoint`): Desired state
- `actual` (`trajectory_msgs/msg/JointTrajectoryPoint`): Actual state
- `error` (`trajectory_msgs/msg/JointTrajectoryPoint`): Error between desired and actual

#### Result Fields
- `error_code` (`int32`): Result code (0 = success)
- `error_string` (`string`): Human-readable error description

---


---

## Hardware Interfaces

The driver exposes the following ros2_control hardware interfaces for each joint:

### Command Interfaces

| Interface | Type | Description |
|-----------|------|-------------|
| `position` | `double` | Target position command in radians |

### State Interfaces

| Interface | Type | Description |
|-----------|------|-------------|
| `position` | `double` | Current joint position in radians |
| `velocity` | `double` | Current joint velocity in radians/second |
| `effort` | `double` | Estimated joint effort (converted from motor current) |
| `current` | `double` | Raw motor current in milliamperes |

---

## Joint Names

The SVH has 9 controllable joints. The joint names depend on whether a left or right hand is configured:

### Right Hand Joints
- `Right_Hand_Thumb_Flexion`
- `Right_Hand_Thumb_Opposition`
- `Right_Hand_Index_Finger_Distal`
- `Right_Hand_Index_Finger_Proximal`
- `Right_Hand_Middle_Finger_Distal`
- `Right_Hand_Middle_Finger_Proximal`
- `Right_Hand_Ring_Finger`
- `Right_Hand_Pinky`
- `Right_Hand_Finger_Spread`

### Left Hand Joints
- `Left_Hand_Thumb_Flexion`
- `Left_Hand_Thumb_Opposition`
- `Left_Hand_Index_Finger_Distal`
- `Left_Hand_Index_Finger_Proximal`
- `Left_Hand_Middle_Finger_Distal`
- `Left_Hand_Middle_Finger_Proximal`
- `Left_Hand_Ring_Finger`
- `Left_Hand_Pinky`
- `Left_Hand_Finger_Spread`

---

## Example Usage

### Sending a Trajectory via Topic

```python
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

msg = JointTrajectory()
msg.joint_names = ['Right_Hand_Thumb_Flexion']

point = JointTrajectoryPoint()
point.positions = [0.5]  # radians
point.time_from_start = Duration(sec=1)

msg.points.append(point)
publisher.publish(msg)
```

### Sending a Trajectory via Action

```bash
ros2 action send_goal /right_hand/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {
    joint_names: ['Right_Hand_Thumb_Flexion'],
    points: [{positions: [0.5], time_from_start: {sec: 1}}]
  }}"
```

### Querying Joint States

```bash
ros2 topic echo /joint_states
```

### Checking Initialization Status

```bash
ros2 topic echo /svh/initialized --once
```

### Listing Available Controllers

```bash
ros2 control list_controllers
```

---

## Notes

- The driver uses the `joint_trajectory_controller` which supports partial joint goals (`allow_partial_joints_goal: True`), allowing control of individual fingers without specifying all joints.
- The controller update rate is 125 Hz. The update rate can be configured in `schunk_svh_driver.yaml`.
- Joint efforts are estimated from motor currents using internal conversion factors.
- The `/svh/initialized` topic is nor published continuously but once during initialization.
