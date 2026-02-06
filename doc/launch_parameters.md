# Launch Parameters Documentation

This document describes the launch parameters for the Schunk SVH ROS2 driver.

---

## Driver Launch Files

### schunk_svh_driver.launch.py

The main launch file for operating the Schunk SVH hand.

```bash
ros2 launch schunk_svh_driver schunk_svh_driver.launch.py [parameters]
```

#### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `device_file` | string | `/dev/ttyUSB0` | Device file of the Schunk SVH serial interface. This is the path to the serial port used to connect the hand. |
| `control` | string | `right_hand` | Specifies if the connected hand is a left or right hand. |

#### Example Usage

**Right hand (default):**
```bash
ros2 launch schunk_svh_driver schunk_svh_driver.launch.py
```

**Left hand:**
```bash
ros2 launch schunk_svh_driver schunk_svh_driver.launch.py control:=left_hand
```

**Custom device file:**
```bash
ros2 launch schunk_svh_driver schunk_svh_driver.launch.py device_file:=/dev/ttyUSB1
```

**Combined parameters:**
```bash
ros2 launch schunk_svh_driver schunk_svh_driver.launch.py device_file:=/dev/ttyUSB1 control:=left_hand
```

---

## Hardware Parameters

The following hardware parameters are configured in the URDF/xacro and passed to the ros2_control system interface:

| Parameter | Source | Description |
|-----------|--------|-------------|
| `device_file` | Launch argument | Serial device path for hardware communication |

Joint-specific controller parameters (current settings, position settings, home settings) are loaded from xacro files in `schunk_svh_driver/urdf/parameters/`. The parameters are described in a seperate [document](../schunk_svh_driver/urdf/parameters/README.md).

---

## Controller Configuration

Controller parameters are defined in `schunk_svh_driver/cfg/schunk_svh_driver.yaml`:

| Parameter | Value | Description |
|-----------|-------|-------------|
| `update_rate` | 125 Hz | Controller manager update frequency |
| `allow_partial_joints_goal` | True | Allows sending trajectory goals for a subset of joints |

