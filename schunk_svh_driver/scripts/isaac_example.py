#!/usr/bin/env python3
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
#
# Modified from https://github.com/SCHUNK-SE-Co-KG/schunk_svh_ros_driver/blob/ros2/schunk_svh_driver/scripts/example.py
# --------------------------------------------------------------------------------
"""
Control the Schunk SVH's joints with graphical sliders

This is a small helper to manually test the SVH with ROS2-control. It provides
a minimal GUI with sliders that change the SVH's state for each joint individually.
It assumes a running `joint_trajectory_controller` for all joints.
It now also publishes a sensor_msgs/msg/JointState message to the
/joint_states_control topic, so that the state of all joints can be used to drive it in Isaac Sim.
"""
import sys
import subprocess
from tkinter import Tk, Scale

import numpy as np
import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState  # new import for joint states


class SVH(Node):
    def __init__(self):
        super().__init__('svh_gui')

        # Check the available ROS2 nodes for a joint trajectory controller.
        # It's either called /left_hand or /right_hand.
        nodes = subprocess.check_output("ros2 node list", stderr=subprocess.STDOUT, shell=True)
        nodes = nodes.decode("utf-8").split('\n')
        if "/left_hand" in nodes:
            controller = "/left_hand"
            joint_prefix = "Left_Hand"
        elif "/right_hand" in nodes:
            controller = "/right_hand"
            joint_prefix = "Right_Hand"
        else:
            rclpy.shutdown()
            print("No suitable hand controller found.")
            sys.exit(0)

        # Publisher to control the hand via joint trajectories.
        self.publisher = self.create_publisher(
            JointTrajectory, f'{controller}/joint_trajectory', 10)

        # New publisher for joint state messages (to drive the simulation).
        self.joint_state_pub = self.create_publisher(JointState, f'{controller}/joint_command', 10)

        # Configuration presets
        self.joint_names = [
            f'{joint_prefix}_Thumb_Flexion',
            f'{joint_prefix}_Thumb_Opposition',
            f'{joint_prefix}_Index_Finger_Distal',
            f'{joint_prefix}_Index_Finger_Proximal',
            f'{joint_prefix}_Middle_Finger_Distal',
            f'{joint_prefix}_Middle_Finger_Proximal',
            f'{joint_prefix}_Ring_Finger',
            f'{joint_prefix}_Pinky',
            f'{joint_prefix}_Finger_Spread',
        ]
        self.fist = [0.6, 0.6, 0.76, 0.7, 0.76, 0.7, 0.6, 0.6, 0.3]
        self.full_spread = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5]

        # Initial position is full_spread.
        self.current_positions = [fs for fs in self.full_spread]

        # Define the ordering for the joint state message.
        # This order is chosen to match the joint hierarchy of the sim model:
        # [Index_Finger_Distal, Thumb_Flexion, Middle_Finger_Distal, Thumb_Opposition,
        #  Ring_Finger, Middle_Finger_Proximal, Pinky, Index_Finger_Proximal, Finger_Spread]
        # Based on the order in self.joint_names, the indices are:
        #   Thumb_Flexion           -> 0
        #   Thumb_Opposition        -> 1
        #   Index_Finger_Distal     -> 2
        #   Index_Finger_Proximal   -> 3
        #   Middle_Finger_Distal    -> 4
        #   Middle_Finger_Proximal  -> 5
        #   Ring_Finger             -> 6
        #   Pinky                   -> 7
        #   Finger_Spread           -> 8
        # The desired order is therefore:
        self.joint_state_order = [2, 0, 4, 1, 6, 5, 7, 3, 8]

    def gui(self):
        """ A GUI with sliders for joint actuation """
        self.percent = 100
        self.window = Tk()
        self.window.title('Schunk SVH')
        self.sliders = {}
        for name in self.joint_names:
            widget_name = name.lower()  # Required by tkinter
            self.sliders[widget_name] = Scale(
                self.window,
                from_=self.percent,
                to=0,
                orient='horizontal',
                length=250,
                label=widget_name,
                name=widget_name)
            self.sliders[widget_name].set(self.percent)
            self.sliders[widget_name].bind("<ButtonRelease-1>", self.slider_changed)
            self.sliders[widget_name].pack()
        self.window.mainloop()

    def slider_changed(self, event):
        widget_name = event.widget._name  # the widget's name corresponds to a lower-case joint name
        state = self.sliders[widget_name].get()
        self.publish(widget_name, state)
        rclpy.spin_once(self, timeout_sec=0)

    def publish(self, slider, state):
        """ Publish a new, single-joint trajectory and update the joint state

        We scale linearly with state=[0, self.percent] between the values for `fist` and
        `full_spread` for each joint.
        """
        # Find the index of the joint corresponding to this slider.
        joint_names_lower = [jn.lower() for jn in self.joint_names]
        try:
            index = joint_names_lower.index(slider)
        except ValueError:
            self.get_logger().error(f"Unknown slider name: {slider}")
            return

        # Compute the new joint position as a linear interpolation between fist and full_spread.
        # When state == self.percent -> full_spread, and when state == 0 -> fist.
        jpos = (state * self.full_spread[index] + (self.percent - state) * self.fist[index]) / self.percent

        self.current_positions[index] = jpos

        # Create and publish a JointTrajectory message for the single joint.
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [self.joint_names[index]]
        jtp = JointTrajectoryPoint()
        jtp.positions = [jpos]
        jtp.time_from_start = Duration(sec=1)
        traj_msg.points.append(jtp)
        self.publisher.publish(traj_msg)

        # Create and publish a full JointState message on /joint_states_control.
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.header.frame_id = ''

        # Rearrange the joint names and positions according to the desired order.
        js_msg.name = [self.joint_names[i] for i in self.joint_state_order]
        js_msg.position = [self.current_positions[i] for i in self.joint_state_order]

        nan_val = float('nan')
        js_msg.velocity = [nan_val] * len(js_msg.name)
        js_msg.effort = [nan_val] * len(js_msg.name)

        self.joint_state_pub.publish(js_msg)


def main(args=None):
    rclpy.init(args=args)
    svh = SVH()
    svh.gui()
    svh.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
