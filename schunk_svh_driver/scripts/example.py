#!/usr/bin/env python3
################################################################################
#
# © Copyright 2022 SCHUNK Mobile Greifsysteme GmbH, Lauffen/Neckar Germany
# © Copyright 2022 FZI Forschungszentrum Informatik, Karlsruhe, Germany
#
# This file is part of the Schunk SVH Driver.
#
# The Schunk SVH Driver is free software: you can redistribute it and/or
# modify it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or (at your
# option) any later version.
#
# The Schunk SVH Driver is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
# Public License for more details.
#
# You should have received a copy of the GNU General Public License along with
# the Schunk SVH Driver. If not, see <https://www.gnu.org/licenses/>.
#
###############################################################################
"""
Control the Schunk SVH's joints with graphical sliders

This is a small helper to manually test the SVH with ROS2-control.  It provides
a minimal GUI with sliders that change the SVH's state for each joint individually.
It assumes a running `joint_trajectory_controller` for all joints.
"""
import numpy as np
from tkinter import Tk, Scale
import rclpy
from rclpy.node import Node
import subprocess
import sys

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


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

        # Control the hand with publishing joint trajectories.
        self.publisher = self.create_publisher(
            JointTrajectory, f'{controller}/joint_trajectory', 10)

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
        self.fist = [0.6, 0.6, 0.7, 0.7, 0.7, 0.7, 0.6, 0.6, 0.3]
        self.full_spread = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5]

    def gui(self):
        """ A GUI with sliders for joint actuation """
        self.percent = 100
        self.window = Tk()
        self.window.title('Schunk SVH')
        self.sliders = {}
        for name in self.joint_names:
            widget_name = name.lower() # Required by tkinter
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
        self.publish(event.widget._name, self.sliders[event.widget._name].get())
        rclpy.spin_once(self, timeout_sec=0)

    def publish(self, slider, state):
        """ Publish a new, single-joint trajectory with an interpolated state

        We scale linearly with state=[0,1] between the values for `fist` and
        `full_spread` for each joint.
        """
        index = [j.lower() for j in self.joint_names].index(slider)
        msg = JointTrajectory()
        msg.joint_names = [self.joint_names[index]]
        jtp = JointTrajectoryPoint()
        jpos = state * self.full_spread[index] + \
            (self.percent - state) * self.fist[index]
        jpos = jpos / self.percent
        jtp.positions = [jpos]
        jtp.time_from_start = Duration(sec=1)

        msg.points.append(jtp)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    svh = SVH()
    svh.gui()
    svh.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
