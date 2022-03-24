#!/usr/bin/env python3
"""
Control the Schunk SVH's joints with a graphical slider

This is a small helper to manually test the SVH with ROS2-control.  It provides
a minimal GUI with a single slider that changes the SVH's state between a
`fist` and a `full spread`.  It assumes a running `joint_trajectory_controller`
for all joints.
"""
import threading
import numpy as np
from tkinter import Tk, Scale
import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class SVH(Node):
    def __init__(self):
        super().__init__('svh_gui')
        self.publisher = self.create_publisher(
            JointTrajectory, 'joint_trajectory_controller/joint_trajectory', 10)

        # Configuration presets
        self.joint_names = [
            'Left_Hand_Thumb_Flexion',
            'Left_Hand_Thumb_Opposition',
            'Left_Hand_Index_Finger_Distal',
            'Left_Hand_Index_Finger_Proximal',
            'Left_Hand_Middle_Finger_Distal',
            'Left_Hand_Middle_Finger_Proximal',
            'Left_Hand_Ring_Finger',
            'Left_Hand_Pinky',
            'Left_Hand_Finger_Spread',
        ]
        self.fist = [0.6, 0.6, 0.7, 0.7, 0.7, 0.7, 0.6, 0.6, 0.3]
        self.full_spread = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5]

    def gui(self):
        """ A GUI with a single slider for opening/closing the SVH """
        self.percent = 100
        self.window = Tk()
        self.window.title('Schunk SVH')
        self.slider = Scale(self.window, from_=self.percent, to=0)
        self.slider.set(self.percent)
        self.slider.bind("<ButtonRelease-1>", self.slider_changed)
        self.slider.pack()
        self.window.mainloop()

    def slider_changed(self, event):
        self.publish(self.slider.get())
        rclpy.spin_once(self, timeout_sec=0)

    def publish(self, opening):
        """ Publish a new joint trajectory with an interpolated state

        We scale linearly with opening=[0,1] between `fist` and `full_spread`.
        """
        jpos = opening * np.array(self.full_spread) + \
            (self.percent - opening) * np.array(self.fist)
        jpos = jpos / self.percent
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        jtp = JointTrajectoryPoint()
        jtp.positions = jpos.tolist()
        jtp.time_from_start = Duration(sec=2)
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
