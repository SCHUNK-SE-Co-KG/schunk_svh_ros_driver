import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from python_qt_binding import QtGui
from std_msgs.msg import Int8, Empty

class SVHResetGui(Plugin):

    def __init__(self, context):
        super(SVHResetGui, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('TrajectoryDesigner')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())

        # Create QWidget
        self._widget = QWidget()

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('schunk_svh_driver'), 'resource', 'SVHResetGui.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('SVHResetGuiUI')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self._widget.connect_button.clicked[bool].connect(self.ConnectButton)
        self._widget.reset_button.clicked[bool].connect(self.ResetButton)
        self._widget.enable_button.clicked[bool].connect(self.EnableButton)

        self._widget.finger_select_box.addItem("All",-1)
        self._widget.finger_select_box.addItem("Thumb Flexion",0)
        self._widget.finger_select_box.addItem("Thumb Opposition",1)
        self._widget.finger_select_box.addItem("Index Finger Distal",2)
        self._widget.finger_select_box.addItem("Index Finger Proximal",3)
        self._widget.finger_select_box.addItem("Middle Finger Distal",4)
        self._widget.finger_select_box.addItem("Middle Finger Proximal",5)
        self._widget.finger_select_box.addItem("Ring Finger",6)
        self._widget.finger_select_box.addItem("Pinky Finger",7)
        self._widget.finger_select_box.addItem("Finger Spread",8)

        self.reset_pub = rospy.Publisher('svh_controller/reset_channel',  Int8, queue_size=1)
        self.enable_pub = rospy.Publisher('svh_controller/enable_channel', Int8, queue_size=1)
        self.connect_pub = rospy.Publisher('svh_controller/connect',  Empty, queue_size=1)


    def ConnectButton(self):
        self.connect_pub.publish(Empty())
        rospy.loginfo("ConnectButton\n")

    def ResetButton(self):
        selected = self._widget.finger_select_box.itemData(self._widget.finger_select_box.currentIndex())
        self.reset_pub.publish(Int8(selected))
        rospy.loginfo("Reset\n" + str(selected))


    def EnableButton(self):
        selected = self._widget.finger_select_box.itemData(self._widget.finger_select_box.currentIndex())
        self.enable_pub.publish(Int8(selected))
        rospy.loginfo("Enabled\n" + str(selected))
