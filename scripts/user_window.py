#!/usr/bin/env python
import os
import rospkg
import pprint
import roslib
import rospy
from gazebo_msgs.srv import ApplyBodyWrench, GetModelState, GetLinkState, BodyRequest, SetModelState
from gazebo_msgs.msg import ContactsState, ModelState
from geometry_msgs.msg import Pose, Wrench, Quaternion,Twist, TwistStamped
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64, String, Int32
import PID
import numpy as np
import math
from create_file import create
from read_file import read_file, check_soil
from robil_lihi.msg import BobcatControl
from sensor_msgs.msg import Joy
import time
from std_srvs.srv import Empty
import random
import sys
from PySide2 import QtGui, QtCore
from PySide2.QtWidgets import *
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
import rospy




class Form(QDialog):
    def __init__(self, parent=None):
        super(Form, self).__init__(parent)
        self.setWindowTitle("User Window")
        # Create widgets
        self.headline = QLabel("Soil type")
        # self.edit = QLineEdit("Write my name here")
        self.button_check = QPushButton("Check soil")
        self.button_stop = QPushButton("Stop Checking")
        # Create layout and add widgets
        layout = QVBoxLayout()
        # layout.addWidget(self.edit)
        layout.addWidget(self.button_check)
        layout.addWidget(self.button_stop)
        layout.addWidget(self.headline)
        # Set dialog layout
        self.setLayout(layout)
        # Add button signal to greetings slot
        self.button_check.clicked.connect(self.greetings)
        self.button_stop.clicked.connect(self.stopcheck)
        pub = rospy.Publisher('chatter', String, queue_size=10)
        rospy.init_node('talker', anonymous=True)

    def greetings(self):
        print ("Hello ")#%s" % self.edit.text())
        os.system('rosrun robil_lihi check_soil.py')
        # print(results)

    def stopcheck(selfself):
        print ("Stoppppp")
        os.system("rosnode kill check_soil")


if __name__ == '__main__':
        app = QApplication(sys.argv)
        form = Form()
        form.show()
    # Run the main Qt loop
        sys.exit(app.exec_())

