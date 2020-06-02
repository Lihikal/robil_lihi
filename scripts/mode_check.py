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


pp = pprint.PrettyPrinter(indent=4)

model_name = 'Bobby'

body_name = 'Bobby::loader'
node_name = 'learn_soil'
bumper_topic = "/test_depth"


def main():
    Controller()
    rospy.spin()

class Controller:
    rospy.init_node(node_name, anonymous=True)
    ns = rospy.get_namespace()

    Hz = 50
    rate = rospy.Rate(Hz)

    ns = rospy.get_namespace()
    if ns == '/':
        topic_states = model_name + '/link_states'
    else:
        topic_states = 'link_states'

    mode=0


    def check_mode(self, data):
        if data.data == 1:
            self.mode = 1
        elif data.data == 2:
            self.mode = 2


    def __init__(self):

        rospy.Subscriber('/system_mode', Int32, self.check_mode)

        # self.pub = rospy.Publisher('/WPD/Speed', TwistStamped, queue_size=10)




        while not rospy.is_shutdown():
            if self.mode == 1:
                os.system('rosrun robil_lihi bobcat_a2c.py -n trpo_bobcat -e MovingBobcat-v0')
            self.rate.sleep()
            if self.mode == 2:
                os.system('rosrun robil_lihi train.py')
            self.rate.sleep()

if __name__ == '__main__':
    try:

        main()
        rospy.spin()



    except rospy.ServiceException as e:
        print
        e
