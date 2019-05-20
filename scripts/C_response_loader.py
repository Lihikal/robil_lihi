#!/usr/bin/env python

import pprint

import rospy
from gazebo_msgs.srv import ApplyBodyWrench, GetModelState, GetLinkState
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Pose, Wrench
from sensor_msgs.msg import Imu
import numpy as np
import PID
import numpy as np
import re
import math


pp = pprint.PrettyPrinter(indent=4)

model_name = 'Bobby'

body_name = 'Bobby::loader'
node_name = 'loader controller'
bumper_topic = "/test_depth"


def main():
    Controller()
    rospy.spin()


def clamp(x, minimum, maximum):
    return max(minimum, min(x, maximum))


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

    # kP = 10
    # kD = 0.1
    # kI = 0.01
    K0 = 50000
    S = 0.04342
    res_wrench = Wrench()
    res_wrench.force.x = 0
    res_wrench.force.y = 0
    res_wrench.force.z = 0
    res_wrench.torque.x = 0
    res_wrench.torque.y = 0
    res_wrench.torque.z = 0
    loader_pos = Pose()
    box2_pos = Pose()
    contacts = ContactsState()
    depth = 0
    angular_vel = 0
    m = (2.7692)/(0.9538 + 0.2)  # the slope of the pile

    def get_depth(self, data):
        if (ContactsState.states != []):
         i=0
         for i in range(len(data.states)):
                 if ('box' in data.states[i].collision2_name) or ('box' in data.states[i].collision1_name):  # check that the string exist in collision2_name/1
                           self.depth = np.mean(data.states[i].depths)
                           # rospy.loginfo(self.depth)

    def get_angular_vel(self, msg):
        self.angular_vel = msg.angular_velocity.y

    def __init__(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        rospy.wait_for_service('/gazebo/get_link_state')
        self.get_link_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
        rospy.wait_for_service('/gazebo/apply_body_wrench')
        self.apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        self.pub = rospy.Publisher('/response', Wrench, queue_size=10)
        rospy.Subscriber('/robot_bumper', ContactsState, self.get_depth)
        rospy.Subscriber('/Bobby/imu', Imu, self.get_angular_vel)

        while not rospy.is_shutdown():
            self.loader_pos = self.get_link_state('Bobby::loader', 'world').link_state.pose
            # self.box2_pos = self.get_model_state('box', 'world').pose
            H = self.m *(self.loader_pos.position.x + 0.42 + 0.2)  # 0.42 is the distance between center mass of the loader to the end
            print("x loader:", self.loader_pos.position.x)
            print(H)
            if self.depth > 0:
                         self.res_wrench.force.x = -(self.K0 * math.cos(self.angular_vel) * self.depth * self.S * 9.81)

            self.apply_body_wrench(body_name=body_name,
                                            reference_frame="",
                                            wrench=self.res_wrench,
                                            start_time=rospy.Time.from_sec(0),
                                            duration=rospy.Duration.from_sec(1.0))

            self.pub.publish(self.res_wrench)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        main()

    except rospy.ServiceException as e:
        print
        e
