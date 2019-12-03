#!/usr/bin/env python
import pprint

import rospy
from robil_lihi.msg import BobcatControl
from gazebo_msgs.srv import *
from std_msgs.msg import Float32

pp = pprint.PrettyPrinter(indent=4)

model_name = 'Bobby'
service_name = '/gazebo/apply_joint_effort'
RW_joint_name = model_name + '::front_right_wheel_joint'
LW_joint_name = model_name + '::front_left_wheel_joint'


node_name = 'tracks_controller'
linear_velocity_multiplier = 10


def main():
    Controller()
    rospy.spin()


class Controller:
    rospy.init_node(node_name)

    ns = rospy.get_namespace()
    if ns == '/':
        topic_states = model_name + '/joint_states'
    else:
        topic_states = 'joint_states'

    if ns == '/':
        topic_CMD = model_name + '/controlCMD'
    else:
        topic_CMD = 'controlCMD'

    state = 0

    Hz = 50
    rate = rospy.Rate(Hz)

    force_multiplier = 1e3

    def __init__(self):
        rospy.wait_for_service(service_name)

        self.apply_RW_effort = rospy.ServiceProxy(service_name, ApplyJointEffort)
        self.apply_LW_effort = rospy.ServiceProxy(service_name, ApplyJointEffort)

        self.RW_effort = ApplyJointEffortRequest()
        self.RW_effort.duration.nsecs = int(1e9 / self.Hz)
        self.RW_effort.joint_name = RW_joint_name
        self.LW_effort = ApplyJointEffortRequest()
        self.LW_effort.duration.nsecs = int(1e9 / self.Hz)
        self.LW_effort.joint_name = LW_joint_name

        self.pub = rospy.Publisher(node_name + '_DEBUG', Float32, queue_size=10)

        rospy.Subscriber(self.topic_CMD, BobcatControl, self.update_cmd)

        while not rospy.is_shutdown():
            print("in")
            try:
                self.apply_efforts()
            except Exception as e:
                rospy.logerr("Exception in Tracks Controller.")
                print(type(e))
                print(e)
                return

            self.rate.sleep()

    def update_cmd(self, data):
        """

        :param data:
        :type data: BobcatControl
        """
        lin = data.linear_velocity
        ang = data.angular_velocity

        self.RW_effort.effort = self.force_multiplier * (linear_velocity_multiplier * lin)*2
        self.LW_effort.effort = self.force_multiplier * (linear_velocity_multiplier * ang)/5

    def apply_efforts(self):

        response = self.apply_RW_effort(self.RW_effort)
        if not response.success:
            pp.pprint(response)

        response = self.apply_LW_effort(self.LW_effort)
        if not response.success:
            pp.pprint(response)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
