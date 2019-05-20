#!/usr/bin/env python
import pprint

import rospy
from robil_lihi.msg import BobcatControl
from gazebo_msgs.srv import *
from std_msgs.msg import Float32

pp = pprint.PrettyPrinter(indent=4)

model_name = 'Bobby'
service_name = '/gazebo/apply_joint_effort'
wFR_joint_name = model_name + '::front_right_wheel_joint'
wFL_joint_name = model_name + '::front_left_wheel_joint'
wBR_joint_name = model_name + '::back_right_wheel_joint'
wBL_joint_name = model_name + '::back_left_wheel_joint'

node_name = 'wheels_controller'
linear_velocity_multiplier = 30
angular_velocity_multiplier = 10


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

    # TODO make tunable through dynamic reconfigure or rosparam
    force_multiplier = 1e3

    def __init__(self):
        rospy.wait_for_service(service_name)

        self.apply_FRw_effort = rospy.ServiceProxy(service_name, ApplyJointEffort)
        self.apply_FLw_effort = rospy.ServiceProxy(service_name, ApplyJointEffort)
        self.apply_BRw_effort = rospy.ServiceProxy(service_name, ApplyJointEffort)
        self.apply_BLw_effort = rospy.ServiceProxy(service_name, ApplyJointEffort)
        self.r_FRw_effort = ApplyJointEffortRequest()
        self.r_FRw_effort.duration.nsecs = int(1e9 / self.Hz)
        self.r_FRw_effort.joint_name = wFR_joint_name
        self.r_FLw_effort = ApplyJointEffortRequest()
        self.r_FLw_effort.duration.nsecs = int(1e9 / self.Hz)
        self.r_FLw_effort.joint_name = wFL_joint_name
        self.r_BRw_effort = ApplyJointEffortRequest()
        self.r_BRw_effort.duration.nsecs = int(1e9 / self.Hz)
        self.r_BRw_effort.joint_name = wBR_joint_name
        self.r_BLw_effort = ApplyJointEffortRequest()
        self.r_BLw_effort.duration.nsecs = int(1e9 / self.Hz)
        self.r_BLw_effort.joint_name = wBL_joint_name

        self.pub = rospy.Publisher(node_name + '_DEBUG', Float32, queue_size=10)

        rospy.Subscriber(self.topic_CMD, BobcatControl, self.update_cmd)

        while not rospy.is_shutdown():
            try:
                self.apply_efforts()
            except Exception as e:
                rospy.logerr("Exception in Wheels Controller.")
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

        self.r_FRw_effort.effort = self.force_multiplier * (linear_velocity_multiplier * lin +
                                                            angular_velocity_multiplier * ang)
        self.r_FLw_effort.effort = self.force_multiplier * (linear_velocity_multiplier * lin -
                                                            angular_velocity_multiplier * ang)
        self.r_BRw_effort.effort = self.force_multiplier * (linear_velocity_multiplier * lin +
                                                            angular_velocity_multiplier * ang)
        self.r_BLw_effort.effort = self.force_multiplier * (linear_velocity_multiplier * lin -
                                                            angular_velocity_multiplier * ang)

    def apply_efforts(self):

        response = self.apply_FRw_effort(self.r_FRw_effort)
        if not response.success:
            pp.pprint(response)

        response = self.apply_FLw_effort(self.r_FLw_effort)
        if not response.success:
            pp.pprint(response)

        response = self.apply_BRw_effort(self.r_BRw_effort)
        if not response.success:
            pp.pprint(response)

        response = self.apply_BLw_effort(self.r_BLw_effort)
        if not response.success:
            pp.pprint(response)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
