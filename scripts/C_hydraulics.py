#!/usr/bin/env python
import pprint

import rospy
from robil_lihi.msg import BobcatControl
from gazebo_msgs.srv import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from dynamic_reconfigure.server import Server
from robil_lihi.cfg import PIDconConfig
import PID

pp = pprint.PrettyPrinter(indent=4)

model_name = 'Bobby'
service_name = '/gazebo/apply_joint_effort'
joint_name = model_name + '::Hydraulics_joint'
node_name = 'hydraulics_controller'


def main():
    Controller()
    rospy.spin()

# def callback(config, level):
#     rospy.loginfo("""Reconfigure Request: {kP}, {kI}, {kD}""".format(**config))
#     global kP, kI, kD
#     kP = config["kP"]
#     kI = config["kI"]
#     kD = config["kD"]
#     return config

def clamp(x, minimum, maximum):
    # type: (float, float, float) -> float
    return max(minimum, min(x, maximum))


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

    Hz = 50
    rate = rospy.Rate(Hz)
    req = ApplyJointEffortRequest()
    req.duration.nsecs = int(1e9 / Hz)
    req.joint_name = joint_name

    kP = 150000  # 150000
    kD = 50000  # 50000
    kI = 15000  # 15000

    state = 0
    position_SP = 0
    cmd = 0

    max_position = 0.38
    min_position = -0.1

    def __init__(self):

        rospy.wait_for_service(service_name)
        self.aje = rospy.ServiceProxy(service_name, ApplyJointEffort)
        self.pub = rospy.Publisher(node_name + '_publisher', Float32, queue_size=10)

        rospy.Subscriber(self.topic_CMD, BobcatControl, self.update_cmd)
        rospy.Subscriber(self.topic_states, JointState, self.update_state)

        # self.srv = Server(PIDconConfig, callback)
        # self.pid = PID.PID(kP, kI, kD)
        self.pid = PID.PID(self.kP, self.kI, self.kD)

        self.pid.setSampleTime(1.0 / self.Hz)

        while not rospy.is_shutdown():
            # self.pid = PID.PID(kP, kI, kD)
            self.position_SP = clamp(self.position_SP + self.cmd * 0.3 / self.Hz, self.min_position, self.max_position)
            self.pid.SetPoint = self.position_SP
            self.pid.update(self.state)

            self.req.effort = self.pid.output

            try:
                response = self.aje(self.req)
                if not response.success:
                    pp.pprint(response)
                self.pub.publish(self.req.effort)
            except Exception as e:
                rospy.logerr("Exception in Hydraulics Controller.")
                print(type(e))
                print(e)
                return

            self.rate.sleep()

    def update_cmd(self, data):
        """

        :param data:
        :type data: BobcatControl
        :return:
        :rtype:
        """
        self.cmd = data.hydraulics_velocity

    def update_state(self, data):
        ind = data.name.index('Hydraulics_joint')
        self.state = data.position[ind]


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
