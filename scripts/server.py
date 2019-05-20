#!/usr/bin/env python

import pprint

import rospy

from gazebo_msgs.srv import ApplyBodyWrench,GetModelState
from geometry_msgs.msg import Pose, Wrench
import PID
from dynamic_reconfigure.server import Server
from robil_lihi.cfg import PIDconConfig

# import dynamic_reconfigure.client
kP = 0
kD = 0
kI = 0

pp = pprint.PrettyPrinter(indent=4)

model_name = 'Box'

body_name = 'unit_box::link'
node_name = 'box_controller'

# def callback(config):
#     rospy.loginfo("Config set to {kP}, {kI}, {kD}".format(**config))

def callback(config, level):
     rospy.loginfo("""Reconfigure Request: {kP}, {kI}, {kD}""".format(**config))
     global kP,kI,kD
     kP = config["kP"]
     kI = config["kI"]
     kD = config["kD"]
     return config

def main():
    Controller()
    rospy.spin()


class Controller:
    rospy.init_node(node_name,anonymous=True)
    ns = rospy.get_namespace()

    Hz = 50
    rate = rospy.Rate(Hz)

    ns = rospy.get_namespace()
    if ns == '/':
        topic_states = model_name + '/link_states'
    else:
        topic_states = 'link_states'


    new_wrench=Wrench()
    wrench = Wrench()
    wrench.force.x = 0
    wrench.force.y = 0
    wrench.force.z = 0
    wrench.torque.x = 0
    wrench.torque.y = 0
    wrench.torque.z = 0
    new_pos=Pose()
    curr_pos=Pose()
    dd=0

    #link_state = GetLinkState()
    def getW(self,msg):
        ind = msg.name.index('unit_box::link')
        self.state = msg.get_Wrench[ind]

    def do(self,data):
        self.new_pos=data

    def do2(self,msg):
        self.new_wrench=msg

    def __init__(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        rospy.wait_for_service('/gazebo/apply_body_wrench')
        self.apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

        self.pub = rospy.Publisher('/new_position', Pose, queue_size=10)
        rospy.Subscriber('/new_position', Pose, self.do) #subscriber to the user new position
        self.srv = Server(PIDconConfig, callback)
        self.pub = rospy.Publisher('/force', Wrench, queue_size=10) #publisher to the force on the box

        self.pidx = PID.PID(kP, kI, kD)
        self.pidy = PID.PID(kP, kI, kD)
        # self.pidz = PID.PID(kP, kI, kD)
        self.pidx.setSampleTime(1.0 / self.Hz)
        self.pidy.setSampleTime(1.0 / self.Hz)
        # self.pidz.setSampleTime(1.0 / self.Hz)

        while not rospy.is_shutdown():
            self.curr_pos = self.get_model_state('unit_box','world').pose
            self.pidx.SetPoint =self.new_pos.position.x
            self.pidx.update(self.curr_pos.position.x)
            self.wrench.force.x = self.pidx.output

            self.pidy.SetPoint=self.new_pos.position.y
            self.pidy.update(self.curr_pos.position.y)
            self.wrench.force.y = self.pidy.output

            # self.pidz.SetPoint=self.new_pos.position.z
            # self.pidz.update(self.curr_pos.position.z)
            # self.wrench.force.z = self.pidz.output

            try:
                    self.apply_body_wrench(body_name="unit_box::link",
                                            reference_frame="",
                                            wrench=self.wrench,
                                            start_time=rospy.Time.from_sec(0),
                                            duration=rospy.Duration.from_sec(1.0))
                    self.pub.publish(self.wrench)
            except Exception as e:
                      rospy.logerr("Exception in Loader Controller.")

            rospy.loginfo(str(kP))

        # return

        # self.rate.sleep()


if __name__ == '__main__':
    try:
        # self.srv = Server(PIDconConfig, callback)
        main()

    except rospy.ServiceException as e:
        print
        e
