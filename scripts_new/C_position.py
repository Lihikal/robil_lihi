#!/usr/bin/env python


import pprint

import rospy
from gazebo_msgs.srv import ApplyBodyWrench, GetModelState
from geometry_msgs.msg import Pose, Wrench
import PID
from dynamic_reconfigure.server import Server
from robil_lihi.cfg import PIDconConfig

pp = pprint.PrettyPrinter(indent=4)

model_name = 'Box'

body_name = 'unit_box::link'
node_name = 'box_controller'
kP = 50
kD = 0.1
kI = 0.01

def main():
    Controller()
    rospy.spin()


def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {kP}, {kI}, {kD}""".format(**config))
    global kP, kI, kD
    kP = config["kP"]
    kI = config["kI"]
    kD = config["kD"]
    return config



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



    state = 0

    new_wrench = Wrench()
    wrench = Wrench()
    wrench.force.x = 0
    wrench.force.y = 0
    wrench.force.z = 0
    wrench.torque.x = 0
    wrench.torque.y = 0
    wrench.torque.z = 0
    new_pos = Pose()
    curr_pos = Pose()
    dd = 0

    # link_state = GetLinkState()


    def do(self, data):
        self.new_pos = data

    def __init__(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        rospy.wait_for_service('/gazebo/apply_body_wrench')
        self.apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        self.pub = rospy.Publisher('/new_position', Pose, queue_size=10)
        self.pub2 = rospy.Publisher('/force', Wrench, queue_size=10)  # publisher to the force on the box

        rospy.Subscriber('/new_position', Pose, self.do)  # subscriber to the user new position
        self.srv = Server(PIDconConfig, callback)

        self.pidx = PID.PID(kP, kI, kD)
        self.pidy = PID.PID(kP, kI, kD)

        self.pidx.setSampleTime(1.0 / self.Hz)
        self.pidy.setSampleTime(1.0 / self.Hz)


        while not rospy.is_shutdown():
            self.curr_pos = self.get_model_state('unit_box', 'world').pose
            self.pidx.SetPoint = self.new_pos.position.x
            self.pidx.update(self.curr_pos.position.x)
            self.wrench.force.x = self.pidx.output

            self.pidy.SetPoint = self.new_pos.position.y
            self.pidy.update(self.curr_pos.position.y)
            self.wrench.force.y = self.pidy.output


            try:
                self.apply_body_wrench(body_name="unit_box::link",
                                       reference_frame="",
                                       wrench=self.wrench,
                                       start_time=rospy.Time.from_sec(0),
                                       duration=rospy.Duration.from_sec(1.0))
                if(self.wrench.force.x>0):
                    rospy.set_param('direction',1)
                elif(self.wrench.force.x < 0):
                    rospy.set_param('direction',0)

                self.pub2.publish(self.wrench)

            except Exception as e:
                rospy.logerr("Exception in Loader Controller.")

            # rospy.loginfo(str(self.curr_pos.position))

        return

        self.rate.sleep()


if __name__ == '__main__':
    try:
        main()

    except rospy.ServiceException as e:
        print
        e
