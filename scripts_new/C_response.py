#!/usr/bin/env python

import pprint

import rospy
from gazebo_msgs.srv import ApplyBodyWrench,GetModelState
from geometry_msgs.msg import Pose, Wrench
import PID


pp = pprint.PrettyPrinter(indent=4)

model_name = 'Box'

body_name = 'unit_box::link'
node_name = 'box_controller'


def main():
    Controller()
    rospy.spin()


def clamp(x, minimum, maximum):
    return max(minimum, min(x, maximum))


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

    # kP = 10
    # kD = 0.1
    # kI = 0.01
    k=1000

    new_wrench=Wrench()
    wrench = Wrench()
    wrench.force.x = 0
    wrench.force.y = 0
    wrench.force.z = 0
    wrench.torque.x = 0
    wrench.torque.y = 0
    wrench.torque.z = 0
    box1_pos=Pose()
    box2_pos=Pose()
    dd=0

    #link_state = GetLinkState()
    def getW(self,msg):
        ind = msg.name.index('unit_box::link')
        self.state = msg.get_Wrench[ind]

    def do(self,data):
        self.new_pos=data

    def __init__(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        rospy.wait_for_service('/gazebo/apply_body_wrench')
        self.apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        self.pub = rospy.Publisher('/response', Wrench, queue_size=10)
        z = 0

        while not rospy.is_shutdown():
            self.box1_pos = self.get_model_state('unit_box','world').pose
            self.box2_pos = self.get_model_state('unit_box2', 'world').pose

            try:
                    # z = 0
                    self.wrench.force.x = self.k * z
                    if ((self.box1_pos.position.x+0.05 > self.box2_pos.position.x-0.05) & (self.box1_pos.position.x-0.05 < self.box2_pos.position.x+0.05)):

                        z=abs(self.box1_pos.position.x+0.05 - self.box2_pos.position.x-0.05)
                        if(rospy.get_param('direction')==0):
                            z=-z;
                        self.wrench.force.x = self.k*z
                        self.apply_body_wrench(body_name="unit_box::link",
                                            reference_frame="",
                                            wrench=self.wrench,
                                            start_time=rospy.Time.from_sec(0),
                                            duration=rospy.Duration.from_sec(1.0))

                    self.pub.publish(self.wrench)

            except Exception as e:
                     rospy.logerr("Exception in Loader Controller.")

            #rospy.loginfo(str(z))

        return

        self.rate.sleep()


if __name__ == '__main__':
    try:
        main()

    except rospy.ServiceException as e:
        print
        e
