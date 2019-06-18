#!/usr/bin/env python

import pprint

import rospy
from gazebo_msgs.srv import ApplyBodyWrench, GetModelState, GetLinkState
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Pose, Wrench, Quaternion,Twist
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64
import PID
import numpy as np
import math


pp = pprint.PrettyPrinter(indent=4)

model_name = 'Bobby'

body_name = 'Bobby::loader'
node_name = 'response_force'
bumper_topic = "/test_depth"


def main():
    Response()
    rospy.spin()


class Response:
    rospy.init_node(node_name, anonymous=True)
    ns = rospy.get_namespace()

    Hz = 50
    rate = rospy.Rate(Hz)

    ns = rospy.get_namespace()
    if ns == '/':
        topic_states = model_name + '/link_states'
    else:
        topic_states = 'link_states'

 ## soil parameters

    K0 = 20  # penetration resistance of Sandy loamy soddy-podzolic soil g/cm^2
    density = 1441  # density of material in kg/m^3
    matirial_gravity = 1602  # specific gravity of material g/cm^3

 ## tool parameters
    S = 0.04342

 ## definitions

    res_wrench = Wrench()
    res_wrench.force.x = 0
    res_wrench.force.y = 0
    res_wrench.force.z = 0
    res_wrench.torque.x = 0
    res_wrench.torque.y = 0
    res_wrench.torque.z = 0
    loader_pos = Pose()
    contacts = ContactsState()
    orientation_q = Quaternion()
    q = Quaternion()

    depth = 0
    angular_vel = 0
    m = (2.7692)/(0.9538 + 0.2)  # the slope of the pile
    z_collision = 0
    roll = pitch = yaw = 0


    def get_depth(self, data):
        if (ContactsState.states != []):
         i=0
         for i in range(len(data.states)):
                 if ('box' in data.states[i].collision2_name) or ('box' in data.states[i].collision1_name):  # check that the string exist in collision2_name/1
                           self.depth = np.mean(data.states[i].depths)
                           self.z_collision = np.mean(data.states[i].contact_positions[0].z)
                           # rospy.loginfo(self.depth)

    def get_angular_vel(self, msg):
        self.angular_vel = msg.angular_velocity.y
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)


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
        self.pub2 = rospy.Publisher('position_real_arm', Float64, queue_size=10)

        while not rospy.is_shutdown():

            self.loader_pos = self.get_link_state('Bobby::loader', 'world').link_state.pose
            # self.box2_pos = self.get_model_state('box', 'world').pose
            z_pile = self.m * (self.loader_pos.position.x + 0.96 + 0.2)  # 0.96 is the distance between center mass of the loader to the end
            H = z_pile - self.z_collision
            # print("z collision:", self.z_collision)
            # print(H)
            # print(self.depth)
            if self.depth > 0.001:
                         F2 = self.K0 * math.cos(self.angular_vel) * self.matirial_gravity * H * self.S * 9.81
                         self.res_wrench.force.x = -(F2* math.cos(self.pitch))
                         self.res_wrench.force.z = -(((self.depth * H *1.66 *self.density) / 2) * 9.81 + F2 * math.sin(self.pitch))  # 1.66 is the tool width
            if self.depth <= 0.001:
                         self.res_wrench.force.x = 0
                         self.res_wrench.force.z = 0
            self.apply_body_wrench(body_name=body_name,
                                            reference_frame="",
                                            wrench=self.res_wrench,
                                            start_time=rospy.Time.from_sec(0),
                                            duration=rospy.Duration.from_sec(1.0))
            self.pub.publish(self.res_wrench)
            self.q = self.get_link_state('Bobby::main_arm', 'world').link_state.pose.orientation
            orientation_list =[self.q.x, self.q.y, self.q.z, self.q.w]
            (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)
            self.pub2.publish(self.pitch)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        main()
        rospy.spin()

    except rospy.ServiceException as e:
        print
        e