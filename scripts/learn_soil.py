#!/usr/bin/env python

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

"""
-=soil parameters=-
type     | K0      |density  | matirial gravity
```   ```|`````````|`````````|``````
dry sand |    20   |  1441   |   1602
wet sand |    12   |  1922   |   2082
garbel   |    X    |         |
  3      |    Y    |
  4      |    LB   |
  5      |    RB   |

`````````````````````````````````````````````
"""
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

    # soil_type = {"dry_sand": 0, "wet_sand": 1} #, "garbel": 2}
    # soil = "wet_sand"
    # K0 = 12  # penetration resistance of material
    # density = 1922  # density of material in kg/m^3
    # matirial_gravity = 2082  # specific gravity of material g/cm^3

 ## tool parameters
    S = 0.04342

 ## definitions

    linear_velocity_multiplier = 30
    angular_velocity_multiplier = 10
    force_multiplier = 1e3
    loader_pos = Pose()
    contacts = ContactsState()
    orientation_q = Quaternion()
    force_on_bobcat = 0
    joy_val = 0
    c=0
    command = TwistStamped()
    depth = 0
    angular_vel = 0
    m = (2.7692)/(0.9538 + 0.2)  # the slope of the pile
    z_collision = 0
    roll = pitch = yaw = 0
    model_state = ModelState()
    model_state.pose = Pose()
    model_state.pose.position.x = -3
    model_state.pose.position.y = 0
    model_state.pose.position.z = 0.09
    model_state.pose.orientation.x = 0
    model_state.pose.orientation.y = -0.00287877431194
    model_state.pose.orientation.z = 0
    model_state.pose.orientation.w = 1

    model_state.model_name = "Bobby"


    def get_depth(self, data):
        if (ContactsState.states != []):
         i=0
         for i in range(len(data.states)):
                 if ('box' in data.states[i].collision2_name) or ('box' in data.states[i].collision1_name):  # check that the string exist in collision2_name/1
                           self.depth = np.mean(data.states[i].depths)
                           self.z_collision = np.mean(data.states[i].contact_positions[0].z)
                           # rospy.loginfo(self.depth)
                 if ('ground' in data.states[i].collision2_name) or ('ground' in data.states[i].collision1_name):
                           self.c = 1

    def get_angular_vel(self, msg):
        self.angular_vel = msg.angular_velocity.y
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)


    def get_wrench(self, data):
        self.force_x = data.force.x
        self.force_z = data.force.z

    def get_hyd(self, data):
        self.hyd = data

    def get_vel(self, msg):
        self.bobcat_vel = msg.twist.linear.x

    def get_soil(self, msg):
        self.soil = msg.data


    def __init__(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        rospy.wait_for_service('/gazebo/get_link_state')
        self.get_link_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)

        rospy.Subscriber('/robot_bumper', ContactsState, self.get_depth)
        rospy.Subscriber('/response', Wrench, self.get_wrench)
        rospy.Subscriber('/bobcat/arm/hydraulics', Float64, self.get_hyd)
        rospy.Subscriber('/WPD/Speed', TwistStamped, self.get_vel)
        rospy.Subscriber('/soil_type', Int32, self.get_soil)
        self.pub = rospy.Publisher('/WPD/Speed', TwistStamped, queue_size=10)
        self.pub2 = rospy.Publisher('/bobcat/arm/hydraulics', Float64, queue_size=10)
        self.pub3 = rospy.Publisher('/bobcat/arm/loader', Float64, queue_size=10)
        self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.clear_wrench = rospy.ServiceProxy('/gazebo/clear_body_wrenches', BodyRequest)
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.g_pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.g_unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)

        self.loader_pos = self.get_link_state('Bobby::loader', 'world').link_state.pose
        while self.loader_pos.position.z > 0.13: #self.c == 0:
                self.pub2.publish(-0.2)
                self.loader_pos = self.get_link_state('Bobby::loader', 'world').link_state.pose
        self.pub2.publish(0)

        self.loader_pos = self.get_link_state('Bobby::loader', 'world').link_state.pose
        orientation_q = self.loader_pos.orientation
        orientation_loader = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_loader)

        while self.pitch > 0.0:
                self.pub3.publish(0.3)
                self.loader_pos = self.get_link_state('Bobby::loader', 'world').link_state.pose
                orientation_q = self.loader_pos.orientation
                orientation_loader = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_loader)
        self.pub3.publish(0)
        self.command.twist.linear.x = round(random.uniform(0.06, 0.5), 3)


        while not rospy.is_shutdown():

            self.pub.publish(self.command)
            self.loader_pos = self.get_link_state('Bobby::loader', 'world').link_state.pose
            z_pile = self.m * (self.loader_pos.position.x + 0.96)  # 0.96 is the distance between center mass of the loader to the end
            H = z_pile - self.z_collision
            self.bobby_true_vel = self.get_model_state('Bobby', 'world').twist.linear
            # print("linear:", self.command.linear.x)
            # print("vel:", self.bobby_true_vel.x)
            # print("WPD- vel:", self.command.twist.linear.x)
            if self.bobby_true_vel.x < 0.01 and self.depth > 0.05 and self.loader_pos.position.x > -1.25:
                 # build data for the learning algorithm
                 create(self.soil, self.bobcat_vel, self.force_x, self.force_z, self.depth, ((self.depth * H *1.66) /2))
                 self.command.twist.linear.x = 0
                 self.pub.publish(self.command)
                 # time.sleep(1)
                 self.depth = 0
                 rospy.wait_for_service('/gazebo/clear_body_wrenches')
                 try:
                     # reset_proxy.call()
                     self.clear_wrench('Bobby::loader')
                     time.sleep(1)
                     rospy.wait_for_service("/gazebo/pause_physics")
                     try:
                        self.g_pause()
                     except Exception as e:
                        rospy.logerr('Error on calling service: %s', str(e))
                     # rospy.sleep(3)
                     time.sleep(1)
                     rospy.wait_for_service('/gazebo/reset_simulation')
                     try:
                        # self.set_model_state(self.model_state)
                        # self.reset_world()
                        self.reset_simulation()

                     except Exception as e:
                        rospy.logerr('Error on calling service: %s', str(e))
                     time.sleep(1)
                     rospy.wait_for_service("/gazebo/unpause_physics")
                     try:
                        self.g_unpause()
                     except Exception as e:
                        rospy.logerr('Error on calling service: %s', str(e))
                     self.command.twist.linear.x = round(random.uniform(0.06, 0.5), 3)
                     time.sleep(2)
                     # time.sleep(5)
                     # rospy.sleep(2)
                 except Exception as e:
                    print("/gazebo/reset_simulation service call failed")
            # self.body_pos = self.get_link_state('Bobby::body', 'world').link_state.pose
            # self.command.linear.x = 0.1
            # self.pub.publish(self.command)



            self.rate.sleep()


if __name__ == '__main__':
    try:
        main()
        rospy.spin()

    except rospy.ServiceException as e:
        print
        e
