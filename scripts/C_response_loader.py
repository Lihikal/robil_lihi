#!/usr/bin/env python

import pprint

import rospy
from gazebo_msgs.srv import ApplyBodyWrench, GetModelState, GetLinkState
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Pose, Wrench, Quaternion,Twist
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64, String, Int32
import numpy as np
import math
from sensor_msgs.msg import Joy

"""
-=soil parameters=-
type     | K0      |density  | matirial gravity
```   ```|`````````|`````````|``````
dry sand |  20/35  |  1441   |   1602
wet sand |  12/40  |  1922   |   2082
garbel   |    X    |         |
  3      |    Y    |
  4      |    LB   |
  5      |    RB   |
`````````````````````````````````````````````
"""
pp = pprint.PrettyPrinter(indent=4)

model_name = 'Bobby'

body_name = 'Bobby::loader'
node_name = 'response_force'
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

    soil_type = {"dry_sand": 0, "wet_sand": 1} #, "garbel": 2}

    soil = "wet_sand"
    K0 = 40  # penetration resistance of material
    density = 1922  # density of material in kg/m^3
    matirial_gravity = 2082  # specific gravity of material g/cm^3

 ## tool parameters
    S = 0.04342

 ## definitions

    linear_velocity_multiplier = 30
    angular_velocity_multiplier = 10
    force_multiplier = 1e3
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
    force_on_bobcat = 0
    joy_val = 0
    x=0
    z_collision = 0
    volume = 0
    volume_sum=0
    last_z_pile = 0
    last_x_step = 0
    last_z_collision = 0

    depth = 0
    angular_vel = 0
    m = (2.1213)/(1.9213 + 0.2)  # the slope of the pile
    z_collision = 0
    roll = pitch = yaw = 0


    def get_depth(self, data):
        if (ContactsState.states != []):
         i=0
         for i in range(len(data.states)):
                 if ('box' in data.states[i].collision2_name) or ('box' in data.states[i].collision1_name):  # check that the string exist in collision2_name/1
                           self.depth = np.mean(data.states[i].depths)
                           self.z_collision = np.mean(data.states[i].contact_positions[0].z)

    def get_angular_vel(self, msg):
        self.angular_vel = msg.angular_velocity.y
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)

    def get_linear_vel(self, msg):
        lin = msg.linear_velocity
        ang = msg.angular_velocity
        self.force_on_bobcat = 4* self.force_multiplier * (self.linear_velocity_multiplier * lin +
                                                            self.angular_velocity_multiplier * ang)

    def get_joy(self, msg):
        self.joy_val = msg.linear_velocity

    def __init__(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.get_link_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)

        self.apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        self.pub = rospy.Publisher('/response', Wrench, queue_size=10)
        self.pub2 = rospy.Publisher('/soil_type', Int32, queue_size=10)
        self.pub3 = rospy.Publisher('/system_mode', Int32, queue_size=10)

        rospy.Subscriber('/robot_bumper', ContactsState, self.get_depth)
        rospy.Subscriber('/Bobby/imu', Imu, self.get_angular_vel)
        rospy.Subscriber('/joy', Joy, self.get_joy)
        self.pub3.publish(0)

        while not rospy.is_shutdown():
            self.pub2.publish(self.soil_type[self.soil])
            rospy.wait_for_service('/gazebo/get_link_state')
            self.loader_pos = self.get_link_state('Bobby::loader', 'world').link_state.pose
            self.body_pos = self.get_link_state('Bobby::body', 'world').link_state.pose
            z_pile = self.m * (self.loader_pos.position.x + 0.96 * math.cos(self.pitch) + 0.2)  # 0.96 is the distance between center mass of the loader to the end
            H = z_pile - self.z_collision

            if self.depth > 0.001 :
                         z_pile = self.m * (self.loader_pos.position.x + 0.96 * math.cos(
                            self.pitch) + 0.2)  # 0.96 is the distance between center mass of the loader to the end
                         x = self.loader_pos.position.x + 0.96 * math.cos(self.pitch)
                         big_trapezoid = (x - self.last_x_step) * (z_pile + self.last_z_pile) / 2
                         small_trapezoid = (x - self.last_x_step) * (self.z_collision + self.last_z_collision) / 2
                         volume = (big_trapezoid - small_trapezoid) * 1.66 * self.density  # 1.66 is the tool width
                         if z_pile > 0 and self.z_collision > 0 and z_pile > self.z_collision:
                             self.volume_sum = self.volume_sum + volume
                         self.last_z_pile = z_pile
                         self.last_x_step = x
                         self.last_z_collision = self.z_collision
                         F2 = self.K0 * math.cos(self.angular_vel) * self.matirial_gravity * H * self.S * 9.81
                         self.res_wrench.force.x = -(F2 * math.cos(self.pitch))
                         self.res_wrench.force.z = -(volume * 9.81 + F2 * math.sin(self.pitch))
                         # build data for the learning algorithm
            if self.depth <= 0.001:
                         self.res_wrench.force.x = 0
                         self.res_wrench.force.z = 0

            rospy.wait_for_service('/gazebo/apply_body_wrench')
            try:
                self.apply_body_wrench(body_name=body_name,
                                            reference_frame="",
                                            wrench=self.res_wrench,
                                            start_time=rospy.Time.from_sec(0),
                                            duration=rospy.Duration.from_sec(1.0))
            except Exception as e:
                print("/gazebo/reset_simulation service call failed")

            self.pub.publish(self.res_wrench)

            self.rate.sleep()


if __name__ == '__main__':
    try:
        main()
        rospy.spin()

    except rospy.ServiceException as e:
        print
        e