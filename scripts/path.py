#!/usr/bin/env python
import os
import rospkg
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
import sys
import csv
from gazebo_connection import GazeboConnection



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
    BobcatPos_x=0

    model_state.model_name = "Bobby"
    flag=0
    hyd=0
    score = 0
    tic = time.time()
    toc = 0
    done = 0

    def getBobcatPos_x(self):  # Normalizing the position x of loader to 0-5
        self.loader_pos = self.get_link_state('Bobby::loader', 'world').link_state.pose
        self.BobcatPos_x = math.floor((self.loader_pos.position.x+0.2)/(0.95+0.2)*5)
        # print(self.BobcatPos_x)
        if self.BobcatPos_x > 5:
            self.BobcatPos_x = 5
        if self.BobcatPos_x < 0:
            self.BobcatPos_x = 0
        return self.BobcatPos_x

    def getBobcatPos_z(self):  # Normalizing the position z of loader to 0-5
        self.loader_pos = self.get_link_state('Bobby::loader', 'world').link_state.pose
        self.BobcatPos_z = math.floor((self.loader_pos.position.z-0)/(3-0)*5)
        if self.BobcatPos_z > 5:
            self.BobcatPos_z = 5
        if self.BobcatPos_z < 0:
            self.BobcatPos_z = 0
        return self.BobcatPos_z

    def getBobcatVel(self):  # Normalizing the bobcat velocity to 0-3
        self.bobby_true_vel = self.get_model_state('Bobby', 'world').twist.linear
        self.BobcatVel = math.floor((self.bobby_true_vel.x-0)/(0.5-0)*3)
        if self.BobcatVel > 3:
            self.BobcatVel = 3
        if self.BobcatVel < 0:
            self.BobcatVel = 0
        return self.BobcatVel

    def getArm_Bucket_Vel(self):  # Normalizing the arm velocity to 0-2 and the bucket velocity to 0-2
        self.ArmVel = math.floor((float(self.hyd))/(0.5-0.0)*2)
        if self.ArmVel > 2:
            self.ArmVel = 2
        if self.ArmVel < 0:
            self.ArmVel = 0
        self.BucketVel = math.floor((float(self.bucket_vel))/(0.5-0.0)*2)
        if self.BucketVel > 2:
            self.BucketVel = 2
        if self.BucketVel < 0:
            self.BucketVel = 0

    def DoAction(self, action):  # ["Bobcat x velocity", "Arm velocity", "Bobcat and Arm velocity", "Bucket velocity"]
        global score, action_count
        # self.score = self.score - 10

        if action == 1:  # action bobcat velocity
            # r = round(random.uniform(0.1, 0.5), 3)
            self.command.twist.linear.x = round(random.uniform(0.1, 0.5), 3)
            self.pub.publish(self.command)
            self.pub2.publish(0)
            self.pub3.publish(0)
            self.scoreCalc()
            # print("action 1")
        if action == 2:  # action arm velocity
            r = round(random.uniform(-0.1, -0.5), 3)
            self.command.twist.linear.x = 0
            self.pub.publish(self.command)
            self.pub2.publish(r)
            self.pub3.publish(0)
            self.scoreCalc()
            # print("action 2")
        if action == 3:  # action bobcat&arm velocity
            r = round(random.uniform(-0.1, -0.5), 3)
            self.command.twist.linear.x = round(random.uniform(0.1, 0.5), 3)
            self.pub.publish(self.command)
            self.pub2.publish(r)
            self.pub3.publish(0)
            self.scoreCalc()
            # print("action 3")
        if action == 4:  # bucket velocity
            r = np.random.uniform(0.1, 0.5)
            self.command.twist.linear.x = 0
            self.pub.publish(self.command)
            self.pub2.publish(0)
            self.pub3.publish(r)
            self.scoreCalc()
            # print("action 4")

        self.BobcatPos_x = self.getBobcatPos_x()
        self.BobcatPos_z = self.getBobcatPos_z()
        self.BobcatVel = self.getBobcatVel()
        self.getArm_Bucket_Vel()
        self.FieldState = np.array([self.BobcatPos_x, self.BobcatPos_z, self.BobcatVel, self.ArmVel, self.BucketVel])

    def scoreCalc(self):  # function for score fail/success
        global score
        self.BobcatPos_x = self.getBobcatPos_x()
        self.BobcatPos_z = self.getBobcatPos_z()
        self.BobcatVel = self.getBobcatVel()
        # self.score = self.score + 1  # just for now, need to delete
        self.toc = time.time()
        print("toc - tic =", self.toc - self.tic)
        if self.toc-self.tic > 20:
            self.resetSim()
            self.score = self.score - 50


    def epsilon_greedy(Q, epsilon, n_actions, s, train=False):
        """
        @param Q Q values state x action -> value
        @param epsilon for exploration
        @param s number of states
        @param train if true then no random actions selected
        """
        if train or np.random.rand() < epsilon:
            action = np.argmax(Q[s, :])
        else:
            action = np.random.randint(0, n_actions)
        return action

    def resetSim(self):
        self.depth = 0

        # rospy.wait_for_service('/gazebo/clear_body_wrenches')
        # try:
        #         self.flag=1
        #         self.command.twist.linear.x = 0
        #         # reset_proxy.call()
        #         self.clear_wrench('Bobby::loader')
        #         time.sleep(1)
        #         rospy.wait_for_service("/gazebo/pause_physics")
        #         try:
        #                 self.g_pause()
        #         except Exception as e:
        #                 rospy.logerr('Error on calling service: %s', str(e))
        #         # rospy.sleep(3)
        #         time.sleep(1)
        #         rospy.wait_for_service('/gazebo/reset_simulation')
        #         try:
        #                 # self.set_model_state(self.model_state)
        #                 # self.reset_world()
        #                 self.reset_simulation()
        #
        #         except Exception as e:
        #                 rospy.logerr('Error on calling service: %s', str(e))
        #         time.sleep(1)
        #         rospy.wait_for_service("/gazebo/unpause_physics")
        #         try:
        #                 self.g_unpause()
        #         except Exception as e:
        #                 rospy.logerr('Error on calling service: %s', str(e))
        #         self.command.twist.linear.x = round(random.uniform(0.06, 0.5), 3)
        #         time.sleep(2)
        #
        #         self.pub.publish(self.command)
        # except Exception as e:
        #             print("/gazebo/reset_simulation service call failed")

        self.gazebo.pauseSim()
        self.gazebo.clearBodyWrenches()
        self.gazebo.unpauseSim()
        self.gazebo.pauseSim()
        self.gazebo.resetSim()
        self.gazebo.unpauseSim()

        self.command.twist.linear.x = round(random.uniform(0.06, 0.5), 3)
        time.sleep(2)
        self.pub.publish(self.command)
        self.tic = time.time()
        self.depth = 0

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

    def get_hyd(self, data):  # subscriber to arm vel
        self.hyd = data.data

    def get_bucket_vel(self, data):  # subscriber to bucket vel
        self.bucket_vel = data.data

    def get_vel(self, msg):
        self.bobcat_vel = msg.twist.linear.x

    def get_soil(self, msg):
        self.soil = msg.data

    def qlearning(self):
        global score
        # np.random.seed(1)
        # print("Enter qlearning -------------------------------------------")
        # learning parameters
        gamma = 0.9    # discount factor
        alpha = 0.2    # learning rate
        epsilon = 0.92  # exploration probability (1-epsilon = exploit / epsilon = explore)
        # states
        pos_x_y = np.arange(6)  # num of range to divide
        vel_bob = np.arange(4)
        vel_hyd = np.arange(3)
        all_states = np.fliplr(np.array(np.meshgrid(vel_hyd, vel_hyd, vel_bob, pos_x_y, pos_x_y)).T.reshape(-1, 5))
        self.BobcatPos_x = self.getBobcatPos_x()
        self.BobcatPos_z = self.getBobcatPos_z()
        self.BobcatVel = self.getBobcatVel()
        self.getArm_Bucket_Vel()

        self.FieldState = np.array([self.BobcatPos_x, self.BobcatPos_z, self.BobcatVel, self.ArmVel, self.BucketVel])

        all_action = np.array([1, 2, 3, 4])  # set action array
        actionName = ["Bobcat x velocity", "Arm velocity", "Bobcat and Arm velocity", "Bucket velocity"]
        self.DoAction(all_action[0])
        #  initial Q matrix
        if os.path.isfile('Qmatrix.csv'):
            Q = np.array(list(csv.reader(open('Qmatrix.csv', "r"), delimiter=","))).astype("float")
        else:
            Q = np.zeros((len(all_states), len(all_action)))
        episodes = 100  # maximum number of frames

        current_state_inx = np.where(np.all(all_states == self.FieldState, axis=1))  # the initial state
        # the main loop of the algorithm
        if self.depth > 0.01:
            for episode in range(episodes):
            # if episode == 0:
            #     self.tic = time.time()
                print("Episode: ", episode)
                self.BobcatPos_x = self.getBobcatPos_x()
                self.BobcatPos_z = self.getBobcatPos_z()
                self.BobcatVel = self.getBobcatVel()
                self.getArm_Bucket_Vel()
                self.FieldState = np.array([self.BobcatPos_x, self.BobcatPos_z, self.BobcatVel, self.ArmVel, self.BucketVel])
                t = 0
                St = self.FieldState  # Get the current state
                current_state_inx = np.where(np.all(all_states == St, axis=1))
                r = np.random.rand()  # get 1 uniform random number
                x = sum(r >= np.cumsum([0, 1-epsilon, epsilon]))  # check it to be in which probability area

                #  availible action = [2,3,4,5,6];
                # choose either explore or exploit
                if x == 1:   # exploit
                    current_action = np.argmax(Q[current_state_inx, :])
                else:        # explore NEED TO ADD THAT IN THE BOT/TOP NOT POSSIBLE TO MOVE DOWN/UP
                    current_action = np.random.randint(0, all_action.all())
                # current_action = self.epsilon_greedy(Q, epsilon, all_action, current_state_inx)
                while t < 3:
                    # self.done = 1
                    t = t + 1
                    self.DoAction(current_action)
                    action_ind = np.where(all_action == current_action+1)  # id of the chosen action
            # UPDATE State to State_time_plus_1

                    next_state_ind = np.where(np.all(all_states == self.FieldState, axis=1))  # id of the next state
                    print("next state", next_state_ind)
                    print("Q in is", Q[next_state_ind, :])
            # a_ = np.argmax(Q[next_state_ind, :])
            # update the Q matrix using the Q-learning rule
            # Q(current_state_inx,action_idx) = Q(current_state_inx,action_idx) + alpha * (score + gamma* max(Q(next_state_idx,:)) - Q(current_state_inx,action_idx))
                    Q[current_state_inx, action_ind] += alpha * (self.score + gamma * np.argmax(Q[next_state_ind, :]) - Q[current_state_inx, action_ind])
        np.savetxt("Qmatrix.csv", Q.astype(int), delimiter=",")

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
        rospy.Subscriber('/bobcat/arm/loader', Float64, self.get_bucket_vel)
        self.pub = rospy.Publisher('/WPD/Speed', TwistStamped, queue_size=10)
        self.pub2 = rospy.Publisher('/bobcat/arm/hydraulics', Float64, queue_size=10)
        self.pub3 = rospy.Publisher('/bobcat/arm/loader', Float64, queue_size=10)

        self.gazebo = GazeboConnection()
        # inputs, targets = read_file("scores.csv")
        # check_soil(inputs, targets)
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

        # self.pub.publish(self.command)

        while not rospy.is_shutdown():

            # self.pub.publish(self.command)

            self.loader_pos = self.get_link_state('Bobby::loader', 'world').link_state.pose
            z_pile = self.m * (self.loader_pos.position.x + 0.96)  # 0.96 is the distance between center mass of the loader to the end
            H = z_pile - self.z_collision
            self.bobby_true_vel = self.get_model_state('Bobby', 'world').twist.linear
            # print("linear:", self.command.linear.x)
            # print("vel:", self.bobby_true_vel.x)
            # print("WPD- vel:", self.command.twist.linear.x)
            # while self.depth > 0.01:
            #     self.qlearning()
            # if self.bobby_true_vel.x < 0.01 and self.depth > 0.05 and self.loader_pos.position.x > -1.25:
            #      while self.depth>0.05:
            #         self.loader_pos = self.get_link_state('Bobby::loader', 'world').link_state.pose
            #         self.pub2.publish(0.2)
            #      self.pub2.publish(0)


            self.rate.sleep()

if __name__ == '__main__':
    try:

        main()
        rospy.spin()



    except rospy.ServiceException as e:
        print
        e
