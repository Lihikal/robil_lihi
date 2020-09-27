#!/usr/bin/env python
import rospy
import numpy as np
import math
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from gazebo_connection import GazeboConnection
from gym.utils import seeding
from gym.envs.registration import register
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ContactsState, ModelState, LinkStates
from geometry_msgs.msg import Pose, Wrench, Quaternion,Twist, TwistStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64, String, Int32, Float32
import gym
from gym import spaces
import random
import shlex
from psutil import Popen

class CustomEnv(gym.Env):
  """Custom Environment that follows gym interface"""
  metadata = {'render.modes': ['human']}
reg = register(
    id='MovingBobcat-v0',
    entry_point='bobcat_env:MovingBobcatEnv',
    max_episode_steps=2000
    )

class MovingBobcatEnv(gym.Env):

    def __init__(self):

        self.number_actions = rospy.get_param('/moving_cube/n_actions')
        self.field_division = rospy.get_param("/moving_cube/field_division")
        self.action_space = spaces.Discrete(self.number_actions)
        self.observation_space = spaces.Discrete((self.field_division+1)*(self.field_division+1)*3*3*4)
        self._seed()

        #get configuration parameters
        self.init_roll_vel = rospy.get_param('/moving_cube/init_roll_vel')
        # Actions
        self.roll_speed_fixed_value = rospy.get_param('/moving_cube/roll_speed_fixed_value')
        self.roll_speed_increment_value = rospy.get_param('/moving_cube/roll_speed_increment_value')
        self.start_point = Point()
        self.start_point.x = rospy.get_param("/moving_cube/init_cube_pose/x")
        self.start_point.y = rospy.get_param("/moving_cube/init_cube_pose/y")
        self.start_point.z = rospy.get_param("/moving_cube/init_cube_pose/z")
        self.hyd = 0
        self.bucket_vel = 0
        self.depth = 0
        self.m = (2.1213)/(1.9213 + 0.2)  # the slope of the pile
        self.density = 1922  # density of material in kg/m^3
        self.z_collision = 0
        self.last_volume = 0
        self.volume_sum = 0
        self.last_z_pile = 0
        self.last_x_step = 0
        self.last_z_collision = 0
        self.tip_position = Point()
        self.last_reward = 0
        self.pitch = 0
        self.flag = 0
        self.max_volume = 860  # max bucket operation load for tool

        # Done
        self.max_pitch_angle = rospy.get_param('/moving_cube/max_pitch_angle')
        # Rewards
        self.move_distance_reward_weight = rospy.get_param("/moving_cube/move_distance_reward_weight")
        self.y_linear_speed_reward_weight = rospy.get_param("/moving_cube/y_linear_speed_reward_weight")
        self.y_axis_angle_reward_weight = rospy.get_param("/moving_cube/y_axis_angle_reward_weight")
        self.end_episode_points = rospy.get_param("/moving_cube/end_episode_points")

        # stablishes connection with simulator
        self.gazebo = GazeboConnection()
        self.gazebo.unpauseSim()
        self.check_all_sensors_ready()
        self.pub_bucket_tip = rospy.Publisher('/bobcat/tip_position', Point, queue_size=10)
        rospy.Subscriber('/bobcat/arm/hydraulics', Float64, self.get_hyd)
        rospy.Subscriber('/WPD/Speed', TwistStamped, self.get_vel)
        rospy.Subscriber('/bobcat/arm/loader', Float64, self.get_bucket_vel)
        rospy.Subscriber("/Bobby/joint_states", JointState, self.joints_callback)
        rospy.Subscriber("/Bobby/odom", Odometry, self.odom_callback)
        rospy.Subscriber('/robot_bumper', ContactsState, self.get_depth)
        # rospy.Subscriber('/bobcat/tip_position', Point, self.get_tip_position)
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.cb_link_states)
        self.pub = rospy.Publisher('/WPD/Speed', TwistStamped, queue_size=10)
        self.pub2 = rospy.Publisher('/bobcat/arm/hydraulics', Float64, queue_size=10)
        self.pub3 = rospy.Publisher('/bobcat/arm/loader', Float64, queue_size=10)
        self.pub4 = rospy.Publisher('/volume', Float32, queue_size=10)
        self.pub5 = rospy.Publisher('/penetration_z', Float32, queue_size=10)

        self.gazebo.pauseSim()
        self.command = TwistStamped()
        self.node_process = Popen(shlex.split('rosrun robil_lihi C_response_loader.py'))
        self.node_process.terminate()

    def cb_link_states(self, msg):
        self.link_states = msg
        self.loader_index = self.link_states.name.index("Bobby::loader")
        self.loader_pos = self.link_states.pose[self.loader_index]

    def get_angular_vel(self, msg):
        self.angular_vel = msg.angular_velocity.y
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)

    def _seed(self, seed=None): #overriden function
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):#overriden function

        self.gazebo.unpauseSim()
        self.set_action(action)
        self.gazebo.pauseSim()
        obs = self._get_obs()
        done = self._is_done(obs)
        info = {}
        reward = self.compute_reward(obs, done)
        simplified_obs = self.convert_obs_to_state(obs)
        return simplified_obs, reward, done, info

    def reset(self):

        self.gazebo.unpauseSim()


        self.check_all_sensors_ready()
        self.z_collision = 0
        self.depth = 0
        self.command.twist.linear.x = 0
        self.pub.publish(self.command)
        self.pub2.publish(0)
        self.pub3.publish(0)
        self.node_process.terminate()
        self.set_init_pose()

        self.gazebo.pauseSim()
        self.gazebo.resetSim()
        self.node_process = Popen(shlex.split('rosrun robil_lihi C_response_loader.py'))
        self.gazebo.unpauseSim()
        self.command.twist.linear.x = 0
        self.pub.publish(self.command)
        self.pub2.publish(0)
        self.pub3.publish(0)
        self.set_init_pose()
        self.init_env_variables()
        self.check_all_sensors_ready()
        self.gazebo.pauseSim()
        self.init_env_variables()
        obs = self._get_obs()
        simplified_obs = self.convert_obs_to_state(obs)
        return simplified_obs
    def _render(self, mode='human', close=False):
            pass
    def init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        self.depth = 0.0
        self.last_volume = 0.0
        self.total_distance_moved = 0.0
        self.volume_sum = 0
        self.last_z_pile = 0
        self.last_x_step = 0
        self.last_z_collision = 0
        self.z_collision = 0
        self.last_reward = 0
        self.pitch = 0
        self.flag = 0

    def _is_done(self, observations):



        # if self.volume_sum > 87 or (self.depth < 0.01 and self.volume_sum > 5):

        now = rospy.get_rostime()
        if self.max_volume-5 < self.volume_sum or now.secs > 40:  # term to restart the episode
            done = True
        else:
            done = False

        return done
    def set_action(self, action):
        command = TwistStamped()
        if action == 0:  # action bobcat velocity
            command.twist.linear.x = round(random.uniform(0.1, 0.5), 3)
            self.pub.publish(command)
            self.pub2.publish(0)
            self.pub3.publish(0)
        elif action == 1:  # action bobcat & arm velocity
            r = round(random.uniform(0.1, 0.5), 3)
            command.twist.linear.x = round(random.uniform(0.1, 0.5), 3)
            self.pub.publish(command)
            self.pub2.publish(r)
            self.pub3.publish(0)
        elif action == 2:  # bucket velocity
            r = np.random.uniform(0.1, 0.5)
            command.twist.linear.x = 0
            self.pub.publish(command)
            self.pub2.publish(0)
            self.pub3.publish(r)
        elif action == 3:  # action arm velocity
            r = round(random.uniform(0.1, 0.5), 3)
            command.twist.linear.x = 0
            self.pub.publish(command)
            self.pub2.publish(r)
            self.pub3.publish(0)
    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have acces to, we need to read the
        MyCubeSingleDiskEnv API DOCS
        :return:
        """
        # We get the orientation of the bucket in RPY
        roll, pitch, yaw = self.get_orientation_euler()
        BobcatPos_x = math.floor((self.loader_pos.position.x + 0.96 * math.cos(self.pitch)+1.495)/(-0.1311+1.495)*self.field_division)
        if BobcatPos_x > self.field_division:
            BobcatPos_x = float(self.field_division)
        elif BobcatPos_x < 0:
            BobcatPos_x = 0.0
        BobcatPos_z = math.floor((self.loader_pos.position.z + 0.96 * math.sin(self.pitch)-0.0924)/(0.4037-0.0924)*self.field_division)
        if BobcatPos_z > self.field_division:
            BobcatPos_z = float(self.field_division)
        elif BobcatPos_z < 0:
            BobcatPos_z = 0.0
        # We get the velocity of the bucket
        BobcatVel = math.floor((self.odom.twist.twist.linear.x-0)/(0.5-0)*3)
        if BobcatVel > 3:
            BobcatVel = 3
        if BobcatVel < 0:
            BobcatVel = 0
        ArmVel = math.floor((float(self.hyd))/(0.5-0.0)*2)
        if ArmVel > 2:
            ArmVel = 2
        if ArmVel < 0:
            ArmVel = 0
        BucketVel = math.floor((float(self.bucket_vel))/(0.5-0.0)*2)
        if BucketVel > 2:
            BucketVel = 2
        if BucketVel < 0:
            BucketVel = 0
        bucket_observations = [BobcatPos_x, BobcatPos_z, BobcatVel, ArmVel, BucketVel]
        return bucket_observations
    def get_orientation_euler(self):
        # We convert from quaternions to euler
        orientation_list = [self.odom.pose.pose.orientation.x,
                            self.odom.pose.pose.orientation.y,
                            self.odom.pose.pose.orientation.z,
                            self.odom.pose.pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        return roll, pitch, yaw
    def get_roll_velocity(self):
        # We get the current joint roll velocity
        roll_vel = self.joints.velocity[0]
        return roll_vel
    def get_y_linear_speed(self):
        # We get the current joint roll velocity
        y_linear_speed = self.odom.twist.twist.linear.y
        return y_linear_speed
    def get_y_dir_distance_from_start_point(self, start_point):
        """
        Calculates the distance from the given point and the current position
        given by odometry. In this case the increase or decrease in y.
        :param start_point:
        :return:
        """
        y_dist_dir = self.odom.pose.pose.position.y - start_point.y
        return y_dist_dir
    def compute_reward(self, observations, done):
        if not done:
            reward = -10
            self.calc_volume()
            self.pub4.publish(self.volume_sum)
            if self.volume_sum > 0 and self.depth <= 0.01 and self.flag == 0:
                self.flag = 1
                self.pub5.publish(self.loader_pos.position.z + 0.96 * math.sin(self.pitch))
            if self.volume_sum > self.max_volume:  # failed to take the bucket out, too many soil
                reward -= self.volume_sum
                rospy.logwarn("############### Fail=>" + str(self.volume_sum))
            elif self.volume_sum > 0.01:
                reward += 0.1 * self.volume_sum
            reward = reward - 20 * (self.loader_pos.position.z + 0.96 * math.sin(self.pitch))
        else:  # The episode didn't success so we need to give a big negative reward

            reward = 0
            self.depth = 0
        return reward


    def joints_callback(self, data):
        self.joints = data
    def odom_callback(self, data):
        self.odom = data
        roll, pitch, yaw = self.get_orientation_euler()
        self.tip_position.x = self.odom.pose.pose.position.x + 0.96 * math.cos(pitch)
        self.tip_position.y = self.odom.pose.pose.position.y
        self.tip_position.z = self.odom.pose.pose.position.z + 0.96 * math.sin(pitch)
        self.pub_bucket_tip.publish(self.tip_position)
    def check_all_sensors_ready(self):
        self.check_joint_states_ready()
        self.check_odom_ready()
        rospy.logdebug("ALL SENSORS READY")
    def get_hyd(self, data):  # subscriber for arm vel
        self.hyd = data.data
    def get_bucket_vel(self, data):  # subscriber for bucket vel
        self.bucket_vel = data.data
    def get_vel(self, msg):  # subscriber for bobcat velocity
        self.bobcat_vel = msg.twist.linear.x
    def check_joint_states_ready(self):
        self.joints = None
        while self.joints is None and not rospy.is_shutdown():
            try:
                self.joints = rospy.wait_for_message("/Bobby/joint_states", JointState, timeout=1.0)
                rospy.logdebug("Current Bobby/joint_states READY=>" + str(self.joints))
            except:
                rospy.logerr("Current Bobby/joint_states not ready yet, retrying for getting Bobby")
        return self.joints
    def check_odom_ready(self):
        self.odom = None
        while self.odom is None and not rospy.is_shutdown():
            try:
                self.odom = rospy.wait_for_message("/Bobby/odom", Odometry, timeout=1.0)
                rospy.logdebug("Current /Bobby/odom READY=>" + str(self.odom))
            except:
                rospy.logerr("Current /Bobby/odom not ready yet, retrying for getting odom")
        return self.odom


    def set_init_pose(self):
        """Sets the Robot in its init pose
        """
        while not rospy.is_shutdown():
            self.command.twist.linear.x = 0
            self.pub.publish(self.command)
            while self.odom.pose.pose.position.z > 0.13:
                self.pub2.publish(-0.5)
            while self.odom.pose.pose.position.z < 0.03:
                self.pub2.publish(0.5)
            self.pub2.publish(0)
            roll, pitch, yaw = self.get_orientation_euler()
            while pitch > 0.0:
                self.pub3.publish(0.3)
                roll, pitch, yaw = self.get_orientation_euler()
            while pitch < -0.05:
                self.pub3.publish(-0.3)
                roll, pitch, yaw = self.get_orientation_euler()
            self.pub3.publish(0)
            break

    def go_up(self):
        """get the bucket out of soil
        """
        self.gazebo.unpauseSim()  # must in order to get the bucket up
        self.check_all_sensors_ready()
        self.command.twist.linear.x = 0
        while not rospy.is_shutdown():
            self.command.twist.linear.x = 0
            self.pub.publish(self.command)
            self.pub2.publish(0)
            self.pub3.publish(0)
            roll, pitch, yaw = self.get_orientation_euler()
            print("pitch:", pitch)
            print("z:", self.odom.pose.pose.position.z)
            while pitch > -0.45:
                self.pub3.publish(0.3)
                roll, pitch, yaw = self.get_orientation_euler()
            self.pub3.publish(0)
            while self.odom.pose.pose.position.z < 1.3:
                self.pub2.publish(0.2)
            self.pub2.publish(0)
            print("total volume:", self.volume_sum)
            break
    def convert_obs_to_state(self, observations):
        """
        Converts the observations used for reward and so on to the essentials for the robot state
        In this case we only need the orientation of the cube and the speed of the disc.
        The distance doesnt condition at all the actions
        """
        BobcatPos_x = observations[0]
        BobcatPos_z = observations[1]
        BobcatVel = observations[2]
        ArmVel = observations[3]
        BucketVel = observations[4]
        state_converted = [BobcatPos_x, BobcatPos_z, BobcatVel, ArmVel, BucketVel]
        return state_converted
    def get_depth(self, data):
        if (ContactsState.states != []):
            i = 0
            for i in range(len(data.states)):
                 if ('box' in data.states[i].collision2_name) or ('box' in data.states[i].collision1_name):  # check that the string exist in collision2_name/1
                           self.depth = np.mean(data.states[i].depths)
                           self.z_collision = np.mean(data.states[i].contact_positions[0].z)
    def calc_volume(self):
        z_pile = self.m * (self.loader_pos.position.x + 0.96 * math.cos(
            self.pitch) + 0.2)  # 0.96 is the distance between center mass of the loader to the end
        H = z_pile - self.z_collision
        x = self.loader_pos.position.x + 0.96 * math.cos(self.pitch)
        z = self.z_collision
        big_trapezoid = (x-self.last_x_step)*(z_pile+self.last_z_pile)/2
        small_trapezoid = (x-self.last_x_step)*(self.z_collision+self.last_z_collision)/2
        volume = (big_trapezoid - small_trapezoid) * 1.612 * self.density  # 1.612 is the tool width [m]
        if z_pile > 0 and z > 0 and z_pile > z:
            self.volume_sum = self.volume_sum + volume
        self.last_z_pile = z_pile
        self.last_x_step = x
        self.last_z_collision = self.z_collision
        return self.volume_sum
