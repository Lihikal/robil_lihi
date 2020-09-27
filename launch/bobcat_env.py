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
from geometry_msgs.msg import Pose, Wrench, Quaternion, Twist, TwistStamped
from gazebo_msgs.srv import ApplyBodyWrench, GetModelState, GetLinkState, BodyRequest, SetModelState
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64, String, Int32,Float32


import gym
from gym import spaces
import shlex
from psutil import Popen
from matplotlib import pyplot as plt

reg = register(
    id='MovingBobcat-v0',
    entry_point='bobcat_env:MovingBobcatEnv',
    max_episode_steps = 1600
)


class MovingBobcatEnv(gym.Env):
    STATE_SIZE = 2
    ACTION_SIZE = 3

    def __init__(self):

        self.number_actions = rospy.get_param('/moving_cube/n_actions')
        self.field_division = rospy.get_param("/moving_cube/field_division")
        self.seed()
        self.viewer = None

        self.action_space = spaces.Box(0.0, 0.5, (self.ACTION_SIZE,), dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, +np.inf, shape=(self.STATE_SIZE,), dtype=np.float32)

        # get configuration parameters
        self.init_roll_vel = rospy.get_param('/moving_cube/init_roll_vel')
        self.get_link_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)

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
        self.m = (2.1213) / (1.9213 + 0.2)  # the slope of the pile
        self.density = 1922  # density of material in kg/m^3
        self.z_collision = 0
        self.last_volume = 0
        self.volume_sum = 0
        self.last_z_pile = 0
        self.last_x_step = 0
        self.last_z_collision = 0
        self.tip_position = Point()
        self.last_reward = 0
        self.pitch=0
        self.counter=0
        self.c=0
        self.flag = 0
        self.max_volume = 850  # max bucket operation load for tool

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
        rospy.Subscriber('/Bobby/imu', Imu, self.get_angular_vel)
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

    def plot_x(self):
        rospy.wait_for_service('/gazebo/get_link_state')
        loader_pos = self.get_link_state('Bobby::loader', 'world').link_state.pose
        x = np.linspace(-0.2, 1.5, 100)
        y = self.m * x + 0.2
        plt.plot(x, y, color='goldenrod')
        if self.counter % 20 == 0:
            plt.plot(loader_pos.position.x + 0.96 * math.cos(self.pitch), loader_pos.position.z + 0.96 * math.sin(self.pitch),
                     'bo')
            plt.title("Bucket position")
            plt.ylabel("z axis")
            plt.xlabel("x axis")
            plt.axis([-1, 1.5, 0, 1.5])
            plt.axis('equal')
            plt.draw()
            plt.pause(0.00000000001)

        self.counter += 1

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        self.gazebo.unpauseSim()
        now = rospy.get_rostime()
        action[0] = 0.25 * action[0] + 0.25
        action[1] = 0.25 * action[1] + 0.25
        action[2] = 0.25 * action[2] + 0.25
        self.set_action(action)
        self.gazebo.pauseSim()
        obs = self._get_obs()
        done = self._is_done(obs, now)
        reward = self.compute_reward(obs, done, now)
        simplified_obs = self.convert_obs_to_state(obs)

        self.plot_x()  # plot graph of bucket tip position
        plt.ion()
        plt.show()
        return np.array(simplified_obs, dtype=np.float32), reward, done, {}

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
        obs = self._get_obs()
        now = rospy.get_rostime()
        self._is_done(obs, now)

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
        rospy.sleep(1)
        self.gazebo.pauseSim()
        self.init_env_variables()
        obs = self._get_obs()
        simplified_obs = self.convert_obs_to_state(obs)
        plt.clf()
        return simplified_obs

    def render(self, mode='human', close=False):
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

    def _is_done(self, observations, now):

        if self.max_volume-5 < self.volume_sum or now.secs > 15 or (self.depth < 0.001 and self.volume_sum > 5):  # term to restart the episode
            done = True
        else:
            done = False

        return done

    def set_action(self, action):
        'execute the action choosen in the gazebo environment'
        command = TwistStamped()
        command.twist.linear.x = action[0]
        self.pub.publish(command)
        self.pub2.publish(action[1])
        self.pub3.publish(action[2])

    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have acces to, we need to read the
        MyCubeSingleDiskEnv API DOCS
        :return:
        """

        bucket_observations = [self.loader_pos.position.x + 0.96 * math.cos(self.pitch),
                               self.loader_pos.position.z + 0.96 * math.sin(self.pitch)]
        return bucket_observations

    def get_orientation_euler(self):
        # We convert from quaternions to euler
        orientation_list = [self.odom.pose.pose.orientation.x,
                            self.odom.pose.pose.orientation.y,
                            self.odom.pose.pose.orientation.z,
                            self.odom.pose.pose.orientation.w]

        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        return roll, pitch, yaw

    def compute_reward(self, observations, done, now):

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
                reward += 0.3 * self.volume_sum
            if self.volume_sum == 0:
                reward = reward - 10 * (self.loader_pos.position.z + 0.96 * math.sin(self.pitch))
        else:  # The episode didn't success so we need to give a big negative reward
            reward = 0
            self.depth = 0
        return reward



    def joints_callback(self, data):
        self.joints = data

    def odom_callback(self, data):
        self.odom = data
        roll, pitch, yaw = self.get_orientation_euler()

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
        now = rospy.get_rostime()
        while not rospy.is_shutdown():
            self.command.twist.linear.x = 0
            self.pub.publish(self.command)
            while self.odom.pose.pose.position.z > 0.13:
                if now.secs > 45:
                    break
                self.pub2.publish(-0.5)
            while self.odom.pose.pose.position.z < 0.03:
                if now.secs > 45:
                    break
                self.pub2.publish(0.5)
            self.pub2.publish(0)

            roll, pitch, yaw = self.get_orientation_euler()
            while pitch > 0.0:
                if now.secs > 45:
                    break
                self.pub3.publish(0.3)
                roll, pitch, yaw = self.get_orientation_euler()
            while pitch < -0.05:
                if now.secs > 45:
                    break
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
            print("pitch:", self.pitch)
            print("z:", self.loader_pos.position.z)
            while self.pitch > -0.45:
                self.pub3.publish(0.3)
            self.pub3.publish(0)
            while self.loader_pos.position.z < 1.0:
                self.pub2.publish(0.2)
            self.pub2.publish(0)
            print("total volume:", self.volume_sum)
            self.plot_x()  # plot graph of bucket tip position
            plt.ion()
            plt.show()
            break

    def convert_obs_to_state(self, observations):
        """
        Converts the observations used for reward and so on to the essentials for the robot state
        In this case we only need the orientation of the cube and the speed of the disc.
        The distance doesnt condition at all the actions
        """
        BobcatPos_x = observations[0]
        BobcatPos_z = observations[1]

        state_converted = [BobcatPos_x, BobcatPos_z]

        return state_converted

    def get_depth(self, data):
        if (ContactsState.states != []):
            i = 0
            for i in range(len(data.states)):
                if ('box' in data.states[i].collision2_name) or (
                        'box' in data.states[i].collision1_name):  # check that the string exist in collision2_name/1
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
        volume = (big_trapezoid - small_trapezoid) * 1.612 * self.density  # 1.66 is the tool width
        if z_pile > 0 and z > 0 and z_pile > z:
            self.volume_sum = self.volume_sum + volume
        self.last_z_pile = z_pile
        self.last_x_step = x
        self.last_z_collision = self.z_collision
