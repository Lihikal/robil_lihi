#!/usr/bin/env python
import numpy as np
import tensorflow as tf
import os
import datetime

import sys
sys.path.append('/home/robil/catkin_ws/gym')
import gym
from gym import wrappers
import bobcat_env
import rospy
import rospkg
from gazebo_msgs.msg import ContactsState, ModelState
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Wrench, Quaternion,Twist, TwistStamped
from gazebo_msgs.srv import ApplyBodyWrench, GetModelState, GetLinkState, BodyRequest, SetModelState
from nav_msgs.msg import Odometry



class MyModel(tf.keras.Model):
    def __init__(self, num_states, hidden_units, num_actions):
        super(MyModel, self).__init__()
        self.input_layer = tf.keras.layers.InputLayer(input_shape=(num_states,))
        self.hidden_layers = []
        for i in hidden_units:
            self.hidden_layers.append(tf.keras.layers.Dense(
                i, activation='tanh', kernel_initializer='RandomNormal'))
        self.output_layer = tf.keras.layers.Dense(
            num_actions, activation='linear', kernel_initializer='RandomNormal')

    @tf.function
    def call(self, inputs):
        z = self.input_layer(inputs)
        for layer in self.hidden_layers:
            z = layer(z)
        output = self.output_layer(z)
        return output


class DQN:
    def __init__(self, num_states, num_actions, hidden_units, gamma, max_experiences, min_experiences, batch_size, lr):
        self.num_actions = num_actions
        self.batch_size = batch_size
        self.optimizer = tf.optimizers.Adam(lr)
        self.gamma = gamma
        self.model = MyModel(num_states, hidden_units, num_actions)
        self.experience = {'s': [], 'a': [], 'r': [], 's2': [], 'done': []}
        self.max_experiences = max_experiences
        self.min_experiences = min_experiences

    def predict(self, inputs):
        return self.model(np.atleast_2d(inputs.astype('float32')))

    @tf.function
    def train(self, TargetNet):
        if len(self.experience['s']) < self.min_experiences:
            return 0
        ids = np.random.randint(low=0, high=len(self.experience['s']), size=self.batch_size)
        states = np.asarray([self.experience['s'][i] for i in ids])
        actions = np.asarray([self.experience['a'][i] for i in ids])
        rewards = np.asarray([self.experience['r'][i] for i in ids])
        states_next = np.asarray([self.experience['s2'][i] for i in ids])
        dones = np.asarray([self.experience['done'][i] for i in ids])
        value_next = np.max(TargetNet.predict(states_next), axis=1)
        actual_values = np.where(dones, rewards, rewards+self.gamma*value_next)

        with tf.GradientTape() as tape:
            selected_action_values = tf.math.reduce_sum(
                self.predict(states) * tf.one_hot(actions, self.num_actions), axis=1)
            loss = tf.math.reduce_sum(tf.square(actual_values - selected_action_values))
        variables = self.model.trainable_variables
        gradients = tape.gradient(loss, variables)
        self.optimizer.apply_gradients(zip(gradients, variables))


    def get_action(self, states, epsilon):
        if np.random.random() < epsilon:
            return np.random.choice(self.num_actions)
        else:
            return np.argmax(self.predict(np.atleast_2d(states))[0])

    def add_experience(self, exp):
        if len(self.experience['s']) >= self.max_experiences:
            for key in self.experience.keys():
                self.experience[key].pop(0)
        for key, value in exp.items():
            self.experience[key].append(value)

    def copy_weights(self, TrainNet):
        variables1 = self.model.trainable_variables
        variables2 = TrainNet.model.trainable_variables
        for v1, v2 in zip(variables1, variables2):
            v1.assign(v2.numpy())


def play_game(env, TrainNet, TargetNet, epsilon, copy_step):
    rewards = 0
    iter = 0
    done = False
    observations = env.reset()
    while not done:
        action = TrainNet.get_action(observations, epsilon)
        prev_observations = observations
        observations, reward, done, _ = env.step(action)
        rewards += reward
        if done:
            # reward = 0
            env.reset()

        exp = {'s': prev_observations, 'a': action, 'r': reward, 's2': observations, 'done': done}
        TrainNet.add_experience(exp)
        TrainNet.train(TargetNet)
        iter += 1
        if iter % copy_step == 0:
            TargetNet.copy_weights(TrainNet)
    return rewards


def make_video(env, TrainNet):
    env = wrappers.Monitor(env, os.path.join(os.getcwd(), "videos"), force=True)
    rewards = 0
    steps = 0
    done = False
    observation = env.reset()
    while not done:
        env.render()
        action = TrainNet.get_action(observation, 0)
        observation, reward, done, _ = env.step(action)
        steps += 1
        rewards += reward
    print("Testing steps: {} rewards {}: ".format(steps, rewards))

def get_depth(data):
    global depth
    if (ContactsState.states != []):
            i=0
            for i in range(len(data.states)):
                 if ('box' in data.states[i].collision2_name) or ('box' in data.states[i].collision1_name):  # check that the string exist in collision2_name/1
                           depth = np.mean(data.states[i].depths)
                           z_collision = np.mean(data.states[i].contact_positions[0].z)


def odom_callback(data):
    global odom
    odom = data


def main():
    env = gym.make('MovingBobcat-v0')
    gamma = 0.99
    copy_step = 25
    num_states = env.observation_space.n
    num_actions = env.action_space.n
    hidden_units = [200, 200]
    max_experiences = 10000
    min_experiences = 100
    batch_size = 32
    lr = 0.01
    current_time = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    log_dir = 'logs/dqn/' + current_time
    summary_writer = tf.summary.create_file_writer(log_dir)
    print("traininggggggg")
    TrainNet = DQN(num_states, num_actions, hidden_units, gamma, max_experiences, min_experiences, batch_size, lr)
    TargetNet = DQN(num_states, num_actions, hidden_units, gamma, max_experiences, min_experiences, batch_size, lr)
    N = 20000
    total_rewards = np.empty(N)
    epsilon = 0.99
    decay = 0.9999
    min_epsilon = 0.1
    for n in range(N):
        epsilon = max(min_epsilon, epsilon * decay)
        total_reward = play_game(env, TrainNet, TargetNet, epsilon, copy_step)
        total_rewards[n] = total_reward
        avg_rewards = total_rewards[max(0, n - 100):(n + 1)].mean()
        with summary_writer.as_default():
            tf.summary.scalar('episode reward', total_reward, step=n)
            tf.summary.scalar('running avg reward(100)', avg_rewards, step=n)
        if n % 100 == 0:
            print("episode:", n, "episode reward:", total_reward, "eps:", epsilon, "avg reward (last 100):", avg_rewards)
        print("episode reward:", total_reward)
    print("avg reward for last 100 episodes:", avg_rewards)
    make_video(env, TrainNet)
    env.close()


if __name__ == '__main__':
    rospy.init_node('movingbobcat_gym', anonymous=True, log_level=rospy.WARN)
    global depth, command, odom, pub2
    command = TwistStamped()
    depth = 0
    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    rospy.wait_for_service('/gazebo/get_link_state')
    get_link_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
    pub = rospy.Publisher('/WPD/Speed', TwistStamped, queue_size=10)
    pub2 = rospy.Publisher('/bobcat/arm/hydraulics', Float64, queue_size=10)
    pub3 = rospy.Publisher('/bobcat/arm/loader', Float64, queue_size=10)
    rospy.Subscriber('/robot_bumper', ContactsState, get_depth)
    rospy.Subscriber("/Bobby/odom", Odometry, odom_callback)

    # Create the Gym environment
    env = gym.make('MovingBobcat-v0')
    print("Start------------------------------------")
    rospy.loginfo("Gym environment done")

    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('robil_lihi')

    outdir = pkg_path + '/training_results'
    env = wrappers.Monitor(env, outdir, force=True)
    rospy.loginfo("Monitor Wrapper started")

    last_time_steps = np.ndarray(0)

    # Loads parameters from the ROS param server
    # Parameters are stored in a yaml file inside the config directory
    # They are loaded at runtime by the launch file
    Alpha = rospy.get_param("/moving_cube/alpha")
    Epsilon = rospy.get_param("/moving_cube/epsilon")
    Gamma = rospy.get_param("/moving_cube/gamma")
    epsilon_discount = rospy.get_param("/moving_cube/epsilon_discount")
    nepisodes = rospy.get_param("/moving_cube/nepisodes")
    nsteps = rospy.get_param("/moving_cube/nsteps")

    running_step = rospy.get_param("/moving_cube/running_step")

    main()

