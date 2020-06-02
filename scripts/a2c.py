#!/usr/bin/env python
import os
import math
import ptan
import time
import gym
import argparse
from tensorboardX import SummaryWriter

from lib import model, common, test_net, calc_logprob, build_parser

import torch
import torch.optim as optim
import torch.nn.functional as F
import numpy as np
import tensorflow as tf

GAMMA = 0.99
REWARD_STEPS = 5
BATCH_SIZE = 32
LEARNING_RATE_ACTOR = 1e-5
LEARNING_RATE_CRITIC = 1e-3
ENTROPY_BETA = 1e-3
ENVS_COUNT = 16

TEST_ITERS = 100000





def s_reshaper(s):
    return np.reshape(s, (1, 4))


class SolverA2C:
    def __init__(self, param_dict_critic, param_dict_actor):
        self.time_step = 0  # updated at the end of each time step - plotting critic loss in tensorboard
        self.most_recent_score = tf.Variable(0, name='most_recent_score')
        tf.summary.scalar('most_recent_score', self.most_recent_score)

        # Initialize actor and critic
        self.__init_actor(**param_dict_actor)
        self.__init_critic(**param_dict_critic)

        # Create custom individual summaries, merge summaries
        self.overall_summary = tf.summary.merge_all()

        # Initialize TensorFlow session and writer
        self.__init_session()
        self.summary_writer = tf.summary.FileWriter('path', self.session.graph)

    def __init_critic(self, gamma, HL_nodes_critic, adam_learning_rate_critic):
        """ Initialize critic. We model the critics value function by a neural network with one hidden layer
        :param gamma: discount rate
        :type gamma: float
        :param HL_nodes_critic: number of nodes in the (only) hidden layer
        :type HL_nodes_critic: int
        :param adam_learning_rate_critic: adam learning rate
        :type adam_learning_rate_critic: float
        """
        # Parameters
        self.gamma = gamma
        self.HL_nodes_critic = HL_nodes_critic
        self.adam_learning_rate_critic = adam_learning_rate_critic

        # Placeholders
        self.input_critic = tf.placeholder(tf.float32, [1, 4], name='input_critic')
        self.target_critic = tf.placeholder(tf.float32, [1, 1], name='target_critic')

        # Variables
        self.W1_critic = tf.Variable(tf.truncated_normal([4, self.HL_nodes_critic]))
        self.HL_critic = tf.nn.relu(tf.matmul(self.input_critic, self.W1_critic))
        self.W2_critic = tf.Variable(tf.truncated_normal([self.HL_nodes_critic, 1]))

        # Value function and loss function
        self.value_function = tf.matmul(self.HL_critic, self.W2_critic)
        self.loss_critic = tf.reduce_sum(tf.square(self.target_critic - self.value_function))

        # Optimization and Summary operations
        self.train_critic_ops = tf.train.AdamOptimizer(self.adam_learning_rate_critic).minimize(self.loss_critic)
        tf.summary.scalar('loss_critic', self.loss_critic)

    def __init_actor(self, HL_nodes_actor, adam_learning_rate_actor):
        """ Initialize actor. We model the policy by a neural network with one hidden layer
        :param HL_nodes_actor: number of nodes in the (only) hidden layer
        :type HL_nodes_actor: int
        :param adam_learning_rate_actor: adam learning rate
        :type adam_learning_rate_actor: float
        """
        # Parameters
        self.HL_nodes_actor = HL_nodes_actor
        self.adam_learning_rate_actor = adam_learning_rate_actor

        # Placeholders
        self.input_actor = tf.placeholder(tf.float32, [1, 4], name='input_actor')
        self.target_actor = tf.placeholder(tf.float32, [1, 2], name='target_actor')

        # Variables
        self.W1_actor = tf.Variable(tf.truncated_normal([4, self.HL_nodes_actor]))
        self.HL_actor = tf.nn.relu(tf.matmul(self.input_actor, self.W1_actor))
        self.W2_actor = tf.Variable(tf.truncated_normal([self.HL_nodes_actor, 2]))

        # Policy and loss function
        self.policy = tf.nn.softmax(tf.matmul(self.HL_actor, self.W2_actor))
        self.loss_actor = tf.reduce_sum(
            tf.nn.softmax_cross_entropy_with_logits(logits=tf.matmul(self.HL_actor, self.W2_actor),
                                                    labels=self.target_actor))

        # Optimization and Summary operations
        self.train_actor_ops = tf.train.AdamOptimizer(self.adam_learning_rate_actor).minimize(self.loss_actor)
        tf.summary.scalar('loss_actor', self.loss_actor)

    def choose_action(self, s):
        """ Given an observed state s, we perform an action based on actors policy network. Since the
        network outputs a 2-element probability vector we sample from this vector
        :param s: observed state
        :type s: ndarray
        :return: chosen action, 0 or 1
        :rtype: int
        """
        p = self.session.run(self.policy, feed_dict={self.input_actor: s}).ravel()
        return np.random.choice([0, 1], 1, p=p)[0]

    def train(self, s_old, a, r, s_new, done):
        """ Trains both the actors policy network and critics state value network.
        :param s_old: old state
        :type s_old: ndarray
        :param a: action we performed
        :type a: int
        :param r: reward we got
        :type r: int
        :param s_new: new state
        :type s_new: ndarray
        :param done: True if game over due to bad state (NOT TIME OUT)
        :type done: bool
        """
        # Initialize targets
        target_critic = np.zeros((1, 1))
        target_actor = np.zeros((1, 2))

        # Preliminary computations
        V_s_old = self.session.run(self.value_function, feed_dict={self.input_critic: s_old})
        V_s_new = self.session.run(self.value_function, feed_dict={self.input_critic: s_new})

        if done:
            # The value function of s_new must be zero because the state leads to game end
            target_critic[0][0] = r
            target_actor[0][a] = r - V_s_old
        else:
            target_critic[0][0] = r + self.gamma * V_s_new
            target_actor[0][a] = r + self.gamma * V_s_new - V_s_old

        # Train and Write summaries
        _, _, new_summary_string = self.session.run([self.train_critic_ops, self.train_actor_ops, self.overall_summary],
                                                    feed_dict={self.input_critic: s_old,
                                                               self.target_critic: target_critic,
                                                               self.input_actor: s_old,
                                                               self.target_actor: target_actor})
        self.summary_writer.add_summary(new_summary_string, self.time_step)

        # Increment timestep
        self.time_step += 1

    def feed_most_recent_score(self, score):
        """ Feeds the most recent score into our solver class so that we can visualize it in tensorboard
        :param score: most recent score
        :type score: int
        """
        op = self.most_recent_score.assign(score)
        self.session.run(op)

    def __init_session(self):
        self.session = tf.InteractiveSession()
        self.session.run(tf.global_variables_initializer())