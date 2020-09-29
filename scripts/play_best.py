#!/usr/bin/env python
import argparse
from gym import wrappers
from lib import model, kfac, make_env
from PIL import Image
import os
import numpy as np
import torch
import bobcat_env
import rospy
from matplotlib import pyplot as plt
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.srv import ApplyBodyWrench, GetModelState, GetLinkState
from sensor_msgs.msg import Imu



ENV_ID = "MovingBobcat-v0"
NHID = 64


def get_angular_vel(msg):
    global pitch
    orientation_q = msg.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

def plot_x():
    global counter
    rospy.wait_for_service('/gazebo/get_link_state')
    loader_pos = get_link_state('Bobby::loader', 'world').link_state.pose
    x = np.linspace(-0.2, 1.5, 100)
    y = 1 * x + 0.2
    plt.plot(x, y, color='goldenrod')

    if counter % 20 == 0:
        plt.plot(loader_pos.position.x + 0.96 * math.cos(pitch), loader_pos.position.z + 0.96 * math.sin(pitch), 'bo')
        plt.ylabel("z position")
        plt.xlabel("x position")
        plt.axis([-1, 1.5, 0, 1.5])
        plt.axis('equal')
        plt.draw()
        plt.pause(0.00000000001)

    counter += 1


if __name__ == "__main__":
    rospy.init_node('movingbobcat_play_best', anonymous=True, log_level=rospy.WARN)
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--model", required=True, help="Model file to load")
    parser.add_argument("-e", "--env", default=ENV_ID, help="Environment name to use, default=" + ENV_ID)
    parser.add_argument("-d", "--datafile", required=False, help="Name of data file to load")
    parser.add_argument("--hid", default=NHID, type=int, help="Hidden units, default=" + str(NHID))
    parser.add_argument("-r", "--record", help="If specified, sets the recording dir, default=Disabled")
    parser.add_argument("-s", "--save", type=int, help="If specified, save every N-th step as an image")
    parser.add_argument("--acktr", default=False, action='store_true', help="Enable Acktr-specific tweaks")
    args = parser.parse_args()
    get_link_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
    pitch = 0
    rospy.Subscriber('/Bobby/imu', Imu, get_angular_vel)

    counter = 0
    env = make_env(args)
    if args.record:
        env = wrappers.Monitor(env, args.record)

    net = model.ModelActor(env.observation_space.shape[0], env.action_space.shape[0], args.hid)
    if args.acktr:
        opt = kfac.KFACOptimizer(net)
    net.load_state_dict(torch.load(args.model))

    obs = env.reset()
    total_reward = 0.0
    total_steps = 0

    while True:
        obs_v = torch.FloatTensor(obs)
        mu_v = net(obs_v)
        action = mu_v.squeeze(dim=0).data.numpy()
        action = np.clip(action, -1, 1)
        if np.isscalar(action): 
            action = [action]
        obs, reward, done, _ = env.step(action)
        total_reward += reward
        total_steps += 1
        rospy.Subscriber('/Bobby/imu', Imu, get_angular_vel)
        # plot_x()
        # plt.ion()
        # plt.show()
        if done:
            break
        if args.save is not None and total_steps % args.save == 0:
            o = env.render('rgb_array')
            img = Image.fromarray(o.values)
            if not os.path.exists('images'):
                os.mkdir('images')
            img.save("images/img_%05d.png" % total_steps)



    print("In %d steps we got %.3f reward" % (total_steps, total_reward))


    env.go_up()
    env.close()
