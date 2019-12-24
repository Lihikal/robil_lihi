#!/usr/bin/env python
import sys
sys.path.append('/home/robil/catkin_ws/gym')
import gym
import numpy
import time
import functools
import qlearn
from gym import wrappers

# ROS packages required
import rospy
import rospkg
from gazebo_msgs.msg import ContactsState, ModelState
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Wrench, Quaternion,Twist, TwistStamped
import numpy as np
import random
from gazebo_msgs.srv import ApplyBodyWrench, GetModelState, GetLinkState, BodyRequest, SetModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry

# import our training environment
import bobcat_env


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

def writefile(fname, s):
        with open(path.join(outdir, fname), 'w') as fh: fh.write(s)


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

    last_time_steps = numpy.ndarray(0)

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


    # Initialises the algorithm that we are going to use for learning
    qlearn = qlearn.QLearn(actions=range(env.action_space.n),
                    alpha=Alpha, gamma=Gamma, epsilon=Epsilon, env=env)
    initial_epsilon = qlearn.epsilon

    start_time = time.time()
    highest_reward = 0

    # Starts the main training loop: the one about the episodes to do
    for x in range(nepisodes):
        rospy.logwarn("############### START EPISODE=>" + str(x))

        cumulated_reward = 0
        done = False
        if qlearn.epsilon > 0.05:
            qlearn.epsilon *= epsilon_discount

        # Initialize the environment and get first state of the robot
        observation = env.reset()
        state = ''.join(map(str, observation))


        episode_time = rospy.get_rostime().to_sec()
        # for each episode, we test the robot for nsteps

        for i in range(nsteps):
            rospy.loginfo("############### Start Step=>"+str(i))
        # Pick an action based on the current state
            action = qlearn.chooseAction(state)
            rospy.loginfo("Next action is:%d", action)
        # Execute the action in the environment and get feedback
            observation, reward, done, info = env.step(action)  # do the action


            rospy.loginfo(str(observation) + " " + str(reward))
            cumulated_reward += reward
            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward

            nextState = ''.join(map(str, observation))

        # Make the algorithm learn based on the results
            rospy.logwarn("############### state we were=>" + str(state))
            rospy.logwarn("############### action that we took=>" + str(action))
            rospy.logwarn("############### reward that action gave=>" + str(reward))
            rospy.logwarn("############### State in which we will start next step=>" + str(nextState))
            qlearn.learn(state, action, reward, nextState)

            if not(done):
                state = nextState
            else:
                rospy.loginfo("DONE")
                last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
                break
            rospy.loginfo("############### END Step=>" + str(i))
        #raw_input("Next Step...PRESS KEY")
        #rospy.sleep(2.0)
        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        rospy.logwarn ( ("EP: "+str(x+1)+" - [alpha: "+str(round(qlearn.alpha,2))+" - gamma: "+str(round(qlearn.gamma,2))+" - epsilon: "+str(round(qlearn.epsilon,2))+"] - Reward: "+str(cumulated_reward)+"     Time: %d:%02d:%02d" % (h, m, s)))

    rospy.loginfo ( ("\n|"+str(nepisodes)+"|"+str(qlearn.alpha)+"|"+str(qlearn.gamma)+"|"+str(initial_epsilon)+"*"+str(epsilon_discount)+"|"+str(highest_reward)+"| PICTURE |"))

    # l = last_time_steps.tolist()
    # l.sort()

    np.save(outdir+'/data.npy', qlearn.q)
    # writefile('info.json', json.dumps(str(qlearn.q)))
    #print("Parameters: a="+str)
    # rospy.loginfo("Overall score: {:0.2f}".format(last_time_steps.mean()))
    # rospy.loginfo("Best 100 score: {:0.2f}".format(functools.reduce(lambda x, y: x + y, l[-5:]) / len(l[-5:])))

    env.close()