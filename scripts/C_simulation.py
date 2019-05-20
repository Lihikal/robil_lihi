#!/usr/bin/env python
import math
import tf

import rospy
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *
from geometry_msgs.msg import *
from os.path import expanduser


class Main:
    """
    This node controls the simulation with gazebo-ros services
    It's main function is to spawn and delete sand models in order
    to visualize the bobcat's actions
    """
    home = expanduser('~')
    sand_big_sdf = home + '/.gazebo/models/sand_big/model.sdf'
    sand_pile_small_sdf = home + '/.gazebo/models/sand_pile_small/model.sdf'
    sand_pile_big_sdf = home + '/.gazebo/models/sand_pile_big/model.sdf'
    model_name = 'Bobby'

    def __init__(self):
        rospy.init_node('SimulationControlNode')

        rospy.Subscriber("/gazebo/link_states", LinkStates, self.link_states_callback)
        self.loaded = 0
        self.curr_pile_scale = 2

        # wait until services are available to avoid errors
        rospy.wait_for_service("/gazebo/set_model_state")
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        rospy.wait_for_service("/gazebo/delete_model")

        self.spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.get_world_properties_service = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        self.delete_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

        rospy.loginfo("[SimulationControlNode]: started")
        rospy.spin()

    def link_states_callback(self, data):
        """callback for /gazebo/link_states Subscriber
        * load sand from start pile if loader is in sand pile
        * unload sand to end pile if self.loaded and bobby is in the right position
        Args:
            data (LinkStates): state of all links in the simulation
        """
        # get the loader index so we could get it's pose
        loader_idx = 4
        for i, name in enumerate(data.name):
            if name == 'Bobby::loader':
                loader_idx = i
                break

        x = data.pose[loader_idx].position.x
        y = data.pose[loader_idx].position.y
        z = data.pose[loader_idx].position.z
        orientation = data.pose[loader_idx].orientation

        (roll, pitch, theta) = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])

        # distances from both piles
        dist_start = math.hypot(x - 6, y)
        dist_end = math.hypot(x - (-6), y)

        # load sand
        if self.curr_pile_scale != 0 and z < 1 and math.fabs(dist_start) < 1.9 and (not self.loaded) and (not pitch > 0.2):
            self.loaded = 1
            self.spawn_sand_on_loader(x, y, z, orientation)
            self.spawn_start_pile()
        # spill sand
        elif pitch > 0.2 and self.loaded and dist_end < 1.9:
            self.loaded = 0
            self.spawn_end_pile(x, y, 0, orientation)

    def del_all_models(self, name):
        """ delete all models whose name contains given name
        Args:
            name (string): name of model group
        """
        resp = self.get_world_properties_service()
        for model in resp.model_names:
            if name in model:
                self.delete_service(model)
                rospy.sleep(0.2)

    def spawn_sand_on_loader(self, x, y, z, orientation):
        """ spawn sand grains on top of the loader
        Args:
            x,y,z (floats): current position of Bobby's loader
            orientation (geometry_msgs/Quaternion.msg): current orientation of Bobby's loader
        """
        r = SpawnModelRequest()
        f = open(self.sand_big_sdf)
        r.model_xml = f.read()

        (roll, pitch, theta) = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])

        initial_pose = Pose()
        initial_pose.position.x = x + math.cos(theta)
        initial_pose.position.y = y + math.sin(theta)
        initial_pose.position.z = z + 0.3
        r.initial_pose = initial_pose

        # spawn 5 grains
        for i in range(0, 5):
            r.model_name = "sand_big_{}".format(i)
            self.spawn_srv(r)
            rospy.sleep(0.1)

    def spawn_end_pile(self, x, y, z, orientation):
        """ Spills the sand from loader to the end pile
        Args:
            x,y,z (floats): current position of Bobby's loader
            orientation (geometry_msgs/Quaternion.msg): current orientation of Bobby's loader

        """
        # first delete the existing end pile
        self.del_all_models("sand_pile_end")

        # then spawn a bigger pile to replace it
        r = SpawnModelRequest()

        (roll, pitch, theta) = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])
        initial_pose = Pose()
        initial_pose.position.x = x + math.cos(theta)
        initial_pose.position.y = y + math.sin(theta)
        initial_pose.position.z = z
        r.model_name = "sand_pile_end_{}".format(round(rospy.get_time()))
        r.initial_pose = initial_pose

        f = open(self.sand_pile_big_sdf)
        r.model_xml = f.read()
        r.model_xml = r.model_xml.replace("<scale>2 2 2</scale>", "<scale>{0} {0} {0}</scale>".format(2.8 - self.curr_pile_scale))

        self.spawn_srv(r)

        # delete grains from loader
        self.del_all_models("sand_big")

    def spawn_start_pile(self):
        """
         Decrease start pile every time bobby take sand
        """
        # first spawn a new, smaller, pile
        r = SpawnModelRequest()

        initial_pose = Pose()
        initial_pose.position.x = 6
        initial_pose.position.y = 0
        initial_pose.position.z = 0
        r.model_name = "sand_pile_start_{}".format(self.curr_pile_scale - 0.2)
        r.initial_pose = initial_pose

        self.curr_pile_scale -= 0.2

        if self.curr_pile_scale > 1:
            f = open(self.sand_pile_big_sdf)
            r.model_xml = f.read()
            r.model_xml = r.model_xml.replace("<scale>2 2 2</scale>", "<scale>{0} {0} {0}</scale>".format(self.curr_pile_scale))

            self.spawn_srv(r)

        # then delete older, bigger, pile
        if self.curr_pile_scale == 1.8:
            self.del_all_models("sand_pile_big")
        else:
            self.del_all_models("sand_pile_start_{}".format(self.curr_pile_scale + 0.2))


if __name__ == '__main__':
    try:
        Main()
        pass
    except rospy.ROSInterruptException:
        pass
