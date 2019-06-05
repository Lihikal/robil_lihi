#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Float64
import numpy as np
import math

def get_depth(data):
        global depth
        if (ContactsState.states != []):
            i=0
            for i in range(len(data.states)):
                 if ('box' in data.states[i].collision2_name) or ('box' in data.states[i].collision1_name):  # check that the string exist in collision2_name/1
                           depth = np.mean(data.states[i].depths)
        elif (ContactsState.states == []):
            depth = 0
            # z_collision = np.mean(data.states[i].contact_positions[0].z)
            # rospy.loginfo(self.depth)

def main():
    rospy.init_node('depth', anonymous=True)
    ns = rospy.get_namespace()

    Hz = 50
    rate = rospy.Rate(Hz)

    ns = rospy.get_namespace()
    # depth = 0.0
    rospy.loginfo("inside main function")

    pub = rospy.Publisher('/depth', Float64, queue_size=10)

    while not rospy.is_shutdown():
            rospy.Subscriber('/robot_bumper', ContactsState, get_depth)
            rospy.loginfo(depth)
            pub.publish(depth)
            rate.sleep()

if __name__ == '__main__':
    try:
        global depth
        depth=0.0
        main()
        rospy.spin()

    except rospy.ServiceException as e:
        print
        e
