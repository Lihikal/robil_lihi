#!/usr/bin/env python
import pprint

import rospy
from gazebo_msgs.srv import *
from sensor_msgs.msg import JointState

pp = pprint.PrettyPrinter(indent=4)

model_name = 'Bobby'
service_name = '/gazebo/get_joint_properties'
joints = ['Hydraulics_joint', 'loader_joint', 'brackets_joint', 'back_left_wheel_joint', 'back_right_wheel_joint',
          'front_left_wheel_joint', 'front_right_wheel_joint']
joints_list = [model_name+'::'+joint for joint in joints]
node_name = model_name + 'joint_publisher'


def main():
    CustomPublisher()
    rospy.spin()


class CustomPublisher:
    rospy.init_node(node_name)

    Hz = 100
    rate = rospy.Rate(Hz)

    def __init__(self):
        req = GetJointPropertiesRequest()
        rospy.wait_for_service(service_name)
        service = rospy.ServiceProxy(service_name, GetJointProperties)
        pub = rospy.Publisher('joint_states', JointState, queue_size=10)

        while not rospy.is_shutdown():
            state = JointState()
            for i, j in enumerate(joints_list):
                req.joint_name = j
                try:
                    response = service(req)
                    if response.success:
                        state.name.append(j.split('::')[-1])
                        state.position.append(response.position[0])
                        state.velocity.append(response.rate[0])
                    else:
                        pass
                except Exception as e:
                    rospy.logerr("Exception in Joint State Publisher.")
                    print(type(e))
                    print(e)
                    return

            pub.publish(state)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
