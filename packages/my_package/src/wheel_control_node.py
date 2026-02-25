#!/usr/bin/env python3

import os
import math
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import String


# angular velocities for each wheel (quarter rotation a second)
W_LEFT = 1/4 * (2 * math.pi)
W_RIGHT = 1/4 * (2 * math.pi)

class WheelControlNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(WheelControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        wheel_radius_param = f"/{vehicle_name}/kinematics_node/radius"
        # get duckiebot's wheel radius
        self._wheel_radius = rospy.get_param(wheel_radius_param)
        # compute linear speeds
        self._vel_left = W_LEFT * self._wheel_radius
        self._vel_right = W_RIGHT * self._wheel_radius
        # construct publisher
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        # construct subscriber for other nodes to use
        # self._subscriber = rospy.Subscriber('chatter', String, self.callback)


    def run(self, velocity_left: float, velocity_right: float, time: float):
        """
            This method sets the robot's wheels to move 
        """
        
        # publish 10 messages every second (10 Hz)
        rate = rospy.Rate(0.1)
        message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)
        while not rospy.is_shutdown():
            self._publisher.publish(message)
            rate.sleep()

    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher.publish(stop)

if __name__ == '__main__':
    # create the node
    node = WheelControlNode(node_name='wheel_control_node')
    # run node
    node.run()
    # keep the process from terminating
    rospy.spin()