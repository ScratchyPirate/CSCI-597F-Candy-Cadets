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

        # construct subscriber for other nodes to use to control the wheel
        self._subscriber = rospy.Subscriber('wheel_control', String, self.run)

    def parse_yaml(raw_input: str):
        """
          Parses a YAML formatted string, following the format:
          ---
          <argument>: <value>
          <argument>: <value>
          ---
        """
        output = {} # Create an empty dictionary as output

        lines = raw_input.split("\n") # Split our yaml based on lines

        for line in lines: # for each of the lines
            variable = line.split(":") # store the argument and its value
            output[variable[0].strip(" -")] = variable[1].strip(" -") # put the argument and its value into our output dictionary

        return output # return the dictionary containing all arguments and their corresponding values

    def duration_from_distance(distance: float, velocity):
        """Given a distance in cm and using velocity
        determine the length of time needed to reach that distance"""
        # TODO
        return -1
    
    def duration_from_angle(distance: float, left_vel, right_vel):
        """Given an angle and left and right wheel velocities
        determine the length of time needed to turn that amount"""
        # TODO
        return -1

    def run(self, movement_type: std_msgs.msg.String):
        """
            This method sets the robot's wheels to move

            Expected format movement_type:{type}\narg:{argument}
              Where 
              {type} is either 'forward' or 'turn' 
              {argument} is the angle to rotate the robot by when type is turn or the distance
                  to drive the robot forward in centimeters when type is forward

        """
        
        """
        arguments dictionary has the following form:
        {"movement_type": "forward", "argument": 3.14159}
        """
        arguments = self.parse_yaml(movement_type)
        
        match arguments["movement_type"]: # check the value of movement_type
            case "forward":
                left_wheel_vel = self._vel_left
                right_wheel_vel = self._vel_right
                duration = self.duration_from_distance(arguments["argument"], left_wheel_vel) # find the duration to move based on velocity and distance
            case "turn":
                """
                if turning, move the wheels at the velocity 
                needed to attain the specified angle of rotation.
                """
                if arguments["argument"] > 0: # if positive angle, turn clockwise
                    left_wheel_vel = self._vel_left
                    right_wheel_vel = -self._vel_right
                else: # if negative angle, turn counter-clockwise
                    left_wheel_vel = -self._vel_left
                    right_wheel_vel = self._vel_right
                duration = self.duration_from_angle(arguments["argument"], left_wheel_vel, right_wheel_vel) # find the duration to move based on velocity and distance
                
            # default case if no other case matches
            case _:
                print(f"Incorrect argument given to run. Format should be the following:",
                      f"movement_type:<type>",
                      f"arg:<argument>", sep="\n")
                return 
        
        # publish 10 messages every second (10 Hz) for duration of time
        rate = rospy.Rate(0.1) # 10 Hz
        message = WheelsCmdStamped(vel_left=left_wheel_vel, vel_right=right_wheel_vel) #move wheels at quarter of a rotation
        while not rospy.is_shutdown(): # TODO: how to make sure this runs for the given duration?
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