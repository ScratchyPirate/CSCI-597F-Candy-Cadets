#!/usr/bin/env python3
"""
    camera_reader_node.py
    Created: 2.25.26
    Purpose: 
        This file defines a node that reads camera data
        from the Duckiebot and displays the camera feed in a cv2 window.

    Credit: https://docs.duckietown.com/ente/devmanual-software/beginner/ros/wheel-control.html
"""

#   Imports
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge


class CameraReaderNode(DTROS):
    """
        Node to gather camera feed and post to a window.
    """

    def __init__(self, node_name):
        """
            Default constructor.
            Create the node with a specified name and subscribe to the
            camera topic. Create a window to display camera feed in
        """
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)

        #   Fields
        self._vehicle_name: str = None
        self._camera_topic: str = None
        self._bridge: CvBridge = None
        self._window: str = None
        self._camera_topic_subscriber: rospy.Subscriber = None


        #   Init fields
        #       Strings
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._window = "camera-reader"

        #       CV window and bridge
        self._bridge = CvBridge()
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        
        #       Camera subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

    def callback(self, msg):
        """
            Function to execute when camera publishes a new image
        """

        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        # display frame
        cv2.imshow(self._window, image)
        cv2.waitKey(1)

if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='camera_reader_node')
    # keep spinning
    rospy.spin()