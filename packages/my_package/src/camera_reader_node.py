#!/usr/bin/env python3
"""
    camera_reader_node.py
    Purpose: 
        This file defines a node that reads camera data
        from the Duckiebot and displays the camera feed in a cv2 window.

    Credit: https://docs.duckietown.com/ente/devmanual-software/beginner/ros/camera-reader.html
"""
#!/usr/bin/env python3

import os
from packages.my_package.src.globals import CAMERA_APRIL_TAG_LOCATIONS_SUB, CAMERA_STREAM_SUB
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64MultiArray
import dt_apriltags
from dt_apriltags import Detector, Detection
import numpy as np
import cv2
from cv_bridge import CvBridge

CAMERA_READER_NODE_RATE = 10 #   Refresh rate for camera subscriber

class CameraReaderNode(DTROS):

    def __init__(self, node_name):


        #   Default constructor
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        
        #   Fields
        self._vehicle_name = os.environ['VEHICLE_NAME'] #   Name of bot
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"  #   Camera topic
        self._bridge = CvBridge()   #   Window to display camera in when publishing images
        self._window = "camera-reader"  #   Name of window to display camera images
        #       Image related
        self.raw_image: list[int] = [] #  To hold the latest raw image data
        self.colored_image: cv2.typing.MatLike = None #    To hold the latest image as a colored image
        self.grey_image: cv2.typing.MatLike = None #   To hold the latest image as a b/w image
        self.detector = dt_apriltags.Detector() #   April tags detector
        self.latest_camera_message: CompressedImage = None  #   To hold the latest camera reader msg
        self.new_image_loaded: bool = False #   A bool that represents whether there is a new image ready to process.
                                            #   False implies there isn't while true implies there is.
                                            #   Whenever an image is received and stored with the 'store_image' method
                                            #   set this value to true.
                                            #   Whenever a stored image is processed by

        #   Create window for camera
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)

        #   Create subscriber to retrieve images
        self.camera_sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.store_image)

        #   Create publisher to publish 
        self.camera_stream_pub = rospy.Publisher(CAMERA_STREAM_SUB, CompressedImage)
        self.camera_april_tag_loc_pub = rospy.Publisher(CAMERA_APRIL_TAG_LOCATIONS_SUB, )


    def store_image(self, msg: CompressedImage):
        """
            This method stores image information as it is received from the camera            
        """
        #   Locals
        april_tags: list[float] = []

        #       Gather image as colored and b/w image.
        self.raw_image = np.frombuffer(msg.data, np.uint8)
        self.colored_image = cv2.imdecode(self.raw_image, cv2.IMREAD_COLOR)
        self.grey_image = cv2.cvtColor(self.colored_image, cv2.COLOR_BGR2GRAY)
        self.grey_image = self.grey_image[1* len(self.colored_image) // 4 : 2 * len(self.colored_image) // 3]
        self.colored_image = self.colored_image[1 * len(self.colored_image) // 4 : 2 * len(self.colored_image) // 3]
        self.latest_camera_message = msg

        #       Notify that a new image has been found.
        self.new_image_loaded = True


    def publish_latest_image(self):
        """
            Publishes the latest image received from the camera reader topic
            using the camera stream publisher. Do so only if an image has been 
            received.
        """
        if (self.latest_camera_message != None):
            self.camera_stream_pub.publish(self.latest_camera_message.data)

    def display_latest_image(self):
        """
            Displays the latest image received from the camera reader topic
            using this node's cv window. Do so only if an image has been 
            received.
        """
        #   Locals
        image: cv2.typing.MatLike = None

        if (self.latest_camera_message != None):
            image = self._bridge.compressed_imgmsg_to_cv2(self.latest_camera_message)
            cv2.imshow(self._window, image)

    def detect_tags(self):
        """
            If an image is available
        """
        #       Extract april tag data (if it exists)

        
        if (self.new_image_loaded):

            #   Detect tags
            tags: list[Detection] = self.detector.detect(img=self.grey_img, estimate_tag_pose=True)


    def publish_april_tags(self):
        """
            Publishes the latest detected aptil tags from the 'detect_tags' method
        """
        #       For each april tag square, store in the form of:
        #           [id, top_left_x, top_left_y, top_right_x, top_right_y, bottom_right_x, bottom_right_y, bottom_left_x, bottom_left_y]
        #           Where id is the id of the april tag and the others are coordinates of the different
        #           corners of the april tag.
        #       If multiple april tags are detected, they are stored sequentially in the order parsed
        #       and so the list 
        pass

if __name__ == '__main__':
    
    #   Instantiate nod
    node = CameraReaderNode(node_name='camera_reader_node')

    #   Set the rate to run/publish content to CAMERA_READER_NODE_RATE (CAMERA_READER_NODE_RATE times a second)
    rate = rospy.Rate(CAMERA_READER_NODE_RATE)

    #   Run node until shutoff
    while not rospy.is_shutdown() and node.run:

        #   Display the latest image (if available)
        node.display_latest_image()

        #   Detect april tags (if image is available)
        node.detect_tag()

        #   Publish april tags (if found in previous method)
        node.publish_april_tags()

        #   Wait
        rate.sleep()