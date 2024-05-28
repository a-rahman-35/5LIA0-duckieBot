#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage

import cv2
from cv_bridge import CvBridge

class ObjectDetectionNode(DTROS):

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(ObjectDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        
        self._vehicle_name = os.environ['VEHICLE_NAME']  # get the vehicle name from environment variable
        self._camera_topic = f'/{self._vehicle_name}/camera_node/image/compressed' # camera topic

        # Get the package path
        self.package_path = rospy.get_param('object_detection_node/package_path', os.path.join(os.path.dirname(os.path.abspath(__file__)), '../..'))

        # Load the yolo5 object detection model
        self.model = cv2.dnn.readNet(os.path.join(self.package_path, 'model', 'yolov5s.torchscript.pt'))
        
        # Initialize the cv_bridge and ROS subscriber
        self.bridge = CvBridge()

        # Subscribe to the camera topic
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

    def callback(self, msg):
        # Convert the compressed image to a cv image
        image = self.bridge.compressed_imgmsg_to_cv2(msg)

        # Do the object detection
        # ...

        # Publish the image with the detected objects
        # ...