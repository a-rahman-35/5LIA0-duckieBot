#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped
from cv_bridge import CvBridge, CvBridgeError
import torch
import cv2
import numpy as np
import os

class ObjectDetectionNode:
    def __init__(self):
        rospy.init_node('object_detection_node', anonymous=True)
        self.veh = rospy.get_namespace().strip("/")
        self.avoid_objects = False

        # Construct publishers
        self.cmd_pub = rospy.Publisher(f"/{self.veh}/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=10)
        self.image_pub = rospy.Publisher("~detections_image/compressed", CompressedImage, queue_size=10)

        # Construct subscribers
        self.image_sub = rospy.Subscriber(f"/{self.veh}/camera_node/image/compressed", CompressedImage, self.image_cb, queue_size=1)
        
        self.bridge = CvBridge()

        # Load YOLOv5 model
        model_path = os.path.join(os.path.dirname(__file__), '..', 'models', 'yolov5_weights_blocks.pt')
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)
        self.model.eval()

    def image_cb(self, image_msg):
        try:
            bgr = self.bridge.compressed_imgmsg_to_cv2(image_msg)
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            self.process_image(rgb)
        except CvBridgeError as e:
            rospy.logerr("Could not decode image: %s" % e)

    def process_image(self, rgb_image):
        results = self.model(rgb_image)
        detections = results.pandas().xyxy[0]

        self.avoid_objects = not detections.empty

        self.publish_commands()
        self.publish_debug_image(rgb_image, detections)

    def publish_commands(self):
        wheels_cmd = WheelsCmdStamped()
        if self.avoid_objects:
            wheels_cmd.vel_left = 0.0
            wheels_cmd.vel_right = 0.0
        else:
            wheels_cmd.vel_left = 0.2
            wheels_cmd.vel_right = 0.2
        self.cmd_pub.publish(wheels_cmd)

    def publish_debug_image(self, rgb_image, detections):
        for _, row in detections.iterrows():
            x1, y1, x2, y2, conf, cls = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax']), row['confidence'], row['class']
            cv2.rectangle(rgb_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{self.model.names[int(cls)]}: {conf:.2f}"
            cv2.putText(rgb_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        compressed_img_msg = self.bridge.cv2_to_compressed_imgmsg(bgr_image)
        self.image_pub.publish(compressed_img_msg)

if __name__ == '__main__':
    try:
        node = ObjectDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
