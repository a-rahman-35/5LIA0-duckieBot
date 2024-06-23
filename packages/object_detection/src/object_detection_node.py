#!/usr/bin/env python3

import cv2
import numpy as np
import rospy

from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import EpisodeStart
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

from nn_model.constants import IMAGE_SIZE
from nn_model.model import Wrapper

from nn_model.integration_activity import \
    NUMBER_FRAMES_SKIPPED, \
    filter_by_classes, \
    filter_by_bboxes, \
    filter_by_scores

class ObjectDetectionNode(DTROS):
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(ObjectDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.initialized = False
        self.log("Initializing!")

        self.veh = rospy.get_namespace().strip("/")
        self.avoid_duckies = False

        # Construct the publisher for debug images
        self.pub_detections_image = rospy.Publisher(
            "~image/compressed",
            CompressedImage,
            queue_size=1,
            dt_topic_type=TopicType.DEBUG
        )

        # Construct the subscriber for compressed images
        self.sub_image = rospy.Subscriber(
            f"/{self.veh}/camera_node/image/compressed",
            CompressedImage,
            self.image_cb,
            buff_size=10000000,
            queue_size=1,
        )

        self.bridge = CvBridge()

        aido_eval = rospy.get_param("~AIDO_eval", False)
        self.log(f"AIDO EVAL VAR: {aido_eval}")
        self.log("Starting model loading!")
        self._debug = False
        self.model_wrapper = Wrapper()
        self.log("Finished model loading!")
        self.frame_id = 0
        self.first_image_received = False
        self.initialized = True
        self.log("Initialized!")
        
        # Define colors and names for the new dataset
        self.colors = {
            0: (0, 255, 255), 1: (0, 165, 255), 2: (0, 250, 0), 3: (0, 0, 255),
            4: (255, 0, 0), 5: (255, 255, 0), 6: (255, 0, 255), 7: (0, 255, 0),
            8: (0, 255, 127), 9: (255, 127, 0), 10: (127, 0, 255), 11: (127, 255, 0),
            12: (0, 127, 255), 13: (127, 127, 127), 14: (255, 255, 255), 15: (0, 0, 127),
            16: (127, 0, 0), 17: (127, 127, 0)
        }
        self.names = [
            'bridge_blue', 'bridge_red', 'cube_large', 'cube_small_wooden', 'cube_small_yellow',
            'cylinder_large', 'cylinder_medium_brown', 'cylinder_medium_green', 'cylinder_small_blue',
            'cylinder_small_brown', 'cylinder_small_green', 'rectangle_brown', 'rectangle_wooden', 'step',
            'triangle_large', 'triangle_small_blue', 'triangle_small_green', 'triangle_small_red'
        ]

    def image_cb(self, image_msg):
        if not self.initialized:
            return

        self.frame_id += 1
        self.frame_id = self.frame_id % (1 + NUMBER_FRAMES_SKIPPED())
        if self.frame_id != 0:
            return

        # Decode from compressed image with OpenCV
        try:
            bgr = self.bridge.compressed_imgmsg_to_cv2(image_msg)
        except ValueError as e:
            self.logerr("Could not decode image: %s" % e)
            return

        rgb = bgr[..., ::-1]
        rgb = cv2.resize(rgb, (IMAGE_SIZE, IMAGE_SIZE))
        bboxes, classes, scores = self.model_wrapper.predict(rgb)

        detection = self.det2bool(bboxes, classes, scores)

        detected_objects = []
        if detection:
            font = cv2.FONT_HERSHEY_SIMPLEX
            for clas, bbox, score in zip(classes, bboxes, scores):
                clas_int = int(clas)  # Convert class ID to integer
                object_name = self.get_object_name(clas_int)
                detected_objects.append(f"{object_name} (score: {score:.2f}, bbox: {bbox})")
                self.log(f"Detected class ID: {clas_int}, name: {object_name}, score: {score:.2f}, bbox: {bbox}")
                
                pt1 = np.array([int(bbox[0]), int(bbox[1])])
                pt2 = np.array([int(bbox[2]), int(bbox[3])])
                pt1 = tuple(pt1)
                pt2 = tuple(pt2)
                
                # self.log(f"Detected class: {names[clas_int]}")
                color = tuple(reversed(self.colors[clas_int])) 
                # reverse the colors to convert from BGR to RGB
                name = self.names[clas_int]
                # draw bounding box
                rgb = cv2.rectangle(rgb, pt1, pt2, color, 2)
                # label location
                text_location = (pt1[0], min(pt2[1] + 30, IMAGE_SIZE))
                # draw label underneath the bounding box
                rgb = cv2.putText(rgb, name, text_location, font, 1, color, thickness=2)
            # self.log(f"Objects detected: {', '.join(detected_objects)}")
            bgr = rgb[..., ::-1]
            obj_det_img = self.bridge.cv2_to_compressed_imgmsg(bgr)
            self.pub_detections_image.publish(obj_det_img)

#         if self._debug:
#             # Define colors and names for the new dataset
#             colors = {
#                 0: (0, 255, 255), 1: (0, 165, 255), 2: (0, 250, 0), 3: (0, 0, 255),
#                 4: (255, 0, 0), 5: (255, 255, 0), 6: (255, 0, 255), 7: (0, 255, 0),
#                 8: (0, 255, 127), 9: (255, 127, 0), 10: (127, 0, 255), 11: (127, 255, 0),
#                 12: (0, 127, 255), 13: (127, 127, 127), 14: (255, 255, 255), 15: (0, 0, 127),
#                 16: (127, 0, 0), 17: (127, 127, 0)
#             }
#             names = [
#                 'bridge_blue', 'bridge_red', 'cube_large', 'cube_small_wooden', 'cube_small_yellow',
#                 'cylinder_large', 'cylinder_medium_brown', 'cylinder_medium_green', 'cylinder_small_blue',
#                 'cylinder_small_brown', 'cylinder_small_green', 'rectangle_brown', 'rectangle_wooden', 'step',
#                 'triangle_large', 'triangle_small_blue', 'triangle_small_green', 'triangle_small_red'
#             ]
            
# #            colors = {0: (0, 255, 255), 1: (0, 165, 255), 2: (0, 250, 0), 3: (0, 0, 255)}
# #            names = {0: "duckie", 1: "cone", 2: "truck", 3: "bus"}
#             font = cv2.FONT_HERSHEY_SIMPLEX
#             for clas, box in zip(classes, bboxes):
#                 clas_int = int(clas)  # Convert class ID to integer
#                 pt1 = np.array([int(box[0]), int(box[1])])
#                 pt2 = np.array([int(box[2]), int(box[3])])
#                 pt1 = tuple(pt1)
#                 pt2 = tuple(pt2)
#                 self.log(f"Detected class: {names[clas_int]}")
#                 color = tuple(reversed(colors[clas_int])) 
#                 # reverse the colors to convert from BGR to RGB
#                 name = names[clas_int]
#                 # draw bounding box
#                 rgb = cv2.rectangle(rgb, pt1, pt2, color, 2)
#                 # label location
#                 text_location = (pt1[0], min(pt2[1] + 30, IMAGE_SIZE))
#                 # draw label underneath the bounding box
#                 rgb = cv2.putText(rgb, name, text_location, font, 1, color, thickness=2)

#             bgr = rgb[..., ::-1]
#             obj_det_img = self.bridge.cv2_to_compressed_imgmsg(bgr)
#             self.pub_detections_image.publish(obj_det_img)

    def get_object_name(self, class_id):
        if class_id < len(self.names):
            return self.names[int(class_id)]
        return 'unknown'

    def det2bool(self, bboxes, classes, scores):
        box_ids = np.array(list(map(filter_by_bboxes, bboxes))).nonzero()[0]
        cla_ids = np.array(list(map(filter_by_classes, classes))).nonzero()[0]
        sco_ids = np.array(list(map(filter_by_scores, scores))).nonzero()[0]

        box_cla_ids = set(list(box_ids)).intersection(set(list(cla_ids)))
        box_cla_sco_ids = set(list(sco_ids)).intersection(set(list(box_cla_ids)))

        return len(box_cla_sco_ids) > 0

if __name__ == "__main__":
    # Initialize the node
    object_detection_node = ObjectDetectionNode(node_name="object_detection_node")
    # Keep it spinning
    rospy.spin()
