#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
import os
from datetime import datetime
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import Twist2DStamped
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Range
from nn_model.constants import IMAGE_SIZE
from nn_model.model import Wrapper
from nn_model.integration_activity import filter_by_classes, filter_by_bboxes, filter_by_scores, NUMBER_FRAMES_SKIPPED

class ObjectDetectionNode(DTROS):
    def __init__(self, node_name):
        super(ObjectDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh = rospy.get_namespace().strip("/")
        self.initialized = False
        self.log("Initializing!")

        self.bridge = CvBridge()
        self.model_wrapper = Wrapper()
        self.frame_id = 0
        self.first_image_received = False
        self.pub_detections_image = rospy.Publisher("~image/compressed", CompressedImage, queue_size=1, dt_topic_type=TopicType.DEBUG)
        self.sub_image = rospy.Subscriber(f"/{self.veh}/camera_node/image/compressed", CompressedImage, self.image_cb, buff_size=10000000, queue_size=1)
        # self.tof_subscriber = rospy.Subscriber(f"/{self.veh}/front_center_tof_driver_node/range", Range, self.tof_cb)
        self.chassis_publisher = rospy.Publisher(f"/{self.veh}/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)
        
        # self.objects = ["triangle_small_blue", "triangle_small_green"]
        self.objects = ["Duckie"]
        # self.objects = ["cube_small_wooden"]
        self.drop_off_object = "Cone"
        self.obstacle_object = "triangle_small_red"
        self.detected_objects = []
        self.state = "search"
        self.holding_object = False
        self.objects_picked_up = 0
        self.total_objects_to_pick = len(self.objects)

        self.Kp = 0.6
        self.Ki = 0.01
        self.Kd = 0.01
        self.min_angular_velocity = 0.01
        self.max_angular_velocity = 1.2
        self.angle_threshold = 0.05
        self.integral_error = 0
        self.previous_error = 0
        self.integral_limit = 1.0  # Anti-windup for PID controller
        self.ToF_distance = 0.1
        self.tof_distance = 0.5
        self.image_center_x = IMAGE_SIZE / 2

        self.search_duration = 4  # Duration to spin in place (seconds)
        self.spin_timer = rospy.Time.now()
        self.spin_interval = rospy.Duration(4)  # Pause interval between spins

        # Threshold values for bounding box sizes
        self.pickup_bb_threshold = 0.03  # Adjust this threshold based on actual size
        self.dropoff_bb_threshold = 0.02  # Larger threshold for drop-off object

        # Bayesian filter parameters
        self.detection_probabilities = {obj: 0.5 for obj in self.objects + [self.drop_off_object, self.obstacle_object]}

        self.initialized = True
        self.log("Initialized!")

    def image_cb(self, image_msg):
        if not self.initialized:
            return
        self.frame_id += 1
        self.frame_id = self.frame_id % (1 + NUMBER_FRAMES_SKIPPED())
        if self.frame_id != 0:
            return

        try:
            bgr = self.bridge.compressed_imgmsg_to_cv2(image_msg)
        except ValueError as e:
            self.logerr("Could not decode image: %s" % e)
            return

        rgb = bgr[..., ::-1]
        rgb = cv2.resize(rgb, (IMAGE_SIZE, IMAGE_SIZE))
        bboxes, classes, scores = self.model_wrapper.predict(rgb)
        detected_objects = self.process_detections(bboxes, classes, scores)
        detected_objects = self.apply_bayes_filter(detected_objects)

        self.detected_objects = detected_objects
        self.log_detected_objects()
        self.run_state_machine()

    def process_detections(self, bboxes, classes, scores):
        detected_objects = []
        for clas, bbox, score in zip(classes, bboxes, scores):
            if filter_by_bboxes(bbox) and filter_by_classes(clas) and filter_by_scores(score):
                clas_int = int(clas)  # Convert class ID to integer
                object_name = self.get_object_name_duckie(clas_int)
                if object_name in self.objects + [self.drop_off_object, self.obstacle_object]:  # Only consider relevant objects
                    center = [(bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2]
                    detected_objects.append({"name": object_name, "score": score, "bbox": bbox, "center": center})
        return detected_objects

    def apply_bayes_filter(self, detected_objects):
        updated_detections = []
        for obj in detected_objects:
            name = obj["name"]
            if name not in self.detection_probabilities:
                self.detection_probabilities[name] = 0.5
            self.detection_probabilities[name] = 0.9 * self.detection_probabilities[name] + 0.1 * obj["score"]
            if self.detection_probabilities[name] > 0.5:
                updated_detections.append(obj)
        return updated_detections

    def get_object_name_duckie(self, class_id):
        names = ['Duckie', 'Cone']
        if class_id < len(names):
            return names[class_id]
        return 'unknown'

    def log_detected_objects(self):
        for obj in self.detected_objects:
            bbox_size = self.bbox_size(obj['bbox'])
            self.log(f"Detected: {obj['name']} with bbox {obj['bbox']} and score {obj['score']}, BBox Size = {bbox_size}")

    def move(self, v, omega):
        twist = Twist2DStamped(v=v, omega=omega)
        self.chassis_publisher.publish(twist)

    def pid_control(self, error):
        self.integral_error += error
        if self.integral_error > self.integral_limit:
            self.integral_error = self.integral_limit
        elif self.integral_error < -self.integral_limit:
            self.integral_error = -self.integral_limit
        derivative_error = error - self.previous_error
        control_effort = self.Kp * error + self.Ki * self.integral_error + self.Kd * derivative_error
        self.previous_error = error
        return control_effort
        
    def reset_pid(self):
        self.integral_error = 0
        self.previous_error = 0

    def search(self):
        self.log("Searching for objects...")
        if rospy.Time.now() - self.spin_timer < self.spin_interval:
            self.move(0, 0)
        else:
            self.spin_timer = rospy.Time.now()
            self.move(0.0, 0.5)  # Turn to search for objects

        if self.detected_objects:
            detected_names = [obj['name'] for obj in self.detected_objects]
            if any(name in self.objects for name in detected_names):
                self.log(f"Target object detected during search")
                self.state = "approach_object"
            elif self.holding_object and self.drop_off_object in detected_names:
                self.log(f"Drop-off object detected during search")
                self.state = "approach_drop_off"

    def approach_object(self):
        if not self.detected_objects:
            self.log("Lost object, returning to search")
            self.reset_pid()
            self.state = "search"
            return

        if self.holding_object:
            target_objects = [obj for obj in self.detected_objects if obj['name'] == self.drop_off_object]
            if not target_objects:
                self.log("No drop-off objects found, returning to search")
                self.reset_pid()
                self.state = "search_drop_off"
                return
        else:
            target_objects = [obj for obj in self.detected_objects if obj['name'] in self.objects]
            if not target_objects:
                self.log("No target objects found, returning to search")
                self.reset_pid()
                self.state = "search"
                return

        closest_object = max(target_objects, key=lambda obj: obj['score'])
        obj_center_x, _ = closest_object['center']
        
        error_x = obj_center_x - self.image_center_x

        control_effort = self.pid_control(error_x / self.image_center_x)
        
        if abs(control_effort) < self.min_angular_velocity:
            control_effort = self.min_angular_velocity * np.sign(control_effort)
        
        control_effort = np.clip(control_effort, -self.max_angular_velocity, self.max_angular_velocity)
        
        self.move(0.05, -control_effort)  # Move forward while adjusting direction
        
        bbox_size = self.bbox_size(closest_object['bbox'])
        self.log(f"Centering and approaching object: Error X = {error_x}, Control effort = {-control_effort}, BBox Size = {bbox_size}")

        if self.holding_object:
            if bbox_size > self.dropoff_bb_threshold:  # Larger threshold for drop-off object
                self.move(0, 0)
                self.log("Object dropped off")
                self.holding_object = False
                self.objects_picked_up += 1
                if self.objects_picked_up >= self.total_objects_to_pick:
                    self.state = "idle"
                else:
                    self.state = "search"
                self.reset_pid()
        else:
            if bbox_size > self.pickup_bb_threshold:  # Threshold for pickup objects
                self.move(0, 0)
                self.log("Object reached and picked up")
                self.holding_object = True
                self.state = "search_drop_off"
                self.reset_pid()

    def search_drop_off(self):
        self.log("Searching for drop-off location...")
        if rospy.Time.now() - self.spin_timer < self.spin_interval:
            self.move(0, 0)
        else:
            self.spin_timer = rospy.Time.now()
            self.move(0.0, 0.5)  # Turn to search for drop-off location

        if self.detected_objects:
            detected_names = [obj['name'] for obj in self.detected_objects]
            if self.drop_off_object in detected_names:
                self.log(f"Drop-off object detected during search")
                self.state = "approach_drop_off"

    def bbox_size(self, bbox):
        return (bbox[2] - bbox[0]) * (bbox[3] - bbox[1]) / (IMAGE_SIZE ** 2)

    def run_state_machine(self):
        self.loginfo(f"State: {self.state}")
        if self.state == "search":
            self.search()
        elif self.state == "approach_object":
            self.approach_object()
        elif self.state == "search_drop_off":
            self.search_drop_off()
        elif self.state == "approach_drop_off":
            self.approach_object()
        elif self.state == "idle":
            pass
            
    def on_shutdown(self):
        self.move(0, 0)
        rospy.loginfo("Object Detection Node is shutting down.")    

    # def tof_cb(self, tof_msg):
    #    self.tof_distance = tof_msg.range

if __name__ == "__main__":
    object_detection_node = ObjectDetectionNode(node_name="object_detection_node")
    rospy.on_shutdown(object_detection_node.on_shutdown)
    rospy.spin()

