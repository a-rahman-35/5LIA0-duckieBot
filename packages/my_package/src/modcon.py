#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import String

# throttle and direction for each wheel
THROTTLE_LEFT = 0.5        # 50% throttle
DIRECTION_LEFT = 1         # forward
THROTTLE_RIGHT = 0.3       # 30% throttle
DIRECTION_RIGHT = -1       # backward

class ModconNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(ModconNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters for the node (vehicle name) 
        vehicle_name = os.environ.get('VEHICLE_NAME', 'default_vehicle')
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        
        # form the message to be published to the wheels_driver_node
        self._vel_left = THROTTLE_LEFT * DIRECTION_LEFT
        self._vel_right = THROTTLE_RIGHT * DIRECTION_RIGHT

        # construct subscriber to receive commands from the modcon_orders topic
        self.sub = rospy.Subscriber('modcon_orders', String, self.callback, queue_size=10) # self.callback is the callback function to process the received command

        # construct publisher for wheels commands (to the wheels_driver_node)
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=10) 
        rospy.loginfo(f"Subscribed to topic: modcon_orders and publishing to {wheels_topic}")

    # callback function to process the received command. 
    def callback(self, data): # data is the received command
        rospy.loginfo(f"I heard: {data.data}")
        # Process the received command (Start or Stop)
        if data.data == "Start":
            self.publish_wheels_command(self._vel_left, self._vel_right) # publish the command to start the robot
        elif data.data == "Stop":
            self.publish_wheels_command(0.0, 0.0) # publish the command to stop the robot
        else:
            rospy.logwarn(f"Unknown command received: {data.data}")

    def publish_wheels_command(self, vel_left, vel_right):
        msg = WheelsCmdStamped() # create a WheelsCmdStamped message
        msg.vel_left = vel_left # set the left wheel velocity
        msg.vel_right = vel_right  # set the right wheel velocity
        self._publisher.publish(msg) # publish the message
        rospy.loginfo(f"Published wheels command: left={vel_left}, right={vel_right}") # log the published command

if __name__ == '__main__':
    # create the node
    node = ModconNode(node_name='modcon') # node_name is the name of the node
    # keep the process from terminating
    rospy.spin()

