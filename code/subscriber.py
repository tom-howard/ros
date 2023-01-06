#!/usr/bin/env python3
# A simple ROS subscriber node in Python

import rospy # (1)
from std_msgs.msg import String

class Subscriber(): # (2)

    def callback(self, topic_message): # (3)
        print(f"The '{self.node_name}' node obtained the following message: '{topic_message.data}'")

    def __init__(self): # (4)
        self.node_name = "simple_subscriber"
        topic_name = {BLANK}

        rospy.init_node(self.node_name, anonymous=True)
        self.sub = rospy.Subscriber(topic_name, String, self.callback)
        rospy.loginfo(f"The '{self.node_name}' node is active...")

    def main_loop(self):
        rospy.spin() # (5)

if __name__ == '__main__': # (6)
    subscriber_instance = Subscriber()
    subscriber_instance.main_loop()