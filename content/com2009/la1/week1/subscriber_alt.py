#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

node_name = "simple_subscriber"
topic_name = {BLANK}

def callback_function(topic_message):
    print(f"The '{node_name}' node obtained the following message: '{topic_message.data}'")

rospy.init_node(node_name, anonymous=True)
sub = rospy.Subscriber(topic_name, String, callback_function)
rospy.loginfo(f"The '{node_name}' node is active...")

rospy.spin()