#!/usr/bin/env python3
# A simple ROS publisher node in Python

import rospy # (1)
from std_msgs.msg import String # (2)

class Publisher(): # (3)
    
    def __init__(self): # (4)
        self.node_name = "simple_publisher" # (5)
        topic_name = "chatter" # (6)

        self.pub = rospy.Publisher(topic_name, String, queue_size=10) # (7)
        rospy.init_node(self.node_name, anonymous=True) # (8)
        self.rate = rospy.Rate(10) # (9)
                
        self.ctrl_c = False # (10)
        rospy.on_shutdown(self.shutdownhook) 
        
        rospy.loginfo(f"The '{self.node_name}' node is active...") # (11)

    def shutdownhook(self): # (12)
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True

    def main_loop(self):
        while not self.ctrl_c: # (13)
            publisher_message = f"rospy time is: {rospy.get_time()}"
            self.pub.publish(publisher_message)
            self.rate.sleep()

if __name__ == '__main__': # (14)
    publisher_instance = Publisher() # (15)
    try:
        publisher_instance.main_loop() # (16)
    except rospy.ROSInterruptException:
        pass