#! /usr/bin/env python3
# search_client.py

import rospy
import actionlib

from tuos_ros_msgs.msg import SearchAction, SearchGoal, SearchFeedback

class SearchActionClient():
    goal = SearchGoal()

    def feedback_callback(self, feedback_data: SearchFeedback):
        ## TODO: get the current distance travelled, from the feedback message
        ## and assign this to a class variable...
        self.distance = ...
        


    def __init__(self):
        self.distance = 0.0
        
        self.action_complete = False
        rospy.init_node("search_action_client")
        self.rate = rospy.Rate(1)
        
        ## TODO: setup a "simple action client" with a callback function
        ## and wait for the server to be available...
        self.client = ...



        rospy.on_shutdown(self.shutdown_ops)

    def shutdown_ops(self):
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            ## TODO: cancel the goal request, if this node is shutdown before the action has completed...
            
            
            
            rospy.logwarn("Goal Cancelled...")
        
        ## TODO: Print the result here...


                        
    def main_loop(self):
        ## TODO: assign values to all goal parameters
        ## and send the goal to the action server...
        self.goal...
        
        
        
        while self.client.get_state() < 2:
            ## TODO: Construct an if statement and cancel the goal if the 
            ## distance travelled exceeds 2 meters...
            if self.distance ...
                
                
                # break out of the while loop to stop the node:
                break
            
            self.rate.sleep()
        
        self.action_complete = True

if __name__ == '__main__':
    ## TODO: Instantiate the node and call the main_loop() method from it...
