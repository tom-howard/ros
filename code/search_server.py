#! /usr/bin/env python3
# search_server.py

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib

# Import all the necessary ROS message types:
from tuos_ros_msgs.msg import SearchAction, SearchFeedback, SearchResult, SearchGoal

# Import the tb3 modules from tb3.py
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

# Import some other useful Python Modules
from math import sqrt, pow

class SearchActionServer():
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        ## TODO: create a "simple action server" with a callback function, and start it...
        self.actionserver = actionlib.SimpleActionServer(...)
        



        # pull in some useful publisher/subscriber functions from the tb3.py module:
        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()

        rospy.loginfo("The 'Search Action Server' is active...")
    
    # The action's "callback function":
    def action_server_launcher(self, goal: SearchGoal):
        rate = rospy.Rate(10)

        ## TODO: Implement some checks on the "goal" input parameter(s)
        success = True
        if goal...


            success = False
            

        if not success:
            ## TODO: abort the action server if an invalid goal has been requested...
            
            
            
            return

        ## TODO: Print a message to indicate that the requested goal was valid
        print(f"...")

        # Get the robot's current odometry from the Tb3Odometry() class:
        self.posx0 = self.tb3_odom.posx
        self.posy0 = self.tb3_odom.posy
        # Get information about objects up ahead from the Tb3LaserScan() class:
        self.closest_object = self.tb3_lidar.min_distance
        self.closest_object_location = self.tb3_lidar.closest_object_position
        
        ## TODO: set the robot's forward velocity (as specified in the "goal")...
        self.vel_controller...
        
        ## TODO: establish a conditional statement so that the  
        ## while loop continues as long as the distance to the closest object
        ## ahead of the robot is always greater than the "approach distance"
        ## (as specified in the "goal")...
        while {something} > {something_else}:
            # update LaserScan data:
            self.closest_object = self.tb3_lidar.min_distance
            self.closest_object_location = self.tb3_lidar.closest_object_position
            
            ## TODO: publish a velocity command to make the robot start moving 
            self.vel_controller...
            
            # check if there has been a request to cancel the action mid-way through:
            if self.actionserver.is_preempt_requested():
                ## TODO: take appropriate action if the action is cancelled (peempted)...
                
                

                success = False
                # exit the loop:
                break
            
            # determine how far the robot has travelled so far:
            self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))
            
            ## TODO: update all feedback message values and publish a feedback message:
            self.feedback...



            ## TODO: update all result parameters:
            self.result...



            rate.sleep()

        if success:
            rospy.loginfo("approach completed successfully.")
            ## TODO: Set the action server to "succeeded" and stop the robot...
            


            
if __name__ == '__main__':
    rospy.init_node("search_action_server")
    SearchActionServer()
    rospy.spin()
