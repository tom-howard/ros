#!/usr/bin/env python3

import rospy
import actionlib

from tuos_msgs.msg import CameraSweepAction, CameraSweepGoal, CameraSweepFeedback

class preemptiveActionClient(): # (1)!
    goal = CameraSweepGoal() # (2)!
   
    def feedback_callback(self, feedback_data: CameraSweepFeedback): # (3)!
        self.captured_images = feedback_data.current_image
        print(f"FEEDBACK: Current yaw: {feedback_data.current_angle:.1f} degrees. "
              f"Image(s) captured so far: {self.captured_images}...")

    def __init__(self): # (4)!
        self.captured_images = 0
        self.action_complete = False

        node_name = "preemptive_camera_sweep_action_client"
        action_server_name = "/camera_sweep_action_server"
        
        rospy.init_node(node_name)

        self.rate = rospy.Rate(1)

        self.client = actionlib.SimpleActionClient(action_server_name, 
                    CameraSweepAction)
        self.client.wait_for_server()

        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self): # (5)!
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            self.client.cancel_goal()
            rospy.logwarn("Goal Cancelled...")
        
        # get the result: (6) 
        rospy.sleep(1) # wait for the result to come in
        print("RESULT:")
        print(f"  * Action State = {self.client.get_state()}")
        print(f"  * {self.captured_images} image(s) saved to {self.client.get_result()}")

    def send_goal(self, images, angle): # (7)!
        self.goal.sweep_angle = angle
        self.goal.image_count = images
        
        # send the goal to the action server:
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)        

    def main(self):
        self.send_goal(images = 0, angle = 0) # (8)!
        i = 1 # (9)!
        print("While we're waiting, let's do our seven-times tables...")
        while self.client.get_state() < 2:
            print(f"STATE: Current state code is {self.client.get_state()}")
            print(f"TIMES TABLES: {i} times 7 is {i*7}")
            i += 1
            self.rate.sleep()
        self.action_complete = True # (10)!

if __name__ == '__main__':
    {BLANK} # (11)!