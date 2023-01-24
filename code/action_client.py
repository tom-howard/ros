#!/usr/bin/env python3

import rospy # (1)!
import actionlib # (2)!

from tuos_ros_msgs.msg import CameraSweepAction, CameraSweepGoal, CameraSweepFeedback # (3)!

node_name = "camera_sweep_action_client"
action_server_name = "/camera_sweep_action_server"

captured_images = 0
def feedback_callback(feedback_data: CameraSweepFeedback): # (4)!
    global captured_images
    captured_images = feedback_data.{BLANK}
    print(f"FEEDBACK: Current yaw: {feedback_data.current_angle:.1f} degrees. "
          f"Image(s) captured so far: {captured_images}...")

rospy.init_node(node_name) # (5)!

client = actionlib.SimpleActionClient(action_server_name, 
            CameraSweepAction) # (6)!
client.wait_for_server() # (7)!

goal = CameraSweepGoal()
goal.sweep_angle = 0
goal.image_count = 0

client.send_goal(goal, feedback_cb=feedback_callback) # (8)!

client.wait_for_result() # (9)!

print(f"RESULT: Action State = {client.get_state()}") # (10)!
print(f"RESULT: {captured_images} images saved to {client.get_result()}")