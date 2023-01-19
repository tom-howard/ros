#!/usr/bin/env python3

import rospy
import actionlib

from tuos_ros_msgs.msg import CameraSweepAction, CameraSweepGoal, CameraSweepFeedback

node_name = "camera_sweep_action_client"
action_server_name = "/camera_sweep_action_server"

captured_images = 0
def feedback_callback(feedback_data: CameraSweepFeedback):
    global captured_images
    captured_images = feedback_data.{BLANK}
    print(f"FEEDBACK: Current yaw: {feedback_data.current_angle:.1f} degrees. "
          f"Image(s) captured so far: {captured_images}...")

rospy.init_node(node_name)

client = actionlib.SimpleActionClient(action_server_name, 
            CameraSweepAction)
client.wait_for_server()

goal = CameraSweepGoal()
goal.sweep_angle = 0
goal.image_count = 0

client.send_goal(goal, feedback_cb=feedback_callback)

client.wait_for_result()

print(f"RESULT: Action State = {client.get_state()}")
print(f"RESULT: {captured_images} images saved to {client.get_result()}")