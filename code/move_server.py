#!/usr/bin/env python3 

import rospy
from {BLANK}.msg import Twist # (1)!
from tuos_ros_msgs.srv import SetBool, SetBoolResponse # (2)!

service_name = "move_service"

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) # (3)!
vel = Twist()

def callback_function(service_request): # (4)!

    service_response = SetBoolResponse() # (5)!

    if service_request.request_signal == True: # (6)!
        print(f"The '{service_name}' Server received a 'true' request and the robot will now move for 5 seconds...") # (7)!

        StartTime = rospy.get_rostime() # (8)!

        vel.linear.x = 0.1
        pub.publish(vel) # (9)!

        rospy.loginfo('Published the velocity command to /cmd_vel')
        while (rospy.get_rostime().secs - StartTime.secs) < 5: # (10)!
            continue

        rospy.loginfo('5 seconds have elapsed, stopping the robot...')

        vel.linear.x = 0.0
        pub.publish(vel) # (11)!

        service_response.response_signal = True # (12)!
        service_response.response_message = "Request complete."
    else: # (13)!
        service_response.response_signal = False
        service_response.response_message = "Nothing happened, set request_signal to 'true' next time."
    return service_response

rospy.init_node(f"{service_name}_server") # (14)!
my_service = rospy.Service(service_name, SetBool, callback_function) # (15)!
rospy.loginfo(f"the '{service_name}' Server is ready to be called...") # (16)!
rospy.spin() # (17)!