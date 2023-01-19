#!/usr/bin/env python3

import rospy # (1)!
from tuos_ros_msgs.srv import SetBool, {BLANK} # (2)!
import sys # (3)!

service_name = "move_service" # (4)!

rospy.init_node(f"{service_name}_client") # (5)!

rospy.wait_for_service(service_name) # (6)!

service = rospy.ServiceProxy(service_name, SetBool) # (7)!

service_request = {BLANK}() # (8)!
service_request.request_signal = True # (9)!

service_response = service(service_request) # (10)!
print(service_response) # (11)!