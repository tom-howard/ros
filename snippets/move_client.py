#!/usr/bin/env python3

import rospy # (1)!
from tuos_msgs.srv import SetBool, {BLANK} # (2)!

service_name = "move_service" # (3)!

rospy.init_node(f"{service_name}_client") # (4)!

rospy.wait_for_service(service_name) # (5)!

service = rospy.ServiceProxy(service_name, SetBool) # (6)!

request_to_server = {BLANK}() # (7)!
request_to_server.request_signal = True # (8)!

response_from_server = service(request_to_server) # (9)!
print(response_from_server) # (10)!