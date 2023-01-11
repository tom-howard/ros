#!/usr/bin/env python3

import rospy
from tuos_ros_msgs.srv import SetBool, {BLANK}
import sys

service_name = "move_service"

rospy.init_node(f"{service_name}_client")

rospy.wait_for_service(service_name)

service = rospy.ServiceProxy(service_name, SetBool)

service_request = {BLANK}()
service_request.request_signal = True

service_response = service(service_request)
print(service_response)