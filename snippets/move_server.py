#!/usr/bin/env python3 

import rospy
from {BLANK}.msg import Twist # (1)!
from tuos_msgs.srv import SetBool, SetBoolResponse # (2)!

class moveService():

    def __init__(self):
        service_name = "move_service"
        rospy.init_node(f"{service_name}_server") # (14)!

        self.service = rospy.Service(service_name, SetBool, self.srv_callback) # (15)!
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) # (3)!

        rospy.loginfo(f"the '{service_name}' Server is ready to be called...") # (16)!

    def srv_callback(self, request_from_client): # (4)!
        vel = Twist()
        response_from_server = SetBoolResponse() # (5)!

        if request_from_client.request_signal == True: # (6)!
            print(f"Server received a 'true' request and the robot will now move for 5 seconds...") # (7)!

            StartTime = rospy.get_rostime() # (8)!

            vel.linear.x = 0.1
            self.pub.publish(vel) # (9)!

            rospy.loginfo('Published the velocity command to /cmd_vel')
            while (rospy.get_rostime().secs - StartTime.secs) < 5: # (10)!
                continue

            rospy.loginfo('5 seconds have elapsed, stopping the robot...')

            vel.linear.x = 0.0
            self.pub.publish(vel) # (11)!

            response_from_server.response_signal = True # (12)!
            response_from_server.response_message = "Request complete."
        else: # (13)!
            response_from_server.response_signal = False
            response_from_server.response_message = "Nothing happened, set request_signal to 'true' next time."
        return response_from_server

    def main(self):
        rospy.spin() # (17)!

if __name__ == '__main__':
    server = moveService()
    server.main() 