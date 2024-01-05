#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from tb3 import Tb3Move

class LineFollower():
    def __init__(self):
        node_name = "line_follower"
        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(5)

        self.cvbridge_interface = CvBridge()
        self.img_sub = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self.camera_cb)
        self.robot_controller = Tb3Move()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def camera_cb(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(
                img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.imshow("camera image", cv_img)

        height, width, _ = cv_img.shape
        ## TODO 1 (1)

        ## TODO 2 (2)

        ## TODO 3 (3)

        cv2.waitKey(1)

    def main(self):
        while not self.ctrl_c:
            self.rate.sleep()

if __name__ == '__main__':
    node = LineFollower()
    try:
        node.main()
    except rospy.ROSInterruptException:
        pass
