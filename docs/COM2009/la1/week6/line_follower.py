#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from tb3 import Tb3Move

class LineFollower(object):
    def __init__(self):
        node_name = "line_follower"
        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(5)

        self.cvbridge_interface = CvBridge()
        self.img_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_cb)
        self.robot_controller = Tb3Move()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def camera_cb(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            
        _, width, _ = cv_img.shape
        crop_width = 500
        crop_height = 40
        crop_x = int((width / 2) - (crop_width / 2))
        cropped_img = cv_img[10:crop_height, 10:crop_x+crop_width]
        
        hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)

        lower = (100, 100, 100)
        upper = (255, 255, 255)
        mask = cv2.inRange(hsv_img, lower, upper)
        res = cv2.bitwise_and(cropped_img, cropped_img, mask = mask)

        m = cv2.moments(mask)
        cy = m['m10'] / (m['m00'] + 1e-5)
        cz = m['m01'] / (m['m00'] + 1e-5)
                
        cv2.circle(res, (int(cy), int(cz)), 10, (255, 0, 0), 2)
        cv2.imshow("filtered image", res)
        cv2.waitKey(1)
                
        y_error = cy - (width / 2)
        
        kp = 1.0 / 50.0

        fwd_vel = 0.1
        ang_vel = kp * y_error
        
        print(f"Y-error = {y_error:.3f} pixels, ang_vel = {ang_vel:.3f} rad/s")
        self.robot_controller.set_move_cmd(fwd_vel, ang_vel)
        self.robot_controller.{BLANK}    # publish the velocity command

    def main(self):
        while not self.ctrl_c:
            self.rate.sleep()

if __name__ == '__main__':
    lf_instance = LineFollower()
    try:
        lf_instance.main()
    except rospy.ROSInterruptException:
        pass