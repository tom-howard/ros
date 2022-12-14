#!/usr/bin/env python3

import rospy
from pathlib import Path

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

node_name = "object_detection_node"
rospy.init_node(node_name)
print(f"Launched the '{node_name}' node. Currently waiting for an image...")
rate = rospy.Rate(5)

base_image_path = Path.home().joinpath("myrosdata/week6_images")
base_image_path.mkdir(parents=True, exist_ok=True)

cvbridge_interface = CvBridge()

waiting_for_image = True

def show_and_save_image(img, img_name):
    full_image_path = base_image_path.joinpath(f"{img_name}.jpg")

    cv2.imshow(img_name, img)
    cv2.waitKey(0)

    cv2.imwrite(str(full_image_path), img)
    print(f"Saved an image to '{full_image_path}'\n"
        f"image dims = {img.shape[0]}x{img.shape[1]}px\n"
        f"file size = {full_image_path.stat().st_size} bytes")

def camera_cb(img_data):
    global waiting_for_image  
    try:
        cv_img = cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
    except CvBridgeError as e:
        print(e)

    if waiting_for_image == True:
        height, width, channels = cv_img.shape

        print(f"Obtained an image of height {height}px and width {width}px.")

        show_and_save_image(cv_img, img_name = "step1_original")

        waiting_for_image = False

rospy.Subscriber("/camera/rgb/image_raw", Image, camera_cb)

while waiting_for_image:
    rate.sleep()

cv2.destroyAllWindows()