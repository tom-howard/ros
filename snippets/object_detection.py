#!/usr/bin/env python3

import rospy
from pathlib import Path # (1)!

import cv2
from cv_bridge import CvBridge, CvBridgeError # (2)!

from sensor_msgs.msg import Image # (3)!

# Initialisations: (4)
node_name = "object_detection"
rospy.init_node(node_name)
print(f"Launched the '{node_name}' node. Currently waiting for an image...")
rate = rospy.Rate(5)

base_image_path = Path.home().joinpath("myrosdata/object_detection/")
base_image_path.mkdir(parents=True, exist_ok=True) # (5)!

cvbridge_interface = CvBridge() # (6)!

waiting_for_image = True # (7)!

def show_and_save_image(img, img_name): # (8)!
    full_image_path = base_image_path.joinpath(f"{img_name}.jpg") # (9)!

    print("Opening the image in a new window...")
    cv2.imshow(img_name, img) # (10)!
    print(f"Saving the image to '{full_image_path}'...")
    cv2.imwrite(str(full_image_path), img) # (11)!
    print(f"Saved an image to '{full_image_path}'\n"
        f"image dims = {img.shape[0]}x{img.shape[1]}px\n"
        f"file size = {full_image_path.stat().st_size} bytes") # (12)!
    print("**IMPORTANT: Close the image pop-up window to continue!**")
    cv2.waitKey(0) # (13)!

def camera_cb(img_data): # (14)!
    global waiting_for_image # (15)!
    try:
        cv_img = cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8") # (16)!
    except CvBridgeError as e:
        print(e)

    if waiting_for_image == True: # (17)!
        height, width, channels = cv_img.shape

        print(f"Obtained an image of height {height}px and width {width}px.")

        show_and_save_image(cv_img, img_name = "step1_original")

        waiting_for_image = False

rospy.Subscriber("/camera/rgb/image_raw", Image, camera_cb) # (18)!

while waiting_for_image: # (19)!
    rate.sleep()

cv2.destroyAllWindows() # (20)!