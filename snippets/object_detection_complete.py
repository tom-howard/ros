#!/usr/bin/env python3

import rospy
from pathlib import Path

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

node_name = "object_detection"
rospy.init_node(node_name)
print(f"Launched the '{node_name}' node. Currently waiting for an image...")
rate = rospy.Rate(5)

base_image_path = Path.home().joinpath("myrosdata/object_detection/")
base_image_path.mkdir(parents=True, exist_ok=True)

cvbridge_interface = CvBridge()

waiting_for_image = True

def show_and_save_image(img, img_name):
    full_image_path = base_image_path.joinpath(f"{img_name}.jpg")

    print("Opening the image in a new window...")
    cv2.imshow(img_name, img)
    print(f"Saving the image to '{full_image_path}'...")
    cv2.imwrite(str(full_image_path), img)
    print(f"Saved an image to '{full_image_path}'\n"
        f"image dims = {img.shape[0]}x{img.shape[1]}px\n"
        f"file size = {full_image_path.stat().st_size} bytes")
    print("**IMPORTANT: Close the image pop-up window to continue!**")
    cv2.waitKey(0)

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

        crop_width = width - 400
        crop_height = 400
        crop_y0 = int((width / 2) - (crop_width / 2))
        crop_z0 = int((height / 2) - (crop_height / 2))
        cropped_img = cv_img[crop_z0:crop_z0+crop_height, crop_y0:crop_y0+crop_width]

        show_and_save_image(cropped_img, img_name = "step2_cropping")

        hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
        lower_threshold = (115, 225, 100)
        upper_threshold = (130, 255, 255)
        img_mask = cv2.inRange(hsv_img, lower_threshold, upper_threshold)

        show_and_save_image(img_mask, img_name = "step3_image_mask")

        filtered_img = cv2.bitwise_and(cropped_img, cropped_img, mask = img_mask)

        # FINDING THE IMAGE CENTROID: (1)
        m = cv2.moments(img_mask) # (2)!
        cy = m['m10'] / (m['m00'] + 1e-5)
        cz = m['m01'] / (m['m00'] + 1e-5) # (3)!
        cv2.circle(
            filtered_img, 
            (int(cy), int(cz)), 
            10, (0, 0, 255), 2) # (4)!

        show_and_save_image(filtered_img, img_name = "step4_filtered_image")

        waiting_for_image = False

rospy.Subscriber("/camera/rgb/image_raw", Image, camera_cb)

while waiting_for_image:
    rate.sleep()

cv2.destroyAllWindows()