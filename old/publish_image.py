#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import numpy as np
import json
import re

# Instantiate CvBridge
bridge = CvBridge()


def image_callback(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        file = open('data/raw_data.txt', 'w')
        file.write(str(msg))
        file.close()
        cv2_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgra8")

    except CvBridgeError as e:
        print(e)
    else:
        image = np.array(cv2_img, dtype=np.float)
        np.savetxt('data.csv', image, delimiter=',')
        # print(image.shape)
        # normalize image
        file = open('data/image_array.txt', 'w')
        file.write(str(image))
        file.close()
        cv2.imwrite('data/input.jpg', cv2_img)


def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/zed2/zed_node/rgb_raw/image_raw_color"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()


if __name__ == '__main__':
    main()
