#! /usr/bin/python

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
import os

# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg, name):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        cv2.imwrite("/home/teodor/ros_ws/src/baxter_morti/images/data/" + name + ".jpg", cv2_img)

def screenshot_left(name):
    # Define your image topic
    image_topic = "/cameras/left_hand_camera/image"
    msg = rospy.wait_for_message(image_topic, Image)
    # Set up your subscriber and define its callback
    image_callback(msg, name)

    
def screenshot_right(name):
    # Define your image topic
    image_topic = "/cameras/right_hand_camera/image"
    msg = rospy.wait_for_message(image_topic, Image)
    # Set up your subscriber and define its callback
    image_callback(msg, name)
