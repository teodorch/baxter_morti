#!/usr/bin/env python
import baxter_interface
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
from feature_matching import match


bridge = CvBridge()
i = [0]

display_pub= rospy.Publisher('/robot/xdisplay',Image)
def republish(msg):
        """
            Sends the camera image to baxter's display
        """             
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        match(cv2_img, i)

        display_pub.publish(msg)
    


def main():
    rospy.init_node("my_cam")
    right_hand_camera = baxter_interface.CameraController("left_hand_camera")
    right_hand_camera.resolution =(640,400)
    right_hand_camera.open()
    camera_name = "left_hand_camera"
    sub = rospy.Subscriber('/cameras/' + camera_name + "/image", Image, republish, None, 1)
    rospy.spin()

if __name__ == '__main__':
    main()
