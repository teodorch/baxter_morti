#! /usr/bin/python

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

import moveit_commander
import sys
import geometry_msgs.msg

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

import arms_setup as arms

# Instantiate CvBridge
bridge = CvBridge()

def get_background(image_topic, dir_path):
    for i in range(0, 10):
        name = dir_path + "frame%04d" % i
        msg = rospy.wait_for_message(image_topic, Image)
        image_callback(msg, name)

def image_callback(msg, name):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        cv2.imwrite(name + '.jpeg', cv2_img)

def main():
    arms.setup()
    moveit_commander.roscpp_initialize(sys.argv)
    
    robot = moveit_commander.RobotCommander()
    
    group = moveit_commander.MoveGroupCommander("left_arm")
    group2 = moveit_commander.MoveGroupCommander("right_arm")
    
    pose_target = geometry_msgs.msg.Pose()  
    
    pose_target.orientation.x = 0.236456535828
    pose_target.orientation.y = 0.820423215731
    pose_target.orientation.z = 0.515155768566
    pose_target.orientation.w = 0.0748905058489
    pose_target.position.x = 0.82703028839
    pose_target.position.y = -0.19754152288
    pose_target.position.z = 0.0727368002211

  
    group2.set_pose_target(pose_target)
    group2.set_start_state_to_current_state();
    plan_right = group2.plan()
  
    group2.go(plan_right, wait=True)
    
    image_topic = "/cameras/right_hand_camera/image"
    get_background(image_topic, "/home/teodor/ros_ws/src/baxter_morti/images/backgrounds/right/")

        
    pose_target.orientation.x = 0.149708761075
    pose_target.orientation.y = 0.98833322076
    pose_target.orientation.z = 0.00626127071314
    pose_target.orientation.w = 0.0273043602521
    pose_target.position.x = 0.866593664014
    pose_target.position.y = 0.093401659553
    pose_target.position.z = 0.114774653079


    group.set_pose_target(pose_target)
    plan_left_1 = group.plan()
    group.go(plan_left_1, wait=True)
    
    image_topic = "/cameras/left_hand_camera/image"
    get_background(image_topic, "/home/teodor/ros_ws/src/baxter_morti/images/backgrounds/left_1/")
    
    pose_target.orientation.x = 0.237811083067
    pose_target.orientation.y = 0.876081535264
    pose_target.orientation.z = -0.353145266939
    pose_target.orientation.w = 0.226308313544
    pose_target.position.x = 0.860610694833
    pose_target.position.y = 0.144940110983
    pose_target.position.z = 0.0313208117722


    group.set_pose_target(pose_target)
    plan_left_2 = group.plan()
    group.go(plan_left_2, wait=True)
    
    image_topic = "/cameras/left_hand_camera/image"
    get_background(image_topic, "/home/teodor/ros_ws/src/baxter_morti/images/backgrounds/left_2/")


if __name__ == '__main__':
    main()
