#!/usr/bin/env python
#title echo.py
#author ariel anders
   
import rospy
import roslib; roslib.load_manifest("pr2_controllers_msgs")
from pr2_controllers_msgs.msg import JointTrajectoryControllerState
   
def callback(data):
  print data.actual.positions
   
def listener():
  rospy.init_node('listener')
  rospy.Subscriber("l_arm_controller/state", JointTrajectoryControllerState, callback)
  rospy.spin()
   
listener()

