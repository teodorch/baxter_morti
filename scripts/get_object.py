#!/usr/bin/env python

import sys
import copy
import rospy
import os
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import sqrt
import numpy as np

import take_screenshot as ss

from std_msgs.msg import String 
import arms_setup as arms

def move_group_python_interface_tutorial():
  ## First initialize moveit_commander and rospy.
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_arm_movement',anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()


  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the left
  ## arm.  This interface can be used to plan and execute motions on the left
  ## arm.
  group = moveit_commander.MoveGroupCommander("left_arm")
  group2 = moveit_commander.MoveGroupCommander("right_arm")

  print "============ Starting tutorial "

  ## Getting Basic Information
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^
  ##
  ## We can get the name of the reference frame for this robot
  print "============ Reference frame: %s" % group.get_planning_frame()

  ## We can also print the name of the end-effector link for this group
  print "============ Reference frame: %s" % group.get_end_effector_link()

  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()

  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"


  ## Planning to a Pose goal
  ## ^^^^^^^^^^^^^^^^^^^^^^^
  ## We can plan a motion for this group to a desired pose for the
  ## end-effector
  print "============ Generating plan right"
  pose_target = geometry_msgs.msg.Pose()  
  #print group2.get_current_pose()

  
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
  ss.screenshot_right("right")

  print "============ After group.go..."
  print group2.get_current_pose()    
  
  
  pose_target.orientation.x = 0.149708761075
  pose_target.orientation.y = 0.98833322076
  pose_target.orientation.z = 0.00626127071314
  pose_target.orientation.w = 0.0273043602521
  pose_target.position.x = 0.866593664014
  pose_target.position.y = 0.093401659553
  pose_target.position.z = 0.114774653079


  group.set_pose_target(pose_target)
  group.set_start_state_to_current_state();
  plan2 = group.plan()
  
  
  #group.go(plan2, wait=True)
  #ss.screenshot_left("left_1")
  print "============ After group.go..."
  print group.get_current_pose() 
  
  
  pose_target.orientation.x = 0.237811083067
  pose_target.orientation.y = 0.876081535264
  pose_target.orientation.z = -0.353145266939
  pose_target.orientation.w = 0.226308313544
  pose_target.position.x = 0.860610694833
  pose_target.position.y = 0.144940110983
  pose_target.position.z = 0.0313208117722


  group.set_pose_target(pose_target)
  plan_left = group.plan()

  print "============ Before group.go..."
  #group.go(plan_left, wait=True)
  #ss.screenshot_left("left_2")
  print "============ After group.go..."
  print group.get_current_pose() 
  
  pose_target.orientation.x = 0.0756030883698
  pose_target.orientation.y = 0.846973156725
  pose_target.orientation.z = 0.0483242020131
  pose_target.orientation.w = 0.524008984957
  pose_target.position.x = 0.802510985464
  pose_target.position.y = 0.0763729576788
  pose_target.position.z = -0.00833527975835


  group.set_pose_target(pose_target)
  plan2 = group.plan()
  
  if not plan2.joint_trajectory.header.frame_id:
    print("ERROR: No solution plan2")

  print "============ Before group.go..."
  #group.go(plan2, wait=True)
  #ss.screenshot_left("left_3")
  ## When finished shut down moveit_commander.
  #arms.setup()
  moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL
  return 0
  print "============ STOPPING"


if __name__=='__main__':
  try:
    sys.exit(move_group_python_interface_tutorial())
  except rospy.ROSInterruptException:
    pass
