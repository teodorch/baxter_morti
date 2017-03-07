#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import sqrt
import numpy as np

from std_msgs.msg import String

def normalize(v, tolerance=0.00001):
    mag2 = sum(n * n for n in v)
    if abs(mag2 - 1.0) > tolerance:
        mag = sqrt(mag2)
        v = tuple(n / mag for n in v)
    return np.array(v)

def move_group_python_interface_tutorial():
  ## First initialize moveit_commander and rospy.
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  #scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the left
  ## arm.  This interface can be used to plan and execute motions on the left
  ## arm.
  group = moveit_commander.MoveGroupCommander("left_arm")
  group2 = moveit_commander.MoveGroupCommander("right_arm")

  ## We create this DisplayTrajectory publisher which is used below to publish
  ## trajectories for RVIZ to visualize.
  #display_trajectory_publisher = rospy.Publisher(
  #                                    '/move_group/display_planned_path',
  #                                    moveit_msgs.msg.DisplayTrajectory)

  ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
  #print "============ Waiting for RVIZ..."
  #rospy.sleep(10)
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
  print "============ Generating plan 1"
  pose_target = geometry_msgs.msg.Pose()  
  #print group.get_current_pose()
  print group2.get_current_pose()
  
  v = [0.99, -0.05, -0.02, 0.006]
  v = normalize(v)
  
  pose_target.orientation.x = 0.236787613412
  pose_target.orientation.y = 0.820419029664
  pose_target.orientation.z = 0.514920718837
  pose_target.orientation.w = 0.0755042728073
  pose_target.position.x = 0.839694837099
  pose_target.position.y = -0.328787907362
  pose_target.position.z = -0.0227358943576
  
  group2.set_pose_target(pose_target)
  group2.set_start_state_to_current_state();
  plan = group2.plan()
  
  group2.go(plan, wait=True)

  
  
  #pose_target.orientation.x = v[0]
  #pose_target.orientation.y = v[1]
  #pose_target.orientation.z = v[2]
  #pose_target.orientation.w = v[3]
  #pose_target.position.x = 0.581650163136
  #pose_target.position.y = 0.179503175447
  #pose_target.position.z = 0.103167938468
  
  #group.set_pose_target(pose_target)
  #group.set_goal_orientation_tolerance(0.1);

  #plan1 = group.plan()
  

  #if not plan1.joint_trajectory.header.frame_id:
  #  print("ERROR: No solution plan1")

  print "============ Before group.go..."
  #group.go(plan1, wait=True)
  print "============ After group.go..."
  print group.get_current_pose()    
  
  #pose_target.orientation.x = 0.999840890638
  #pose_target.orientation.y = 0.0137420026584
  #pose_target.orientation.z = 0.0083877951816
  #pose_target.orientation.w = 0.00768086343309
  #pose_target.position.x = 0.775108192524
  #pose_target.position.y = 0.126942174004
  #pose_target.position.z = 0.115825261414  
  
  pose_target.orientation.x = -0.992331033695
  pose_target.orientation.y = -0.00639551602537
  pose_target.orientation.z = -0.119816072059
  pose_target.orientation.w = 0.0297039697313
  pose_target.position.x = 0.795391990662
  pose_target.position.y = 0.127373565438
  pose_target.position.z = 0.120531619255

  group.set_pose_target(pose_target)
  group.set_start_state_to_current_state();
  plan2 = group.plan()

  if not plan2.joint_trajectory.header.frame_id:
    print("ERROR: No solution plan2")

  print "============ Before group.go..."
  group.go(plan2, wait=True)
  print "============ After group.go..."
  print group.get_current_pose() 
  
  
  #pose_target.orientation.x = -0.992331033695
  #pose_target.orientation.y = -0.00639551602537
  #pose_target.orientation.z = -0.119816072059
  #pose_target.orientation.w = 0.0297039697313
  #pose_target.position.x = 0.795391990662
  #pose_target.position.y = 0.127373565438
  #pose_target.position.z = 0.120531619255

  #group.set_pose_target(pose_target)
  #plan3 = group.plan()

  #if not plan2.joint_trajectory.header.frame_id:
  #  print("ERROR: No solution plan2")

  print "============ Before group.go..."
  #group.go(plan3, wait=True)
  print "============ After group.go..."
  print group.get_current_pose() 
  
  pose_target.orientation.x = 0.872140449475
  pose_target.orientation.y = -0.0561375806538
  pose_target.orientation.z = 0.106326848312
  pose_target.orientation.w = 0.474251209546
  pose_target.position.x = 0.839400319485
  pose_target.position.y = 0.236851537022
  pose_target.position.z = 0.0362598461986

  group.set_pose_target(pose_target)
  plan_left = group.plan()

  group.go(plan_left, wait=True)
  ## When finished shut down moveit_commander.
  
  pose_target.orientation.x = 0.96408351143
  pose_target.orientation.y = -0.0258245920717
  pose_target.orientation.z = 0.117329760151
  pose_target.orientation.w = 0.236875074283
  pose_target.position.x = 0.79526741433
  pose_target.position.y = 0.126852889789
  pose_target.position.z = 0.120477367407


  group.set_pose_target(pose_target)
  plan2 = group.plan()
  group.go(plan2, wait=True)
  moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL
  return 0
  print "============ STOPPING"


if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
