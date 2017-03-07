#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String

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
  pose_target.position.x =  0.54
  pose_target.position.y =  0.71
  pose_target.position.z = -0.14
  pose_target.orientation.x = -0.32
  pose_target.orientation.y =  0.94
  pose_target.orientation.z =  0.00
  pose_target.orientation.w = -0.07

  group.set_pose_target(pose_target)

  plan1 = group.plan()

  if not plan1.joint_trajectory.header.frame_id:
    print("ERROR: No solution plan1")

  print "============ Before group.go..."
  #group.go(plan1,wait=False)
  print "============ After group.go..."

  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL

  print "============ STOPPING"
  return 0

if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
    return 0
  except rospy.ROSInterruptException:
    pass
