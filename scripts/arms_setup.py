#!/usr/bin/env python

import argparse
import struct
import sys
import copy
  
import rospy
import rospkg
     
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)
     
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
    
import baxter_interface

class ArmMovement(object):
    def __init__(self, limb, hover_distance = 0.15, verbose=True):
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
     
    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")
        
    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

def setup():
    main()

def main():
    rospy.init_node("arm_movement")
    limb1 = 'left'
    limb2 = 'right'
    hover_distance = 0.15 # meters
        # Starting Joint angles for left arm
    starting_joint_angles = {'left_w0': 0.6718835850938112,
                             'left_w1': 1.031985575049912,
                             'left_w2': -0.476301034638421,
                             'left_e0': -1.189968899785275,
                             'left_e1': 1.9400238130755056,
                             'left_s0': -0.08000397926829805,
                             'left_s1': -0.9999781166910306}
    
    starting_joint_angles2 = {'right_w0': -0.668432128321069,
                             'right_w1': 1.0316020798529408,
                             'right_w2':  0.49777676566881673,
                             'right_e0': 1.1873011298233103,
                             'right_e1': 1.9385682206901915,
                             'right_s0': 0.08015049616701286,
                             'right_s1': -0.9974710073224903}
                             
                
    arm_left = ArmMovement(limb1, hover_distance)
    
    arm_right = ArmMovement(limb2, hover_distance)    
        
        
    arm_left.move_to_start(starting_joint_angles)
    arm_right.move_to_start(starting_joint_angles2)
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
















