#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import baxter_interface
from sensor_msgs.msg import JointState
from baxter_core_msgs.msg import EndEffectorState

class JointGripperStateMerger():
    def __init__(self):
        self.left_gripper = None
        self.right_gripper = None
        self.joint_states = None
        self.l_finger_joints = ["left_gripper_l", "left_gripper_r"]
        self.r_finger_joints = ["right_gripper_l", "right_gripper_r"]
        upper = 0.0095
        self.lower = -0.0125
        self.diff = (upper - self.lower)/100
        self.state_pub = rospy.Publisher("/inria/joint_states",JointState)
        self.joint_states_sub = rospy.Subscriber("/robot/joint_states",JointState,self.updateJointState,None,1)
        self.right_gripper_state_sub = rospy.Subscriber("/robot/end_effector/right_gripper/state",EndEffectorState,self.updateRightGripperState,None,1)
        self.left_gripper_state_sub = rospy.Subscriber("/robot/end_effector/left_gripper/state",EndEffectorState,self.updateLeftGripperState,None,1)
        rospy.loginfo("Joint gripper state merger initialized!")
        
    def updateJointState(self,msg):
        self.joint_states = msg
        self.mergeStates()
        self.state_pub.publish(self.joint_states)
        
    def updateLeftGripperState(self,msg):
        self.left_gripper = msg
        
    def updateRightGripperState(self,msg):
        self.right_gripper = msg
        
    def mergeStates(self):
        try:
            for joint in self.l_finger_joints:
                self.joint_states.name.append(joint+"_finger_joint")
                self.joint_states.position  = self.joint_states.position + (self.lower + self.diff * self.left_gripper.position,)
                self.joint_states.velocity =  self.joint_states.velocity +  (0.0,)
                self.joint_states.effort = self.joint_states.effort +(0.0,)
        except:
            rospy.logwarn("Could not update left finger position")
        try:
            for joint in self.r_finger_joints:
                self.joint_states.name.append(joint+"_finger_joint")
                self.joint_states.position  = self.joint_states.position + (self.lower + self.diff * self.right_gripper.position,)
                self.joint_states.velocity =  self.joint_states.velocity +  (0.0,)
                self.joint_states.effort = self.joint_states.effort +(0.0,)
        except:
            rospy.logwarn("Could not update right finger position")

def main():
    rospy.init_node("joint_gripper_publisher")
    jgsm = JointGripperStateMerger()
    rospy.spin()
    
if __name__ == '__main__':
    main()
