#!/usr/bin/env python3

# Python 2 compatibility imports
from __future__ import absolute_import, division, print_function
from future import standard_library

import sys, rospy, tf, moveit_commander, random
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelStateResponse
from math import pi

#Oscar Command services
from oscar_msgs.srv import ArmControl, ArmControlRequest, ArmControlResponse
from oscar_msgs.srv import GripperControl, GripperControlRequest, GripperControlResponse

#Script to pick a box, go to home position and place it in its original position

#Wait for gazebo
rospy.wait_for_service("gazebo/set_model_state")
move_object=rospy.ServiceProxy("gazebo/set_model_state", SetModelState)

#Wait command services
rospy.wait_for_service("close_right_gripper") #This is the last service to be spawned in oscar_command_services.py

#Arm and gripper commanders
right_arm=rospy.ServiceProxy("right_arm_command", ArmControl)
left_arm=rospy.ServiceProxy("left_arm_command", ArmControl)
right_gripper=rospy.ServiceProxy("close_right_gripper", GripperControl)
left_gripper=rospy.ServiceProxy("close_left_gripper", GripperControl)


#Object Pick Pose
obj_orient = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
obj_pose = Pose(Point( 0.4, -0.35, 0.801), obj_orient)
#Object move service request
move_obj_msg=SetModelStateRequest()
move_obj_msg.model_state.model_name="object"
move_obj_msg.model_state.reference_frame="world"
move_obj_msg.model_state.pose=obj_pose



'''
#End effector orientations
orient = Quaternion(*tf.transformations.quaternion_from_euler(0, pi/4, 0))
orient1 = Quaternion(*tf.transformations.quaternion_from_euler(0, pi/4, pi/4))
orient2 = Quaternion(*tf.transformations.quaternion_from_euler(0, pi/4, -pi/4))

#Predefined poses for the end effector
pose_pick = Pose(Point( 0.4, -0.35, 0.8), orient)
pose_place = Pose(Point( 0.25, 0, 0.801), orient1)
pose_pick_left = Pose(Point( 0.25, 0, 0.8), orient2)
pose_place_left = Pose(Point( 0.4, 0.35, 0.801), orient)

#Predefined poses for the end effector
pose_pre_pick = Pose(Point( 0.4, -0.35, 0.9), orient)
pose_pre_place = Pose(Point( 0.25, 0, 0.9), orient1)
pose_pre_pick_left = Pose(Point( 0.25, 0, 0.9), orient2)
pose_pre_place_left = Pose(Point( 0.4, 0.35, 0.9), orient)
'''

#Predefined poses for right arm
pose_pick_right = ArmControlRequest( 0.4, -0.35, 0.8, 1, '')
pose_place_right = ArmControlRequest( 0.25, 0, 0.85, 1, '') 
pose_pre_pick_right = ArmControlRequest( 0.4, -0.35, 0.9, 1, '')
pose_pre_place_right = ArmControlRequest( 0.25, 0, 0.9, 1, '') 

#Predefined poses for left arm
pose_pick_left = ArmControlRequest( 0.25, 0, 0.8, 1, '')
pose_place_left = ArmControlRequest( 0.4, 0.35, 0.85, 1, '') 
pose_pre_pick_left = ArmControlRequest( 0.25, 0, 0.9, 1, '')
pose_pre_place_left = ArmControlRequest( 0.4, 0.35, 0.9, 1, '') 


'''
#Start Moveit
moveit_commander.roscpp_initialize(sys.argv)
#Commander for the arm and for the robot
right = moveit_commander.MoveGroupCommander("right_arm")
left = moveit_commander.MoveGroupCommander("left_arm")
right_gripper = moveit_commander.MoveGroupCommander("right_gripper")
left_gripper = moveit_commander.MoveGroupCommander("left_gripper")
robot = moveit_commander.RobotCommander()

#Set Velocity
right.set_max_velocity_scaling_factor(1)
right_gripper.set_max_velocity_scaling_factor(1)
left.set_max_velocity_scaling_factor(1)
left_gripper.set_max_velocity_scaling_factor(1)
'''

#Start Node
rospy.init_node('oscar_grab_test',anonymous=True)

#Move Object
rospy.loginfo("Moving Object")
move_obj_resp=move_object(move_obj_msg)
print(move_obj_resp.status_message)




#Upright Right
rospy.loginfo("Going Upright")    
print(right_arm(0,0,0,1,"upright")) 

#Upright Left
rospy.loginfo("Going Upright")    
print(left_arm(0,0,0,1,"upright")) 


###RIGHT PICK AND PLACE


#Gripper
rospy.loginfo("Right Open Gripper")    
print(right_gripper(False)) 

#Pre Grasp Position
rospy.loginfo("Pre Pick")
print(right_arm(pose_pre_pick_right))


#Pre Grasp Position
rospy.loginfo("Right Pick")
print(right_arm(pose_pick_right))

#Gripper
rospy.loginfo("Right Close Gripper")    
print(right_gripper(True)) 


#Home
rospy.loginfo("Right Home")    
print(right_arm(0,0,0,1,"home")) 


#Pre Grasp Position
rospy.loginfo("Right Pre Place")
print(right_arm(pose_pre_place_right))


#Place Position
rospy.loginfo("Right Place")
print(right_arm(pose_place_right))


#Gripper
rospy.loginfo("Right Open Gripper")    
print(right_gripper(False)) 


#Pre Grasp Position
rospy.loginfo("Right Pre Place")
print(right_arm(pose_pre_place_right))


#Home
rospy.loginfo("Right Home")    
print(right_arm(0,0,0,1,"home")) 


### LEFT PICK AND PLACE

#Gripper
rospy.loginfo("Left Open Gripper")    
print(left_gripper(False)) 

#Pre Grasp Position
rospy.loginfo("Left Pre Pick")
print(left_arm(pose_pre_pick_left))


#Pre Grasp Position
rospy.loginfo("Left Pick")
print(left_arm(pose_pick_left))

#Gripper
rospy.loginfo("Left Close Gripper")    
print(left_gripper(True)) 


#Pre Grasp Position
rospy.loginfo("Left Pre Pick")
print(left_arm(pose_pre_pick_left))

#Home
rospy.loginfo("Left Home")    
print(left_arm(0,0,0,1,"home"))  


#Pre Grasp Position
rospy.loginfo("Left Pre Place")
print(left_arm(pose_pre_place_left))


#Pre Grasp Position
rospy.loginfo("Left Place")
print(left_arm(pose_place_left))


#Gripper
rospy.loginfo("Left Open Gripper")    
print(left_gripper(False)) 


#Pre Grasp Position
rospy.loginfo("Left Pre Place")
print(left_arm(pose_pre_place_left))


#Home
rospy.loginfo("Left Home")    
print(left_arm(0,0,0,1,"home"))  





#moveit_commander.roscpp_shutdown()
