#!/usr/bin/env python

# Python 2 compatibility imports
from __future__ import absolute_import, division, print_function
from future import standard_library

import sys, rospy, tf, moveit_commander, random
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelStateResponse
from math import pi

#from thor_msgs.srv import GripperControl, GripperControlRequest, GripperControlResponse 

#Script to pick a box, go to home position and place it in its original position

#Wait for gazebo
rospy.wait_for_service("gazebo/set_model_state")
move_object=rospy.ServiceProxy("gazebo/set_model_state", SetModelState)


#Object Pick Pose
obj_orient = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
obj_pose = Pose(Point( 0.4, -0.35, 0.801), obj_orient)
#Object move service request
move_obj_msg=SetModelStateRequest()
move_obj_msg.model_state.model_name="object"
move_obj_msg.model_state.reference_frame="world"
move_obj_msg.model_state.pose=obj_pose


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




#Start Moveit
moveit_commander.roscpp_initialize(sys.argv)
#Start Node
rospy.init_node('oscar_grab_test',anonymous=True)
#Commander for the arm and for the robot
right = moveit_commander.MoveGroupCommander("right_arm")
left = moveit_commander.MoveGroupCommander("left_arm")
right_gripper = moveit_commander.MoveGroupCommander("right_gripper")
left_gripper = moveit_commander.MoveGroupCommander("left_gripper")
robot = moveit_commander.RobotCommander()

#Move Object
rospy.loginfo("Moving Object")
move_obj_resp=move_object(move_obj_msg)
print(move_obj_resp.status_message)

#Home Right
rospy.loginfo("Going Upright")    
right.set_named_target("upright")
print(right.go(wait=True)) 

#Home Left
rospy.loginfo("Going Upright")    
left.set_named_target("upright")
print(left.go(wait=True)) 


###RIGHT PICK AND PLACE


#Gripper
rospy.loginfo("Open Gripper")    
right_gripper.set_named_target("open")
print(right_gripper.go(wait=True)) 

#Pre Grasp Position
rospy.loginfo("Pre Pick")
right.set_pose_target(pose_pre_pick)
print(right.go(wait=True))


#Pre Grasp Position
rospy.loginfo("Pick")
right.set_pose_target(pose_pick)
print(right.go(wait=True))

#Gripper
rospy.loginfo("Open Gripper")    
right_gripper.set_named_target("closed")
print(right_gripper.go(wait=True)) 


#Home
rospy.loginfo("Home")    
right.set_named_target("home")
print(right.go(wait=True)) 


#Pre Grasp Position
rospy.loginfo("Pre Place")
right.set_pose_target(pose_pre_place)
print(right.go(wait=True))


#Pre Grasp Position
rospy.loginfo("Place")
right.set_pose_target(pose_place)
print(right.go(wait=True))


#Gripper
rospy.loginfo("Open Gripper")    
right_gripper.set_named_target("open")
print(right_gripper.go(wait=True)) 


#Pre Grasp Position
rospy.loginfo("Pre Place")
right.set_pose_target(pose_pre_place)
print(right.go(wait=True))


#Home
rospy.loginfo("Home")    
right.set_named_target("home")
print(right.go(wait=True)) 


### LEFT PICK AND PLACE

#Gripper
rospy.loginfo("Open Gripper")    
left_gripper.set_named_target("open")
print(left_gripper.go(wait=True)) 

#Pre Grasp Position
rospy.loginfo("Pre Pick")
left.set_pose_target(pose_pre_pick_left)
print(left.go(wait=True))


#Pre Grasp Position
rospy.loginfo("Pick")
left.set_pose_target(pose_pick_left)
print(left.go(wait=True))

#Gripper
rospy.loginfo("Open Gripper")    
left_gripper.set_named_target("closed")
print(left_gripper.go(wait=True)) 


#Home
rospy.loginfo("Home")    
left.set_named_target("home")
print(left.go(wait=True)) 


#Pre Grasp Position
rospy.loginfo("Pre Place")
left.set_pose_target(pose_pre_place_left)
print(left.go(wait=True))


#Pre Grasp Position
rospy.loginfo("Place")
left.set_pose_target(pose_place_left)
print(left.go(wait=True))


#Gripper
rospy.loginfo("Open Gripper")    
left_gripper.set_named_target("open")
print(left_gripper.go(wait=True)) 


#Pre Grasp Position
rospy.loginfo("Pre Place")
left.set_pose_target(pose_pre_place_left)
print(left.go(wait=True))


#Home
rospy.loginfo("Home")    
left.set_named_target("home")
print(left.go(wait=True)) 





moveit_commander.roscpp_shutdown()
