#!/usr/bin/env python3

#    change_hands_test.py: Script to pick an object with the right arm, change to left arm and place it in the table again
#    Copyright (C) 2021  Emanuel Fallas (efallashdez@gmail.com)

#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.

#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.

#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <https://www.gnu.org/licenses/>.


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



#SCRIPT START

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


#SCRIPT END




