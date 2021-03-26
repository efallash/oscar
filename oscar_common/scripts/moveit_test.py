#!/usr/bin/env python

# Python 2 compatibility imports
from __future__ import absolute_import, division, print_function
from future import standard_library

import sys, rospy, tf, moveit_commander, random
from geometry_msgs.msg import Pose, Point, Quaternion
from math import pi

#from thor_msgs.srv import GripperControl, GripperControlRequest, GripperControlResponse 

#Script to pick a box, go to home position and place it in its original position


#End effector orientations
orient = Quaternion(*tf.transformations.quaternion_from_euler(0, pi/4, 0))
orient1 = Quaternion(*tf.transformations.quaternion_from_euler(0, pi/4, pi/3))


#Predefined poses for the end effector
test_pose = Pose(Point( 0.4, -0.25, 0.9), orient)
test_pose1 = Pose(Point( 0.2, 0.1, 0.9), orient1)





#Start Moveit
moveit_commander.roscpp_initialize(sys.argv)
#Start Node
rospy.init_node('oscar_grab_test',anonymous=True)
#Commander for the arm and for the robot
group = moveit_commander.MoveGroupCommander("right_arm")
gripper = moveit_commander.MoveGroupCommander("right_gripper")
robot = moveit_commander.RobotCommander()



# We can get the name of the reference frame for this robot:
planning_frame = group.get_planning_frame()
print("============ Reference frame: %s", planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = group.get_end_effector_link()
print("============ End effector: %s", eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Robot Groups:", robot.get_group_names())

#Home
rospy.loginfo("Going Upright")    
group.set_named_target("upright_right")
print(group.go(wait=True)) 

#Pre Grasp Position
rospy.loginfo("Test Pose")
group.set_pose_target(test_pose)
print(group.go(wait=True))

#Pre Grasp Position
rospy.loginfo("Test Pose")
group.set_pose_target(test_pose1)
print(group.go(wait=True))









#Called two times because of syncronization issues
eff_pose=group.get_current_pose()
rospy.sleep(0.1)
eff_pose=group.get_current_pose()

eff_pos=eff_pose.pose.position

eff_quat=eff_pose.pose.orientation

eff_orient=tf.transformations.euler_from_quaternion([eff_quat.x,eff_quat.y,eff_quat.z,eff_quat.w])


print("Orientation")
print(eff_orient)
print("Position")
print(eff_pos)


moveit_commander.roscpp_shutdown()
