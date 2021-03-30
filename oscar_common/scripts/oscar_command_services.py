#!/usr/bin/env python
# This code has been adapted from the ROS Wiki ROS Service tutorials and the HRWROS MOOC
# (http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)

from oscar_msgs.srv import ArmControl, ArmControlRequest, ArmControlResponse
from oscar_msgs.srv import GripperControl, GripperControlRequest, GripperControlResponse

import sys, rospy, tf, moveit_commander, random
import rospy
import numpy as np


#Service servers to command OSCAR's arms

class oscar_command:
    def __init__(self):
        #Initialize Moveit
        moveit_commander.roscpp_initialize(sys.argv)

        #Create the moveit commander for the kinematic groups
        #Arm groups
        self.left_arm = moveit_commander.MoveGroupCommander("left_arm")
        self.right_arm = moveit_commander.MoveGroupCommander("right_arm")
        #Gripper groups
        self.left_gripper = moveit_commander.MoveGroupCommander("left_gripper")
        self.right_gripper = moveit_commander.MoveGroupCommander("right_gripper")
        

        # Create a ROS services & Log message about service availability.
        # Arm services
        self.srv_left_arm = rospy.Service('left_arm_command', ArmControl, self.left_arm_callback)
        rospy.loginfo('Left Arm Service is now available.')
        self.srv_right_arm = rospy.Service('right_arm_command', ArmControl, self.right_arm_callback)
        rospy.loginfo('Right Arm Service is now available.')
        #Gripper services
        self.srv_left_gripper = rospy.Service('close_left_gripper', GripperControl, self.left_gripper_callback)
        rospy.loginfo('Left Gripper Service is now available.')
        self.srv_right_arm = rospy.Service('close_right_gripper', GripperControl, self.right_gripper_callback)
        rospy.loginfo('Right Gripper Service is now available.')
        
        

        



    def left_arm_callback(self,req):
        rospy.loginfo("left_arm_callback")

    def right_arm_callback(self,req):
        rospy.loginfo("right_arm_callback")




    def left_gripper_callback(self,req):
        rospy.loginfo("Left Gripper Service Called")
        # Instantiate the response message object.
        res = self.gripper_control(req, self.left_gripper)
        return res
        

    def right_gripper_callback(self,req):
        rospy.loginfo("Right Gripper Service Called")
        # Instantiate the response message object.
        res = self.gripper_control(req, self.right_gripper)
        return res


    def arm_command(self, req, group, arm_frame):
        pass


   
    def gripper_control(self,req,group):
        rospy.loginfo('Close Gripper Service Called.')


        # Instantiate the response message object.
        res = GripperControlResponse()
            

        if(req.close):
          group.set_named_target("closed")
          res= not group.go(wait=True) #Negated because failure in the go function means some object stopped the gripper
        else:
          group.set_named_target("open")
          res=group.go(wait=True)
                


        #Return the response message.
        return res


    

if __name__ == "__main__":
    rospy.init_node('gripper_control_server', anonymous = False)
    server=oscar_command()
    rospy.spin()

