#!/usr/bin/env python
# This code has been adapted from the ROS Wiki ROS Service tutorials and the HRWROS MOOC
# (http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)

from oscar_msgs.srv import ArmControl, ArmControlRequest, ArmControlResponse

import sys, rospy, tf, moveit_commander, random
import rospy
import numpy as np


#Service servers to command OSCAR's arms

class control_gripper:
    def __init__(self):




        #Initialize Moveit
        moveit_commander.roscpp_initialize(sys.argv)
        

        # Create a ROS service type.
        self.service = rospy.Service('left_arm_commander', GripperControl, self.control_callback)
        self.service = rospy.Service('right_arm_commander', GripperControl, self.control_callback)

        # Log message about service availability.
        rospy.loginfo('Close Gripper Service is now available.')

        #Create the group commander for the gripper
        self.group = moveit_commander.MoveGroupCommander("gripper")




    # Service callback function.
    def control_callback(self,req):
        rospy.loginfo('Close Gripper Service Called.')


        # Instantiate the response message object.
        res = GripperControlResponse()
            

        if(req.close):
          self.group.set_named_target("close")
          res= not self.group.go(wait=True) #Negated because failure in the go function means some object stopped the gripper
        else:
          self.group.set_named_target("open")
          res=self.group.go(wait=True)
                


        #Return the response message.
        return res


    

if __name__ == "__main__":
    rospy.init_node('gripper_control_server', anonymous = False)
    server=control_gripper()
    rospy.spin()

