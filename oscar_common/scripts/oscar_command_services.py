#!/usr/bin/env python3
# This code has been adapted from the ROS Wiki ROS Service tutorials and the HRWROS MOOC
# (http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)

from oscar_msgs.srv import ArmControl, ArmControlRequest, ArmControlResponse
from oscar_msgs.srv import GripperControl, GripperControlRequest, GripperControlResponse

import sys, rospy, moveit_commander
from geometry_msgs.msg import Pose, Point, Quaternion
import tf
import rospy
import numpy as np
from math import pi

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

        #Call the commander function
        plan, execute= self.arm_command(req, self.left_arm, "left_arm")

        # Response message
        res=ArmControlResponse()
        res.plan=plan
        res.execute=execute
        
        return res


    def right_arm_callback(self,req):
        rospy.loginfo("right_arm_callback")

        #Call the commander function
        plan, execute = self.arm_command(req, self.right_arm, "right_arm")

        # Response message
        res=ArmControlResponse()
        res.plan=plan
        res.execute=execute

        return res

    def left_gripper_callback(self,req):
        rospy.loginfo("Left Gripper Service Called")
        # Instantiate the response message object.
        res= self.gripper_control(req, self.left_gripper)
    
        
        return res
        

    def right_gripper_callback(self,req):
        rospy.loginfo("Right Gripper Service Called")
        # Instantiate the response message object.
        res = self.gripper_control(req, self.right_gripper)

        
        return res


    #Commands the arm to a xyz position with simple orientation estimation for the end effector
    def arm_command(self, req, group, arm_frame):

        #Check if velocity command is in allowed values
        if req.vel>0 and req.vel<=1:
            group.set_max_velocity_scaling_factor(req.vel)
        
        if len(req.named_pose)>0 :
            group.set_named_target(req.named_pose)
            plan_success,plan,*_ = group.plan()
            exec_success=False
            if plan_success:              
              rospy.loginfo("Executing Pose")
              exec_success=group.execute(plan,wait=True)
            return(plan_success,exec_success)


        #Defining target position relative to the arm base
        arm_x=req.x
        if arm_frame=="left_arm":
            arm_y=req.y-0.25 #Static transform from the left arm and world
        elif arm_frame=="right_arm":
            arm_y=req.y+0.25 #Static transform from the right arm and world
        #In case of non existant arm_frame
        else:
            rospy.logerr("Wrong arm frame")
            return (False,False)

        
        #Get object azimuth angle respect to the arm
        if req.x==0:
            rospy.logerr("Error: Impossible to obtain x=0 position")
            return (False,False)
        else:
            yaw=np.arctan(arm_y/arm_x)


        #List of pitches to plan
        pitch_list=np.flip(np.linspace(0,pi/2, num=9))

        #Set planning time
        group.set_planning_time(0.05) #Fast Timeout


        #Attempt planning for every pitch angle
        plan_success=False
        exec_success=False
        


        for pitch in pitch_list:
          #Build pose message
          target_pose=Pose(Point(req.x,req.y,req.z),Quaternion(*tf.transformations.quaternion_from_euler(0, pitch, yaw)))
          

          #Plan Pose
          group.set_pose_target(target_pose)
          rospy.loginfo("Planing")
          plan_success,plan,*_ = group.plan()
          
          #If planning was successfull, execute trajectory
          if plan_success:              
              rospy.loginfo("Executing Pose")
              exec_success=group.execute(plan,wait=True)
              break


        return(plan_success,exec_success)






   
    def gripper_control(self,req,group):
        rospy.loginfo('Commanding gripper...')


        # Instantiate the response message object.
        res = GripperControlResponse()
            

        if(req.close):
          group.set_named_target("closed")
          res= not group.go(wait=True) #Negated because failure in the go function means some object stopped the gripper
        else:
          group.set_named_target("open")
          res= not group.go(wait=True)  #Negated because failure in the go function means some object stopped the gripper
                


        #Return the response message.
        return res


    

if __name__ == "__main__":
    rospy.init_node('oscar_command_server', anonymous = False)
    server=oscar_command()
    rospy.spin()

