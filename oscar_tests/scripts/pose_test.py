#!/usr/bin/env python3

#    pose_test.py: Script to perform a pose test with OSCAR robot
#    Copyright (C) 2021  Emanuel Fallas (efallashdez@gmail.com)

#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or any later version.

#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.

#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <https://www.gnu.org/licenses/>.

# Python 2 compatibility imports (Probably not necessary)
from __future__ import absolute_import, division, print_function
from future import standard_library

from datetime import datetime

import sys, rospy, tf, tf2_ros, moveit_commander, random, rospkg
import pandas as pd
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from math import pi

#Model delete service
from gazebo_msgs.srv import DeleteModel, GetPhysicsProperties, GetPhysicsPropertiesResponse, SetPhysicsProperties, SetPhysicsPropertiesRequest

#Oscar Command services
from oscar_msgs.srv import ArmControl, ArmControlRequest, ArmControlResponse
from oscar_msgs.srv import GripperControl, GripperControlRequest, GripperControlResponse

#Script to test different poses for the end effector and record success (planning and execution) and accuracy
def pose_test():

    
    #Start Node
    rospy.init_node('oscar_pose_test',anonymous=True)

    #Log start
    now=datetime.now()
    rospy.loginfo("Start Time: "+str(now))

    #Speed up gazebo simulation
    rospy.loginfo("Accelerating World")
    rospy.wait_for_service("gazebo/set_physics_properties")
    rospy.wait_for_service("gazebo/get_physics_properties")
    get_physics=rospy.ServiceProxy("gazebo/get_physics_properties", GetPhysicsProperties)
    set_physics=rospy.ServiceProxy("gazebo/set_physics_properties", SetPhysicsProperties)
    physics=get_physics()
    fast_physics=SetPhysicsPropertiesRequest(physics.time_step,0,physics.gravity,physics.ode_config)
    set_physics(fast_physics)




    #Delete Objects
    rospy.loginfo("Deleting objects")
    rospy.wait_for_service("gazebo/delete_model")
    gazebo_delete=rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    gazebo_delete('object')
    gazebo_delete('basket')
    gazebo_delete('button')

    #Get experiment parameters

    try:
        controller = rospy.get_param('~controller')
        velocity = rospy.get_param('~velocity')
        load = rospy.get_param('~load')
    except KeyError:
        rospy.logfatal("Experiment parameters not set")
        controller = 0
        velocity = 1
        load = 0

    #Wait command services
    rospy.wait_for_service("close_right_gripper") #This is the last service to be spawned in oscar_command_services.py

    #Arm and gripper commanders
    right_arm=rospy.ServiceProxy("right_arm_command", ArmControl)
    left_arm=rospy.ServiceProxy("left_arm_command", ArmControl)
    right_gripper=rospy.ServiceProxy("close_right_gripper", GripperControl)
    left_gripper=rospy.ServiceProxy("close_left_gripper", GripperControl)

    group=[right_arm,left_arm]


   

    #Bound of the experiment
    

    #x axis    
    x_min=0.1
    x_max=0.625
    #y axis (Uses negative values to explore right and left sides of the robot)
    y_min_right=-0.875
    y_max_right=0.375
    y_min_left=-0.375
    y_max_left=0.875

    #z_axis 
    z_min=0.8
    z_max=1.325

    #Resolution of the experiment
    x_divide=11
    y_divide=13
    z_divide=11


    


    #List of points
    x_points=np.linspace(x_min,x_max, num=x_divide)

    
    y_points_right=np.linspace(y_min_right,y_max_right, num=y_divide)
    y_points_left=np.linspace(y_min_left,y_max_left, num=y_divide)


    z_points=np.linspace(z_min,z_max, num=z_divide)
        
    poses_right=[]
    poses_left=[]
    
    

     
    #Generate a list of all the poses of the experiment
    for x in x_points:
        for y in y_points_right:
            for z in z_points:
                #Create a pose in the x,y,z format
                pose=[x, y, z]
                #Append the pose to the list of poses                    
                poses_right.append(pose)
    for x in x_points:
        for y in y_points_left:
            for z in z_points:
                #Create a pose in the x,y,z format
                pose=[x, y, z]
                #Append the pose to the list of poses                    
                poses_left.append(pose)



    #Shuffle of the poses to avoid bias in the experiment
    random.shuffle(poses_right)
    random.shuffle(poses_left)

    results=[]

    
    #Open grippers
    rospy.loginfo("Opening grippers")
    right_gripper(False)
    left_gripper(False)

    #Home arms
    rospy.loginfo("Homing arms")
    group[0](0,0,0,1,"home")
    group[1](0,0,0,1,"home")

    rospy.loginfo("Starting Experiment...")

    #Get tf transforms
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    for gr_index in [0,1]: #For each arm

        #Start in home
        rospy.loginfo("Going Home")    
        group[gr_index](0,0,0,1,"home")
        
        if gr_index == 0:
            arm_name="right_arm_gripper_link"
            arm_result="right"
            poses=poses_right
        else:
            arm_name="left_arm_gripper_link"
            arm_result="left"
            poses=poses_left
        #Result format: [Arm_name,Setpoint_x,Setpoint_y,Setpoint_z,plan_success, exec_success,reached_x,reached_y,reached_z, error]
        
        for i in poses:
            result=[]
            #Build pose message
            target_pose=ArmControlRequest(i[0],i[1],i[2],velocity,'')

            #Send target to commander
            rospy.loginfo("Sending target")
            command_output=group[gr_index](target_pose)
            
            

            #If planning was successful, get final pose
            if command_output.plan: 
                rospy.loginfo("Successful Plan")
                

                #Get the effector position
                eff_pos= tfBuffer.lookup_transform('world', arm_name, rospy.Time())

                eff_pos=eff_pos.transform.translation

                error=np.sqrt( np.square(eff_pos.x-i[0]) + np.square(eff_pos.y-i[1]) + np.square(eff_pos.z-i[2]) )

                result=[arm_result,i[0],i[1],i[2],True,command_output.execute,eff_pos.x,eff_pos.y,eff_pos.z, error]
                
                

            else:
                rospy.loginfo("Failed Plan")
                result=[arm_result,i[0],i[1],i[2],False,False,0,0,0,0]

            #Store Results
            rospy.loginfo("Storing Result")
            results.append(result)


        #Start in home
        rospy.loginfo("Going Home")    
        group[gr_index](0,0,0,1,"home")
    
    

    rospy.loginfo("Saving Results")
    results_df=pd.DataFrame(results,
        columns=["arm","x_set","y_set","z_set","plan","execute","x_final","y_final","z_final","error"])
    #Search the tests package
    rospack= rospkg.RosPack()

    pkg_path= rospack.get_path('oscar_tests')

    results_df.to_csv(pkg_path+'/results/pose_test/'+f'exp_cont{controller}_vel{velocity}_load{load}.csv', index=False)

    #Log finish time
    now=datetime.now()
    rospy.loginfo("Finish Time: "+str(now))   


#MAIN PROGRAM
if __name__=='__main__':
  try:
    pose_test()
  except rospy.ROSInterruptException:
    pass

