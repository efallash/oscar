#!/usr/bin/env python3

# Python 2 compatibility imports
from __future__ import absolute_import, division, print_function
from future import standard_library

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
    y_min=-0.875
    y_max=0.875
    #z_axis 
    z_min=0.8
    z_max=1.325

    #Resolution of the experiment
    x_divide=11
    y_divide=35
    z_divide=11


    


    #List of points
    x_points=np.linspace(x_min,x_max, num=x_divide)

    
    y_points=np.linspace(y_min,y_max, num=y_divide)
    #Using only y=0
    #y_points=[0]

    z_points=np.linspace(z_min,z_max, num=z_divide)
        
    poses=[]
    
    

     
    #Generate a list of all the poses of the experiment
    for x in x_points:
        for y in y_points:
            for z in z_points:
                #Create a pose in the x,y,z format
                pose=[x, y, z]
                #Append the pose to the list of poses                    
                poses.append(pose)



    #Shuffle of the poses to avoid bias in the experiment
    random.shuffle(poses)

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

    for gr_index in [0,1]:

        #Start in home
        rospy.loginfo("Going Home")    
        group[gr_index](0,0,0,1,"home")
        
        
        #Result format: [Arm_name,Setpoint_x,Setpoint_y,Setpoint_z,plan_success, exec_success,reached_x,reached_y,reached_z, error]
        
        for i in poses:
            result=[]
            #Build pose message
            target_pose=ArmControlRequest(i[0],i[1],i[2],velocity,'')

            #Send target to commander
            rospy.loginfo("Sending target")
            command_output=group[gr_index](target_pose)
            
            if gr_index == 0:
                    arm_name="right_arm_gripper_link"
                    arm_result="right"
            else:
                arm_name="left_arm_gripper_link"
                arm_result="left"

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
          

    


  
        
    


    

    
    


    


if __name__=='__main__':
  try:
    pose_test()
  except rospy.ROSInterruptException:
    pass

