#!/usr/bin/env python3

# Python 2 compatibility imports
from __future__ import absolute_import, division, print_function
from future import standard_library

from datetime import datetime
from copy import deepcopy

import sys, rospy, tf, tf2_ros, moveit_commander, random, rospkg
import pandas as pd
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from math import pi

#Gazebo services
from gazebo_msgs.srv import DeleteModel,SpawnModel 
from gazebo_msgs.srv import GetPhysicsProperties, GetPhysicsPropertiesResponse, SetPhysicsProperties, SetPhysicsPropertiesRequest
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelStateResponse
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetModelStateResponse

#Oscar Command services
from oscar_msgs.srv import ArmControl, ArmControlRequest, ArmControlResponse
from oscar_msgs.srv import GripperControl, GripperControlRequest, GripperControlResponse

#Script to test different pick and place conditions and record the results

def pose_test():

    
    #Start Node
    rospy.init_node('oscar_pick_place_test',anonymous=True)

    #Search the tests package
    rospack= rospkg.RosPack()
    pkg_path= rospack.get_path('oscar_tests')

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
    gazebo_delete('button')

    #Move object service client
    rospy.wait_for_service("gazebo/set_model_state")
    move_object=rospy.ServiceProxy("gazebo/set_model_state", SetModelState)

    #Get object state service client
    rospy.wait_for_service("gazebo/get_model_state")
    get_object=rospy.ServiceProxy("gazebo/get_model_state", GetModelState)

    #Model Spawner
    spawn_object=rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    #Model directories
    box0=pkg_path+"/objects/test_box50.sdf"
    box1=pkg_path+"/objects/test_box200.sdf"
    cyl0=pkg_path+"/objects/test_cylinder50.sdf"
    cyl1=pkg_path+"/objects/test_cylinder200.sdf"
    #Open sdf files
    with open(box0, "r") as f:
        box0 = f.read()
    
    with open(box1, "r") as f:
        box1 = f.read()
    
    with open(cyl0, "r") as f:
        cyl0 = f.read()
    
    with open(cyl1, "r") as f:
        cyl1 = f.read()

    #Object poses (side of the table, out of the workspace)
    pb0=Pose(Point(0.15,0.9,0.8),Quaternion(0,0,0,1))
    pb1=Pose(Point(0.30,0.9,0.8),Quaternion(0,0,0,1))
    pc0=Pose(Point(0.45,0.9,0.8),Quaternion(0,0,0,1))
    pc1=Pose(Point(0.60,0.9,0.8),Quaternion(0,0,0,1))
    default_poses=[[pb0, pb1],[pc0,pc1]] #To access the default poses later

    #Basket pose
    basket_default=SetModelStateRequest()
    basket_default.model_state.reference_frame="world"
    basket_default.model_state.model_name="basket"
    basket_pose = Pose(Point(0.15, 0, 0.78), Quaternion(0,0,0,1)) #Default basket pose
    basket_default.model_state.pose=basket_pose

    #Spawn objects
    spawn_object("box0",box0,"",pb0,"world")
    spawn_object("box1",box1,"",pb1,"world")
    spawn_object("cyl0",cyl0,"",pc0,"world")
    spawn_object("cyl1",cyl1,"",pc1,"world")

    #Set basket position
    move_object(basket_default)

    #Wait command services
    rospy.wait_for_service("close_right_gripper") #This is the last service to be spawned in oscar_command_services.py

    #Arm and gripper commanders
    right_arm=rospy.ServiceProxy("right_arm_command", ArmControl)
    left_arm=rospy.ServiceProxy("left_arm_command", ArmControl)
    right_gripper=rospy.ServiceProxy("close_right_gripper", GripperControl)
    left_gripper=rospy.ServiceProxy("close_left_gripper", GripperControl)


    #Get poses from workspace_pick.csv
    ws_csv=pkg_path+"/scripts/workspace_pick.csv"
    pos_df=pd.read_csv(ws_csv)
    

    #Get factors from pick_and_place_factors.csv
    fact_csv=pkg_path+"/scripts/pick_and_place_factors.csv"
    fact_df=pd.read_csv(fact_csv)
    #Convert to list
    factors=fact_df.values
    
     
    #Open grippers
    rospy.loginfo("Opening grippers")
    right_gripper(False)
    left_gripper(False)

    #Home arms
    rospy.loginfo("Homing arms")
    right_arm(0,0,0,1,"home")
    left_arm(0,0,0,1,"home")

    rospy.loginfo("Starting Experiment...")


    #Experiment
    run=1 #Run counter
    for factor in factors:
        
        #Shuffle poses dataframe
        pos_df = pos_df.sample(frac=1).reset_index(drop=True)
        #Convert to list
        poses=pos_df.values

        #Empty list to store results
        results=[]

        geom=factor[0]
        weight=factor[1]
        vel=factor[2]

        #Load run parameters
        if vel==1:
            velocity=0.1
        elif vel==-1:
            velocity=0.02
        else:
            rospy.logfatal("Incorrect velocity factor")

        if geom==1:
            obj="cyl"
        elif geom==-1:
            obj="box"
            geom=0 #Index of the default object position (line 198)
        else:
            rospy.logfatal("Incorrect geometry factor")

        if weight==1:
            obj=obj+"1"
        elif weight==-1:
            obj=obj+"0"
            weight=0 #Index of the default object position (line 198)
        else:
            rospy.logfatal("Incorrect weight factor")

        #Move object service request
        obj_orient = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
        obj_pose = Pose(Point( 0.6, -0.9, 0.8), obj_orient) #Default pose, edited for each pose
        #Object move service request
        move_obj_msg=SetModelStateRequest()
        move_obj_msg.model_state.model_name=obj
        move_obj_msg.model_state.reference_frame="world"
        move_obj_msg.model_state.pose=obj_pose
        
        #Start position of the object
        move_obj_default=deepcopy(move_obj_msg)
        move_obj_default.model_state.pose=default_poses[geom][weight]



        

        for pose in poses:

            #select arm
            if pose[0]=="right":
                arm=right_arm
                gripper=right_gripper

            elif pose[0]=="left":
                arm=left_arm
                gripper=left_gripper
            else:
                rospy.logfatal("Incorrect Arm Name")
            
            #Move Object
            move_obj_msg.model_state.pose.position.x=pose[1]
            move_obj_msg.model_state.pose.position.y=pose[2]
            move_resp=move_object(move_obj_msg)
            rospy.logdebug(move_resp.status_message)

            #Pick and place object
            rospy.loginfo(f"Run:{run}-Moving to x={pose[1]}, y={pose[2]}. Velocity={velocity}. Arm={pose[0]}")
            pick_and_place(pose[1], pose[2], velocity, arm, gripper)



            #Get object and basket position
            
            obj_pos= get_object(obj,"")

            bskt_pos= get_object("basket","")

            dist_x=abs(bskt_pos.pose.position.x-obj_pos.pose.position.x)
            dist_y=abs(bskt_pos.pose.position.y-obj_pos.pose.position.y)

            #Distance greater than 3.5cm in x or y direction means that place operation failed
            if dist_x>0.035 or dist_y>0.035:
                place_success=0
            else:
                place_success=1
            rospy.loginfo(f"Success={place_success}")


            #Result format [arm,x,y,velocity,object,success]
            result=[pose[0],pose[1],pose[2],velocity,obj,place_success]    

            #Reset basket
            move_object(basket_default)


            #Store Results
            rospy.loginfo("Storing Result")
            results.append(result)





        
        
        move_object(move_obj_default)
        rospy.loginfo("Saving Results")
        results_df=pd.DataFrame(results,columns=['arm','x','y','velocity','object','success'])
        results_df.to_csv(pkg_path+'/results/pick_place_test/'+f'exp_r{run}_{obj}_vel{velocity}.csv', index=False)
        

        #Log finish time of each run
        now=datetime.now()
        rospy.loginfo(f"Finish Time (Run{run}): "+str(now)) 
        run=run+1  

    

def pick_and_place(x, y, velocity, arm, gripper):
    pre_pick_pose=ArmControlRequest(x,y,0.9,velocity,'')
    pick_pose=ArmControlRequest(x,y,0.8,velocity,'')
    place_pose=ArmControlRequest(0.15,0,0.87,velocity,'')
    home=ArmControlRequest(0,0,0,velocity,"home")


    #Pick and place sequence

    #Pick
    arm(pre_pick_pose)
    arm(pick_pose)
    success=gripper(True) #Detects if an object was grasped
    arm(pre_pick_pose)
    arm(home)

    #Place
    arm(place_pose)
    gripper(False)

    #Home
    arm(home)
    


    
    return success
  
        
    


    

    
    


    


if __name__=='__main__':
  try:
    pose_test()



  except rospy.ROSInterruptException:
    pass

