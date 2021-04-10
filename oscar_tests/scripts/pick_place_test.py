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

#Oscar Command services
from oscar_msgs.srv import ArmControl, ArmControlRequest, ArmControlResponse
from oscar_msgs.srv import GripperControl, GripperControlRequest, GripperControlResponse

#Script to test different pick and place conditions and record the results

def pose_test(geom,weight,vel): #Arguments are coded levels of the experiment (0: Low, 1: High)

    
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

    #Model Spawner
    spawn_object=rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    #Model directories
    box0=pkg_path+"/scripts/test_box50.sdf"
    box1=pkg_path+"/scripts/test_box200.sdf"
    cyl0=pkg_path+"/scripts/test_cylinder50.sdf"
    cyl1=pkg_path+"/scripts/test_cylinder200.sdf"
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
    basket_pose = Pose(Point( 0.25, 0, 0.78), Quaternion(0,0,0,1)) #Default basket pose
    basket_default.model_state.pose=basket_pose

    #Spawn objects
    spawn_object("box0",box0,"",pb0,"world")
    spawn_object("box1",box1,"",pb1,"world")
    spawn_object("cyl0",cyl0,"",pc0,"world")
    spawn_object("cyl1",cyl1,"",pc1,"world")

    #Wait command services
    rospy.wait_for_service("close_right_gripper") #This is the last service to be spawned in oscar_command_services.py

    #Arm and gripper commanders
    right_arm=rospy.ServiceProxy("right_arm_command", ArmControl)
    left_arm=rospy.ServiceProxy("left_arm_command", ArmControl)
    right_gripper=rospy.ServiceProxy("close_right_gripper", GripperControl)
    left_gripper=rospy.ServiceProxy("close_left_gripper", GripperControl)



   
    '''
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
    '''

    #Get poses from workspace_pick.csv
    ws_csv=pkg_path+"/scripts/workspace_pick.csv"
    pos_df=pd.read_csv(ws_csv)
    poses=pos_df.values

    print(type(poses))

    print("Sin shuffle")
    print(poses)

    

    random.shuffle(poses)
    print("Shuffle")
    print(poses)

    results=[]

    
    #Open grippers
    rospy.loginfo("Opening grippers")
    right_gripper(False)
    left_gripper(False)

    #Home arms
    rospy.loginfo("Homing arms")
    right_arm(0,0,0,1,"home")
    left_arm(0,0,0,1,"home")

    rospy.loginfo("Starting Experiment...")

    #Get tf transforms
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    #Experiment




    #Load run parameters
    if vel:
        velocity=1
    else:
        velocity=0.5
    if geom:
        obj="cyl"
    else:
        obj="box"
    if weight:
        obj=obj+"1"
    else:
        obj=obj+"0"

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
        if pose[0]=="left":
            arm=left_arm
            gripper=left_gripper
        
        
        #Move Object
        move_obj_msg.model_state.pose.position.x=pose[1]
        move_obj_msg.model_state.pose.position.y=pose[2]
        move_resp=move_object(move_obj_msg)
        rospy.loginfo(move_resp.status_message)

        #Pick and place object
        pick_and_place(pose[1], pose[2], velocity, arm, gripper)



        #Get the object position
        '''
        eff_pos= tfBuffer.lookup_transform('world', arm_name, rospy.Time())

        eff_pos=eff_pos.transform.translation

        error=np.sqrt( np.square(eff_pos.x-i[0]) + np.square(eff_pos.y-i[1]) + np.square(eff_pos.z-i[2]) )

        result=[arm_result,i[0],i[1],i[2],True,command_output.execute,eff_pos.x,eff_pos.y,eff_pos.z, error]
        '''               
            

        #If planning was successful, get final pose
        
            
    
            
            

        #Reset basket
        move_object(basket_default)


        #Store Results
        rospy.loginfo("Storing Result")
        #results.append(result)



    
    
    move_object(move_obj_default)
    #rospy.loginfo("Saving Results")
    #results_df=pd.DataFrame(results,columns=["arm","x_set","y_set","z_set","plan","execute","x_final","y_final","z_final","error"])
    #results_df.to_csv(pkg_path+'/results/pose_test/'+f'exp_cont{controller}_vel{velocity}_load{load}.csv', index=False)

    #Log finish time
    now=datetime.now()
    rospy.loginfo("Finish Time: "+str(now))   

    

def pick_and_place(x, y, velocity, arm, gripper):
    pre_pick_pose=ArmControlRequest(x,y,0.9,velocity,'')
    pick_pose=ArmControlRequest(x,y,0.8,velocity,'')
    place_pose=ArmControlRequest(0.25,0,0.87,velocity,'')
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
    pose_test(0,0,1)



  except rospy.ROSInterruptException:
    pass

