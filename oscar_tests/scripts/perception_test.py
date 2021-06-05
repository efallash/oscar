#!/usr/bin/env python3

#    perception_test.py: Script to perform a test of OSCAR's visual perception system
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

#ROS and standard imports
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

#Gazebo control services
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelStateResponse
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetModelStateResponse
from oscar_msgs.srv import Perception, PerceptionRequest, PerceptionResponse

#Script to place the the object or the basket in different positions and record the perception error
def pose_test():
    #Start Node
    rospy.init_node('oscar_perception_test',anonymous=True)

    #Search the tests package
    rospack= rospkg.RosPack()
    pkg_path= rospack.get_path('oscar_tests')

    #Arm command services
    rospy.wait_for_service("right_arm_command")
    right_arm=rospy.ServiceProxy("right_arm_command", ArmControl)
    rospy.wait_for_service("left_arm_command")
    left_arm=rospy.ServiceProxy("left_arm_command", ArmControl) 

    #Delete Objects
    rospy.wait_for_service("gazebo/delete_model")
    gazebo_delete=rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    

    #Move object service client
    rospy.wait_for_service("gazebo/set_model_state")
    move_object=rospy.ServiceProxy("gazebo/set_model_state", SetModelState)

    #Create perception service proxy
    rospy.loginfo("Creating Perception Service Proxy")
    rospy.wait_for_service("oscar/request_perceptions")
    perception_srv=rospy.ServiceProxy("oscar/request_perceptions", Perception)


    #Log start
    now=datetime.now()
    rospy.loginfo("Start Time: "+str(now))


    try:
        basket = rospy.get_param('~basket')
    except KeyError:
        rospy.logfatal("Experiment parameters not set")
        basket=0


    if basket:
        #Bound of the experiment
        #x axis    
        x_min=0.15
        x_max=0.7
        #y axis 
        y_min=-0.704
        y_max=0.704

        #Resolution of the experiment
        x_divide=55
        y_divide=140


        model_name="basket"
        remove_name="object"
    else:
        #Bound of the experiment
        #x axis    
        x_min=0.1125
        x_max=0.7375
        #y axis 
        y_min=-0.7415
        y_max=0.7415
        #Resolution of the experiment
        x_divide=63
        y_divide=148


        model_name="object"
        remove_name="basket" 

    #List of points
    x_points=np.linspace(x_min,x_max, num=x_divide)
    y_points=np.linspace(y_min,y_max, num=y_divide)


    #Generate a list of all the poses of the experiment
    poses=[]
    for x in x_points:
        for y in y_points:
            #Create a pose in the x,y,z format
            pose=[x, y]
            #Append the pose to the list of poses                    
            poses.append(pose)
    random.shuffle(poses)

    #Home arms
    rospy.loginfo("Lifting arms")
    left_arm(0,0,0,1,"upright")
    right_arm(0,0,0,1,"upright")


    #Move object service request
    obj_orient = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
    obj_pose = Pose(Point( 0.6, -0.9, 0.8), obj_orient) #Default pose, edited for each pose

    #Object move service request
    move_obj_msg=SetModelStateRequest()
    move_obj_msg.model_state.model_name=model_name
    move_obj_msg.model_state.reference_frame="world"
    move_obj_msg.model_state.pose=obj_pose


    #EXPERIMENT START
    rospy.loginfo("Starting Experiment...")

    #Delete unused objects
    rospy.loginfo("Deleting objects")
    gazebo_delete(remove_name)
    gazebo_delete('button')

    #Result format: [Actual_x,Actual_y,Perceived_x,Perceived_y, error]
    results=[]
    for pose in poses:
        #Move Object
        move_obj_msg.model_state.pose.position.x=pose[0]
        move_obj_msg.model_state.pose.position.y=pose[1]
        move_resp=move_object(move_obj_msg)
        rospy.sleep(0.2)

        #Request perception
        perception=perception_srv(PerceptionRequest())
        if basket:
            x=perception.basket.x
            y=perception.basket.y
        else:
            x=perception.red_object.x
            y=perception.red_object.y

        #Calculate error
        error=np.sqrt(np.square(x-pose[0]) + np.square(y-pose[1]))

        #Store result
        result=[pose[0],pose[1],x,y,error]
        results.append(result)

    #Log finish time of each run
    now=datetime.now()
    rospy.loginfo("Finish Time: "+str(now))

    rospy.loginfo("Saving Results")
    results_df=pd.DataFrame(results,
        columns=["Actual_x","Actual_y","Perceived_x","Perceived_y", "error"])
    #Store results
    results_df.to_csv(pkg_path+'/results/perception_test/'+f'perc_{now.date()}_{model_name}.csv', index=False)






#MAIN PROGRAM
if __name__=='__main__':
    try:
        pose_test()
    except rospy.ROSInterruptException:
        pass