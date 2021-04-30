#!/usr/bin/env python3

#ROS Imports
import sys, rospy, moveit_commander
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool, String, Float64
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelStateResponse
import tf
import tf2_ros
import rospy
from rospy.numpy_msg import numpy_msg


#Math imports
import numpy as np
from math import pi

#MDB Messages
from mdb_common.msg import ControlMsg, ObjectListMsg, ObjectMsg

#Oscar Messages
from oscar_msgs.srv import ArmControl, ArmControlRequest, ArmControlResponse
from oscar_msgs.srv import GripperControl, GripperControlRequest, GripperControlResponse
from oscar_msgs.srv import Perception, PerceptionRequest, PerceptionResponse


#Server to interface with the Long Term Memory of the Multilevel Darwinist Brain 

class OscarMDB:
    def __init__(self):

        # Create MDB publishers
        self.red_object_pub = rospy.Publisher('/mdb/oscar/sensor/red_object', ObjectListMsg, queue_size=1, latch=True)
        self.basket_pub = rospy.Publisher('/mdb/oscar/sensor/basket', ObjectListMsg, queue_size=1, latch=True)
        self.obj_in_left_hand_pub = rospy.Publisher('/mdb/oscar/sensor/oscar_in_left_hand', Bool, queue_size=1, latch=True)
        self.obj_in_right_hand_pub = rospy.Publisher('/mdb/oscar/sensor/oscar_in_right_hand', Bool, queue_size=1, latch=True)
        self.reward_pub = rospy.Publisher('mdb/reward', Float64, queue_size=1, latch=True)

        # Perception Messages
        # Object Perception
        self.red_object=ObjectListMsg()
        self.red_object.data.append(ObjectMsg())
        self.red_object.data[0].diameter=0.025
        self.red_object.data[0].id=1
        #Basket Perception
        self.basket=ObjectListMsg()
        self.basket.data.append(ObjectMsg())
        self.basket.data[0].diameter=0.1
        self.basket.data[0].id=2
        #Hands
        self.obj_in_left_hand=False
        self.obj_in_right_hand=False
        #Reward
        self.reward=0

        #Create perception service proxy
        rospy.loginfo("Creating Perception Service Proxy")
        rospy.wait_for_service("oscar/request_perceptions")
        self.perception_srv=rospy.ServiceProxy("oscar/request_perceptions", Perception)

        #Wait command services
        rospy.loginfo("Creating Command Service Proxy")
        rospy.wait_for_service("close_right_gripper") #This is the last service to be spawned in oscar_command_services.py
        #Arm and gripper commanders
        self.right_arm=rospy.ServiceProxy("right_arm_command", ArmControl)
        self.left_arm=rospy.ServiceProxy("left_arm_command", ArmControl)
        self.right_gripper=rospy.ServiceProxy("close_right_gripper", GripperControl)
        self.left_gripper=rospy.ServiceProxy("close_left_gripper", GripperControl)

        #Gazebo service client for moving objects
        rospy.wait_for_service("gazebo/set_model_state")
        self.move_object=rospy.ServiceProxy("gazebo/set_model_state", SetModelState)

        #Go Home
        self.left_arm(0,0,0,1,"home")        
        self.right_arm(0,0,0,1,"home")
        self.right_gripper(False)
        self.left_gripper(False)

        # Create MDB subscribers
        rospy.Subscriber("mdb/ltm/executed_policy", String, self.policy_callback)
        rospy.Subscriber("mdb/oscar/control", ControlMsg, self.reset_world_callback)

    def policy_callback(self, data):
        getattr(self, data.data + "_policy")()

        self.update_perceptions()
        self.update_reward()

        self.publish_perception()

    def reset_world_callback(self, data):
        rospy.loginfo("Reseting World...")
        if data.command=="reset_world":
            self.random_positions()
            #Go Home
            self.left_arm(0,0,0,1,"home")        
            self.right_arm(0,0,0,1,"home")
            self.right_gripper(False)
            self.left_gripper(False)
            self.update_perceptions()
            self.update_reward()
            self.publish_perception()
        else:
            rospy.logfatal("ATTENTION/ACHTUNG: LTM COMMAND NOT RECOGNIZED")

    def grasp_right_policy(self):
        self.update_perceptions()
        rospy.loginfo("Executing grasp_right policy...")

        if not self.perception.obj_in_left_hand and not self.perception.obj_in_left_hand:
            pick_point=self.perception.red_object

            #Check if object is reachable
            plan=self.right_arm(pick_point.x,pick_point.y,pick_point.z,0,"")

            if plan.plan:

                #Go Home and open the gripper
                self.right_arm(0,0,0,1,"home")
                self.right_gripper(False)

                #Go above object
                self.right_arm(pick_point.x,pick_point.y,0.9,1,"")

                #Grasp object
                self.right_arm(pick_point.x,pick_point.y,0.8,1,"")
                self.right_gripper(True)

                #Lift Object
                self.right_arm(pick_point.x,pick_point.y,0.9,1,"")

                #Go Home
                self.right_arm(0,0,0,1,"home")

    def grasp_left_policy(self):
        self.update_perceptions()
        rospy.loginfo("Executing grasp_left policy...")

        if not self.perception.obj_in_left_hand and not self.perception.obj_in_left_hand:
            pick_point=self.perception.red_object

            #Check if object is reachable
            plan=self.left_arm(pick_point.x,pick_point.y,pick_point.z,0,"")

            if plan.plan:
                #Go Home and open the gripper
                self.left_arm(0,0,0,1,"home")
                self.left_gripper(False)

                #Go above object
                self.left_arm(pick_point.x,pick_point.y,0.85,1,"")

                #Grasp object
                self.left_arm(pick_point.x,pick_point.y,0.8,1,"")
                self.left_gripper(True)

                #Lift Object
                self.left_arm(pick_point.x,pick_point.y,0.85,1,"")

                #Go Home
                self.left_arm(0,0,0,1,"home")

    def press_button_policy(self):
        self.update_perceptions()
        rospy.loginfo("Executing press_button policy...")
        if not self.perception.obj_in_left_hand and not self.perception.obj_in_right_hand:
            #Go Home and close the gripper
            self.left_arm(0,0,0,1,"home")
            self.left_gripper(True)

            #Go above the button
            self.left_arm(0.12,0.5,0.85,1,"")

            #Press the button
            self.left_arm(0.12,0.5,0.8,1,"")

            #Check if object is reachable
            pick_point=self.perception.red_object
            left_plan=self.left_arm(pick_point.x,pick_point.y,pick_point.z,0,"")
            right_plan=self.right_arm(pick_point.x,pick_point.y,pick_point.z,0,"")

            if not left_plan.plan and not right_plan.plan:
                self.bring_object_near()

            #Go above the button
            self.left_arm(0.12,0.5,0.85,1,"")

            #Go Home and open the gripper
            self.left_arm(0,0,0,1,"home")
            self.left_gripper(False)

    def place_object_right_policy(self):
        self.update_perceptions()
        rospy.loginfo("Executing place_object_right policy...")

        #Check if basket is reachable
        place_point=self.perception.basket
        plan=self.right_arm(place_point.x,place_point.y,place_point.z,0,"")

        if self.perception.obj_in_right_hand and plan.plan:
            place_point=self.perception.basket

            #Go above basket and release object
            self.right_arm(place_point.x,place_point.y,0.9,1,"")
            self.right_gripper(False)

            #Go Home
            self.right_arm(0,0,0,1,"home")

    def place_object_left_policy(self):
        self.update_perceptions()
        rospy.loginfo("Executing place_object_left policy...")


        #Check if basket is reachable
        place_point=self.perception.basket
        plan=self.left_arm(place_point.x,place_point.y,place_point.z,0,"")

        if self.perception.obj_in_left_hand:
            place_point=self.perception.basket

            #Go above basket and release object
            self.left_arm(place_point.x,place_point.y,0.9,1,"")
            self.left_gripper(False)

            #Go Home
            self.left_arm(0,0,0,1,"home")

    def change_hands_policy(self):
        self.update_perceptions()
        rospy.loginfo("Executing change_hands policy...")
        if self.perception.obj_in_left_hand:
            #Move left hand to give position
            self.left_arm(0,0,0,1,"switch_give")

            #Move right hand to pre-receive position and open gripper
            self.right_arm(0,0,0,1,"pre_switch_collect")
            self.right_gripper(False)

            #Move right hand to receive position and close gripper
            self.right_arm(0,0,0,1,"switch_collect")
            self.right_gripper(True)

            #Open left gripper and retract arm 
            self.left_gripper(False)
            self.left_arm(0,0,0,1,"post_switch_give")
            #Go home
            self.left_arm(0,0,0,1,"home")
            self.right_arm(0,0,0,1,"home")

        if self.perception.obj_in_right_hand:
            #Move right hand to give position
            self.right_arm(0,0,0,1,"switch_give")

            #Move left hand to pre-receive position and open gripper
            self.left_arm(0,0,0,1,"pre_switch_collect")
            self.left_gripper(False)

            #Move left hand to receive position and close gripper
            self.left_arm(0,0,0,1,"switch_collect")
            self.left_gripper(True)

            #Open right gripper and retract arm 
            self.right_gripper(False)
            self.right_arm(0,0,0,1,"post_switch_give")

            #Go home
            self.left_arm(0,0,0,1,"home")
            self.right_arm(0,0,0,1,"home")








    def update_perceptions(self):
        rospy.loginfo("Updating Perceptions....")
        self.perception=self.perception_srv(PerceptionRequest())
        
        #Assign perceptions to the MDB messages
        obj=OscarMDB.cartesian_to_polar(self.perception.red_object)
        bskt=OscarMDB.cartesian_to_polar(self.perception.basket)

        self.red_object.data[0].distance=obj[0]
        self.red_object.data[0].angle=obj[1]

        self.basket.data[0].distance=bskt[0]
        self.basket.data[0].angle=bskt[1]

        self.obj_in_left_hand=self.perception.obj_in_left_hand
        self.obj_in_right_hand=self.perception.obj_in_right_hand




    def update_reward(self):
        self.update_perceptions()
        rospy.loginfo("Reading Reward....")

        basket_x=self.perception.basket.x
        basket_y=self.perception.basket.y

        object_x=self.perception.red_object.x
        object_y=self.perception.red_object.y

        delta_x=abs(basket_x-object_x)
        delta_y=abs(basket_y-object_y)

        #Random reward for the moment
        if delta_x<0.035 and delta_y<0.035:
            self.reward=1
        else:
            self.reward=0
        #print(f"delta_x={delta_x}, delta_y={delta_y}")
        rospy.loginfo(f"Reward obtained: {self.reward}")



    def publish_perception(self):
        rospy.loginfo("Publishing Perceptions....")
        self.red_object_pub.publish(self.red_object)
        self.basket_pub.publish(self.basket)
        self.obj_in_left_hand_pub.publish(self.obj_in_left_hand)
        self.obj_in_right_hand_pub.publish(self.obj_in_right_hand)
        self.reward_pub.publish(self.reward)

        #Combinar bring_object_near y random_positions
    def bring_object_near(self):
        basket_x=self.perception.basket.x
        basket_y=self.perception.basket.y

        object_x=np.random.uniform(low=0.2575,high=0.3625)
        object_y=np.random.uniform(low=-0.458,high=0.458)

        delta_x=basket_x-object_x
        delta_y=basket_y-object_y
        distance=np.sqrt(delta_x*delta_x+delta_y*delta_y)
        while distance<0.15:
            object_x=np.random.uniform(low=0.2575,high=0.3625)
            object_y=np.random.uniform(low=-0.458,high=0.458)

            delta_x=basket_x-object_x
            delta_y=basket_y-object_y
            distance=np.sqrt(delta_x*delta_x+delta_y*delta_y)

        object_pose=Pose(Point(object_x,object_y,0.8),Quaternion(0,0,0,1))
        obj_msg=SetModelStateRequest()

        obj_msg.model_state.model_name="object"
        obj_msg.model_state.pose=object_pose
        obj_msg.model_state.reference_frame="world"
        
        move_resp=self.move_object(obj_msg)
        rospy.logdebug(move_resp.status_message)
        
    def random_positions(self):
        basket_x=np.random.uniform(low=0.225,high=0.25)
        basket_y=np.random.uniform(low=-0.65,high=0.65)

        object_x=np.random.uniform(low=0.1125,high=0.7375)
        object_y=np.random.uniform(low=-0.7415,high=0.7415)

        delta_x=basket_x-object_x
        delta_y=basket_y-object_y
        distance=np.sqrt(delta_x*delta_x+delta_y*delta_y)
        while distance<0.15:
            object_x=np.random.uniform(low=0.1125,high=0.7375)
            object_y=np.random.uniform(low=-0.7415,high=0.7415)

            delta_x=basket_x-object_x
            delta_y=basket_y-object_y
            distance=np.sqrt(delta_x*delta_x+delta_y*delta_y)

        
        basket_pose=Pose(Point(basket_x,basket_y,0.8),Quaternion(0,0,0,1))
        object_pose=Pose(Point(object_x,object_y,0.8),Quaternion(0,0,0,1))

        bskt_msg=SetModelStateRequest()
        obj_msg=SetModelStateRequest()

        bskt_msg.model_state.model_name="basket"
        bskt_msg.model_state.pose=basket_pose
        bskt_msg.model_state.reference_frame="world"

        obj_msg.model_state.model_name="object"
        obj_msg.model_state.pose=object_pose
        obj_msg.model_state.reference_frame="world"

        move_resp=self.move_object(bskt_msg)
        rospy.logdebug(move_resp.status_message)
        move_resp=self.move_object(obj_msg)
        rospy.logdebug(move_resp.status_message)

    @staticmethod
    def cartesian_to_polar(point):
        distance=np.sqrt(point.x*point.x+point.y*point.y)
        angle=np.arctan2(point.y, point.x)
        return distance, angle

if __name__ == "__main__":
    rospy.init_node('oscar_mdb_server', anonymous = False)
    rospy.loginfo("Started OSCAR MDB Server Node")
    server=OscarMDB()
    rospy.loginfo("OSCAR MDB Server Node: Setup Finished")
    rospy.spin()

    
