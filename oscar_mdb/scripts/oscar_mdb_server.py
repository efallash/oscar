#!/usr/bin/env python3



#ROS Imports
import sys, rospy, moveit_commander
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool, String, Float64
import tf
import tf2_ros
import rospy
from rospy.numpy_msg import numpy_msg


#Math imports
import numpy as np
from math import pi

#MDB Messages
from mdb_common.msg import ControlMsg, ObjectListMsg, ObjectMsg

#Server to interface with the Long Term Memory of the Multilevel Darwinist Brain 

class OscarMDB:
    def __init__(self):

        # Create MDB subscribers
        rospy.Subscriber("mdb/ltm/executed_policy", String, self.policy_callback)
        rospy.Subscriber("mdb/oscar/control", ControlMsg, self.reset_world_callback)
        

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

        #Create perception subscribers



    def policy_callback(self, data):
        getattr(self, data.data + "_policy")()

        self.update_perceptions()
        self.update_reward()

        self.publish_perception()



    def reset_world_callback(self, data):
        rospy.loginfo("Reseting World...")

        self.update_perceptions()
        self.update_reward()
        self.publish_perception()
        







    def grasp_right_policy(self):
        rospy.loginfo("Executing grasp_right_policy...")
        rospy.sleep(1)


    def grasp_left_policy(self):
        rospy.loginfo("Executing grasp_left_policy...")
        rospy.sleep(1)

    def press_button_policy(self):
        rospy.loginfo("Executing press_button_policy...")
        rospy.sleep(1)

    def place_object_right_policy(self):
        rospy.loginfo("Executing place_object_right_policy...")
        rospy.sleep(1)

    def place_object_left_policy(self):
        rospy.loginfo("Executing place_object_left_policy...")
        rospy.sleep(1)

    def change_hands_policy(self):
        rospy.loginfo("Executing change_hands_policy...")
        rospy.sleep(1)


    def update_perceptions(self):
        rospy.loginfo("Updating Perceptions....")
        #Random perceptions for the moment
        

        self.red_object.data[0].distance=np.random.uniform(low=0.2,high=1.9)
        self.red_object.data[0].angle=np.random.uniform(low=-1.4,high=1.4)

        self.basket.data[0].distance=np.random.uniform(low=0.2,high=1.9)
        self.basket.data[0].angle=np.random.uniform(low=-1.4,high=1.4)

        if np.random.uniform(low=0, high=1)>0.5:
            self.obj_in_left_hand=True
        else:
            self.obj_in_left_hand=False

        if np.random.uniform(low=0, high=1)>0.5:
            self.obj_in_right_hand=True
        else:
            self.obj_in_right_hand=False



    def update_reward(self):
        rospy.loginfo("Reading Reward....")
        #Random reward for the moment
        if np.random.uniform(low=0, high=1)>0.85:
            self.reward=1
        else:
            self.reward=0



    def publish_perception(self):
        rospy.loginfo("Publishing Perceptions....")
        self.red_object_pub.publish(self.red_object)
        self.basket_pub.publish(self.basket)
        self.obj_in_left_hand_pub.publish(self.obj_in_left_hand)
        self.obj_in_right_hand_pub.publish(self.obj_in_right_hand)
        self.reward_pub.publish(self.reward)






    


             


    

if __name__ == "__main__":
    rospy.init_node('oscar_mdb_server', anonymous = False)
    rospy.loginfo("Started OSCAR MDB Server Node")
    server=OscarMDB()
    rospy.loginfo("OSCAR MDB Server Node: Setup Finished")
    rospy.spin()

    

