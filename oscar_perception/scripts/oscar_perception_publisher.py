#!/usr/bin/env python3

#    oscar_perception_publisher.py: Real-Time publishers of the frames of the objects in the scene
#    WARNING: Not fully implemented
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

#ROS Imports
import sys, rospy, moveit_commander
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool 
from control_msgs.msg import JointControllerState
import tf
import tf2_ros
import rospy
from rospy.numpy_msg import numpy_msg

#Vision Packages
import skimage
from skimage.measure import regionprops, label
#from skimage.io import imsave
import cameratransform as ct

#Math imports
import numpy as np
from math import pi


#MAIN CLASS
class OscarPerception:
    def __init__(self):

        # Create sensors subscribers

        #RGB Camera
        rospy.Subscriber("camera/rgb/image_raw", numpy_msg(Image), self.visual_processing)

        #Depth Camera #Not implemented
        #rospy.Subscriber("kinect/depth/points", Image, self.visual_processing_3d) #No implementado

        # Gripper subscribers #Not implemented
        #rospy.Subscriber("right_gripper_controller/state", JointControllerState, self.tactile_processing)
        #rospy.Subscriber("left_gripper_controller/state", JointControllerState, self.tactile_processing)

        # Create perception publishers

        #tf2 is used instead of these publishers
        #obj_pub = rospy.Publisher('oscar/perception/object', String, queue_size=1)
        #bskt_pub = rospy.Publisher('oscar/perception/basket', String, queue_size=1)

        #Visual publisher (Object coordinates)
        self.tf_br = tf2_ros.TransformBroadcaster() #Broadcasts the detected object frame 

        #Tactile perception publishers #NOT IMPLEMENTED
        #self.hand_left_pub = rospy.Publisher('oscar/perception/object_in_left_hand', Bool, queue_size=1)
        #self.hand_right_pub = rospy.Publisher('oscar/perception/object_in_right_hand', Bool, queue_size=1)





        #CAMERA DEFINITION 

        # intrinsic camera parameters
        f = 3.04    # in mm
        sensor_size = (3.68, 2.76)    # in mm
        image_size = (3280, 2464)    # in px


        #Extrinsic Parameters
        self.camera_elevation = 1.250 #Camera height measured from the table

        #Color Limits
        self.r=(0,5)
        self.g=(0,5)
        self.b=(0,5)
        self.obj_r=(200,255)
        self.bskt_b=(200,255)

        #Object height measured from the table (z=0)
        self.object_z=0.025 #Works for both object and basket
        
        #Camera
        self.cam=ct.Camera(ct.RectilinearProjection(focallength_mm=f,
                                                sensor=sensor_size,
                                                image=image_size),
                    ct.SpatialOrientation(elevation_m=self.camera_elevation,tilt_deg=0))

        #Define Generic Transform
        self.t = TransformStamped()

        #Placeholder header
        self.t.header.stamp = rospy.Time.now()
        self.t.header.frame_id = ""
        self.t.child_frame_id = ""
        #Placeholder translation
        self.t.transform.translation.x = 0
        self.t.transform.translation.y = 0
        self.t.transform.translation.z = 0.0

        #Rotation is ignored in visual 2D algorithm
        self.t.transform.rotation.x = 0
        self.t.transform.rotation.y = 0
        self.t.transform.rotation.z = 0
        self.t.transform.rotation.w = 1


    #This method is called at aprox 10Hz, uses a LOT of bandwidth
    #A real time processing algorithm must be implemented in C++ using
    #the ROS image transport http://wiki.ros.org/image_transport
    def visual_processing(self,data):
        

        im = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        
        # print("saving Image")
        # imsave("test.png",im)

        #Segment object and basket
        obj_bin =OscarPerception.color_segment_rgb(im,self.obj_r,self.g,self.b)
        bskt_bin =OscarPerception.color_segment_rgb(im,self.r,self.g,self.bskt_b)

        #Find object in image
        obj_find=OscarPerception.find_centroid(obj_bin, "red_box")
        bskt_find=OscarPerception.find_centroid(bskt_bin, "basket")

        #Get object poses in the image frame (Centered in the table, y pointing forward, x pointing right)

        #Object
        if obj_find[0]:
            obj_pose_img=self.cam.spaceFromImage([obj_find[1][0],obj_find[1][1]], Z=self.object_z)
        else:
            obj_pose_img=[1, 0.325, self.object_z] #If not found map to the  right corner of the table

        #Basket
        if bskt_find[0]:
            bskt_pose_img=self.cam.spaceFromImage([bskt_find[1][0],bskt_find[1][1]], Z=self.object_z)
        else:
            bskt_pose_img=[-1, 0.325, self.object_z] #If not found map to the left corner of the table

        #Publish Results in tf frames
        frame_time=rospy.Time.now() #Stamp time is generated AFTER processing (this could cause issues)

        #Object frame
        #Header
        self.t.header.stamp = rospy.Time.now()
        self.t.header.frame_id = data.header.frame_id
        self.t.child_frame_id = "perceived_object"
        #Object ranslation
        self.t.transform.translation.x = self.camera_elevation-self.object_z
        self.t.transform.translation.y = -obj_pose_img[0]
        self.t.transform.translation.z = obj_pose_img[1]
        #Publish transform
        self.tf_br.sendTransform(self.t)

        #Basket frame
        #Header
        self.t.child_frame_id = "perceived_basket"
        #Object ranslation
        self.t.transform.translation.x = self.camera_elevation-self.object_z
        self.t.transform.translation.y = -bskt_pose_img[0]
        self.t.transform.translation.z = bskt_pose_img[1]
        #Publish transform
        self.tf_br.sendTransform(self.t)


    def tactile_processing(self,data):
        pass


    @staticmethod        
    def color_segment_rgb(rgb_img, r_limits=(0,255), g_limits=(0,255), b_limits=(0,255)):


        r_img=rgb_img[:, :, 0]
        g_img=rgb_img[:, :, 1]
        b_img=rgb_img[:, :, 2]


        r_img_bin= ((r_img>=r_limits[0])&(r_img<=r_limits[1]))

        g_img_bin= ((g_img>=g_limits[0])&(g_img<=g_limits[1]))

        b_img_bin= ((b_img>=b_limits[0])&(b_img<=b_limits[1]))

        img_bin=(r_img_bin & g_img_bin & b_img_bin)

  

        return img_bin

    @staticmethod
    def find_centroid(img_bin, info_string):
        label_image=label(img_bin)
        region=regionprops(label_image) 
        
        num_region=len(region)
        small_region=False

        if len(region) == 1:
            if region[0].area > 2500:
                position=region[0].centroid
                return True, (position[1],position[0]) #Returns flag and image centroid
            else:
                small_region=True


        #Show warnings if object not found
        if num_region == 0:
            rospy.logwarn(f"Object {info_string} not found.")
        elif small_region:
            rospy.logerr(f"Object {info_string}: Small region found.")
        else:
            rospy.logfatal(f"Object {info_string}: Multiple regions found.")
        return False, (0,0)


             


    
#MAIN PROGRAM: Creates OscarPerception object and spins the thread
if __name__ == "__main__":
    rospy.init_node('oscar_perception_publisher', anonymous = False)
    rospy.loginfo("Started OSCAR Perception Publisher node")
    server=OscarPerception()
    rospy.loginfo("OSCAR Perception Publisher Node: Setup Finished")
    rospy.spin()

    

