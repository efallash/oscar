#!/usr/bin/env python3



#ROS Imports
import sys, rospy, moveit_commander
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from sensor_msgs.msg import Image#, PointCloud2
from std_msgs.msg import Bool 
from control_msgs.msg import JointTrajectoryControllerState
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

#Service file import
from oscar_msgs.srv import Perception, PerceptionResponse

#Service server that provides the perceptual state

class OscarPerception:
    def __init__(self):
        #Service Server
        p_srv=rospy.Service("oscar/request_perceptions", Perception, self.perception_callback)
        rospy.Subscriber("camera/rgb/image_raw", numpy_msg(Image), self.dummy_callback) #Necessary for camera to be updated (REALLY UGLY)
        #Camera configuration
        self.configure_camera() 

        


    def configure_camera(self):
        # intrinsic camera parameters
        self.f = 3.04    # in mm
        self.sensor_size = (3.68, 2.76)    # in mm
        self.image_size = (3280, 2464)    # in px


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
        self.cam= ct.Camera(ct.RectilinearProjection(focallength_mm=self.f,
                                                sensor=self.sensor_size,
                                                image=self.image_size),
                    ct.SpatialOrientation(elevation_m=self.camera_elevation,tilt_deg=0))

    def dummy_callback(self, data):
        pass

    def perception_callback(self, req):
        rospy.loginfo("Waiting Image Data")
        image=rospy.wait_for_message("camera/rgb/image_raw", numpy_msg(Image))

        rospy.loginfo("Waiting Gripper Controller Data")
        left_gripper=rospy.wait_for_message("left_gripper_controller/state", JointTrajectoryControllerState)
        right_gripper=rospy.wait_for_message("right_gripper_controller/state", JointTrajectoryControllerState)

        obj, bskt=self.visual_processing(image)
        left, right=self.tactile_processing(left_gripper,right_gripper)

        response=PerceptionResponse()
        response.red_object=obj
        response.basket=bskt
        response.obj_in_left_hand=left
        response.obj_in_right_hand=right

        return response

    def visual_processing(self,data):
        

        im = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
    

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


        #MANUALLY Transforming from image frame to World frame 

        #Object
        obj=Point()
        obj.x=obj_pose_img[1]+0.425 #Translate 425mm
        obj.y=obj_pose_img[0]*-1 #Invert direction
        obj.z=0.8 #Pick Height

        #Basket
        bskt=Point()
        bskt.x=bskt_pose_img[1]+0.425 #Translate 425mm 
        bskt.y=obj_pose_img[0]*-1 #Invert direction
        bskt.z=0.8 #Pick Height

        return obj,bskt



    def tactile_processing(self, left_hand, right_hand):
        #Error over 5mm in the gripper's PID means an object is gripped
        left_error=[left_hand.error.positions[0]*-1,left_hand.error.positions[1]*-1] #Error is negative
        right_error=[right_hand.error.positions[0]*-1,right_hand.error.positions[1]*-1]

        #Average over the two teeth of the gripper
        left_avg=np.mean(left_error)
        right_avg=np.mean(right_error)

        if left_avg>0.005:
            left=True
        else:
            left=False

        if right_avg>0.005:
            right=True
        else:
            right=False

        return left, right


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


             


    

if __name__ == "__main__":
    rospy.init_node('oscar_perception_server', anonymous = False)
    rospy.loginfo("Started OSCAR Perception server node")
    server=OscarPerception()
    rospy.loginfo("OSCAR Perception Server Node: Setup Finished")
    rospy.spin()

    

