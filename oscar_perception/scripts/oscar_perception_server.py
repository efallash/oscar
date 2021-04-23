#!/usr/bin/env python3



#ROS Imports
import sys, rospy, moveit_commander
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool 
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

#Service servers to generate visual and tactile perceptions

class OscarPerception:
    def __init__(self):

        # Create sensors subscribers

        #RGB Camera
        rospy.Subscriber("camera/rgb/image_raw", numpy_msg(Image), self.visual_processing)

        #Depth Camera
        #rospy.Subscriber("kinect/depth/points", Image, self.visual_processing_3d) #No implementado

        # Gripper subscribers


        # Create perception publishers

        #En vez de estos se utiliza el tf publisher
        #obj_pub = rospy.Publisher('oscar/perception/object', String, queue_size=1)
        #bskt_pub = rospy.Publisher('oscar/perception/basket', String, queue_size=1)

        #Tactile perception publishers
        self.hand_left_pub = rospy.Publisher('oscar/perception/object_in_left_hand', Bool, queue_size=1)
        self.hand_right_pub = rospy.Publisher('oscar/perception/object_in_right_hand', Bool, queue_size=1)

        #Visual publisher (Object coordinates)
        self.tf_br = tf2_ros.TransformBroadcaster() #Broadcasts the detected object frame 



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
        self.cam=cam = ct.Camera(ct.RectilinearProjection(focallength_mm=f,
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
        frame_time=rospy.Time.now() #Stamp time AFTER processing,this could cause issues

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


        #rospy.loginfo(f"Object: ({obj_pose_img[0]},{obj_pose_img[1]},{obj_pose_img[2]})")
        #rospy.loginfo(f"Basket: ({bskt_pose_img[0]},{bskt_pose_img[1]},{bskt_pose_img[2]})")


        


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


             


    

if __name__ == "__main__":
    rospy.init_node('oscar_perception_server', anonymous = False)
    rospy.loginfo("Started OSCAR perception server node")
    server=OscarPerception()
    rospy.loginfo("OSCAR Perception Server Node: Setup Finished")
    rospy.spin()

    

