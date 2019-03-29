#!/usr/bin/env python
import rospy
import message_filters
import time
import numpy as np

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from rgbd_camera.msg import RgbdImage

def getROSCameraParameters(ros_topic_name):
    #doesn't work without initNode first

    camera_info_message = rospy.wait_for_message(ros_topic_name, CameraInfo, timeout=None)
    return camera_info_message.K

def get_xyz_from_rgbd(image_2d_pos, depth_image, depth_camera_matrix):
    """
        from a numpy array of N 2d image_positions ( in a 2 by N numpy array), a numpy depth image and the camera matrix
        returns a numpy array of the 3d_positions N by 3
    """
    pos_3D = []
    for image_pos in image_2d_pos.T:
        #print(image_pos)
        if image_pos[0] < depth_image.shape[1] and image_pos[1] < depth_image.shape[0] and image_pos[0] > 0 and image_pos[1] > 0:
            z = depth_image[image_pos[1], image_pos[0]]
            y = ( image_pos[1] - depth_camera_matrix[1,2] ) * (z / depth_camera_matrix[1,1])
            x = ( image_pos[0] - depth_camera_matrix[0,2] ) * (z / depth_camera_matrix[0,0])
        else:
            z = 0
            y = 0
            x = 0
        pos_3D.append([x, y, z])
    
    return np.transpose(np.asarray(pos_3D))

    

class RGBDCameraAcquisition:
    #Class that implements the logic for acquiring two streams sync (depth and rgb) and put it together in a single image
    #We assume that rgb and depth topic are already aligned

    def __init__(self, depth_camera_info_topic, rgb_camera_info_topic, rgb_image_topic, depth_image_topic):
        #doesn't work without initNode first

        self.depth_camera_intrinsics = getROSCameraParameters(depth_camera_info_topic)
        self.rgb_camera_intrinsics = getROSCameraParameters(rgb_camera_info_topic)
        
        self.rgb_image_subs = message_filters.Subscriber(rgb_image_topic, Image)
        self.depth_image_subs = message_filters.Subscriber(depth_image_topic, Image)
        self.rgbd_sync_subs = message_filters.ApproximateTimeSynchronizer([self.rgb_image_subs, self.depth_image_subs], 1, 0.02)


        self.rgbd_pub = rospy.Publisher('rgbd_image', RgbdImage, queue_size=1)
        
    def images_callback(self, rgb_image_msg, depth_image_msg):
        #print("RGB_header: ", rgb_image_msg.header)
        #print("DEPTH_header: ", depth_image_msg.header)
        rgbd_image = RgbdImage()
        rgbd_image.header = rgb_image_msg.header
        rgbd_image.K_rgb = self.rgb_camera_intrinsics
        rgbd_image.K_depth = self.depth_camera_intrinsics
        rgbd_image.rgb = rgb_image_msg
        rgbd_image.depth = depth_image_msg
        
        self.rgbd_pub.publish(rgbd_image)

    def start_acquisition(self):
        self.rgbd_sync_subs.registerCallback(self.images_callback)