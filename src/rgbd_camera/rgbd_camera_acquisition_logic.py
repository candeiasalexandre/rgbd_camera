#!/usr/bin/env python
import rospy
import message_filters
import time

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from rgbd_camera.msg import RgbdImage

def getROSCameraParameters(ros_topic_name):
        #doesn't work without initNode first

        camera_info_message = rospy.wait_for_message(ros_topic_name, CameraInfo, timeout=None)
        return camera_info_message.K

class RGBDCameraAcquisition:
    #Class that implements the logic for acquiring two streams sync (depth and rgb) and put it together in a single image
    #We assume that rgb and depth topic are already aligned

    def __init__(self, depth_camera_info_topic, rgb_camera_info_topic, rgb_image_topic, depth_image_topic):
        #doesn't work without initNode first

        self.depth_camera_intrinsics = getROSCameraParameters(depth_camera_info_topic)
        self.rgb_camera_intrinsics = getROSCameraParameters(rgb_camera_info_topic)
        
        self.rgb_image_subs = message_filters.Subscriber(rgb_image_topic, Image)
        self.depth_image_subs = message_filters.Subscriber(depth_image_topic, Image)
        self.rgbd_sync_subs = message_filters.ApproximateTimeSynchronizer([self.rgb_image_subs, self.depth_image_subs], 1, 0.01)


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