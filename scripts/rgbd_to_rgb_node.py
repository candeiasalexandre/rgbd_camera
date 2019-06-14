#!/usr/bin/env python

#####################################
#DEBUG NODE
#####################################
import rospy
import cv2

from cv_bridge import CvBridge, CvBridgeError


from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from rgbd_camera.msg import RgbdImage


rgb_image_publisher = None
depth_image_publisher = None
bridge = CvBridge()

def rgbd_callback(data):

    depth_image_publisher.publish(data.depth)
    rgb_image_publisher.publish(data.rgb)


if __name__ == "__main__":
    rospy.init_node('rgbd_to_rgb_depth_node', anonymous=True)
    

    rgbd_image_topic_name = rospy.get_param('~rgbd_topic_name', "/rgbd_image")
    prefix = rospy.get_param('~prefix_name', 'rgbd_to_color_depth')

    rgb_image_topic_name = prefix + "/color/image_raw"
    depth_image_topic_name = prefix + "/depth/image_raw"
    
    rgb_image_publisher = rospy.Publisher(rgb_image_topic_name, Image, queue_size=10)
    depth_image_publisher = rospy.Publisher(depth_image_topic_name, Image, queue_size=10)


    rgbd_subs = rospy.Subscriber(rgbd_image_topic_name, RgbdImage, rgbd_callback)

    rospy.spin()