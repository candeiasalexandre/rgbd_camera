#!/usr/bin/env python
import rospy
import sys

from rgbd_camera.rgbd_camera_acquisition_logic import RGBDCameraAcquisition


if __name__ == "__main__":
    rospy.init_node('rgbd_camera', anonymous=True)

    argvs = rospy.myargv(argv=sys.argv)


    #test get camera intrinsics 
    camera_1 = RGBDCameraAcquisition(depth_camera_info_topic=argvs[1], 
                            rgb_camera_info_topic=argvs[2],
                            rgb_image_topic=argvs[3],
                            depth_image_topic=argvs[4])
    
    #print(test_RGBD.rgb_camera_intrinsics)
    #print(test_RGBD.depth_camera_intrinsics)

    camera_1.start_acquisition()
    rospy.spin()