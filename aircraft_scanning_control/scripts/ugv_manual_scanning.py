#!/usr/bin/env python
import rospy
import numpy as np
from ugv_controller import ugv_arm_controller
from sensor.pointcloud import data_capture
from sensor.camera import realsense_d435
import os
import glob
import struct
import ctypes

if __name__ == '__main__':
    # clean folder for save point cloud file
    temp_folder = "/home/yufeng/Temp/Scanning/"
    files = glob.glob(temp_folder+"*")
    for f in files:
        print("remove ", f)
        os.remove(f)

    rospy.init_node("ugv_manual_scannig", anonymous=True, log_level=rospy.INFO)
    rospy.sleep(1) # wait for other node ready, such a gazebo
    camera = realsense_d435()
    pc_capture = data_capture(camera,temp_folder)
    controller = ugv_arm_controller()
    rospy.sleep(1) #
    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            if not controller.is_initialized():
                controller.initialize()

            key_input = raw_input("press 'enter' for cature data:\n")
            if (key_input == ''):
                mat = controller.transform_m2c()
                pc_capture.scan_and_save(mat)

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
