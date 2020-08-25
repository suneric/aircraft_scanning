#!/usr/bin/env python
import rospy
import numpy as np
import transform
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
    camera = realsense_d435()
    pc_capture = data_capture(camera,temp_folder)
    ugv_controller = ugv_arm_controller()
    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            key_input = raw_input("please enter 'space' for cature data:\n")
            if (key_input == ''):
                ugv_controller.move_to([0,1.57,0,0,0,0,1.57])
                #pc_capture.scan_and_save(mat)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
