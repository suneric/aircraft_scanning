#!/usr/bin/env python
import rospy
import numpy as np
from math import *
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import sensor_msgs.point_cloud2 as pc2
import transform
from uav_trajectory import trajectory_defined
from uav_controller import uav_cam_controller

from sensor.camera import realsense_d435
from sensor.image import depth_img, color_img
from sensor.pointcloud import data_capture

import time
import os
import glob
import struct
import ctypes

class auto_scanning:
    def __init__(self,controller,camera,data_processor):
        self.controller = controller
        self.camera = camera
        self.data = data_processor

        self.takeoff = False
        self.landing = False
        self.status = 'ready' # 'completed', 'ready'

        self.trajectory = trajectory_defined()
        self.ts = 0
        self.te = 0
        self.dataPub = rospy.Publisher('uav_scanning/pointcloud', PointCloud2, queue_size=1)
        self.posePub = rospy.Publisher('uav_scanning/viewpoint', Float64MultiArray, queue_size=1)

    def is_takeoff(self):
        return self.takeoff
    def is_landing(self):
        return self.landing

    def start(self):
        self.ts = time.clock()
        self.controller.takeoff(self._takeoff_callback)

    def terminate(self):
        self.te = time.clock()
        self.controller.landing()
        self.landing == True
        print("Operation time {:.3f} secs".format(self.te-self.ts))

    def fly(self):
        while True:
            if self.status == 'ready':
                self.status = 'flying'
                self._fly_to_next(self.trajectory.next_view())
            elif self.status == 'completed':
                self.terminate()

    def _fly_to_next(self, pose):
        if pose == None:
            self.status = 'completed'
            return
        print("flying to ", pose[0].position.x, pose[0].position.y, pose[0].position.z, "camera joint", pose[1])
        self.controller.execute_camera_joint(pose[1])
        self.controller.execute_quadrotor_pose(pose[0], self._fly_callback)

    def _fly_callback(self):
        print("reached.")
        mat = self.controller.transform_q2c()
        pc = self.camera.point_cloud()
        self.data.scan_and_save(pc,mat) # save
        # publish
        if pc != None:
            vp = Float64MultiArray()
            vp.layout.dim.append(MultiArrayDimension())
            vp.layout.dim.append(MultiArrayDimension())
            vp.layout.dim[0].size=4
            vp.layout.dim[1].size=4
            vp.layout.dim[0].stride=4*4
            vp.layout.dim[1].stride=4
            vp.layout.data_offset=0
            vp.data = mat.flatten().tolist()[0]
            self.posePub.publish(vp)
            self.dataPub.publish(pc)
        self.status = 'ready'

    def _takeoff_callback(self):
        print("takeoff.")
        pose = self._initial_pose(20,-30,5,0.5*pi,0)
        self.controller.execute_quadrotor_pose(pose, self._initial_callback)

    def _initial_callback(self):
        print("initialized.")
        self.takeoff = True

    def _initial_pose(self,x,y,z,yaw,angle):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        qw,qx,qy,qz = transform.eularangle_to_quaternion(yaw, 0.0, 0.0)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        self.controller.execute_camera_joint(angle)
        return pose

# main
if __name__ == '__main__':
    # clean folder for save point cloud file
    temp_folder = "/home/yufeng/Temp/Scanning/"
    files = glob.glob(temp_folder+"*")
    for f in files:
        print("remove ", f)
        os.remove(f)

    # initialize ros node
    rospy.init_node("uav_auto_scanning", anonymous=True, log_level=rospy.INFO)
    rospy.sleep(2)
    controller = uav_cam_controller()
    camera = realsense_d435()
    pc_capture = data_capture(camera,temp_folder)
    auto = auto_scanning(controller, camera, pc_capture)
    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            if auto.is_takeoff() == False:
                rospy.sleep(1)
                auto.start()
            else:
                key_input = raw_input("press enter for start autonomous flying:\n")
                if (key_input == ''):
                    if auto.is_landing() == False:
                        auto.fly()

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
