#!/usr/bin/env python
import rospy
import numpy as np
from math import *

import transform
import pcl

#######################
class data_capture:
    def __init__(self, camera, filepath):
        self.camera = camera
        self.ratio = self._frame_ratio()
        self.filepath = filepath
        self.index = 0

    ##################################################
    # scan and point could process
    # input point could, matrix of camera to global
    # pose: [quadrotor_pose, camera_joint]
    def scan_and_save(self,pose):
        pc = self.camera.point_cloud()
        if pc == None:
            return

        # transform of point cloud to world frame
        q2c_mat = transform.quadrotor2camera(pose[0], pose[1])
        c2p_mat = transform.camera2pointcloud()
        mat = np.dot(q2c_mat,c2p_mat)

        #print(mat)
        cloud = self._cloud_process(pc,mat)
        if cloud != None:
            self._save_cloud(cloud)
        else:
            print("no data captured.")

    ### private functions
    def _cloud_process(self,cloud,mat):
        # filter out the point outside of the frame and depth
        point_list = []
        for data in cloud:
            x,y,z,rgb=data[:4]
            if self._in_box(x,y,z,self._bbox()):
                tp = np.dot(mat,np.array([x,y,z,1])) # transform point
                point_list.append([tp[0],tp[1],tp[2],rgb])
        if len(point_list) > 0:
            pcl_cloud = pcl.PointCloud_PointXYZRGB()
            pcl_cloud.from_list(point_list)
            return pcl_cloud
        else:
            return None

    def _save_cloud(self,cloud):
        file = self.filepath+"point_"+str(self.index)+".ply"
        pcl.save(cloud,file,"ply")
        self.index = self.index + 1

    def _in_box(self,x,y,z, bbox):
        if x < bbox[0]:
            return False
        elif x > bbox[1]:
            return False
        elif y < bbox[2]:
            return False
        elif y > bbox[3]:
            return False
        elif z < bbox[4]:
            return False
        elif z > bbox[5]:
            return False
        else:
            return True

    def _frame_ratio(self):
        ps = self.camera.pixel_size()
        f = self.camera.camera_focal()
        frame = self.camera.image_size()
        x_ratio = frame[0]*ps/f[0]
        y_ratio = frame[1]*ps/f[1]
        return [x_ratio,y_ratio]

    def _bbox(self):
        dist = self.camera.center_distance()
        x = dist*self.ratio[0]
        y = dist*self.ratio[1]
        return np.array([-x,x,-y,y,0.5*dist,1.5*dist])

# if __name__ == '__main__':
#     rospy.init_node("uav_scanning_data", anonymous=True, log_level=rospy.INFO)
#     camera = realsense_d435()
#     data = data_capture(camera)
#     rate = rospy.Rate(10)
#     index = 1
#     try:
#         while not rospy.is_shutdown():
#             key_input = raw_input("please enter 'space' for cature data:\n")
#             if (key_input == ''):
#                 data.scan_and_save(index)
#                 print("data save no. ", index, " data")
#                 index = index+1
#
#             rate.sleep()
#     except rospy.ROSInterruptException:
#         pass
