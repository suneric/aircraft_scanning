#!/usr/bin/env python
import rospy
import numpy as np
from rs_camera import rs_image
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from control_msgs.msg import JointControllerState
import pcl
from math import *

import os
import glob
import struct
import ctypes

#######################
class pc_capture:
    def __init__(self,camera):
        self.camera = camera
        self.temp_folder = "/home/yufeng/Temp/Scanning/"
        self._clean_temp_folder(self.temp_folder)
        self.quadrotor_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self._quadrotor_callback)
        self.camerapose_sub = rospy.Subscriber('uav_scanning/camera_pose_controller/state', JointControllerState, self._camerapose_callback)
        self.quadrotor_pose = None
        self.camera_pose = None

    ##################################################
    # scan and point could process
    # input point could, matrix of camera to global
    def scan_and_save(self,index):
        pc = self.camera.point_cloud()
        if pc == None:
            return
        pt = self.quadrotor_pose
        angle = self.camera_pose
        #print("quadrotor", pt)
        #print("camera joint", angle)
        # calculate transform mation from quadrotor position in global to camera frame
        mat = self._transform(pt, angle)
        #print(mat)
        cloud = self._cloud_process(pc,mat)
        if cloud != None:
            self._save_cloud(cloud,index)
        else:
            print("no data captured.")

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

    def _transform(self,pt,angle):
        half_PI = 0.5*3.1415926535897931
        # quadrotor pose in world frame
        mat0 = self._cartesian_to_matrix(pt)
        #print(mat0)
        # camera joint translation in base frame
        mat1 = np.array([[1,0,0,0.42],
                         [0,1,0,0],
                         [0,0,1,0], # 0.11 meter down the quadrotor
                         [0,0,0,1]])

        # rs435 camera rotation along y axis, tranlate along x axis
        mat2 = np.array([[cos(angle),0,sin(angle),0.0358], # 0.458 meter to the positive x
                         [0,1,0,0],
                         [-sin(angle),0,cos(angle),0],
                         [0,0,0,1]])

        rs_mat = np.dot(mat0,np.dot(mat1,mat2))

        # point cloud frame respect to rs435 frame
        mat_z = np.array([[cos(-half_PI),-sin(-half_PI),0,0],
                          [sin(-half_PI),cos(-half_PI),0,0],
                          [0,0,1,0],
                          [0,0,0,1]])
        mat_x = np.array([[1,0,0,0],
                          [0,cos(-half_PI),-sin(-half_PI),0],
                          [0,sin(-half_PI),cos(-half_PI), 0],
                          [0,0,0,1]])

        pc_mat = np.dot(mat_z,mat_x)
        return np.dot(rs_mat,pc_mat)

    def _quadrotor_callback(self, data):
        index = 2;
        if data.name[index]=="quadrotor":
            self.quadrotor_pose = data.pose[index]

    def _camerapose_callback(self, data):
        self.camera_pose = data.set_point

    def _cartesian_to_matrix(self,cp):
        position = cp.position
        orientation = cp.orientation
        matrix = np.eye(4)
        # translation
        matrix[0,3] = position.x# in meter
        matrix[1,3] = position.y
        matrix[2,3] = position.z
        # quaternion to matrix
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        Nq = w*w + x*x + y*y + z*z
        if Nq < 0.001:
            return matrix

        s = 2.0/Nq
        X = x*s
        Y = y*s
        Z = z*s
        wX = w*X; wY = w*Y; wZ = w*Z
        xX = x*X; xY = x*Y; xZ = x*Z
        yY = y*Y; yZ = y*Z; zZ = z*Z
        matrix=np.array([[1.0-(yY+zZ), xY-wZ, xZ+wY, position.x],
            [xY+wZ, 1.0-(xX+zZ), yZ-wX, position.y],
            [xZ-wY, yZ+wX, 1.0-(xX+yY), position.z],
            [0, 0, 0, 1]])
        return matrix

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

    def _save_cloud(self,cloud,index):
        file = self.temp_folder+"point_"+str(index)+".ply"
        #print("save point cloud to ",file)
        pcl.save(cloud,file,"ply")

    def _bbox(self):
        ps = self.camera.pixel_size()
        f = self.camera.camera_focal()
        dist = self.camera.center_distance()
        frame = self.camera.image_size()
        x = dist*frame[0]*ps/f[0]
        y = dist*frame[1]*ps/f[1]
        return np.array([-x,x,-y,y,0.8*dist,1.2*dist])

    def _clean_temp_folder(self,temp_folder):
        files = glob.glob(temp_folder+"*")
        for f in files:
            print("remove ", f)
            os.remove(f)

if __name__ == '__main__':
    rospy.init_node("uav_scanning_data", anonymous=True, log_level=rospy.INFO)
    camera = rs_image()
    data = pc_capture(camera)
    rate = rospy.Rate(10)
    index = 1
    try:
        while not rospy.is_shutdown():
            key_input = raw_input("please enter 'space' for cature data:\n")
            if (key_input == ''):
                data.scan_and_save(index)
                print("data save no. ", index, " data")
                index = index+1

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
