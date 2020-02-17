#!/usr/bin/env python
import rospy
import numpy as np
from math import *#sin, cos, acos, asin, radians

from geometry_msgs.msg import Pose

import transform
from viewpoint import viewpoint_creator

# agent for determine next viewpoint based on current camera input
class viewpoint_agent:
    def __init__(self, camera, dist):
        self.camera = camera
        self.creator = viewpoint_creator(camera)
        self.dist = dist

    def current_viewpoint(self, current_pose):
        return self._create_viewpoint(current_pose, 'c')

    # return 'l', 'r', 'u', 'd'
    def next_viewpoint(self, current_pose):
        img_color = self.camera.color_image()
        img_depth = self.camera.depth_image()
        type = self._decision(img_color, img_depth)
        return self._create_viewpoint(current_pose, type)

    def _decision(self, color, depth):
        type = 'u'
        return type

    def _create_viewpoint(self,current,type):
        du = 0
        dv = 0
        offset = 50
        if type == 'l':
            du = -offset
        elif type == 'r':
            du = offset
        elif type == 'u':
            dv = -offset
        elif type == 'd':
            dv = offset

        return self.creator.compute_vp(du,dv,self.dist,self._current_transform(current))

    def _current_transform(self,current):
        q2c_mat = transform.quadrotor2camera(current[0],current[1])
        c2p_mat = transform.camera2pointcloud()
        return np.dot(q2c_mat,c2p_mat)


class trajectory_plan:
    def __init__(self, camera, dist):
        self.camera = camera
        self.vpagent = viewpoint_agent(camera,dist)
        self.trajectory = []
        self.index = -1
        self.dist = dist

    def reset(self, current):
        vp = self.vpagent.current_viewpoint(current)
        if vp != None:
            quadrotor, camera = transform.quadrotor_camera_pose(vp)
            self.trajectory.append([quadrotor, camera])
            self.index = 0
        else:
            print("unable to reset the uav position with invalid viewpoint")

    def completed(self):
        return self.index >= len(self.trajectory)

    def next_view(self):
        if self.completed():
            return None
        else:
            cp = self.trajectory[self.index]
            self.index = self.index + 1
            return cp

    def explore_views(self,current):
        vp = self.vpagent.next_viewpoint(current)
        if vp != None:
            quadrotor, camera = transform.quadrotor_camera_pose(vp)
            self.trajectory.append([quadrotor, camera])
            return True
        else:
            return False
