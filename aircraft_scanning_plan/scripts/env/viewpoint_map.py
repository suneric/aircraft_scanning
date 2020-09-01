#! /usr/bin/env python
import numpy as np
from math import *
from geometry_msgs.msg import Pose

class ViewPoint(object):
    def __init__(self,idx,px,py,pz,ox,oy,oz,ow):
        self.idx = idx
        self.cam_pose = Pose();
        self.cam_pose.position.x = px
        self.cam_pose.position.y = py
        self.cam_pose.position.z = pz
        self.cam_pose.orientation.x = ox
        self.cam_pose.orientation.y = oy
        self.cam_pose.orientation.z = oz
        self.cam_pose.orientation.w = ow
        self.voxels = []
    def add_voxel(self, voxel_idx):
        if voxel_idx not in self.voxels:
            self.voxels.append(voxel_idx)
    # viewpoint index
    def index(self):
        return self.idx
    # the pose of quadrotor
    def camera(self):
        return self.cam_pose
    # voxels index in the camera view
    def view(self):
        return self.voxels

# ViewPoint Map for short distance
class ViewPointMap(object):
    def __init__(self,viewpoints):
        self.distmap = self.build_distmap(viewpoints)
    # get number of count neighbors vp indices with meauraing the distance
    def neighbors(self,vpIdx,count,vp_list):
        # return all the vp_list if its size is less than required count
        dist_vps = self.distmap[vpIdx]
        dists = [dist_vps[i] for i in vp_list]
        sorted_dist = [dists[i] for i in range(len(dists))]
        sorted_dist.sort()
        size = count
        if size > len(sorted_dist):
            size = len(sorted_dist)
        sorted_subset = sorted_dist[0:size]

        neighbor_vps = []
        for dist in sorted_subset:
            neighbor_vps.append(vp_list[dists.index(dist)])
        #print("index", vpIdx,"availble",vp_list, "neighbor", neighbor_vps)
        return neighbor_vps

    # build a distance map
    def build_distmap(self,viewpoints):
        size = len(viewpoints)
        # don't use arr=[[0]*size]*size as the arr[0] and arr[1] refer to the same object
        distmap = [[0 for i in range(size)] for j in range(size)]
        for i in range(size):
            vp1 = viewpoints[i]
            for j in range(size):
                vp2 = viewpoints[j]
                distmap[i][j] = self.distance(vp1,vp2)
        return distmap

    # distance of two viewpoints
    def distance(self, vp1, vp2):
        x1 = vp1.camera().position.x;
        y1 = vp1.camera().position.y;
        z1 = vp1.camera().position.z;
        x2 = vp2.camera().position.x;
        y2 = vp2.camera().position.y;
        z2 = vp2.camera().position.z;
        return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2))
