#! /usr/bin/env python
import sys
import os
import numpy as np
from math import *
from geometry_msgs.msg import Pose
from viewpoint_map import ViewPoint, ViewPointMap

class ScanningUtil(object):
    def __init__(self, action_dim):
        self.viewpoints = [] # viewpoint
        self.voxels = [] # voxel indices
        self.action_dim = action_dim # -1 means no limited

    def neighbors(self, vpIdx, vpsState):
        # rule 1 find neighbors in unvisited vps
        # rule 2 find neighbors with smallest overlap
        # rule 3 find neighbors with shortest distance
        vp = self.viewpoints[vpIdx]
        view = vp.view()
        unvisited = np.where(np.asarray(vpsState) == 0)[0]
        if len(unvisited) <= self.action_dim:
            return unvisited

        overlapList, unoverlapList = [],[]
        overlapValues, unoverlapDist = [],[]
        for i in unvisited:
            vpi = self.viewpoints[i]
            viewi = vpi.view()
            common = list(set(view) & set(viewi))
            overlap = float(len(common))
            if (overlap/len(view)) > 0:
                overlapList.append(i)
                overlapValues.append(overlap)
            else:
                unoverlapList.append(i)
                unoverlapDist.append(self.distance_i(vpIdx,i))

        neighbors = []
        if len(overlapList) >= self.action_dim:
            for i in range(self.action_dim):
                idx = np.argmin(overlapValues)
                neighbors.append(overlapList[idx])
                del overlapList[idx]
                del overlapValues[idx]
        else:
            neighbors = overlapList[:]
            for i in range(len(overlapList)-1,self.action_dim):
                idx = np.argmin(unoverlapDist)
                neighbors.append(unoverlapList[idx])
                del unoverlapList[idx]
                del unoverlapDist[idx]
        return neighbors

    def nearest(self, vp):
        distList = [self.distance(vp,self.viewpoints[i]) for i in range(len(self.viewpoints))]
        return np.argmin(distList)

    def distance_i(self, vpIdx1, vpIdx2):
        vp1 = self.viewpoints[vpIdx1]
        vp2 = self.viewpoints[vpIdx2]
        return self.distance(vp1,vp2)

    def distance(self, vp1, vp2):
        x1 = vp1.quadrotor().position.x;
        y1 = vp1.quadrotor().position.y;
        z1 = vp1.quadrotor().position.z;
        x2 = vp2.quadrotor().position.x;
        y2 = vp2.quadrotor().position.y;
        z2 = vp2.quadrotor().position.z;
        return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2))

    def load(self,file):
        with open(file,'r') as reader:
            for line in reader.read().splitlines():
                data = line.split(" ")
                idx = int(data[0])
                px = float(data[1])
                py = float(data[2])
                pz = float(data[3])
                ox = float(data[4])
                oy = float(data[5])
                oz = float(data[6])
                ow = float(data[7])
                angle = float(data[8])
                vp = ViewPoint(idx,px,py,pz,ox,oy,oz,ow,angle)
                voxels = data[9:len(data)]
                # build voxels list
                for v in voxels:
                    vxIdx = int(v)
                    if vxIdx not in self.voxels:
                        self.voxels.append(vxIdx)
                    vp.add_voxel(self.voxels.index(vxIdx))
                # build viewpoints list
                self.viewpoints.append(vp)
        reader.close()
        print("viewpoints count:", len(self.viewpoints), "voxels count:", len(self.voxels))

    # save trajectory
    def save(self,trajectory,file):
        with open(file, 'w') as writer:
            for vpIdx in trajectory:
                vp = self.viewpoints[vpIdx]
                idx = vp.index()
                pos = vp.quadrotor()
                angle = vp.camera()
                voxels = vp.view()

                line = str(idx) + " "\
                     + str(pos.position.x) + " "\
                     + str(pos.position.y) + " "\
                     + str(pos.position.z) + " "\
                     + str(pos.orientation.x) + " "\
                     + str(pos.orientation.y) + " "\
                     + str(pos.orientation.z) + " "\
                     + str(pos.orientation.w) + " "\
                     + str(angle) + " "
                # voxels index of viewpoint
                for i in range(len(voxels)):
                    line = line + str(self.voxels[voxels[i]]) + " "
                line += "\n"
                #print(line)
                writer.write(line)
        writer.close()
