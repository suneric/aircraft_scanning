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
        self.neighborMap = []

    def neighbors(self,vpIdx):
        return self.neighborMap[vpIdx]

    def _build_neighbor_map(self):
        map = []
        for i in range(len(self.viewpoints)):
            neighbors = self._search_neighbors(i)
            map.append(neighbors)
            #print(i, neighbors)
        return map

    def _search_neighbors(self, vpIdx):
        # rule 1 find neighbors in unvisited vps
        # rule 2 find neighbors with smallest overlap
        # rule 3 find neighbors with shortest distance
        vp = self.viewpoints[vpIdx]
        view = vp.view()
        vps_search = [i for i in range(len(self.viewpoints))]
        overlapList, unoverlapList = [],[]
        overlapValues, unoverlapDist = [],[]
        for i in vps_search:
            if i == vpIdx:
                continue
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
        #print(overlapList)
        neighbors = []
        size = self.action_dim
        overlapSize = len(overlapList)
        if overlapSize >= size:
            neighbors[0:size]=self.getMinValues(overlapList,overlapValues,size)[:]
        elif overlapSize < size and overlapSize > 0:
            neighbors[0:overlapSize]=self.getMinValues(overlapList,overlapValues,overlapSize)[:]
            neighbors[overlapSize:size]=self.getMinValues(unoverlapList,unoverlapDist,size-overlapSize)[:]
        elif overlapSize == 0:
            neighbors[0:size] = self.getMinValues(unoverlapList,unoverlapDist,size)[:]
        # half of the neighbors are overlapped vp and largest distance
        # size = int(0.5*self.action_dim)
        # if len(overlapList) >= size and len(unoverlapList) >= size:
        #     neighbors[0:size] = self.getMinValues(overlapList,overlapValues,size)[:]
        #     neighbors[size:2*size] = self.getMinValues(unoverlapList,unoverlapDist,size)[:]
        # elif len(overlapList) >= size and len(unoverlapList) < size:
        #     minSize = len(unoverlapList)
        #     neighbors[0:2*size-minSize] = self.getMinValues(overlapList,overlapValues,2*size-minSize)[:]
        #     neighbors[2*size-minSize:2*size] = self.getMinValues(unoverlapList,unoverlapDist,minSize)[:]
        # elif len(overlapList) < size and len(unoverlapList) >= size:
        #     minSize = len(overlapList)
        #     neighbors[0:minSize] = self.getMinValues(overlapList,overlapValues,minSize)[:]
        #     neighbors[minSize:2*size] = self.getMinValues(unoverlapList,unoverlapDist,2*size-minSize)[:]
        #print(neighbors)
        return neighbors

    def getMinValues(self, itemList, valueList, size):
        if (len(itemList) == size):
            return itemList
        else:
            minValueItem = []
            for _ in range(size):
                idx = np.argmin(valueList)
                minValueItem.append(itemList[idx])
                del itemList[idx]
                del valueList[idx]
            return minValueItem


    def nearest(self, vp):
        distList = [self.distance(vp,self.viewpoints[i]) for i in range(len(self.viewpoints))]
        return np.argmin(distList)

    def distance_i(self, vpIdx1, vpIdx2):
        vp1 = self.viewpoints[vpIdx1]
        vp2 = self.viewpoints[vpIdx2]
        return self.distance(vp1,vp2)

    def distance(self, vp1, vp2):
        x1 = vp1.camera().position.x;
        y1 = vp1.camera().position.y;
        z1 = vp1.camera().position.z;
        x2 = vp2.camera().position.x;
        y2 = vp2.camera().position.y;
        z2 = vp2.camera().position.z;
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
                vp = ViewPoint(idx,px,py,pz,ox,oy,oz,ow)
                voxels = data[8:len(data)]
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
        self.neighborMap = self._build_neighbor_map()

    # save trajectory
    def save(self,trajectory,file):
        with open(file, 'w') as writer:
            for vpIdx in trajectory:
                vp = self.viewpoints[vpIdx]
                idx = vp.index()
                pos = vp.camera()
                voxels = vp.view()

                line = str(idx) + " "\
                     + str(pos.position.x) + " "\
                     + str(pos.position.y) + " "\
                     + str(pos.position.z) + " "\
                     + str(pos.orientation.x) + " "\
                     + str(pos.orientation.y) + " "\
                     + str(pos.orientation.z) + " "\
                     + str(pos.orientation.w) + " "
                # voxels index of viewpoint
                for i in range(len(voxels)):
                    line = line + str(self.voxels[voxels[i]]) + " "
                line += "\n"
                #print(line)
                writer.write(line)
        writer.close()
