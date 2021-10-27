#! /usr/bin/env python
import sys
import os
import numpy as np
from math import *
from geometry_msgs.msg import Pose
import copy

MAXVALUE = 1000000

"""
ViewPoint for camere position with voxels in the view
"""
class ViewPoint(object):
    def __init__(self,id,px,py,pz,ox,oy,oz,ow):
        self.id = id
        self.voxels = set() # grid / voxels in view
        self.camera = self.cameraPose(px,py,pz,ox,oy,oz,ow)

    def cameraPose(self,px,py,pz,ox,oy,oz,ow):
        camera = Pose()
        camera.position.x = px
        camera.position.y = py
        camera.position.z = pz
        camera.orientation.x = ox
        camera.orientation.y = oy
        camera.orientation.z = oz
        camera.orientation.w = ow
        return camera

def vpDistance(vp1, vp2):
    x1 = vp1.camera.position.x;
    y1 = vp1.camera.position.y;
    z1 = vp1.camera.position.z;
    x2 = vp2.camera.position.x;
    y2 = vp2.camera.position.y;
    z2 = vp2.camera.position.z;
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2))

def vpOverlap(vp1, vp2):
    intersect = vp1.voxels & vp2.voxels
    return len(intersect)

class ViewPointUtil(object):
    def __init__(self, vps, actDim=None, cn=0.3):
        self.viewpoints = vps
        self.voxelMaxId = 0
        self.voxels = set()
        self.nbMap = []
        self.actDim = actDim
        self.cn = cn

    def neighbors(self,vpIdx,size=0):
        if size == 0:
            return self.nbMap[vpIdx]
        else:
            return self.nbMap[vpIdx][0:size]

    """
    build a map for viewpoints
    """
    def buildNeighborMap(self):
        # get all voxel
        for vp in self.viewpoints:
            self.voxels |= vp.voxels
        self.voxelMaxId = max(self.voxels)

        print("=== start building neighbor map ===".format())
        dim = len(self.viewpoints)
        self.nbMap = [None]*dim
        for i in range(dim):
            nbvps = self.searchNeighbor(i,self.viewpoints,self.cn)
            self.nbMap[i] = nbvps
        print("=== end building neighbor map ===".format())
        return self.nbMap

    """
    search neighbor viewpoints of a given viewpoint
    considering the distance and overlap
    """
    def searchNeighbor(self,i,vps,cn):
        dim = len(vps)
        vp = vps[i]
        scoreList = [None]*dim
        for j in range(dim):
            vp1 = vps[j]
            if vp1.id == vp.id:
                scoreList[j] = MAXVALUE
            else:
                dist = vpDistance(vp, vp1)
                overlap = vpOverlap(vp, vp1)
                scoreList[j]=cn*dist + (1.0-cn)*overlap
        # return the viewpoint indices with least value
        sortedIndice = np.argsort(np.array(scoreList))
        # sorted = np.lexsort((range(dim), scoreList))[0:self.actDim]
        return sortedIndice
