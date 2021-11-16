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

"""
utility function of viewpoint
"""
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

def nearestViewpoint(pos, vps):
    x, y, z = pos[0],pos[1], pos[2]
    minDist = MAXVALUE
    idx = 0
    for i in range(len(vps)):
        vpx = vps[i].camera.position.x
        vpy = vps[i].camera.position.y
        vpz = vps[i].camera.position.z
        dist = sqrt((x-vpx)*(x-vpx)+(y-vpy)*(y-vpy)+(z-vpz)*(z-vpz))
        if dist < minDist:
            minDist = dist
            idx = i
    print("nearest viewpoint index is {} for pos ({},{},{})".format(i,x,y,z))
    return i

class ViewPointUtil(object):
    def __init__(self, vps, actDim=None, cn=0.3):
        self.voxels = set()
        self.viewpoints = []
        self.initialViewpoints(vps)
        self.nbMap = []
        self.actDim = actDim
        self.cn = cn

    def neighbors(self,vpIdx,size=0):
        if size == 0:
            return self.nbMap[vpIdx]
        else:
            return self.nbMap[vpIdx][0:size]

    def initialViewpoints(self,vps):
        self.originalvps = vps
        allVoxels = set()
        for vp in self.originalvps:
            allVoxels |= vp.voxels

        # update voxels for saving memory
        allVoxelsList = list(allVoxels)
        for vp in self.originalvps:
            newVoxels = set()
            for v in vp.voxels:
                vIdx = allVoxelsList.index(v)
                self.voxels.add(vIdx)
                newVoxels.add(vIdx)
            newVp = copy.deepcopy(vp)
            newVp.voxels = newVoxels
            self.viewpoints.append(newVp)
        return

    def originalViewpoint(self,vp):
        vpIdx = self.viewpoints.index(vp)
        return self.originalvps[vpIdx]

    """
    build a map for viewpoints
    """
    def buildNeighborMap(self):
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
