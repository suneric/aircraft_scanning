#! /usr/bin/env python
import sys
import os
import numpy as np
from math import *
from geometry_msgs.msg import Pose
from viewpoint_map import ViewPoint, ViewPointMap
from viewpoint_util import ScanningUtil

class UAVScanningState(object):
    def __init__(self,util,vpIdx,vpsState,vxsState):
        self.util = util
        self.vpIdx = vpIdx
        self.vpsState = vpsState
        self.vxsState = vxsState
        self.vpsNeighbor = self.util.neighbors(self.vpIdx,self.vpsState)

    # current viewpoint index
    def vp(self):
        return self.vpIdx

    # current viewpoint neighbor indices
    def neighbors(self):
        return self.vpsNeighbor

    def coverage(self):
        visited = np.asarray(np.nonzero(self.vxsState)).flatten()
        return float(len(visited))/float(len(self.vxsState))

    def move(self, vpIdx):
        # copy states
        vpsState = [self.vpsState[i] for i in range(len(self.vpsState))]
        vxsState = [self.vxsState[i] for i in range(len(self.vxsState))]

        # update states
        vpsState[vpIdx] = 1
        vp = self.util.viewpoints[vpIdx]
        voxels = vp.view()
        for v in voxels:
            vxsState[v] = 1

        return UAVScanningState(self.util,vpIdx,vpsState,vxsState)

# envrionment
class UAVScanningEnv(object):
    def __init__(self,file,startVp,action_dim=8):
        self.util = ScanningUtil(action_dim)
        self.util.load(file)
        self.state = None
        self.visited_vps = []
        self.start_vp = startVp

    def utility(self):
        return self.util

    def save(self, file):
        self.util.save(self.visited_vps,file)

    def reset(self):
        self.visited_vps = []
        vpsState = [0 for i in range(len(self.util.viewpoints))]
        vxsState = [0 for i in range(len(self.util.voxels))]
        state = UAVScanningState(self.util,-1,vpsState,vxsState)
        vpIdx = self.util.nearest(self.start_vp)
        self.state = state.move(vpIdx)
        self.visited_vps.append(vpIdx)
        return self.state

    def state(self):
        return self.state

    def play(self, vpNext):
        cState = self.state
        vpCurrent = cState.vp()
        cCoverage = cState.coverage()
        nState = cState.move(vpNext)
        nCoverage = nState.coverage()
        travelDist = self.util.distance_i(vpCurrent, vpNext)
        self.state = nState
        self.visited_vps.append(vpNext)

        done = (nCoverage == 1.0)
        step_penalty = -0.1*travelDist
        step_reward = 100*(nCoverage-cCoverage)
        done_reward = done*100
        reward = step_penalty + step_reward + done_reward
        #print("rewards:", step_penalty, step_reward, done_reward)
        return done, reward, nCoverage, nState

    def trajectory(self):
        return self.visited_vps

if __name__ == '__main__':
    # clean folder for save point cloud file
    vpfile = os.path.join(sys.path[0],'../viewpoint/viewpoints.txt');
    env = UAVScanningEnv(vpfile,ViewPoint(0,0,0,0,0,0,0,1,0),8)
    state = env.reset()
    neighbors = state.neighbors()
    done, rewards = False, []
    step = 0
    while not done:
        nextVp = neighbors[np.random.randint(len(neighbors))]
        done, reward, coverage, state = env.play(nextVp)
        rewards.append(reward)
        neighbors = state.neighbors()
        step += 1
        print("step", step, "vp", nextVp, "neighbors", neighbors)
    #print("trajectory", len(env.trajectory()), env.trajectory())
