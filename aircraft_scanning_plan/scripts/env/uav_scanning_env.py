#! /usr/bin/env python
import sys
import os
import numpy as np
from math import *
from geometry_msgs.msg import Pose
from viewpoint_map import ViewPoint, ViewPointMap

class UAVScanningEnv(object):
    def __init__(self,action_count):
        self.viewpoints = []
        self.voxels = []
        self.vpMap = None
        self.vpindices = []

        self.current_vp = None
        self.trajectory = []
        self.voxels_state = []
        self.viewpoints_state = []
        self.neighbor_vps = []
        self.action_dim = action_count # in action_count neighbor

    # load env from file
    def load(self,file):
        with open(file,'r') as reader:
            for line in reader.read().splitlines():
                data = line.split(" ")
                # print(data)
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
                # find all voxels
                for v in voxels:
                    voxel_idx = int(v)
                    if voxel_idx not in self.voxels:
                        self.voxels.append(voxel_idx)
                    vp.add_voxel(self.voxels.index(voxel_idx))
                # find all viewpoints
                self.viewpoints.append(vp)
        reader.close()
        # build a vp distance map
        self.vpMap = ViewPointMap(self.viewpoints)
        print("viewpoints count:", len(self.viewpoints), "voxels count:", len(self.voxels))
        for i in range(len(self.viewpoints)):
            self.vpindices.append(i)

    # save trajectory
    def save(self,file):
        with open(file, 'w') as writer:
            for vpIdx in self.trajectory:
                vp = self.viewpoints[vpIdx]
                idx = vp.index()
                pos = vp.quadrotor()
                angle = vp.camera()
                voxels = vp.view_voxels()

                line = str(idx) + " "\
                     + str(pos.position.x) + " "\
                     + str(pos.position.y) + " "\
                     + str(pos.position.z) + " "\
                     + str(pos.orientation.x) + " "\
                     + str(pos.orientation.y) + " "\
                     + str(pos.orientation.z) + " "\
                     + str(pos.orientation.w) + " "\
                     + str(angle) + " "
                for i in range(0,len(voxels)-1):
                    line = line + str(voxels[i]) + " "
                line = line+str(voxels[len(voxels)-1])+"\n"
                #print(line)
                writer.write(line)
        writer.close()

    def reset(self, startVp):
        self.current_vp = startVp
        self.neighbor_vps =self._nearest_vps(startVp,self.action_dim)
        self.trajectory = []
        self.voxels_state = [0 for i in range(len(self.voxels))]
        self.viewpoints_state = [0 for i in range(len(self.viewpoints))]

    def state_dimension(self):
        return len(self.voxels)+len(self.viewpoints)
    # return concatenate of voxels state and viewpoints state
    def concatenate_state(self):
        voxels = np.asarray(self.voxels_state)
        vps = np.asarray(self.viewpoints_state)
        return np.concatenate((voxels,vps))

    def action(self, actionIdx):
        newVoxel, dist, coverage = self._move2next(actionIdx)
        done = (coverage == 1.0)
        step_penalty = -0.1
        step_reward = newVoxel
        done_reward = 0
        reward = step_penalty + step_reward + done_reward
        #print(done,"td_penalty",td_penalty,"nv_reward",nv_reward,"fn_reward",fn_reward,"step reward",reward)
        return done, reward, coverage

    def _move2next(self, actionIdx):
        # find next vpIdx
        vpIdx = 0
        if actionIdx < len(self.neighbor_vps):
            vpIdx = self.neighbor_vps[actionIdx]
        nextVp = self.viewpoints[vpIdx]
        self.trajectory.append(vpIdx)
        # update viewpoints state
        self.viewpoints_state[vpIdx] = 1
        # update observed view voxels
        newVoxel = 0
        view = nextVp.view()
        for v in view:
            if self.voxels_state[v] == 0:
                newVoxel+=1
                self.voxels_state[v] = 1
        dist = self.vpMap._distance(nextVp, self.current_vp)
        self.current_vp = nextVp
        # find neighborvps in unvisited viewpoints
        vps = self.vpindices
        self.neighbor_vps =self.vpMap.neighbors(vpIdx,self.action_dim,vps)
        coverage = self._coverage()
        #print("action",actionIdx,"vp",vpIdx, "voxel", newVoxel, "coverage",coverage, "neighbors", self.neighbor_vps)
        return newVoxel, dist, coverage

    def _coverage(self):
        # print(self.voxels_state)
        # find all visisted voxels
        visited = np.asarray(np.nonzero(self.voxels_state)).flatten()
        return float(len(visited))/float(len(self.voxels_state))

    # all vp indices that are visisted or unvisited
    def state_vps(self, bVisited):
        vps = []
        vp_state = np.asarray(self.viewpoints_state)
        if bVisited:
            vps = np.where(vp_state==1)[0]
        else:
            vps = np.where(vp_state==0)[0]
        return vps

    def trajectory_vps(self):
        return self.trajectory

    def _nearest_vps(self, vp, count):
        dist_arr = [0 for i in range(len(self.viewpoints))]
        for i in range(len(self.viewpoints)):
            dist_arr[i] = self.vpMap._distance(vp,self.viewpoints[i])
        sorted = dist_arr[:]
        sorted.sort()
        subset = sorted[0:count]
        vps = []
        for dist in subset:
            vps.append(dist_arr.index(dist))
        return vps

if __name__ == '__main__':
    # clean folder for save point cloud file
    vpfile = os.path.join(sys.path[0],'../viewpoint/viewpoints.txt');
    print("load viewpoints from",vpfile)
    action_dim = 12
    env = UAVScanningEnv(action_dim)
    env.load(vpfile)
    start = ViewPoint(0,0,0,0,0,0,0,1,0)
    env.reset(start)
    done, rewards = False, []
    step = 0
    while not done:
        state = env.concatenate_state()
        print("state",state)
        action_idx = np.random.randint(action_dim)
        done, reward = env.action(action_idx)
        rewards.append(reward)
        step += 1
        #print("state", env.state())
    print("step:", step, "total reward:", sum(rewards)," coverage:", env._coverage())
    # env.save("/home/yufeng/Temp/visitedvps.txt")
