#! /usr/bin/env python
import numpy as np
from math import *
from geometry_msgs.msg import Pose

class ViewPoint(object):
    def __init__(self,idx,px,py,pz,ox,oy,oz,ow,angle):
        self.idx = idx
        self.quadrotor_pose = Pose();
        self.quadrotor_pose.position.x = px
        self.quadrotor_pose.position.y = py
        self.quadrotor_pose.position.z = pz
        self.quadrotor_pose.orientation.x = ox
        self.quadrotor_pose.orientation.y = oy
        self.quadrotor_pose.orientation.z = oz
        self.quadrotor_pose.orientation.w = ow
        self.camera_angle = angle
        self.voxels = []
    def index(self):
        return self.idx
    def quadrotor(self):
        return self.quadrotor_pose
    def camera(self):
        return self.camera_angle
    def add_voxel(self, voxel_idx):
        self.voxels.append(voxel_idx)
    def view_voxels(self):
        return self.voxels
    def print_info(self):
        print(self.idx, self.quadrotor_pose, self.camera_angle, self.voxels)

class UAVScanningEnv(object):
    def __init__(self):
        self.viewpoints = []
        self.allvoxels = []

        # need to update each step
        self.obs_voxels = []
        self.unvisited_vp = []
        self.visited_vp = []
        self.current_vp = None

        # state
        self.voxels_state = []
        self.viewpoints_state = []

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
                for v in voxels:
                    voxel_idx = int(v)
                    vp.add_voxel(voxel_idx)
                    if voxel_idx not in self.allvoxels:
                        self.allvoxels.append(voxel_idx)
                self.viewpoints.append(vp)
        reader.close()
        #print(self.allvoxels)

    def save_visited(self,file):
        with open(file, 'w') as writer:
            for vpIdx in self.visited_vp:
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
        self.obs_voxels = []
        self.visited_vp = []
        self.unvisited_vp = [i for i in range(self.viewpoints_count())]
        #print(self.unvisited_vp)
        # voxels state are not visited
        self.voxels_state = [0]*len(self.allvoxels)
        # viewpoints state are not visited
        self.viewpoints_state = [0]*len(self.viewpoints)

    def voxels_state_array(self):
        return np.asarray(self.voxels_state)
    def viewpoints_state_array(self):
        return np.asarray(self.viewpoints_state)

    def viewpoints_count(self):
        return len(self.viewpoints)
    def voxels_count(self):
        return len(self.allvoxels)

    def unvisited_viewpoints(self):
        return self.unvisited_vp
    def visited_viewpoints(self):
        return self.visited_vp

    def action(self, vpIdx):
        # penatly of traveling from current vp to next vp
        nextVp = self.viewpoints[vpIdx]
        dist = self._distance(nextVp, self.current_vp)
        td_penalty = -dist*0.01
        # reward of new voxel coverage
        voxels_count = len(self.allvoxels)
        newVoxel = self._move2next_and_update(vpIdx, nextVp)
        nv_reward = 100.0*float(newVoxel)/float(voxels_count)
        # done reward
        done, fn_reward = False, 0.0
        if self._coverage() == 1.0:
            fn_reward = 0.1*len(self.unvisited_vp)
            #print(len(self.unvisited_vp), fn_reward)
            done = True
        # step reward
        reward = td_penalty + nv_reward + fn_reward
        #print("td_penalty",td_penalty,"nv_reward",nv_reward,"fn_reward",fn_reward,"step reward",reward)
        return done, reward

    def _move2next_and_update(self, vpIdx, nextVp):
        self.current_vp = nextVp
        # update viewpoint state
        self.viewpoints_state[vpIdx] = 1
        # update visited and unvisied viewpoints
        if vpIdx not in self.visited_vp:
            self.visited_vp.append(vpIdx)
        if vpIdx in self.unvisited_vp:
            self.unvisited_vp.remove(vpIdx)

        # update observed view voxels
        newVoxel = 0
        viewVoxels = nextVp.view_voxels()
        for v in viewVoxels:
            # update voxels_state
            vIndex = self.allvoxels.index(v)
            self.voxels_state[vIndex] = 1
            # update observed voxels
            if v not in self.obs_voxels:
                self.obs_voxels.append(v)
                newVoxel+=1
        return newVoxel

    def _coverage(self):
        obsV = len(self.obs_voxels)
        allV = len(self.allvoxels)
        return float(obsV)/float(allV)

    def _distance(self, vp1, vp2):
        x1 = vp1.quadrotor_pose.position.x;
        y1 = vp1.quadrotor_pose.position.y;
        z1 = vp1.quadrotor_pose.position.z;
        x2 = vp2.quadrotor_pose.position.x;
        y2 = vp2.quadrotor_pose.position.y;
        z2 = vp2.quadrotor_pose.position.z;
        return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2))

if __name__ == '__main__':
    # clean folder for save point cloud file
    viewpoints = "/home/yufeng/Temp/viewpoints.txt"
    env = UAVScanningEnv()
    env.load(viewpoints)

    start = ViewPoint(0,0,0,0,0,0,0,1,0)
    env.reset(start)
    done, rewards = False, []
    while not done:
        vps = env.unvisited_viewpoints()
        if len(vps) == 0:
            done = True
            break
        index = np.random.randint(len(vps))
        done, reward = env.action(vps[index])
        rewards.append(reward)
    print("total reward", sum(rewards)," visited", len(env.visited_viewpoints()),"/",env.viewpoints_count())
    # env.save_visited("/home/yufeng/Temp/visitedvps.txt")
