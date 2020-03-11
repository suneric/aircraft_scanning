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

class VPMap(object):
    def __init__(self,viewpoints):
        self.distmap = self._build_distmap(viewpoints)

    # get number of count neighbors vp indices with meauraing the distance
    # exclude itself
    def neighbors(self,vpIdx,count,unvisited_vps):
        size = len(unvisited_vps)
        if size <= count:
            return unvisited_vps
        dist_vps = self.distmap[vpIdx]

        unvisited_dist_vps = []
        for i in range(size):
            vp_idx = unvisited_vps[i]
            unvisited_dist_vps.append(dist_vps[vp_idx])
        vp_dist_indices = self._shortest_distance_vps(unvisited_dist_vps,count)
        vp_indices = []
        for idx in vp_dist_indices:
            vp_indices.append(unvisited_vps[idx])
        return vp_indices

    def _build_distmap(self,viewpoints):
        size = len(viewpoints)
        distmap = [[0]*size]*size
        for i in range(size):
            for j in range(size):
                distmap[i][j] = self._distance(viewpoints[i],viewpoints[j])
        #print(distmap)
        return distmap

    def _shortest_distance_vps(self, dist_vps, count):
        sorted_vps = dist_vps[:]
        sorted_vps.sort()
        subset = sorted_vps[1:count+1] # exclued itself
        neighbor_indices = []
        for dist in subset:
            neighbor_indices.append(dist_vps.index(dist))
        return neighbor_indices

    def _distance(self, vp1, vp2):
        x1 = vp1.quadrotor_pose.position.x;
        y1 = vp1.quadrotor_pose.position.y;
        z1 = vp1.quadrotor_pose.position.z;
        x2 = vp2.quadrotor_pose.position.x;
        y2 = vp2.quadrotor_pose.position.y;
        z2 = vp2.quadrotor_pose.position.z;
        return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2))


class UAVScanningEnv(object):
    def __init__(self):
        self.viewpoints = []
        self.allvoxels = []

        # need to update each step
        self.obs_voxels = []
        self.unvisited_vp = []
        self.visited_vp = []
        self.current_vp = None
        self.trajectory = []

        # state
        self.voxels_state = []
        self.viewpoints_state = []

        self.vpMap = None

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
        self.vpMap = VPMap(self.viewpoints)
        #print(self.allvoxels)

    def save_visited(self,file):
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

    def neighbor_state(self, vpIdx, count):
        n_indices = self.vpMap.neighbors(vpIdx,count,self.unvisited_vp)
        state = [0]*len(self.viewpoints)
        for idx in n_indices:
            state[idx]=1
        return np.asarray(state)

    def nearest_state(self, vp, count):
        dist_arr = [0]*len(self.viewpoints)
        for i in range(len(self.viewpoints)):
            dist_arr[i] = self._distance(vp,self.viewpoints[i])
        sorted = dist_arr[:]
        sorted.sort()
        subset = sorted[0:count]
        state = [0]*len(self.viewpoints)
        for dist in subset:
            state[dist_arr.index(dist)]=1
        return np.asarray(state)

    def reset(self, startVp):
        self.current_vp = startVp
        self.obs_voxels = []
        self.visited_vp = []
        self.trajectory = []
        self.unvisited_vp = [i for i in range(self.viewpoints_count())]

        self.voxels_state = [0]*len(self.allvoxels)
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
        nextVp, newVoxel, dist = self._move2next_and_update(vpIdx)
        #print("next vp delta info",vpIdx, newVoxel,dist,len(self.obs_voxels),len(self.visited_vp))
        coverage = self._coverage()
        done = (coverage == 1.0)
        td_penalty = 0.0 #-dist*0.01 # base penalty + distance travel
        nv_reward = float(newVoxel)/float(self.voxels_count())
        fn_reward = done*len(self.unvisited_vp)
        reward = td_penalty + nv_reward + fn_reward
        #print(done,"td_penalty",td_penalty,"nv_reward",nv_reward,"fn_reward",fn_reward,"step reward",reward)
        return done, reward

    def _move2next_and_update(self, vpIdx):
        # update viewpoint state
        self.trajectory.append(vpIdx)
        self.viewpoints_state[vpIdx] = 1
        nextVp = self.viewpoints[vpIdx]
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

        dist = self._distance(nextVp, self.current_vp)
        self.current_vp = nextVp
        return nextVp, newVoxel, dist

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

    count_n = 25
    start = ViewPoint(0,0,0,0,0,0,0,1,0)
    neighbors = env.nearest_vps(start,count_n)
    env.reset(start)
    done, rewards = False, []
    while not done:
        vps = env.unvisited_viewpoints()
        if len(vps) == 0:
            done = True
            break
        vp_idx = neighbors[np.random.randint(len(neighbors))]
        done, reward = env.action(vp_idx)
        rewards.append(reward)
        neighbors = env.neighbors(vp_idx,count_n)
        print("current:", vp_idx, "neighbors:", neighbors)
    print("total reward:", sum(rewards)," visited:", len(env.visited_viewpoints()),"/",env.viewpoints_count())
    # env.save_visited("/home/yufeng/Temp/visitedvps.txt")
