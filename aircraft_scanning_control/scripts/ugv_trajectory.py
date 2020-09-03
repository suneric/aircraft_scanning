#!/usr/bin/env python
import rospy
import numpy as np
import os
import sys
from geometry_msgs.msg import Pose
from math import *
import transform
from transform import MMRobotTransform

class trajectory_defined:
    def __init__(self):
        self.trajectory = []
        self.index = 0
        self.transform_util = MMRobotTransform()
        # self._create_trajectory()
        file = os.path.join(sys.path[0],'../../aircraft_scanning_plan/trajectory/ugv/viewpoints.txt');
        self._load_trajectory(file)

    def completed(self):
        return self.index >= len(self.trajectory)

    def next_pose(self):
        if self.completed():
            print("done.")
            return None
        else:
            pose = self.trajectory[self.index]
            self.index = self.index + 1
            print("traverse to ", pose)
            return pose

    def _load_trajectory(self,file):
        print("load trajectory from", file)
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
                pose = Pose();
                pose.position.x = px
                pose.position.y = py
                pose.position.z = pz
                pose.orientation.x = ox
                pose.orientation.y = oy
                pose.orientation.z = oz
                pose.orientation.w = ow
                ugv, arm = self.transform_util.camera2mobilebase(pose)
                self.trajectory.append([ugv,arm])
        reader.close()

    def _create_trajectory(self):
        arm_joint = [0.0,1.57,0.0,-0.3,0.0,-1.87,0.0]
        self.trajectory.append([(0,-28,0.5*pi),arm_joint])
        self._create_updownpath([-27,-22.5],[1.0,-1.0],1.0,arm_joint)
        # self._create_updownpath([-20,-8],[1.0,-1.0])
        # self._create_updownpath([-8,8],[1.5,0,-1.5])
        # self._create_updownpath([8,25],[1.0,-1.0])
        # self.trajectory.append((0,26,0.5*pi))
        # self.trajectory.append((0,27,0.5*pi))

    def _create_updownpath(self, y_range, x_pos, offset, arm_joint):
        i = 0
        y = y_range[0]
        while y < y_range[1]:
            if np.mod(i,2) == 0:
                for x in x_pos:
                    self.trajectory.append([(x,y,0.5*pi),arm_joint])
            else:
                for x in reversed(x_pos):
                    self.trajectory.append([(x,y,0.5*pi),arm_joint])

            i += 1
            y += offset
