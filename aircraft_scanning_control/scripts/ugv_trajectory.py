#!/usr/bin/env python
import rospy
import numpy as np
import os
import sys
from math import *

class trajectory_defined:
    def __init__(self):
        self.trajectory = []
        self.index = 0
        self._create_trajectory()

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

    def _create_trajectory(self):
        self.trajectory.append((0,-28,0.5*pi))
        self._create_updownpath([-27,-22.5],[1.0,-1.0])
        # self._create_updownpath([-20,-8],[1.0,-1.0])
        # self._create_updownpath([-8,8],[1.5,0,-1.5])
        # self._create_updownpath([8,25],[1.0,-1.0])
        # self.trajectory.append((0,26,0.5*pi))
        # self.trajectory.append((0,27,0.5*pi))

    def _create_updownpath(self, y_range, x_pos, offset=1.0):
        i = 0
        y = y_range[0]
        while y < y_range[1]:
            if np.mod(i,2) == 0:
                for x in x_pos:
                    self.trajectory.append((x,y,0.5*pi))
            else:
                for x in reversed(x_pos):
                    self.trajectory.append((x,y,0.5*pi))

            i += 1
            y += offset
