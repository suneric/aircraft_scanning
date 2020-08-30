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
        self._create_updownpath([-27,-22])
        self._create_updownpath([-20,28])

    def _create_updownpath(self, y_range):
        x = 1
        offset = 1.25
        i = 0
        y = y_range[0]
        while y < y_range[1]:
            if np.mod(i,2) == 0:
                self.trajectory.append((x,y,0.5*pi))
                self.trajectory.append((-x,y,0.5*pi))
            else:
                self.trajectory.append((-x,y,0.5*pi))
                self.trajectory.append((x,y,0.5*pi))
            i += 1
            y += offset