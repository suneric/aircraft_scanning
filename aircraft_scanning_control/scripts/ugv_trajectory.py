#!/usr/bin/env python
import rospy
import numpy as np
import os
import sys

class trajectory_defined:
    def __init__(self):
        self.trajectory = []
        self.index = 0
        self._create_trajectory()

    def completed(self):
        return self.index >= len(self.trajectory)

    def next_pose(self):
        if self.completed():
            return None
        else:
            pose = self.trajectory[self.index]
            self.index = self.index + 1
            return pose

    def _create_trajectory(self):
        self.trajectory.append((0,-28,1.57))
        self.trajectory.append((0,-27,1.57))
        self.trajectory.append((-2,-27,3.14))
        self.trajectory.append((-4,-27,3.14))
        self.trajectory.append((-4,-26,1.57))
        self.trajectory.append((-2,-26,0))
        self.trajectory.append((0,-26,0))
        self.trajectory.append((2,-26,0))
        self.trajectory.append((4,-26,0))
        self.trajectory.append((4,-25,1.57))
        self.trajectory.append((2,-25,3.14))
        self.trajectory.append((0,-25,3.14))
