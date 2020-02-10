#!/usr/bin/env python
import rospy
import numpy as np
from quadrotor_camera_control import uav_cam_controller
from geometry_msgs.msg import Pose
from rs_camera import rs_image
from pc_process import pc_capture
from math import *

class auto_scanning:
    def __init__(self, camera, data, controller):
        self.camera = camera
        self.data_processor = data
        self.controller = controller
        self.trajectory = self.scan_trajectory(5)#self.circular_fly(7, 25)
        self.pose_index = 0
        self.flying = False

    def start(self):
        self.controller.takeoff()
        self.flying = True

    def terminate(self):
        self.controller.landing()
        self.flying == False

    def fly(self):
        if self.flying == True:
            self.pose_index = 0
            pose = self.trajectory[self.pose_index][0]
            angle = self.trajectory[self.pose_index][1]
            #print("angle", angle, "pose", pose)
            controller.execute_camera_joint(angle)
            controller.execute_quadrotor_pose(pose, self.fly_callback)

    def fly_callback(self):
        # capture data
        self.data_processor.scan_and_save(self.pose_index)
        self.pose_index = self.pose_index + 1
        if self.pose_index < len(self.trajectory):
            pose = self.trajectory[self.pose_index][0]
            angle = self.trajectory[self.pose_index][1]
            controller.execute_camera_joint(angle)
            #print("angle", angle, "pose", pose)
            controller.execute_quadrotor_pose(pose, self.fly_callback)
        else:
            self.terminate()

    def circular_fly(self, height, radius):
        PI = 3.1415926535897931
        trajectory = []
        theta = 0.0
        while theta < 2*PI:
            pose = Pose()
            pose.position.x = radius*cos(theta)
            pose.position.y = radius*sin(theta)
            pose.position.z = height

            # quaternion
            yaw = PI+theta
            if yaw >= 2*PI:
                yaw = yaw - 2*PI

            qx,qy,qz,qw = self.to_quaternion(yaw, 0.0, 0.0)
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
            angle = atan2(height-5,radius)
            trajectory.append([pose,angle])
            theta = theta + 2*PI/50
        return trajectory

    def scan_trajectory(self, height):
        PI = 3.1415926535897931
        trajectory = []

        pose0 = Pose()
        pose0.position.x = 0
        pose0.position.y = -31
        pose0.position.z = height
        qx,qy,qz,qw = self.to_quaternion(0.5*PI, 0.0, 0.0)
        pose0.orientation.x = qx
        pose0.orientation.y = qy
        pose0.orientation.z = qz
        pose0.orientation.w = qw
        trajectory.append([pose0, 0.0])

        y = -30
        while y < 30:
            pose = Pose()
            pose.position.x = 6
            pose.position.y = y
            pose.position.z = height
            qx,qy,qz,qw = self.to_quaternion(PI, 0.0, 0.0)
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
            trajectory.append([pose, 0.0])
            y = y + 5

        pose1 = Pose()
        pose1.position.x = 0
        pose1.position.y = 32
        pose1.position.z = height
        qx,qy,qz,qw = self.to_quaternion(1.5*PI, 0.0, 0.0)
        pose1.orientation.x = qx
        pose1.orientation.y = qy
        pose1.orientation.z = qz
        pose1.orientation.w = qw
        trajectory.append([pose1, 0.0])

        y = 30
        while y >= -30:
            pose = Pose()
            pose.position.x = -6
            pose.position.y = y
            pose.position.z = height
            qx,qy,qz,qw = self.to_quaternion(0.0, 0.0, 0.0)
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
            trajectory.append([pose, 0.0])
            y = y - 5

        x = 0
        y = -30
        while y <= 20:
            pose = Pose()
            pose.position.x = 0.0
            pose.position.y = y
            pose.position.z = height + 4
            qx,qy,qz,qw = self.to_quaternion(PI, 0.0, 0.0)
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
            trajectory.append([pose, 0.5*PI])
            y = y + 4

        x = -20
        while x <= 20:
            pose = Pose()
            pose.position.x = x
            pose.position.y = 0.0
            pose.position.z = height + 4
            qx,qy,qz,qw = self.to_quaternion(0.0, 0.0, 0.0)
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
            trajectory.append([pose, 0.5*PI])
            x = x + 4

        return trajectory

    def to_quaternion(self,yaw,pitch,row):
        cy = cos(0.5*yaw)
        sy = sin(0.5*yaw)
        cp = cos(0.5*pitch)
        sp = sin(0.5*pitch)
        cr = cos(0.5*row)
        sr = sin(0.5*row)

        w = cy*cp*cr+sy*sp*sr
        x = cy*cp*sr-sy*sp*cr
        y = sy*cp*sr+cy*sp*cr
        z = sy*cp*cr-cy*sp*sr
        return x,y,z,w


if __name__ == '__main__':
    rospy.init_node("uav_scanning_auto", anonymous=True, log_level=rospy.INFO)
    camera = rs_image()
    data = pc_capture(camera)
    controller = uav_cam_controller()
    auto = auto_scanning(camera, data, controller)
    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            key_input = raw_input("please enter '+' and '-' for change the angle of camera joint:\n")
            if (key_input == '+'):
                auto.start()
            elif(key_input == '-'):
                auto.terminate()
            elif(key_input == '.'):
                auto.fly()

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
