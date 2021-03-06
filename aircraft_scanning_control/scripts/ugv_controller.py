#!/usr/bin/env python
import rospy
import numpy as np

import time
import math
from math import *
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import transform
from transform import MMRobotTransform

class arm_controller:
    def __init__(self):
        self.arm_pose = None
        self.joint_1_pub = rospy.Publisher('iiwa/PositionJointInterface_J1_controller/command', Float64, queue_size=1)
        self.joint_2_pub = rospy.Publisher('iiwa/PositionJointInterface_J2_controller/command', Float64, queue_size=1)
        self.joint_3_pub = rospy.Publisher('iiwa/PositionJointInterface_J3_controller/command', Float64, queue_size=1)
        self.joint_4_pub = rospy.Publisher('iiwa/PositionJointInterface_J4_controller/command', Float64, queue_size=1)
        self.joint_5_pub = rospy.Publisher('iiwa/PositionJointInterface_J5_controller/command', Float64, queue_size=1)
        self.joint_6_pub = rospy.Publisher('iiwa/PositionJointInterface_J6_controller/command', Float64, queue_size=1)
        self.joint_7_pub = rospy.Publisher('iiwa/PositionJointInterface_J7_controller/command', Float64, queue_size=1)
        self.arm_sub = rospy.Subscriber('iiwa/joint_states', JointState, self._arm_callback)
        self.goal = None
        self.status = 'stop'

    def _arm_callback(self,data):
        self.arm_pose = data.position

    def pose(self):
        return self.arm_pose

    def init(self):
        self._move([0.0,1.57,0.0,-0.3,0.0,-1.87,0.0])

    # set a goal with catersian pose [position, quanternion]
    def set_goal(self, goal):
        self.goal = goal

    def drive(self):
        if self.goal != None and self.status == 'stop':
            self.status = 'moving'
            self._move_to_goal()
            self.status = 'stop'
            self.goal = None

    def _move_to_goal(self):
        if self.goal == None:
            return
        # check if joint reach target positions
        while not self._goal_reached(self.goal):
            self._move(self.goal)
            rospy.sleep(1)

    def _goal_reached(self,goal,tolerance=0.01):
        joints = self.pose()
        if not abs(joints[0]-goal[0]) < tolerance:
            return False
        if not abs(joints[1]-goal[1]) < tolerance:
            return False
        if not abs(joints[2]-goal[2]) < tolerance:
            return False
        if not abs(joints[3]-goal[3]) < tolerance:
            return False
        if not abs(joints[4]-goal[4]) < tolerance:
            return False
        if not abs(joints[5]-goal[5]) < tolerance:
            return False
        if not abs(joints[6]-goal[6]) < tolerance:
            return False
        return True

    def _move(self,joint_positions):
        self.joint_1_pub.publish(joint_positions[0])
        self.joint_2_pub.publish(joint_positions[1])
        self.joint_3_pub.publish(joint_positions[2])
        self.joint_4_pub.publish(joint_positions[3])
        self.joint_5_pub.publish(joint_positions[4])
        self.joint_6_pub.publish(joint_positions[5])
        self.joint_7_pub.publish(joint_positions[6])

# ugv controller
class ugv_controller:
    def __init__(self):
        self.ugv_pose = None
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.ugv_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self._ugv_callback)
        self.goal = None
        self.status = 'stop' # 'moving'
        self.max_speeds = [5.0,pi]

    def _ugv_callback(self, data):
        index = data.name.index('iiwa')
        self.ugv_pose = data.pose[index]
        # print(self.ugv_pose)

    # catesian pose with position and orientation in quaternion
    def pose(self):
        return self.ugv_pose

    # eular angle representation of pose, x, y, yaw in global coordinate system
    def euler_pose(self):
        current = self.ugv_pose
        px = current.position.x
        py = current.position.y
        ow = current.orientation.w
        ox = current.orientation.x
        oy = current.orientation.y
        oz = current.orientation.z
        yaw,pitch,roll = transform.quaternion_to_eularangle(ow,ox,oy,oz)
        return (px,py,yaw)

    def stop(self):
        self._move(0,0)
    # set goal position, x, y, yaw (startin from x axis)
    def set_goal(self,goal):
        self.goal = goal #(x,y,yaw)

    def drive(self):
        if self.goal != None and self.status == 'stop':
            self.status = 'moving'
            self._move_to_goal()
            self.status = 'stop'
            self.goal = None

    def _goal_reached(self,goal,tolerance=0.01):
        (gx,gy,gyaw) = self.goal
        (px,py,yaw) = self.euler_pose()
        print("goal reach check", self.goal, self.euler_pose())
        if abs(gx-px) > tolerance:
            return False
        if abs(gy-py) > tolerance:
            return False
        if abs(gyaw-yaw) > tolerance or abs(gyaw-yaw)-2*pi > tolerance:
            return False
        return True

    def _move_to_goal(self,tolerance=0.001):
        # as the skid_steer, velocity control only in x and z, not y
        # so the drive control
        (gx,gy,gyaw) = self.goal
        # step 1: pre rotate
        (px,py,yaw) = self.euler_pose()
        if self._goal_reached(self.goal):
            return

        angle = self._direction2goal(px,py,gx,gy)
        yaw_err = self._yaw2goal(yaw,angle)
        while not abs(yaw_err) < tolerance:
            self._rotate_controller(yaw_err)
            (px,py,yaw) = self.euler_pose()
            yaw_err = self._yaw2goal(yaw,angle)
            #print("pre rotate yaw_err: ", yaw_err)
        self.stop()

        # step 2: move to goal position
        dist_err, yaw_err = self._distance2goal(self.euler_pose(),self.goal)
        v,vz,dist_i,yaw_i = 1.0,pi,0.0,0.0
        while not abs(dist_err) < 10.0*tolerance:
            v,vz,dist_i,yaw_i = self._pid_controller(dist_err,yaw_err,v,vz,dist_i,yaw_i)
            # print("errors:", dist_err,yaw_err,v,vz)
            dist_err, yaw_err = self._distance2goal(self.euler_pose(),self.goal)
        self.stop()

        # step 3: post rotate
        (px,py,yaw) = self.euler_pose()
        yaw_err = self._yaw2goal(yaw,gyaw)
        while not abs(yaw_err) < tolerance:
            self._rotate_controller(yaw_err)
            (px,py,yaw) = self.euler_pose()
            yaw_err = self._yaw2goal(yaw,gyaw)
            #print("post rotate yaw_err: ", yaw_err)
        self.stop()

    # PID control for ugv drive
    # the yaw angle is in range of [-pi,pi],
    # it is not continuous when its change from pi to -pi
    # so the angluar speed should not be too large
    def _pid_controller(self,dist_err,yaw_err,v,vz,dist_i,yaw_i,time_duration=0.001):
        Kp_v,Ki_v,Kd_v = 5,0.0,0.1
        Kp_z,Ki_z,Kd_z = 5,0.0,0.1
        dist_i += time_duration*dist_err
        yaw_i += time_duration*yaw_err
        v = Kp_v*dist_err + Ki_v*dist_i + Kd_v*v
        vz = Kp_z*yaw_err + Ki_z*yaw_i + Kd_z*vz
        v,vz = self._velocity_bounded(v,vz)
        self._move(v,vz)
        rospy.sleep(time_duration)
        return v,vz,dist_i,yaw_i

    # rotate from a yaw to goal yaw
    def _rotate_controller(self,yaw_err,time_duration=0.001):
        if yaw_err > 0:
            self._move(0,self.max_speeds[1])
        elif yaw_err < 0:
            self._move(0,-self.max_speeds[1])
        else:
            self._move(0,0)
        rospy.sleep(time_duration)

    # direction from pose to goal
    def _direction2goal(self,px,py,gx,gy):
        direction = 0.0
        if abs(gx-px) <= 1e-6:
            if gy-py > 0:
                direction = pi
            else:
                direction = 0.0
        else:
            direction = math.atan2((gy-py),(gx-px))
        return direction

    def _yaw2goal(self,yaw,gyaw):
        # err in range of [-pi,pi]
        err = gyaw-yaw
        if err > pi:
            return err-2*pi
        elif err < -pi:
            return err+2*pi
        else:
            return err

    # distance from pose to target
    def _distance2goal(self,pose,goal):
        (px,py,yaw) = pose
        (gx,gy,gyaw) = goal
        dist = math.sqrt((gx-px)*(gx-px)+(gy-py)*(gy-py))
        direction = self._direction2goal(px,py,gx,gy)
        angle = self._yaw2goal(yaw,direction)
        #print("direction: ", direction, "yaw: ", yaw)
        return dist, angle

    def _velocity_bounded(self, v,vz):
        max_v, max_vz = self.max_speeds[0], self.max_speeds[1]
        if v > max_v:
            v = max_v
        elif v < -max_v:
            v = -max_v
        if vz > max_vz:
            vz = max_vz
        elif vz < -max_vz:
            vz = -max_vz
        return v,vz

    def _move(self,vx=1.0,vz=1.0):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = vz
        self.vel_pub.publish(msg)


# controller
class ugv_arm_controller:
    def __init__(self):
        self.transform_util = MMRobotTransform()
        self.ugv = ugv_controller()
        self.arm = arm_controller()
        self.initialized = False

    def is_initialized(self):
        return self.initialized

    def initialize(self):
        self.initialized = True
        print("initialize mobile manipulator.")
        self.arm.init()
        self.ugv.stop()

    def drive_ugv(self,goal):
        self.ugv.set_goal(goal)
        self.ugv.drive()

    def drive_arm(self,goal):
        self.arm.set_goal(goal)
        self.arm.drive()

    def drive(self,goal):
        self.drive_ugv(goal[0])
        self.drive_arm(goal[1])

    def transform_m2c(self):
        mat = self.transform_util.mobilebase2camera(self.ugv.pose(), self.arm.pose())
        return mat
