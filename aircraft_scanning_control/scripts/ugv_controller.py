#!/usr/bin/env python
import rospy
import numpy as np

import time
import math
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import transform

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

    def _arm_callback(self,data):
        self.arm_pose = data.position

    def pose(self):
        return self.arm_pose

    def move(self,joint_positions):
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

    def _ugv_callback(self, data):
        index = data.name.index('iiwa')
        self.ugv_pose = data.pose[index]

    # catesian pose with position and orientation in quaternion
    def pose(self):
        return self.ugv_pose

    # eular angle representation of pose, x, y, yaw in global coordinate system
    def eular_pose(self):
        current = self.ugv_pose
        px = current.position.x
        py = current.position.y
        ow = current.orientation.w
        ox = current.orientation.x
        oy = current.orientation.y
        oz = current.orientation.z
        yaw,pitch,roll = transform.quaternion_to_eularangle(ow,ox,oy,oz)
        return (px,py,yaw)

    def move(self,vx=1.0,vz=1.0):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = vz
        self.vel_pub.publish(msg)

    def stop(self):
        self.move(0,0)

    # set goal position, x, y, yaw (startin from x axis)
    def set_goal(self,goal):
        self.goal = goal #(x,y,yaw)

    def drive(self):
        if self.goal != None and self.status == 'stop':
            self.status = 'moving'
            # as the skid_steer, velocity control only in x and z, not y
            # so the drive control
            v, vz = 1,3.14
            (gx,gy,gyaw) = self.goal
            (px,py,yaw) = self.eular_pose()
            # step 1: change direction
            angle = math.atan2((gy-py),(gx-px))
            angle_err = angle-yaw
            while not abs(angle_err) <=0.001:
                if angle_err > 0:
                    self.move(0,3.14)
                else:
                    self.move(0,-3.14)
                (px,py,yaw) = self.eular_pose()
                angle = math.atan2((gy-py),(gx-px))
                angle_err = angle-yaw
            self.stop()

            # step 2: move to goal position
            dist_err = math.sqrt((gx-px)*(gx-px)+(gy-py)*(gy-py))
            while not dist_err <= 0.01:
                v,vz = self.PD_controller(self.eular_pose(),self.goal,v,vz)
                self.move(v,vz)
                (px,py,yaw) = self.eular_pose()
                dist_err = math.sqrt((gx-px)*(gx-px)+(gy-py)*(gy-py))
            self.stop()

            # step 3: rotate ugv if needed
            yaw_err = gyaw - yaw
            while not abs(yaw_err) <= 0.001:
                if yaw_err > 0:
                    self.move(0,3.14)
                else:
                    self.move(0,-3.14)
                (px,py,yaw) = self.eular_pose()
                yaw_err = gyaw - yaw
            self.stop()

            self.status = 'stop'
            self.goal = None

    # PD control for ugv drive
    def PD_controller(self,pose,goal,v,yaw_rate):
        # error
        (px,py,yaw) = pose
        (gx,gy,gyaw) = goal
        dist_err = math.sqrt((px-gx)*(px-gx)+(py-gy)*(py-gy))
        angle = math.atan2((gy-py),(gx-px))
        if abs(angle-yaw) > 1.57:
            dist_err *= -1
        yaw_err = angle-yaw
        # PD gains
        Kp_v,Kd_v, Kp_z,Kd_z = 5,0.1, 10,0.1
        v = Kp_v*dist_err + Kd_v*v
        vz = Kp_z*yaw_err + Kd_z*yaw_rate
        v,vz = self._velocity_bounded(v, vz)
        #print('goal: ',gx,gy,gyaw, 'current: ', px,py,yaw, "control input: ", v, vz)
        return v,vz

    def _velocity_bounded(self, v, vz):
        max_v, max_vz = 5.0, 6.28
        if v > max_v:
            v = max_v
        elif v < -max_v:
            v = -max_v

        if vz > max_vz:
            vz = max_vz
        elif vz < -max_vz:
            vz = -max_vz

        return v, vz

# controller
class ugv_arm_controller:
    def __init__(self):
        self.ugv = ugv_controller()
        self.arm = arm_controller()
        self.initialized = False

    def is_initialized(self):
        return self.initialized

    def initialize(self):
        self.initialized = True
        print("initialize mobile manipulator.")
        self.arm.move([0.0,1.57,0.0,0.0,0.0,-1.57,0.0])
        self.ugv.move(0,0)

    def drive_ugv(self,goal):
        self.ugv.set_goal(goal)
        self.ugv.drive()

    def drive_arm(self, arm_pose):
        self.arm.move(arm_pose)

    def transform_m2c(self):
        mat = transform.mobilebase2camera(self.ugv.pose(), self.arm.pose())
        return mat
