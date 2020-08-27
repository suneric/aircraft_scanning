#!/usr/bin/env python
import rospy
import numpy as np
import time

from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

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

    def joint_pos(self):
        return self.arm_pose

    def move(self,joint_positions):
        self.joint_1_pub.publish(joint_positions[0])
        self.joint_2_pub.publish(joint_positions[1])
        self.joint_3_pub.publish(joint_positions[2])
        self.joint_4_pub.publish(joint_positions[3])
        self.joint_5_pub.publish(joint_positions[4])
        self.joint_6_pub.publish(joint_positions[5])
        self.joint_7_pub.publish(joint_positions[6])

class ugv_controller:
    def __init__(self):
        self.ugv_pose = None
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.ugv_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self._ugv_callback)

    def _ugv_callback(self, data):
        index = data.name.index('iiwa')
        self.ugv_pose = data.pose[index]

    def pose(self):
        return self.ugv_pose

    def move(self,vx=1.0,vz=1.0):
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
        self.ugv = ugv_controller()
        self.arm = arm_controller()

    def move_to(self,arm_pose):
        self.arm.move(arm_pose)
        ugv_pose = self.ugv.pose()
        arm_pose = self.arm.joint_pos()
        print('ugv', ugv_pose)
        print('arm', arm_pose)
