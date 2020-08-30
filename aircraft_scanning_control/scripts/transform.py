#!/usr/bin/env python
import numpy as np
from numpy import pi, sqrt, cos, sin, arctan2, array, matrix
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_matrix, quaternion_matrix
from math import *

# transform for mobile manipulator scanning

def mobilebase2camera(ugv_pos, arm_pos):
    mat0 = cartesian_to_matrix(ugv_pos)
    mat1 = mobile_base_to_arm()
    mat2 = arm_to_ee(arm_pos)
    mat3 = arm_ee_to_camera()
    return mat0*mat1*mat2*mat3

def mobile_base_to_arm():
    return matrix([[1,0,0,-0.35],
                     [0,1,0,0],
                     [0,0,1,0.7], # offset 0.7 meters in z
                     [0,0,0,1]])

def arm_ee_to_camera():
    mat_rz = matrix([[cos(-0.5*pi),-sin(-0.5*pi),0,0],
                      [sin(-0.5*pi),cos(-0.5*pi),0,0],
                      [0,0,1,0],
                      [0,0,0,1]])
    mat_tz = matrix([[1,0,0,0],
                      [0,1,0,0],
                      [0,0,1,0.0208],
                      [0,0,0,1]])
    return mat_rz*mat_tz

def arm_to_ee(joint_pos):
    l02 = 0.36
    l24 = 0.42
    l46 = 0.4
    l6E = 0.126

    H02 = Hrrt(joint_pos[1],joint_pos[0],l02)
    H24 = Hrrt(-joint_pos[3],joint_pos[2],l24)
    H46 = Hrrt(joint_pos[5],joint_pos[4],l46)
    H6E = Hrrt(0.0,joint_pos[6],l6E)
    H0E = H02 * H24 * H46 * H6E
    return H0E

def Hrrt(ty, tz, l):
  cy, sy = trig(ty)
  cz, sz = trig(tz)
  return matrix([[cy * cz, -sz, sy * cz, 0.0],
                 [cy * sz, cz, sy * sz, 0.0],
                 [-sy, 0.0, cy, l],
                 [0.0, 0.0, 0.0, 1.0]])

# transform for quadrotor scanning
def quadrotor2camera(pose, angle):
    # quadrotor position matrix
    mat0 = cartesian_to_matrix(pose)
    mat1 = quadrotorbase()
    mat2 = camerajoint(angle)
    mat3 = camera2pointcloud()
    return mat0*mat1*mat2*mat3

# decompose the viewpoint to quadrotor position and angle of camera joint
def quadrotor_camera_pose(viewpoint):
    # pose*T_base*T_camjoint = vp
    matvp = cartesian_to_matrix(viewpoint)
    # find position of quadrotor and angle of camera joint
    qw = viewpoint.orientation.w
    qx = viewpoint.orientation.x
    qy = viewpoint.orientation.y
    qz = viewpoint.orientation.z
    yaw,pitch,roll = quaternion_to_eularangle(qw,qx,qy,qz)
    #print("yaw,pitch,roll",yaw,pitch,roll)

    angle = pitch # pitch turns to angle of camera joint

    mat0 = quadrotorbase()
    mat1 = camerajoint(angle)
    mat2 = camera2pointcloud()
    rs_mat = mat0*mat1*mat2

    quadrotor_mat = matvp*np.linalg.inv(rs_mat)
    quadpose = matrix_to_cartesian(quadrotor_mat)

    return quadpose,angle


def quadrotorbase():
    # camera joint translation in base frame
    return matrix([[1,0,0,0.42],
                     [0,1,0,0],
                     [0,0,1,0], # 0.11 meter down the quadrotor
                     [0,0,0,1]])
def camerajoint(angle):
    # rs435 camera rotation along y axis, translate along x axis
    return matrix([[cos(angle),0,sin(angle),0.0358],
                     [0,1,0,0],
                     [-sin(angle),0,cos(angle),0],
                     [0,0,0,1]])

def camera2pointcloud():
    # point cloud frame respect to rs435 frame
    mat_z = matrix([[cos(-0.5*pi),-sin(-0.5*pi),0,0],
                      [sin(-0.5*pi),cos(-0.5*pi),0,0],
                      [0,0,1,0],
                      [0,0,0,1]])
    mat_x = matrix([[1,0,0,0],
                      [0,cos(-0.5*pi),-sin(-0.5*pi),0],
                      [0,sin(-0.5*pi),cos(-0.5*pi), 0],
                      [0,0,0,1]])
    return mat_z*mat_x


def trig(angle):
    return cos(angle),sin(angle)

# convert eular angle to quaternion
# yaw (Z)-pitch (Y)-roll (X)
# return q1,q1,q3,q4 which is w,x,y,z
def eularangle_to_quaternion(yaw, pitch, roll):
    cy, sy = trig(0.5*yaw)
    cp, sp = trig(0.5*pitch)
    cr, sr = trig(0.5*roll)
    w = sy*sp*sr+cy*cp*cr
    x = -sy*sp*cr+cy*cp*sr
    y = sy*cp*sr+cy*sp*cr
    z = sy*cp*cr-cy*sp*sr
    return w,x,y,z

def quaternion_to_eularangle(w,x,y,z):
    sr_cp = 2*(w*x+y*z)
    cr_cp = 1-2*(x*x+y*y)
    roll = atan2(sr_cp, cr_cp)
    sp = 2*(w*y-z*x)
    pitch = asin(sp)
    if abs(sp) >= 1:
        pitch = copysign(0.5*np.pi, sp)
    sy_cp = 2*(w*z+x*y)
    cy_cp = 1-2*(y*y+z*z)
    yaw = atan2(sy_cp, cy_cp)
    return yaw,pitch,roll

def rotation_translation(rotation, translation):
    Cx,Sx = trig(rotation[0])
    Cy,Sy = trig(rotation[1])
    Cz,Sz = trig(rotation[2])
    dX = translation[0]
    dY = translation[1]
    dZ = translation[2]
    mat_trans = matrix([[1,0,0,dX],
                          [0,1,0,dY],
                          [0,0,1,dZ],
                          [0,0,0,1]])
    mat_rotX = matrix([[1,0,0,0],
                         [0,Cx,-Sx,0],
                         [0,Sx,Cx,0],
                         [0,0,0,1]])
    mat_rotY = matrix([[Cy,0,Sy,0],
                         [0,1,0,0],
                         [-Sy,0,Cy,0],
                         [0,0,0,1]])
    mat_rotZ = matrix([[Cz,-Sz,0,0],
                         [Sz,Cz,0,0],
                         [0,0,1,0],
                         [0,0,0,1]])
    return mat_rotZ*mat_rotY*mat_rotX*mat_trans

def cartesian_to_matrix(cp):
    position = cp.position
    orientation = cp.orientation
    mat = np.eye(4)
    # translation
    mat[0,3] = position.x# in meter
    mat[1,3] = position.y
    mat[2,3] = position.z
    # quaternion to matrix
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w

    Nq = w*w + x*x + y*y + z*z
    if Nq < 0.001:
        return mat

    s = 2.0/Nq
    X = x*s
    Y = y*s
    Z = z*s
    wX = w*X; wY = w*Y; wZ = w*Z
    xX = x*X; xY = x*Y; xZ = x*Z
    yY = y*Y; yZ = y*Z; zZ = z*Z
    mat = matrix([[1.0-(yY+zZ), xY-wZ, xZ+wY, position.x],
            [xY+wZ, 1.0-(xX+zZ), yZ-wX, position.y],
            [xZ-wY, yZ+wX, 1.0-(xX+yY), position.z],
            [0, 0, 0, 1]])
    return mat

# homougenous matrix to quaternion
# return Pose()
def matrix_to_cartesian(mat):
    rot = np.array([mat[0,0:3],mat[1,0:3],mat[2,0:3]])
    trans = np.array([mat[0,3],mat[1,3],mat[2,3]])
    x = trans[0]
    y = trans[1]
    z = trans[2]
    qw = 0.5*sqrt(1+rot[0,0]+rot[1,1]+rot[2,2])
    qx = (rot[2,1]-rot[1,2])/(4*qw)
    qy = (rot[0,2]-rot[2,0])/(4*qw)
    qz = (rot[1,0]-rot[0,1])/(4*qw)

    cp = Pose()
    cp.position.x = x
    cp.position.y = y
    cp.position.z = z
    cp.orientation.w = qw
    cp.orientation.x = qx
    cp.orientation.y = qy
    cp.orientation.z = qz
    return cp

# return Pose()
def eular_to_cartesian(self,p,a,b,c):
    cp = Pose()
    cp.position.x = p[0]
    cp.position.y = p[1]
    cp.position.z = p[2]
    w,x,y,z = eularangle_to_quaternion(c,b,a)
    cp.orientation.w = w
    cp.orientation.x = x
    cp.orientation.y = y
    cp.orientation.z = z
    return cp
