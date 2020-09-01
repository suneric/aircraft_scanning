#!/usr/bin/env python
import numpy as np
from numpy import pi, sqrt, cos, sin, arctan2, array, matrix
from numpy.linalg import norm
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_matrix, quaternion_matrix, euler_from_quaternion
from math import *

# transform for mobile manipulator scanning

def valid_catersian_pose(arm_pose):
    pE0 = matrix([[arm_pose.position.x],
                  [arm_pose.position.y],
                  [arm_pose.position.z]])
    qE0 = array([arm_pose.orientation.x,
                 arm_pose.orientation.y,
                 arm_pose.orientation.z,
                 arm_pose.orientation.w])
    l02 = 0.36
    l24 = 0.42
    l46 = 0.4
    l6E = 0.126
    pE6 = matrix([[0.0], [0.0], [l6E]])
    p20 = matrix([[0.0], [0.0], [l02]])

    RE0 = matrix(quaternion_matrix(qE0)[:3,:3])
    p6E0 = RE0 * pE6
    p60 = pE0 - p6E0
    p260 = p60 - p20

    s = norm(p260)
    if s > self.l24 + self.l46:
      print('invalid pose command')
      return False

    return True

def catersian_to_joint(arm_pose):
    t = 7*[0.0]
    pE0 = matrix([[arm_pose.position.x],
                  [arm_pose.position.y],
                  [arm_pose.position.z]])
    qE0 = array([arm_pose.orientation.x,
                 arm_pose.orientation.y,
                 arm_pose.orientation.z,
                 arm_pose.orientation.w])
    l02 = 0.36
    l24 = 0.42
    l46 = 0.4
    l6E = 0.126
    pE6 = matrix([[0.0], [0.0], [l6E]])
    p20 = matrix([[0.0], [0.0], [l02]])

    RE0 = matrix(quaternion_matrix(qE0)[:3,:3])
    p6E0 = RE0 * pE6
    p60 = pE0 - p6E0
    p260 = p60 - p20

    s = norm(p260)
    if s > self.l24 + self.l46:
      print('invalid pose command')
      return

    (tys, tzs) = rr(p260)
    tp24z0 = 1/(2.0 * s) * (self.l24**2 - self.l46**2 + s**2)
    tp240 = matrix([[-sqrt(self.l24**2 - tp24z0**2)], [0.0], [tp24z0]])
    p240 = Ryz(tys, tzs) * Rz(self.tr) * tp240
    (t[1], t[0]) = rr(p240)

    R20 = Ryz(t[1], t[0])
    p40 = p20 + p240
    p460 = p60 - p40
    p462 = R20.T * p460
    (t[3], t[2]) = rr(p462)
    t[3] = -t[3]

    R42 = Ryz(-t[3], t[2])
    R40 = R20 * R42
    p6E4 = R40.T * p6E0
    (t[5], t[4]) = rr(p6E4)

    R64 = Ryz(t[5], t[4])
    R60 = R40 * R64
    RE6 = R60.T * RE0
    t[6] = arctan2(RE6[1,0], RE6[0,0])

    return t

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

def rr(p):
  ty = arctan2(sqrt(p[0,0]**2 + p[1,0]**2), p[2,0])
  tz = arctan2(p[1,0], p[0,0])

  if tz < -pi/2.0:
    ty = -ty
    tz += pi
  elif tz > pi/2.0:
    ty = -ty
    tz -= pi

  return (ty, tz)

def Rz(tz):
  (cz, sz) = trigonometry(tz)
  return matrix([[ cz, -sz, 0.0],
                 [ sz,  cz, 0.0],
                 [0.0, 0.0, 1.0]])

def Ryz(ty, tz):
  (cy, sy) = trigonometry(ty)
  (cz, sz) = trigonometry(tz)
  return matrix([[cy * cz, -sz, sy * cz],
                 [cy * sz, cz, sy * sz],
                 [-sy, 0.0, cy]])


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
    # find position of quadrotor and angle of camera joint
    qw = viewpoint.orientation.w
    qx = viewpoint.orientation.x
    qy = viewpoint.orientation.y
    qz = viewpoint.orientation.z
    eular_angle = euler_from_quaternion([qx,qy,qz,qw])
    # as the camera rs_d435 rotates about z axis -pi/2
    angle = eular_angle[0]
    # pitch = eular_angle[1]
    # yaw = eular_angle[2]
    #print("yaw,pitch,roll",yaw,pitch,roll)
    
    # pose*T_base*T_camjoint = vp
    matvp = cartesian_to_matrix(viewpoint)
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
    cp = Pose()
    cp.position.x = mat[0,3]
    cp.position.y = mat[1,3]
    cp.position.z = mat[2,3]
    q = quaternion_from_matrix(mat)
    cp.orientation.x = q[0]
    cp.orientation.y = q[1]
    cp.orientation.z = q[2]
    cp.orientation.w = q[3]
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
