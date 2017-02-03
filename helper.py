
import rospy
import sensor_msgs.msg
import geometry_msgs.msg
import message_filters
import numpy as np
import math
import os
import sys
import yaml
import pprint
import tf
import cv2
import cPickle
from PyKDL import *
import tf_conversions
import transformations
import se3d
pi = math.pi

def originrpy2mtx(rpy,origin):
    rot = transformations.quaternion_from_euler(*rpy)
    H = transformations.quaternion_matrix(rot)  
    H[0:3, 3] = origin 
    return H

def mtx2originrpy(mtx):
    q = transformations.quaternion_from_matrix(mtx)
    e = transformations.euler_from_quaternion(q)
    return mtx[0:3, 3],e

def parent_child2urdf(mtx):
    #r = np.linalg.inv(mtx)
    return mtx2originrpy(mtx)

def pose2mtx(m):
    return tf_conversions.toMatrix(tf_conversions.fromMsg(m))

def tf2mtx(p, q):
    return tf_conversions.toMatrix(Frame(Rotation.Quaternion(*q), Vector(*p)))

def list2colvec(x):
    w = np.array(x,dtype=np.float32)
    return w.reshape(w.size,1)

def posequat2pose(p, q):
    return mtx2pose(tf2mtx(p, q))

def ci2dict(ci):
    r = dict()
    r["width"] = ci.width
    r["height"] = ci.height
    #np is RM
    r["camera_matrix"] = np.reshape(np.array(ci.K),(3,3))
    r["dist"] = np.array(ci.D)
    return r

def mtx2pose(m):
    return tf_conversions.toMsg(tf_conversions.fromMatrix(m))

#origin->SetAttribute("xyz", pose_xyz_str);
#origin->SetAttribute("rpy", pose_rpy_str);

def posrvec2mtx(tvec,rvec):
    tvec = np.array(tvec,dtype=np.float64)
    rvec = np.array(rvec,dtype=np.float64)
    q = np.identity(4)
    a,b = cv2.Rodrigues(rvec)
    q[0:3,0:3] = a
    q[0,3] = tvec[0]
    q[1,3] = tvec[1]
    q[2,3] = tvec[2]
    return q

def posrvec2pose(p, r):
    return mtx2pose(posrvec2mtx(p,r))

def quat2aa(q):
    #xyzw to wxyz
    R = transformations.quaternion_matrix(q)  
    w,J = cv2.Rodrigues(R)
    n = np.linalg.norm(w)
    return [180*n/math.pi,w/n]

