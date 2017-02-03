
import rospy
import sensor_msgs.msg
import geometry_msgs.msg
import ramcip_msgs.msg
import message_filters
import numpy as np
import cv2
import math
import os
import sys
import yaml
import pprint
import tf
from cv_bridge import CvBridge, CvBridgeError
import cPickle
from PyKDL import *
import tf_conversions
from ramcip_calib_sssa.srv import *
import transformations
import se3d


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
    q = np.identity(4)
    a,b = cv2.Rodrigues(rvec)
    q[0:3,0:3] = a
    q[0,3] = tvec[0]
    q[1,3] = tvec[1]
    q[2,3] = tvec[2]
    return q

def posrvec2pose(p, r):
    return mtx2pose(posrvec2mtx(p,r))


pos =[-0.04225474 ,0.05194644, 0.00081353]
quat_xyzw = [0.10640583735010758, 0.008415271547103753, 0.004443887926837299, 0.9942772414385762]

urdf_pos,urdf_rpy =parent_child2urdf(pose2mtx(posequat2pose(pos,quat_xyzw)))
print "<origin xyz=\"%f %f %f\" rpy=\"%f %f %f\"/>" % (urdf_pos[0],urdf_pos[1],urdf_pos[2],	urdf_rpy[0],urdf_rpy[1],urdf_rpy[2])
print urdf_rpy[0]*180/math.pi,urdf_rpy[1]*180/math.pi,urdf_rpy[2]*180/math.pi

pos = [0,0,0]
quat_xyzw = [0.10452846326765346, 0, 0 ,0.9945218953682733]
urdf_pos,urdf_rpy =parent_child2urdf(pose2mtx(posequat2pose(pos,quat_xyzw)))
print "<origin xyz=\"%f %f %f\" rpy=\"%f %f %f\"/>" % (urdf_pos[0],urdf_pos[1],urdf_pos[2],	urdf_rpy[0],urdf_rpy[1],urdf_rpy[2])
print urdf_rpy[0]*180/math.pi,urdf_rpy[1]*180/math.pi,urdf_rpy[2]*180/math.pi