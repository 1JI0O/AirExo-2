# -*- coding: utf-8 -*-
from __future__ import print_function
import numpy as np
import sys, io

def mm(*args):
    result = args[0]
    for a in args[1:]:
        result = np.dot(result, a)
    return result

extrinsics = {
    '105422061350': np.array([
        [-0.03884323, -0.99906784,  0.01883331,  0.01281536],
        [-0.91665816,  0.02812369, -0.39868146,  0.024272  ],
        [ 0.39778018, -0.03274978, -0.91689605,  0.68936044],
        [ 0.,          0.,          0.,          1.        ]], dtype=np.float32),
    '104122064161': np.array([
        [-0.94874346, -0.25861034, -0.18167697,  0.07677209],
        [-0.18934393,  0.9253561 , -0.32842812, -0.03099394],
        [ 0.25305077, -0.2771946 , -0.9268918 ,  0.38643917],
        [ 0.,          0.,          0.,          1.        ]], dtype=np.float32),
    '104122061330': np.array([
        [ 0.9448085 , -0.29227304,  0.14803174, -0.09153452],
        [-0.2114339 , -0.8890973 , -0.4059578 , -0.0019359 ],
        [ 0.25026512,  0.35225344, -0.9018231 ,  0.39790273],
        [ 0.,          0.,          0.,          1.        ]], dtype=np.float32),
}

robot_left_tcp_pose  = np.array([ 0.47990388,  0.05316564,  0.39616618,
                                   0.1864741 ,  0.5887938 ,  0.78571755, -0.0346007 ], dtype=np.float32)
robot_right_tcp_pose = np.array([ 0.5281115 , -0.00951796,  0.3995147 ,
                                   0.18649498, -0.56400096,  0.8018281 ,  0.06476301], dtype=np.float32)

ROBOT_LEFT_FLANGE_TO_CAM = np.array([
    [-0.01240050, 0.99905890, 0.04157753, -0.09342833],
    [-0.99978572, -0.01307791, 0.01605635,  0.02158097],
    [ 0.01658444, -0.04136945, 0.99900651, -0.00380356],
    [ 0.,          0.,          0.,          1.        ]], dtype=np.float32)

ROBOT_RIGHT_FLANGE_TO_CAM = np.array([
    [-0.10785065, 0.99415284, 0.00529436, -0.08709522],
    [-0.99405175, -0.10791755, 0.01465851,  0.00088501],
    [ 0.01514411, -0.00368180, 0.99987853, -0.00513650],
    [ 0.,          0.,          0.,          1.        ]], dtype=np.float32)

ROBOT_TCP_TO_FLANGE = np.array([
    [1., 0., 0.,  0.    ],
    [0., 1., 0.,  0.    ],
    [0., 0., 1., -0.170 ],
    [0., 0., 0.,  1.    ]], dtype=np.float32)

ROBOT_LEFT_TCP_TO_CAM  = mm(ROBOT_TCP_TO_FLANGE, ROBOT_LEFT_FLANGE_TO_CAM)
ROBOT_RIGHT_TCP_TO_CAM = mm(ROBOT_TCP_TO_FLANGE, ROBOT_RIGHT_FLANGE_TO_CAM)
ROBOT_LEFT_CAM_TO_TCP  = np.linalg.inv(ROBOT_LEFT_TCP_TO_CAM)
ROBOT_RIGHT_CAM_TO_TCP = np.linalg.inv(ROBOT_RIGHT_TCP_TO_CAM)

ROBOT_LEFT_REAL_BASE_TO_REAL_BASE = np.array([
    [1., 0., 0.,  0.    ],
    [0., 1., 0., -0.135 ],
    [0., 0., 1.,  0.    ],
    [0., 0., 0.,  1.    ]], dtype=np.float32)

ROBOT_RIGHT_REAL_BASE_TO_REAL_BASE = np.array([
    [1., 0., 0., 0.    ],
    [0., 1., 0., 0.135 ],
    [0., 0., 1., 0.    ],
    [0., 0., 0., 1.    ]], dtype=np.float32)

ROBOT_PREDEFINED_TRANSFORMATION = np.array([
    [0., 1., 0., 0.],
    [0., 0., 1., 0.],
    [1., 0., 0., 0.],
    [0., 0., 0., 1.]], dtype=np.float32)


def quat_wxyz_to_matrix(q_wxyz):
    w, x, y, z = float(q_wxyz[0]), float(q_wxyz[1]), float(q_wxyz[2]), float(q_wxyz[3])
    mat = np.eye(4, dtype=np.float32)
    mat[0,0] = 1 - 2*(y*y + z*z)
    mat[0,1] = 2*(x*y - z*w)
    mat[0,2] = 2*(x*z + y*w)
    mat[1,0] = 2*(x*y + z*w)
    mat[1,1] = 1 - 2*(x*x + z*z)
    mat[1,2] = 2*(y*z - x*w)
    mat[2,0] = 2*(x*z - y*w)
    mat[2,1] = 2*(y*z + x*w)
    mat[2,2] = 1 - 2*(x*x + y*y)
    return mat

def matrix_to_quat_wxyz(R):
    trace = R[0,0] + R[1,1] + R[2,2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2,1] - R[1,2]) * s
        y = (R[0,2] - R[2,0]) * s
        z = (R[1,0] - R[0,1]) * s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        w = (R[2,1] - R[1,2]) / s
        x = 0.25 * s
        y = (R[0,1] + R[1,0]) / s
        z = (R[0,2] + R[2,0]) / s
    elif R[1,1] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        w = (R[0,2] - R[2,0]) / s
        x = (R[0,1] + R[1,0]) / s
        y = 0.25 * s
        z = (R[1,2] + R[2,1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        w = (R[1,0] - R[0,1]) / s
        x = (R[0,2] + R[2,0]) / s
        y = (R[1,2] + R[2,1]) / s
        z = 0.25 * s
    return np.array([w, x, y, z], dtype=np.float32)

def xyz_rot_to_mat(xyz_rot):
    t = xyz_rot[:3]
    q = xyz_rot[3:]
    mat = quat_wxyz_to_matrix(q)
    mat[:3, 3] = t
    return mat

def mat_to_xyz_rot(mat):
    t = mat[:3, 3]
    q = matrix_to_quat_wxyz(mat[:3, :3])
    return np.concatenate([t, q])

def average_xyz_rot_quat(mat1, mat2):
    v1 = mat_to_xyz_rot(mat1)
    v2 = mat_to_xyz_rot(mat2)
    t  = (v1[:3] + v2[:3]) / 2.0
    q1, q2 = v1[3:], v2[3:]
    if np.dot(q1, q2) < 0:
        q2 = -q2
    q = (q1 + q2) / 2.0
    q /= np.linalg.norm(q)
    res = np.concatenate([t, q])
    return xyz_rot_to_mat(res)


serial_global   = '105422061350'
serial_inhand_l = '104122064161'
serial_inhand_r = '104122061330'

tcp_mat_left  = xyz_rot_to_mat(robot_left_tcp_pose)
tcp_mat_right = xyz_rot_to_mat(robot_right_tcp_pose)

left_cam_to_base  = mm(extrinsics[serial_global],
                       np.linalg.inv(extrinsics[serial_inhand_l]),
                       ROBOT_LEFT_CAM_TO_TCP,
                       np.linalg.inv(tcp_mat_left))

right_cam_to_base = mm(extrinsics[serial_global],
                       np.linalg.inv(extrinsics[serial_inhand_r]),
                       ROBOT_RIGHT_CAM_TO_TCP,
                       np.linalg.inv(tcp_mat_right))

left_cam_to_base_real  = mm(left_cam_to_base,  ROBOT_LEFT_REAL_BASE_TO_REAL_BASE)
right_cam_to_base_real = mm(right_cam_to_base, ROBOT_RIGHT_REAL_BASE_TO_REAL_BASE)

cam_to_base_real = average_xyz_rot_quat(left_cam_to_base_real, right_cam_to_base_real)
base_to_cam_real = np.linalg.inv(cam_to_base_real)

out = io.open(sys.stdout.fileno(), "w", encoding="utf-8", closefd=False)
np.set_printoptions(precision=6, suppress=True)

def pr(label, mat):
    out.write(u"\n===== " + label + u" =====\n")
    lines = repr(mat).split('\n')
    for l in lines:
        out.write(u"  " + l.decode('utf-8') + u"\n")
    v = mat_to_xyz_rot(mat)
    out.write(u"  7D [x,y,z,qw,qx,qy,qz]: " + repr(v) + u"\n")

pr(u"left  cam_to_base (real_base=True)", left_cam_to_base_real)
pr(u"right cam_to_base (real_base=True)", right_cam_to_base_real)
pr(u"averaged cam_to_base (real_base=True) = T_{cam<-real_base}", cam_to_base_real)
pr(u"base_to_cam = T_{real_base<-cam} = camera POSE in real_base", base_to_cam_real)
out.write(u"  camera origin (xyz only): " + repr(base_to_cam_real[:3,3]) + u"\n")

cam_to_base_urdf = mm(cam_to_base_real, np.linalg.inv(ROBOT_PREDEFINED_TRANSFORMATION))
base_to_cam_urdf = np.linalg.inv(cam_to_base_urdf)
pr(u"base_to_cam in URDF_base frame (real_base=False)", base_to_cam_urdf)
