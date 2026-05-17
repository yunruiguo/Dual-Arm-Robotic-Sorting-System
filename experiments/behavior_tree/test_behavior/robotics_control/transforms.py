import numpy as np
from scipy.spatial.transform import Rotation as R
import math

def pose_to_matrix(pose):
    """"""
    trans = pose[:3]
    rot = R.from_euler('xyz', pose[3:]).as_matrix()
    matrix = np.eye(4)
    matrix[:3, :3] = rot
    matrix[:3, 3] = trans
    return matrix

def transform_pose(pose, transformation):
    """"""
    pose_matrix = pose_to_matrix(pose)
    transformed = np.dot(transformation, pose_matrix)
    translation = transformed[:3, 3]
    rotation = R.from_matrix(transformed[:3, :3]).as_euler('xyz')
    return np.concatenate((translation, rotation))

def invert_transform(matrix):
    """"""
    rot_inv = matrix[:3, :3].T
    trans_inv = -rot_inv @ matrix[:3, 3]
    inv_matrix = np.eye(4)
    inv_matrix[:3, :3] = rot_inv
    inv_matrix[:3, 3] = trans_inv
    return inv_matrix
def create_transformation(translation, rotation, is_radians=True, rot_6d=None):
    """
    
    :param translation: [x, y, z] 
    :param rotation: [rx, ry, rz] [rx, ry, rz, rw]
    :param is_radians: 
    :return: 4x4
    """
    matrix = np.eye(4)
    matrix[:3, 3] = translation

    if rot_6d:
        rot = np.array(rot_6d).reshape(3,3)
        matrix[:3, :3] = rot
    else:   
        if len(rotation) == 3:  # 
            if not is_radians:
                rotation = np.radians(rotation)
            matrix[:3, :3] = R.from_euler('xyz', rotation).as_matrix()
        elif len(rotation) == 4:  # 
            matrix[:3, :3] = R.from_quat(rotation).as_matrix()
    return matrix

def generate_grasp_pose( u, v, z,cx,cy,fx,fy):
    """
    get 3d grasp points in camera color frame
    """
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    return [x, y, z]
def get_width(u, v, z, width ,cx,cy,fx,fy):
    #
    x1 = (u - cx) * z / fx
    y1= (v - cy) * z / fy
    x2 = (u+width - cx) * z / fx
    y2= (v - cy) * z / fy
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return(distance)