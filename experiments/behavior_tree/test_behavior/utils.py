'''
FilePath: utils.py
Author: Yunrui Guo
Date: 2024-12-18 15:56:35
LastEditors: Please set LastEditors
LastEditTime: 2025-03-26 14:20:16

Descripttion: 
'''
import numpy as np

def transform_grasp_pose(T_grasp, R_from_cam, R_to_cam):
    """
    
    
    :param T_grasp: np.array (4,4) - 
    :param R_from_cam: np.array (4,4) - 
    :param R_to_cam: np.array (4,4) -  ( R_base_cam )
    :return: np.array (4,4) - 
    """
    return R_to_cam @ R_from_cam @ T_grasp

def project_to_2d(point_3d, K):
    """ 3D  2D image"""
    X, Y, Z = point_3d
    u = int((K['fx'] * X) / Z + K['cx'])
    v = int((K['fy'] * Y) / Z + K['cy'])
    return u, v

def select_best_grasp_dual(pred_gg_left, pred_gg_right, masks_dict, K_left, K_right, R1, R2, R3):
    """
    ,.
    
    :param pred_gg_left:  (dict: translations, rotations, scores)
    :param pred_gg_right:  (dict: translations, rotations, scores)
    :param masks_dict:  ( segmentation )
    :param K_left: 
    :param K_right: 
    :param R1:  (4x4)
    :param R2:  (4x4)
    :param R3:  (4x4)
    
    :return: best_grasps -  (,,,)
    """
    
    best_grasps = {}

    # 
    for i in range(len(pred_gg_left["scores"])):
        translation = [x * 1000 for x in pred_gg_left["translations"][i]]  #  mm
        rotation = pred_gg_left["rotations"][i]
        score = pred_gg_left["scores"][i]
        
        #  2D
        u, v = project_to_2d(translation, K_left)

        for object_id, mask in masks_dict.items():
            if (0 <= u < mask['segmentation'].shape[1] and 0 <= v < mask['segmentation'].shape[0]) and mask['segmentation'][v, u]:
                category = object_id.split('-')[0]
                
                if object_id not in best_grasps or best_grasps[object_id]["score"] < score:
                    best_grasps[object_id] = {"score": score, "translation": translation, "rotation": rotation, "category": category, "source": "left"}
                break

    # ,
    for i in range(len(pred_gg_right["scores"])):
        T_right_cam = np.eye(4)
        T_right_cam[:3, :3] = pred_gg_right["rotations"][i]
        T_right_cam[:3, 3] = [x * 1000 for x in pred_gg_right["translations"][i]]  #  mm
        
        # 
        T_left_cam = transform_grasp_pose(T_right_cam, R1, np.linalg.inv(R2) @ R3)
        translation = T_left_cam[:3, 3]  # 
        rotation = T_left_cam[:3, :3]  # 
        score = pred_gg_right["scores"][i]
        
        # image
        u, v = project_to_2d(translation, K_left)

        for object_id, mask in masks_dict.items():
            if (0 <= u < mask['segmentation'].shape[1] and 0 <= v < mask['segmentation'].shape[0]) and mask['segmentation'][v, u]:
                category = object_id.split('-')[0]
                
                if object_id not in best_grasps or best_grasps[object_id]["score"] < score:
                    best_grasps[object_id] = {"score": score, "translation": translation, "rotation": rotation, "category": category, "source": "right"}
                break

    return best_grasps
