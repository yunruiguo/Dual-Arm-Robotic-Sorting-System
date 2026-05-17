
import cv2, time
import socket
import pickle
import struct
import numpy as np
from transforms import pose_to_matrix, transform_pose, invert_transform
import cv2
import numpy as np

def project_to_2d(point_3d, K):
        """ 3D  2D image"""
        X, Y, Z = point_3d
        u = int((K['fx'] * X) / Z + K['cx'])
        v = int((K['fy'] * Y) / Z + K['cy'])
        return u, v

def select_best_grasp(pred_gg, masks_dict, K):
    """
    .
    
    :param pred_gg: ,, translations, rotations, scores.
    :param masks_dict: ,,image.
    :param K: .
    :return: .
    """
    
    best_grasps = {}
    # for i in range(pred_gg.translations.shape[0]):
    #     translation = pred_gg.translations[i]  # 3D 
    #     rotation = pred_gg.rotations[i].flatten()  # 
    #     score = pred_gg.scores[i].item()  # 
    for i in range(len(pred_gg["scores"])):
        translation = [x * 1000 for x in pred_gg["translations"][i]]  # 3D  mm
        rotation = pred_gg["rotations"][i] # 
        score = pred_gg["scores"][i] # 
        
        #  2D
        u, v = project_to_2d(translation, K)
        
        found_object = False
        for object_id, mask in masks_dict.items():
            # image
            if (0 <= u < mask['segmentation'].shape[1] and 0 <= v < mask['segmentation'].shape[0]) and mask['segmentation'][v, u]:
                category = object_id.split('-')[0]  # 
                
                # 
                if object_id not in best_grasps or best_grasps[object_id][0] < score:
                    best_grasps[object_id] = (score, translation, rotation, category)
                found_object = True
                break  # 
            
        if not found_object:
            print(f"Point ({u}, {v}) does not belong to any object.")
    
    return best_grasps

# def select_best_grasp_dual(pred_gg_left, pred_gg_right, masks_dict, K_left, K_right, camera2base_matrix,
#                            obj_in_other_arm_poses):
#     """
#     ,.
    
#     :param pred_gg_left:  (dict: translations, rotations, scores)
#     :param pred_gg_right:  (dict: translations, rotations, scores)
#     :param masks_dict:  ( segmentation )
#     :param K_left: 
#     :param K_right: 
#     :param camera2base_matrix:  (4x4)
#     :param obj_in_other_arm_poses:  (4x4)
#     :return: best_grasps -  (,,,,)
#     """
    
#     best_grasps = {}

#     # 
#     for i in range(len(pred_gg_left["scores"])):
#         translation = [x * 1000 for x in pred_gg_left["translations"][i]]  #  mm
#         rotation = pred_gg_left["rotations"][i]
#         score = pred_gg_left["scores"][i]
        
#         #  2D
#         u, v = project_to_2d(translation, K_left)

#         for object_id, mask in masks_dict.items():
#             if (0 <= u < mask['segmentation'].shape[1] and 0 <= v < mask['segmentation'].shape[0]) and mask['segmentation'][v, u]:
#                 category = object_id.split('-')[0]
                
#                 if object_id not in best_grasps or best_grasps[object_id]["score"] < score:
#                     best_grasps[object_id] = {"score": score, "translation": translation, "rotation": rotation, "category": category, "source": "left"}
#                 break

#     # ,
#     for i in range(len(pred_gg_right["scores"])):
#         x ,y, z = obj_in_other_arm_poses[i][:3]
#         P_world = np.array([x,y,z, 1]).reshape(4, 1)
#         P_cam = invert_transform(camera2base_matrix) @ P_world
#         P_cam = P_cam[:3,0]
        
#         translation = [x * 1000 for x in pred_gg_right["translations"][i]]  #  mm
#         rotation = pred_gg_right["rotations"][i]
#         score = pred_gg_right["scores"][i]
        
#         # image
#         u, v = project_to_2d(P_cam, K_left)

#         for object_id, mask in masks_dict.items():
#             if (0 <= u < mask['segmentation'].shape[1] and 0 <= v < mask['segmentation'].shape[0]) and mask['segmentation'][v, u]:
#                 category = object_id.split('-')[0]
                
#                 if object_id not in best_grasps or best_grasps[object_id]["score"] < score:
#                     best_grasps[object_id] = {"score": score, "translation": translation, "rotation": rotation, "category": category, "source": "right"}
#                 break

#     return best_grasps


def select_best_grasp_dual(pred_gg_left, pred_gg_right, masks_dict, K_left, K_right, camera2base_matrix,
                           obj_in_other_arm_poses):
    """
    ,.
    
    :param pred_gg_left:  (dict: translations, rotations, scores)
    :param pred_gg_right:  (dict: translations, rotations, scores)
    :param masks_dict:  ( segmentation )
    :param K_left: 
    :param K_right: 
    :param camera2base_matrix:  (4x4)
    :param obj_in_other_arm_poses:  (4x4)
    :return: best_grasps -  (,,,,)
    """
    
    best_grasps = {}
    counter = 1

    #  RGB image 2D 
    mask_height, mask_width = list(masks_dict.values())[0]['segmentation'].shape
    combined_mask_image = np.zeros((mask_height, mask_width, 3), dtype=np.uint8)

    # ,
    left_color = (255, 0, 0)   # 
    right_color = (0, 0, 255)  # 

    def draw_point_with_label(image, u, v, label, color):
        """Draw points on the image."""
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 1
        text_size = cv2.getTextSize(label, font, font_scale, thickness)[0]
        
        # 
        cv2.circle(image, (u, v), radius=5, color=color, thickness=-1)
        
        # ,image
        text_x = max(u - text_size[0] // 2, 0)
        text_y = min(v + text_size[1], image.shape[0])
        
        # 
        cv2.putText(image, label, (text_x, text_y), font, font_scale, color, thickness, cv2.LINE_AA)

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
                
                # image 2D ,
                draw_point_with_label(combined_mask_image, u, v, f"L{counter}", left_color)
                counter += 1
                break

    # ,
    for i in range(len(pred_gg_right["scores"])):
        x ,y, z = obj_in_other_arm_poses[i][:3]
        P_world = np.array([x,y,z, 1]).reshape(4, 1)
        P_cam = invert_transform(camera2base_matrix) @ P_world
        P_cam = P_cam[:3,0]
        
        translation = [x * 1000 for x in pred_gg_right["translations"][i]]  #  mm
        rotation = pred_gg_right["rotations"][i]
        score = pred_gg_right["scores"][i]
        
        # image
        u, v = project_to_2d(P_cam, K_left)

        for object_id, mask in masks_dict.items():
            if (0 <= u < mask['segmentation'].shape[1] and 0 <= v < mask['segmentation'].shape[0]) and mask['segmentation'][v, u]:
                category = object_id.split('-')[0]
                
                if object_id not in best_grasps or best_grasps[object_id]["score"] < score:
                    best_grasps[object_id] = {"score": score, "translation": translation, "rotation": rotation, "category": category, "source": "right"}
                
                # image 2D ,
                draw_point_with_label(combined_mask_image, u, v, f"R{counter}", right_color)
                counter += 1
                break

    # image
    for idx, (object_id, mask) in enumerate(masks_dict.items()):
        colored_mask = np.zeros_like(combined_mask_image)
        colored_mask[mask['segmentation'] > 0] = (255, 255, 255)  # 
        combined_mask_image = cv2.addWeighted(combined_mask_image, 1, colored_mask, 0.3, 0)  # 

    # # 
    # cv2.imshow("Combined Mask with 2D Projections", combined_mask_image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    return best_grasps

def send_to_server(server_address, serialized_data):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            start_time = time.time()
            client_socket.connect(server_address)
            client_socket.sendall(struct.pack('>L', len(serialized_data)) + serialized_data)

            # 
            raw_data = b''
            while len(raw_data) < 4:
                raw_data += client_socket.recv(4)
            data_size = struct.unpack('>L', raw_data[:4])[0]
            raw_data = raw_data[4:]

            while len(raw_data) < data_size:
                raw_data += client_socket.recv(4096)
            processed_data = pickle.loads(raw_data[:data_size])
            end_time = time.time()
            print(f"Cloud services process time for {server_address}: {end_time - start_time}")
            return processed_data
    except socket.error as se:
        print(f"Socket error: {se}")
    except pickle.UnpicklingError as pe:
        print(f"Unpickling error: {pe}")
    except Exception as e:
        print(f"Unexpected error in send_image: {e}")
    return None

def create_rotation_matrix(axis, theta_deg):
        """"""
        theta_rad = np.radians(theta_deg)
        c, s = np.cos(theta_rad), np.sin(theta_rad)
        
        if axis == 'x':
            rotation_matrix_3x3 = np.array([
                [1, 0, 0],
                [0, c, -s],
                [0, s, c]
            ])
        elif axis == 'y':
            rotation_matrix_3x3 = np.array([
                [c, 0, s],
                [0, 1, 0],
                [-s, 0, c]
            ])
        elif axis == 'z':
            rotation_matrix_3x3 = np.array([
                [c, -s, 0],
                [s, c, 0],
                [0, 0, 1]
            ])
        else:
            raise ValueError("Axis must be 'x', 'y', or 'z'")

        rotation_matrix = np.eye(3)
        rotation_matrix = rotation_matrix_3x3
    
        return rotation_matrix