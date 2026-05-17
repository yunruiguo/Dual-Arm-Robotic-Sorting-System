'''
FilePath: src/vision/box_opening_keypoint_node.py
Author: Yunrui Guo
Date: 2024-12-24 15:06:11
LastEditors: Please set LastEditors
LastEditTime: 2024-12-25 16:02:53

Descripttion: 
'''
'''
FilePath: src/vision/box_opening_keypoint_node.py
Author: Yunrui Guo
Date: 2024-07-30 15:43:30
LastEditors: Please set LastEditors
LastEditTime: 2024-12-23 17:15:51

Descripttion: 
'''
import argparse
import os
import sys
import time
import numpy as np
import json
import torch
import re
from cv_bridge import CvBridge
from PIL import Image
from pathlib import Path
sys.path.append(os.path.join(os.getcwd(), "GroundingDINO"))
sys.path.append(os.path.join(os.getcwd(), "segment_anything"))
sys.path.append(os.path.join(os.getcwd(), "box_centerline_utils"))
sys.path.append(os.path.join(os.getcwd(), "box_centerline_utils/transform_pix2world"))
import rclpy
from rclpy.node import Node
import threading
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image as ROSImage
from sensor_msgs.msg import CameraInfo
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray,String
# Grounding DINO
import GroundingDINO.groundingdino.datasets.transforms as T
from GroundingDINO.groundingdino.models import build_model
from GroundingDINO.groundingdino.util.slconfig import SLConfig
from GroundingDINO.groundingdino.util.utils import clean_state_dict, get_phrases_from_posmap
from box_centerline_utils.transform_pix2world.get_robot_extrinsic_and_worldPoint import enable_robot, GetRobotExtrinsic
from box_centerline_utils.box_centerLine import box_centerLine_and_9_points

# segment anything
from segment_anything import (
    sam_model_registry,
    sam_hq_model_registry,
    SamPredictor
)
import cv2
import matplotlib.pyplot as plt
#import pyrealsense2 as rs
from typing import Union, Any, Optional
from pyorbbecsdk import *
import pdb# Python debugging tool
from GraspNet.model.FGC_graspnet import FGC_graspnet
from GraspNet.model.decode import pred_decode
# from GraspNet.utils.data_utils import CameraInfo, create_point_cloud_from_depth_image
from GraspNet.utils.collision_detector import ModelFreeCollisionDetector



from graspnetAPI import GraspGroup
import open3d as o3d
from scipy.ndimage import map_coordinates
def parse_args():
    parser = argparse.ArgumentParser("Grounded-Segment-Anything Demo", add_help=False)
    parser.add_argument("--config", type=str,default="GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py",help="path to config file")
    parser.add_argument(
        "--grounded_checkpoint", type=str, default="groundingdino_swint_ogc.pth",help="path to checkpoint file"
    )
    parser.add_argument(
        "--sam_version", type=str, default="vit_h",help="SAM ViT version: vit_b / vit_l / vit_h"
    )
    parser.add_argument(
        "--sam_checkpoint", type=str,default="sam_vit_h_4b8939.pth",help="path to sam checkpoint file"
    )
    parser.add_argument(
        "--sam_hq_checkpoint", type=str, default=None, help="path to sam-hq checkpoint file"
    )
    parser.add_argument(
        "--use_sam_hq", action="store_true", help="using sam-hq for prediction"
    )
    parser.add_argument("--input_image", type=str, help="path to image file")
    # parser.add_argument("--text_prompt", type=str, required=True,  help="text prompt")
    parser.add_argument("--text_prompt", type=str,default="box",  help="text prompt")
    parser.add_argument(
        "--output_dir", "-o", type=str, default="outputs",help="output directory"
    )

    parser.add_argument("--box_threshold", type=float, default=0.3, help="box threshold")
    parser.add_argument("--text_threshold", type=float, default=0.25, help="text threshold")
    parser.add_argument('--image_path', default='assets/input_image')
    parser.add_argument('--depth_path', default='assets/input_depth')
    parser.add_argument("--device", type=str, default="cuda", help="running on cpu only!, default=False")
    #grasp_net
    parser.add_argument('--checkpoint_grasp_path', default='checkpoint_fgc.tar', help='Model checkpoint path')
    parser.add_argument('--num_point', type=int, default=12000, help='Point Number [default: 20000]')
    parser.add_argument('--num_view', type=int, default=300, help='View Number [default: 300]')
    parser.add_argument('--collision_thresh', type=float, default=0.01, help='Collision Threshold in collision detection [default: 0.01]')
    parser.add_argument('--voxel_size', type=float, default=0.01, help='Voxel Size to process point clouds before collision detection [default: 0.01]')
    parser.add_argument('--output_dir_grasp', default='outputs')
    args = parser.parse_args()
    return args
def load_image(image_path):
    # load image
    image_pil = Image.open(image_path).convert("RGB")  # load image

    transform = T.Compose(
        [
            T.RandomResize([800], max_size=1333),
            T.ToTensor(),
            T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
        ]
    )
    image, _ = transform(image_pil, None)  # 3, h, w
    return image_pil, image


def load_model(model_config_path, model_checkpoint_path, device):
    args = SLConfig.fromfile(model_config_path)
    args.device = device
    model = build_model(args)
    checkpoint = torch.load(model_checkpoint_path, map_location="cpu")
    load_res = model.load_state_dict(clean_state_dict(checkpoint["model"]), strict=False)
    print(load_res)
    _ = model.eval()
    return model


def get_grounding_output(model, image, caption, box_threshold, text_threshold,color_array, with_logits=True, device="cpu"):
    caption = caption.lower()
    caption = caption.strip()
    if not caption.endswith("."):
        caption = caption + "."
    model = model.to(device)
    image = image.to(device)
    with torch.no_grad():
        outputs = model(image[None], captions=[caption])
    logits = outputs["pred_logits"].cpu().sigmoid()[0]  # (nq, 256)
    boxes = outputs["pred_boxes"].cpu()[0]  # (nq, 4)
    logits.shape[0]

    # filter output
    logits_filt = logits.clone()
    boxes_filt = boxes.clone()
    filt_mask = logits_filt.max(dim=1)[0] > box_threshold
    logits_filt = logits_filt[filt_mask]  # num_filt, 256
    boxes_filt = boxes_filt[filt_mask]  # num_filt, 4
    logits_filt.shape[0]

    # get phrase
    tokenlizer = model.tokenizer
    tokenized = tokenlizer(caption)
    # build pred
    pred_phrases = []
    for logit, box in zip(logits_filt, boxes_filt):
        pred_phrase = get_phrases_from_posmap(logit > text_threshold, tokenized, tokenlizer)
        if with_logits:
            pred_phrases.append(pred_phrase + f"({str(logit.max().item())[:4]})")
        else:
            pred_phrases.append(pred_phrase)

    return boxes_filt, pred_phrases

def show_mask(mask, ax, random_color=False):
    if random_color:
        color = np.concatenate([np.random.random(3), np.array([0.6])], axis=0)
    else:
        color = np.array([30/255, 144/255, 255/255, 0.6])
    h, w = mask.shape[-2:]
    mask_image = mask.reshape(h, w, 1) * color.reshape(1, 1, -1)
    ax.imshow(mask_image)


def show_box(box, ax, label):
    x0, y0 = box[0], box[1]
    w, h = box[2] - box[0], box[3] - box[1]
    ax.add_patch(plt.Rectangle((x0, y0), w, h, edgecolor='green', facecolor=(0,0,0,0), lw=2))
    ax.text(x0, y0, label)


def save_mask_data(output_dir, mask_list, box_list, label_list):
    value = 0  # 0 for background

    mask_img = torch.zeros(mask_list.shape[-2:])
    for idx, mask in enumerate(mask_list):
        mask_img[mask.cpu().numpy()[0] == True] = value + idx + 1
    plt.figure(figsize=(10, 10))
    plt.imshow(mask_img.numpy())
    plt.axis('off')
    plt.savefig(os.path.join(output_dir, 'mask.jpg'), bbox_inches="tight", dpi=300, pad_inches=0.0)

    json_data = [{
        'value': value,
        'label': 'background'
    }]
    for label, box in zip(label_list, box_list):
        value += 1
        name, logit = label.split('(')
        logit = logit[:-1] # the last is ')'
        json_data.append({
            'value': value,
            'label': name,
            'logit': float(logit),
            'box': box.numpy().tolist(),
        })
    with open(os.path.join(output_dir, 'mask.json'), 'w') as f:
        json.dump(json_data, f)
    return  mask_img.numpy()

def get_contour(img):
    """Get connected components

    :param img: input image
    :return: largest connected component
    """
    # convert to grayscale, binarize, connected-component analysis
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, img_bin = cv2.threshold(img_gray, 127, 255, cv2.THRESH_BINARY)

    contours, hierarchy = cv2.findContours(img_bin, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    print(f"Number of contour points: {len(contours)}")
    #print(f"Contour range: x_min={contours[:, :, 0].min()}, x_max={contours[:, :, 0].max()}, y_min={contours[:, :, 1].min()}, y_max={contours[:, :, 1].max()}")
    return img_gray, contours[0]
def apply_mask(image, mask, color=(0, 255, 0), alpha=0.5):
    """
    Apply the mask to the image..

    Parameters:
        image (numpy.ndarray): input image.
        mask (numpy.ndarray): binary mask.
        color (tuple): mask color.
        alpha (float): mask alpha.
    Returns:
        numpy.ndarray: image with the mask applied.
    """
    for c in range(3):
        image[:, :, c] = np.where(mask, image[:, :, c] * (1 - alpha) + color[c] * alpha, image[:, :, c])
    return image
def bilinear_interpolate(x, y, points):
    ''' 
    x, y are the fractional parts relative to the top-left corner of the 2x2 grid.
    points is a 2x2 matrix with known values.
    '''
    return (points[0, 0] * (1 - x) * (1 - y) +
            points[1, 0] * x * (1 - y) +
            points[0, 1] * (1 - x) * y +
            points[1, 1] * x * y)

def fill_depth_with_larger_kernel(depth_array, x, y, kernel_size=3):
    if len(depth_array.shape) == 3:
        depth_array = depth_array[:, :, 0]

    rows, cols = depth_array.shape
    half_k = kernel_size // 2

    # Get the neighborhood bounds
    x_min = max(0, x - half_k)
    x_max = min(cols, x + half_k + 1)
    y_min = max(0, y - half_k)
    y_max = min(rows, y + half_k + 1)
    
    # Extract depth values in the neighborhood
    points = depth_array[y_min:y_max, x_min:x_max]
    
    # Filter invalid values
    valid_points = points[points > 0]
    
    if len(valid_points) == 0:
        return 0.0  # No valid values remain in the neighborhood
    else:
        # Return the mean of valid values in the neighborhood
        return np.mean(valid_points)
def inpaint(img, missing_value=0):
    """
    Inpaint missing values in depth image.
    :param missing_value: Value to fill in teh depth image.
    """
    img = cv2.copyMakeBorder(img, 1, 1, 1, 1, cv2.BORDER_DEFAULT)
    mask = (img == missing_value).astype(np.uint8)

    # Scale to keep as float, but has to be in bounds -1:1 to keep opencv happy.
    scale = np.abs(img).max()
    img = img.astype(np.float32) / scale  # Has to be float32, 64 not supported.
    img = cv2.inpaint(img, mask, 1, cv2.INPAINT_NS)

    # Back to original size and value range.
    img = img[1:-1, 1:-1]
    img = img * scale

    return img

def draw_points(color_array, points, color, radius=1):
    """
    Draw points on the image..
    :param color_array: image
    :param points: 2D point list [(x, y), ...]
    :param color: point color (B, G, R)
    :param radius: point radius
    """
    for point in points:
        cv2.circle(color_array, point, radius=radius, color=color, thickness=-1)
    
class ImageProcessor(Node):
    def __init__(self):
        super().__init__('BOX_line')
        self.bridge = CvBridge()
        self.depth_array = None
        self.color_array = None
        self.device = torch.device("cuda:0")
        self.net = None
        self.depth = None
        self.lock = threading.Lock()  # Added mutex lock
        # Create a publisher for the '/grasppose' topic
        self.grasp_pose_pub = self.create_publisher(Float64MultiArray, 'right/grasppose', 10)
        # Subscribe to synchronized depth and color images with message_filters
        depth_sub = Subscriber(self, ROSImage, '/right_camera/depth/image_raw')
        color_sub = Subscriber(self, ROSImage, '/right_camera/color/image_raw')

        self.subscription = self.create_subscription(
            CameraInfo,
            '/right_camera/color/camera_info',
            self.camera_info_callback,
            10
        )
        # Synchronize streams with ApproximateTimeSynchronizer
        self.ats = ApproximateTimeSynchronizer([depth_sub, color_sub], queue_size=10, slop=0.5)
        self.ats.registerCallback(self.process_synced_images)

    def load_network(self, network_path):
        """Load the neural network model"""
        self.net = torch.load(network_path)
    def camera_info_callback(self, msg):
        # Extract camera intrinsics
        self.camera_intrinsics = {
            "fx": msg.k[0],
            "fy": msg.k[4],
            "cx": msg.k[2],
            "cy": msg.k[5]
        }

        # self.get_logger().info(f"Camera intrinsics: {self.camera_intrinsics}")

    def process_synced_images(self, depth_image, color_image):
        """Process synchronized depth and color images"""
        try:
            depth_cv2 = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding='passthrough')
            depth_crop = (depth_cv2).astype(np.float32)
            self.depth= depth_cv2.copy()
            depth_crop = inpaint(depth_crop)
            # depth_crop[np.isnan(depth_crop)] = 0
            # print(depth_crop)
            self.depth_array = depth_crop
            # print("Depth image updated")  # Debug output

            color_cv2 = self.bridge.imgmsg_to_cv2(color_image, desired_encoding="bgr8")
            color_array = np.asanyarray(color_cv2).astype(np.uint8)
            # color_array = cv2.cvtColor(color_array, cv2.COLOR_BGR2RGB)
            self.color_array=color_array
            # Synchronized depth and color images can now be processed safely
            # print("Processing synchronized images")
        except Exception as e:
            print("Error processing synchronized images:", e)

    def get_3d_points_2d(self,points, depth_array,):
        """
        Set the middle 3D point to the average of the first and third 3D points..
        :param points: contains three pixel coordinates [(u1, v1), (u2, v2), (u3, v3)]
        :param depth_array: depth image
        :param fill_depth_with_larger_kernel: function used to fill missing depth values
        :param camera_intrinsics: camera intrinsic dictionary including fx, fy, cx, cy
        :return: updated list of 3D points [(x1, y1, z1), (x_middle, y_middle, z_middle), (x3, y3, z3)]
        """
        assert len(points) == 3, "Three pixel points are required"
        fx, fy = self.camera_intrinsics["fx"], self.camera_intrinsics["fy"]
        cx, cy = self.camera_intrinsics["cx"], self.camera_intrinsics["cy"]

        # Compute the 3D coordinate of each point
        points_3d = []
        for i, (u, v) in enumerate(points):
            z = depth_array[v, u][0]
            if z == 0:
                z = fill_depth_with_larger_kernel(depth_array, u, v)
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
            points_3d.append((x, y, z))

        # Replace the middle point with the average of the first and third points
        points_3d[1] = tuple(
            (p1 + p3) / 2 for p1, p3 in zip(points_3d[0], points_3d[2])
        )

        return np.array(points_3d)
    def start_processing_thread(self, args):
        # Start the image-processing thread
        self.processing_thread = threading.Thread(target=self.run_main, args=(args,))
        self.processing_thread.start()
    def run_main(self,args):
        """Main loop for image processing and grasp publishing"""
        extrinsics_path = [
            {
                "left_cam2tool": "./box_centerline_utils/transform_pix2world/left_camera_pose.txt",
                "left_top2base": "./box_centerline_utils/transform_pix2world/top_leftcamera_pose.txt",
                "right_cam2tool": "./box_centerline_utils/transform_pix2world/right_camera_pose.txt",
                "right_top2base": "./box_centerline_utils/transform_pix2world/top_rightcamera_pose.txt"
            }
        ]
        robot_cfg = [["left_robot", "192.168.2.12"],
                     ["right_robot", "192.168.2.11"]]

        right_robot = enable_robot(robot_cfg[1][0], robot_cfg[1][1])
        robot_node = GetRobotExtrinsic(extrinsics_path, right_robot, "right_cam")# left_cam  top_cam

        # cfg
        config_file = args.config  # change the path of the model config file
        grounded_checkpoint = args.grounded_checkpoint  # change the path of the model
        sam_version = args.sam_version
        sam_checkpoint = args.sam_checkpoint
        sam_hq_checkpoint = args.sam_hq_checkpoint
        use_sam_hq = args.use_sam_hq
        #image_path = args.input_image
        text_prompt = args.text_prompt
        output_dir = args.output_dir
        box_threshold = args.box_threshold
        text_threshold = args.text_threshold
        device = args.device
        os.makedirs(output_dir, exist_ok=True)
         # load model
        model = load_model(config_file, grounded_checkpoint, device=device)
        # pdb.set_trace()
        # Initialize SAM predictor.
        if use_sam_hq:
            predictor = SamPredictor(sam_hq_model_registry[sam_version](checkpoint=sam_hq_checkpoint).to(device))
        else:
            predictor = SamPredictor(sam_model_registry[sam_version](checkpoint=sam_checkpoint).to(device))

        while rclpy.ok():
            with self.lock:
                color_array = self.color_array
                depth_array = self.depth_array
            if self.color_array is not None and self.depth_array is not None:
                depth_array = np.expand_dims(depth_array, axis=2)
                image_path = os.path.join(args.image_path, "0003000.png")
                cv2.imwrite(image_path, color_array)
                
                depth_image_path = os.path.join(args.depth_path, "0003000.png")
                # if depth_array.ndim == 3:
                #     depth_array_1 = depth_array.copy()
                #     depth_array_1 = np.squeeze(depth_array_1)
                #     depth_array_1 = depth_array_1.astype(np.uint16)
                #     cv2.imwrite("outputs/debug/depth_4.png", depth_array_1)
                # cv2.imwrite(depth_image_path, depth_array)

                # load image
                image_pil, color_image = load_image(image_path)
                # visualize raw image
                #image_pil.save(os.path.join(output_dir, "raw_image.jpg"))

                # run grounding dino model
                boxes_filt, pred_phrases = get_grounding_output(
                    model, color_image, text_prompt, box_threshold,text_threshold,color_array,device=device
                )
                if boxes_filt.size(0)==0:
                    print("no boxes")
                    continue
                
                # yr
                color_image = cv2.imread(image_path)
                color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
                predictor.set_image(color_image)
                size = image_pil.size
                H, W = size[1], size[0]
                for i in range(boxes_filt.size(0)):
                    boxes_filt[i] = boxes_filt[i] * torch.Tensor([W, H, W, H])
                    boxes_filt[i][:2] -= boxes_filt[i][2:] / 2
                    boxes_filt[i][2:] += boxes_filt[i][:2]
                    print("----------yr------------: ", boxes_filt[i])

                boxes_filt = boxes_filt.cpu()
                transformed_boxes = predictor.transform.apply_boxes_torch(boxes_filt, color_image.shape[:2]).to(device)

                masks, _, _ = predictor.predict_torch(
                    point_coords=None,
                    point_labels=None,
                    boxes=transformed_boxes.to(device),
                    multimask_output=False,  # FalseGenerate only one mask
                )

                if len(masks) == 0:
                    print("No masks detected, skipping display update.")
                    continue  # Skip display update

                points_infos, top_points, center_points, down_points, rect_points = box_centerLine_and_9_points(masks, color_array, depth_array, robot_node, self.camera_intrinsics)

                if not top_points  or (not down_points)  or (not center_points) or (not rect_points):
                    print("generate 9 points from mask is failed")
                    continue
                
                top_points_3d = np.array([]) 
                center_points_3d = np.array([]) 
                down_points_3d = np.array([]) 

                for expanded_points, label, top_points, down_points, center_points in zip(rect_points, pred_phrases, top_points, down_points, center_points):
                    print(f"label: {label}, type: {type(label)}")

                             
                    # Draw points and bounding boxes
                    draw_points(color_array, center_points, color=(255, 0, 0))  # Center points are red
                    draw_points(color_array, top_points, color=(0, 0, 255))    # Top points are blue
                    draw_points(color_array, down_points, color=(0, 255, 0))   # Bottom points are green

                    if len(expanded_points) == 4:
                        # Draw the quadrilateral with red lines (0, 0, 255)
                        for i in range(4):
                            start_point = tuple(map(int, expanded_points[i]))
                            end_point = tuple(map(int, expanded_points[(i + 1) % 4]))  # Connect the last point to the first point
                            cv2.line(color_array, start_point, end_point, (0, 255, 0), 2)
                        # Draw label
                        cv2.putText(color_array, label, (start_point[0], start_point[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

                    match = re.search(r"\(([-+]?\d*\.?\d+)\)", label)
                    if match:
                        # Extract and convert to float
                        number = float(match.group(1))
                        # Check the value
                        if number > 0.5:
                            # Compute 3D points
                            top_points_3d = self.get_3d_points_2d(top_points, depth_array)
                            center_points_3d = self.get_3d_points_2d(center_points, depth_array)
                            down_points_3d = self.get_3d_points_2d(down_points, depth_array)
                            
                            # Print debug information
                            print(f"Top points 3D: {top_points_3d}")
                            print(f"Center points 3D: {center_points_3d}")
                            print(f"Down points 3D: {down_points_3d}")
                        else:  
                            print("Probability is less than or equal to0.5")
                    else:
                        print("Probability value not found")
            
                # Show image
                cv2.imshow('Image with Center Points and Corners', color_array)
                # Press key to exit
                key = cv2.waitKey(1) & 0xFF
                if key == 27:
                    cv2.destroyAllWindows()
                    break
                #cv2.destroyAllWindows()  # Ensure all OpenCV windows are closed
                grasp_data = Float64MultiArray()
                # Pack data into a list
                combined_data = []
                combined_data.extend(top_points_3d.flatten())
                  
                combined_data.extend(center_points_3d.flatten())
                combined_data.extend(down_points_3d.flatten())
                # Set the data field
                grasp_data.data = combined_data
                print(grasp_data.data)

                # Main loop
                # rate = 10  # publish rate:10Hz
                # num_iterations = 50
                # period = 1.0 / rate  # Compute period
                # for _ in range(num_iterations):
                    # Publish message
                self.grasp_pose_pub.publish(grasp_data)
                #rclpy.spin_once(node)
                    # time.sleep(period)
                # Stop node
                #rclpy.shutdown()

        

if __name__ == "__main__":
    rclpy.init(args=None)
    image_processor = ImageProcessor()
    args = parse_args()
       # Create a multithreaded executor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(image_processor)

    # Start the image-processing thread
    image_processor.start_processing_thread(args)

    try:
        # Start the executor to process ROS 2 callbacks
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        rclpy.shutdown()
            
