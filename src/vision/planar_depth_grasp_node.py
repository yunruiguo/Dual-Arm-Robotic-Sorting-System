'''
FilePath: src/vision/planar_depth_grasp_node.py
Author: Yunrui Guo
Date: 2024-11-16 19:45:07
LastEditors: Please set LastEditors
LastEditTime: 2025-03-20 00:45:11

Descripttion: 
'''
# -*- coding: utf-8 -*-
import argparse#for command-line parsing
import logging
import random# for logging
import torch.utils.data# for data processing
import warnings #suppress warnings
from torch.serialization import SourceChangeWarning#suppress warnings
warnings.filterwarnings("ignore", category=SourceChangeWarning)#suppress warnings
from models.common import post_process_output# for model inference post-processing
from utils.dataset_processing import evaluation, grasp, image# for dataset processing and grasp-pose computation
from utils.data import get_dataset# get dataset
from matplotlib import pyplot as plt# for plotting
import numpy as np # for data processing
import os # for path handling
import json# for reading and writing JSON files
import pdb# Python debugging tool
import cv2 # OpenCV tool
import math# for mathematical operations
import tifffile# for reading and writing TIFF files
import time# for timing
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
############modified
import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray# standard ROS message types
logging.basicConfig(level=logging.INFO)# logging level
torch.nn.Module.dump_patches = True# record PyTorch model patch information

DEFAULT_NETWORK_PATH = os.environ.get(
    "PLANAR_GRASP_NETWORK",
    "models/planar_grasp/c_d_epoch_36_iou_0.9943"
)
DEFAULT_OUTPUT_DIR = os.environ.get(
    "PLANAR_GRASP_OUTPUT_DIR",
    "outputs/planar_grasp"
)


def parse_args():# Define function for parsing command-line arguments
    parser = argparse.ArgumentParser(description='Evaluate GG-CNN')# Create parser
    # Network
  #Original code  parser.add_argument('--network', type=str, default='ggcnn_weights_cornell/ggcnn_epoch_23_cornell', help='Path to saved network to evaluate')
    parser.add_argument('--network', type=str, default=DEFAULT_NETWORK_PATH,
     help='Path to saved network to evaluate')# Number of grasps to consider
    parser.add_argument('--use-depth', type=int, default=1, help='Use Depth image for training (1/0)')
    parser.add_argument('--use-rgb', type=int, default=0, help='Use RGB image for training (0/1)')
    parser.add_argument('--TOG_FLAG', type=int, default=0, help='UseTOG_FLAG for image or video stream(0/1)')
    parser.add_argument('--num-workers', type=int, default=8, help='Dataset workers')
    parser.add_argument('--n-grasps', type=int, default=1, help='Number of grasps to consider per image')
    args = parser.parse_args()
    return args
def get_rgb_from_array(rgb_array, rot=0, zoom=1.0, output_size=224, normalise=True):
    rgb_img = image.Image.from_array(rgb_array)
    rgb_img.rotate(rot)
    rgb_img.zoom(zoom)
    rgb_img.resize((output_size, output_size))
    if normalise:
        rgb_img.normalise()
        rgb_img.img = rgb_img.img.transpose((2, 0, 1))
    return rgb_img.img

def get_depth_from_array(depth_array, rot=0, zoom=1.0, output_size=224):
    depth_img = image.DepthImage.from_array(depth_array)
    depth_img.inpaint()
    depth_img.rotate(rot)
    depth_img.normalise()
    depth_img.zoom(zoom)
    depth_img.resize((output_size, output_size))
    return depth_img.img
def numpy_to_torch(s):
    if len(s.shape) == 2:
        return torch.from_numpy(np.expand_dims(s, 0).astype(np.float32))
    else:
        return torch.from_numpy(s.astype(np.float32))
def rotate_points(pt, center, th):
    """
    th -> -th
    """
    x, y = pt
    cx, cy = center

    x -= cx
    y -= cy

    xx = x
    x = x*math.cos(-th) - y*math.sin(-th)
    y = xx * math.sin(-th) + y*math.cos(-th)

    x += cx
    y += cy

    return [x,y]


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

def get_rotated_points(center, length, width, angle):
    """ Compute rotated rectangle vertices for the grasp region """
    cx, cy = center
    half_l, half_w = length // 2, width // 2
    return np.array([
        rotate_points((cy - half_l, cx - half_w), (cy, cx), angle),
        rotate_points((cy + half_l, cx - half_w), (cy, cx), angle),
        rotate_points((cy + half_l, cx + half_w), (cy, cx), angle),
        rotate_points((cy - half_l, cx + half_w), (cy, cx), angle)
    ], np.int32).reshape((-1, 1, 2))

def get_gripper_region(center, length, width, angle, is_upper):
    """ Compute the descent region for one side of the gripper """
    cx, cy = center
    offset = length * 5 // 8 if is_upper else -length * 5 // 8
    half_w = width // 4
    return np.array([
        rotate_points((cy + offset, cx - half_w), (cy, cx), angle),
        rotate_points((cy + length // 2 if is_upper else cy - length // 2, cx - half_w), (cy, cx), angle),
        rotate_points((cy + length // 2 if is_upper else cy - length // 2, cx + half_w), (cy, cx), angle),
        rotate_points((cy + offset, cx + half_w), (cy, cx), angle)
    ], np.int32).reshape((-1, 1, 2))

def generate_grasp_pose(u, v, z, cx, cy, fx, fy):
    """ Convert pixel coordinates (u, v) to the camera frame """
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return [x, y, z]

def get_width(u, v, z, width, cx, cy, fx, fy):
    """ Compute the real gripper width in the camera frame """
    x1, y1 = (u - cx) * z / fx, (v - cy) * z / fy
    x2, y2 = (u + width - cx) * z / fx, (v - cy) * z / fy
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def get_real_depth(depth_array, u, v, cx, cy, fx, fy):
    """Compute vertical height from the point cloud to the camera plane."""
    D = depth_array[v, u]  # Depth in the raw depth image as Euclidean distance.
    scale = np.sqrt(1 + ((u - cx) / fx) ** 2 + ((v - cy) / fy) ** 2)  # Compute scale factor
    Z = D / scale  # Compute projected height along the Z axis
    return Z

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('plen_grasp')
        self.bridge = CvBridge()
        self.depth_array = None
        self.color_array = None
        self.device = torch.device("cuda:0")
        self.net = None
        self.depth = None
        self.camera_intrinsics = None

        # Subscribe to topics
        self.subscription = self.create_subscription(
            CameraInfo,
            '/left_camera/color/camera_info',
            self.camera_info_callback,
            10
        )
        self.lock = threading.Lock()  # Added mutex lock
        self.declare_parameter('network_model', DEFAULT_NETWORK_PATH)
        self.network = self.get_parameter('network_model').get_parameter_value().string_value
        self.get_logger().info(f'Parameter value: {self.network}')
        self.load_network(self.network)
        # Create a publisher for the '/grasppose' topic
        self.grasp_pose_pub = self.create_publisher(Float64MultiArray, 'left/grasppose', 10)
        # Subscribe to synchronized depth and color images with message_filters
        depth_sub = Subscriber(self, Image, '/left_camera/depth/image_raw')
        color_sub = Subscriber(self, Image, '/left_camera/color/image_raw')
  

        # Synchronize streams with ApproximateTimeSynchronizer
        self.ats = ApproximateTimeSynchronizer([depth_sub, color_sub], queue_size=10, slop=0.5)
        self.ats.registerCallback(self.process_synced_images)

    def camera_info_callback(self, msg):
        # Extract camera intrinsics
        self.camera_intrinsics = {
            "fx": msg.k[0],
            "fy": msg.k[4],
            "cx": msg.k[2],
            "cy": msg.k[5]
        }

        self.get_logger().info(f"Camera intrinsics: {self.camera_intrinsics}")
        
            # Unsubscribe to ensure this runs only once
        self.destroy_subscription(self.subscription)
        self.get_logger().info("Camera info subscription destroyed.")
    def load_network(self, network_path):
        """Load the neural network model"""
        self.net = torch.load(network_path)
        
    def process_synced_images(self, depth_image, color_image):
        """Process synchronized depth and color images"""
        try:
            depth_cv2 = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding='passthrough')
            depth_crop = (depth_cv2 / 1000.0).astype(np.float32)
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
    def start_processing_thread(self, args):
        # Start the image-processing thread
        self.processing_thread = threading.Thread(target=self.run_main, args=(args,))
        self.processing_thread.start()

    def calculate_grasp_depth(self,depth_array, u_center,v_center,pts3, pts4):
        def get_min_depth_in_rotated_rect(depth_array, rotated_rect):
            # Get the minimum-area enclosing rectangle
            rect = cv2.minAreaRect(rotated_rect)
            # Get the four rectangle corners
            box = cv2.boxPoints(rect)
            box = np.intp(box)
            # Build mask
            mask = np.zeros(depth_array.shape, dtype=np.uint8)
            cv2.drawContours(mask, [box], 0, (255), -1)
            # Find pixels and depth values inside the mask
            indices = np.argwhere(mask > 0)
            
            if indices.size > 0:
                u, v = indices[:, 1], indices[:, 0]  # pixel coordinates
                vertical_heights =depth_array[v,u]
                # Filter depth values by percentile
                lower_value = np.percentile(vertical_heights, 2)
                upper_value = np.percentile(vertical_heights, 92)
                valid_range = vertical_heights[(vertical_heights >= lower_value) & (vertical_heights <= upper_value)]
                sorted_indices = np.sort(valid_range)
                partitioned_indices = np.array_split(sorted_indices, 9)
                # Compute the mean vertical height of each partition
                partition_means = [np.mean(partition) for partition in partitioned_indices]
          
                # Return the minimum mean across all partitions
                return min(partition_means)
            else:
                # Return a default value if the mask contains no points
                return 0.001
        #Depth values represent distance in the camera frame and can be treated as vertical distance here
        # real_depth = get_real_depth(depth_array, u_center, v_center,self.camera_intrinsics["cx"], self.camera_intrinsics["cy"], self.camera_intrinsics["fx"], self.camera_intrinsics["fy"])
        # print('real_depth:',real_depth)
        min_depth_pts3 = get_min_depth_in_rotated_rect(depth_array, pts3)
        # print('min_depth_pts3:',min_depth_pts3)

        min_depth_pts4 = get_min_depth_in_rotated_rect(depth_array, pts4)
        print('min_depth_pts4:',min_depth_pts4)
        # Compare the two gripper-side depth estimates.
        min_depth = min(min_depth_pts3 - depth_array[v_center, u_center], min_depth_pts4 - depth_array[v_center, u_center])
        
        # Return the value directly if it is negative;otherwise,if greater than 0.35,return 0.35;otherwise return the minimum value
        return min(min_depth, 0.051) if min_depth >= 0 else min_depth
    def run_main(self,args):
        """Main loop for image processing and grasp publishing"""
       
        self.load_network(args.network)
        BasePath = DEFAULT_OUTPUT_DIR
        os.makedirs(BasePath, exist_ok=True)
        img_index = len(os.listdir(BasePath)) // 2
        include_depth = args.use_depth
        include_rgb = args.use_rgb
        cv2.namedWindow('detected grasps')
        frame_count = 0
        start_time = time.time()
        # pdb.set_trace()
        while rclpy.ok():
            with self.lock:
                color_array = self.color_array
                depth_array = self.depth_array
            if self.color_array is not None and self.depth_array is not None:
                print(color_array.shape)
                print(depth_array.shape)
                depth_array1 = np.expand_dims(depth_array, axis=2)
                rgb_im = get_rgb_from_array(color_array, normalise=False)
                depth_im = get_depth_from_array(depth_array1)
                print(depth_im.shape)
                if rgb_im is None or depth_im is None:
                    print("rgb_im or depth_im is empty! Warning: Invalid RGB or depth data.")
                    continue

                if include_depth:
                    depth_img = get_depth_from_array(depth_array1)
                    depth_img = np.expand_dims(depth_img, axis=0)
                if include_rgb:
                    rgb_img = get_rgb_from_array(color_array)
                if include_depth and include_rgb:
                    print(np.expand_dims(depth_img, 0).shape)
                    print(np.expand_dims(rgb_img, 0).shape)
                    x = numpy_to_torch(
                        np.concatenate(
                            (np.expand_dims(depth_img, 0), np.expand_dims(rgb_img, 0)),
                            axis=1
                        )
                    )
                elif include_depth:
                    x = numpy_to_torch(np.expand_dims(depth_img, 0))
                elif include_rgb:
                    x = numpy_to_torch(np.expand_dims(rgb_img, 0))

                TOG_FLAG = args.TOG_FLAG
                with torch.no_grad():
                    preds = self.net.predict(x.to(self.device))
                    q_img, ang_img, width_img = post_process_output(preds['pos'], preds['cos'], preds['sin'], preds['width'])
                    if TOG_FLAG:
                        out_grasps = evaluation.plot_output2(rgb_im, depth_im, q_img, ang_img, no_grasps=args.n_grasps, grasp_width_img=width_img)
                    else:
                        out_grasps = evaluation.compute_grasps2(rgb_im, depth_im, q_img, ang_img, no_grasps=args.n_grasps, grasp_width_img=width_img)

                graspt = []
                for grasp in out_grasps:
                    grasp_x, grasp_y = grasp.center
                    quality, width, length, angle = grasp.quality, grasp.width, grasp.length, grasp.angle
                    depth = depth_array[grasp_x, grasp_y]*1000

                    print(f"center: {grasp.center}, quality: {quality:.6f}, depth: {depth:.6f}, width: {width:.6f}, length: {length:.6f}, angle: {angle:.6f}")

                    # Compute grasp region
                    pts = get_rotated_points(grasp.center, length, width, angle)
                    
                    # Compute the descent region for one side of the gripper
                    pts3 = get_gripper_region(grasp.center, length, width, angle, is_upper=False)
                    pts4 = get_gripper_region(grasp.center, length, width, angle, is_upper=True)

                    # Compute grasp depth
                    graspdepth = self.calculate_grasp_depth(depth_array, grasp_y, grasp_x, pts3, pts4)*1000
                    # **Compute the grasp point in the camera frame**
                    grasp_cam_coord = generate_grasp_pose(grasp_y, grasp_x, depth, self.camera_intrinsics["cx"], self.camera_intrinsics["cy"], self.camera_intrinsics["fx"], self.camera_intrinsics["fy"])

                    # **Compute gripper width in the camera frame**
                    grasp_width_real = get_width(grasp_y, grasp_x, depth, length, self.camera_intrinsics["cx"], self.camera_intrinsics["cy"], self.camera_intrinsics["fx"], self.camera_intrinsics["fy"])

                    # Record grasp information(including camera-frame x, y, z and real width)
                    graspt.append([grasp_cam_coord[0], grasp_cam_coord[1], grasp_cam_coord[2], angle, grasp_width_real, quality,graspdepth])

                    # Record grasp information
                    
                    # graspt.append([grasp_x, grasp_y, depth, angle, length, quality])

                    rect_color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
                    cv2.polylines(color_array, [pts], True, rect_color, 2)
                    cv2.polylines(color_array, [pts4], True, (255, 255, 255), 2)
                    cv2.polylines(color_array, [pts3], True, (255, 255, 255), 2)
                    cv2.putText(color_array, str(graspdepth), (grasp.center[1] + 1, grasp.center[0] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                    cv2.circle(color_array, (grasp.center[1], grasp.center[0]), 1, (255, 255, 255), 2)

                print(graspt)
                self.grasp_pose_pub.publish(Float64MultiArray(data=list(np.array(graspt).flatten())))

                frame_count += 1
                if frame_count % 10 == 0:
                    end_time = time.time()
                    elapsed_time = end_time - start_time
                    fps = frame_count / elapsed_time
                    print("FPS:", fps)
                    frame_count = 0
                    start_time = time.time()
                    # Draw FPS on the image
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    org = (10, 30)
                    font_scale = 0.8
                    color = (255, 255, 255)
                    thickness = 2
                    text = "FPS: {:.2f}".format(fps)
                    cv2.putText(color_array, text, org, font, font_scale, color, thickness, cv2.LINE_AA)

                cv2.imshow('detected grasps', color_array)
                key = cv2.waitKey(1) & 0xFF
                if key == 27:
                    break
                elif key == ord('s'):
                    depth_name = str(img_index) + '.tiff'
                    color_name = str(img_index) + '.png'
                    cv2.imwrite(os.path.join(BasePath, color_name), color_array)
                    tifffile.imsave(os.path.join(BasePath, depth_name), depth_array)
                    print("RGB and depth images %d saved" % img_index)
                    img_index += 1

        cv2.destroyAllWindows()

if __name__ == '__main__':
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
