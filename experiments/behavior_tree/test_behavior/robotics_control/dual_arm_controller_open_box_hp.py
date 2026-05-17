'''
FilePath: dual_arm_controller_open_box_hp.py
Author: Yunrui Guo
Date: 2024-12-26 11:59:14
LastEditors: Please set LastEditors
LastEditTime: 2025-04-03 11:50:09

Descripttion: 
'''
import sys
import os
from pathlib import Path
REAL_CONFIG_DIR = Path(__file__).resolve().parents[1] / "real"
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from typing import Callable
import numpy as np    
import time                        # library
import threading
from collections import deque
import tf_transformations                     # library
from scipy.spatial.transform import Rotation as R  # 
from rclpy.node import Node                   # ROS2 
from std_msgs.msg import Float64MultiArray    # ROS,
from geometry_msgs.msg import PoseArray       # ROS,
from grasp_msgs.msg import ArmCommand, GripperCommand  # ,
from grasp_msgs.action import ArmControl, GripperControl   # ,
from action_msgs.msg import GoalStatus        #  Action 
from rclpy.subscription import Subscription     # ROS2 ()
from grasp_msgs.msg import FeedBackMsg          # 
from rclpy.action import ActionClient         # ROS2 Action 
from rclpy.exceptions import ROSInterruptException  # ROSexception
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup  # ,
import logging
from transforms import pose_to_matrix, transform_pose, invert_transform,create_transformation,generate_grasp_pose,get_width
from action_queue import ActionQueue
from message import Message
from sensor_msgs.msg import CameraInfo
from action import Action
from feedback_msgs import ArmFeedbackMsg, GripperFeedbackMsg
from arm_controller import ArmController


# -----------------------------------------------------------------------------
# GraspTaskManager : ROS2 
# -----------------------------------------------------------------------------
class GraspTaskManager(Node):
    def __init__(self):
        super().__init__('grasp_task_manager_node')
        # 
        self.load_camera_poses()
        self.action_history = []  # ,
        self.pending_decision_logic = None  # 
        self.is_paused = False  # 
        self.pose_update_needed = False  # box
        self.grasp_pose_update_needed = True # 
        self.aruco_grasp_position =[]
        self.aruco_grasp_euler =[]
        self.pose_buffer = deque(maxlen=8)  #  5 
        self.std_threshold = 2  #  (: mm),
        self.min_stable_frames = 2  # 
        self.stable_frame_count = 0  # 
        self.pose_received_event = threading.Event()  # 
        self.task_queue = []  # 
        self.grasp_points = {}
        self.grasp_info = {}
        # 
        self.action_info =[]          # 
        self.is_action_in_progress = False  # 
        self.waiting_for_condition = False  # 
        self.current_action_index = 0
        self.logger = logging.getLogger('GraspTaskManager')
        self.logger.setLevel(logging.INFO)
        handler = logging.StreamHandler()
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
        self.logger.addHandler(handler)
        self.is_executing = False  # 
        self.initialized = False  # 
        self.camera_intrinsics = None
        self.action_queue = ActionQueue()  # 

        # ()
        self.arm_state = {'left': None, 'right': None}
         # 
        self.waiting_arms = {
            0: {"waiting": False, "start_time": None, "duration": 0.0, "decision_logic": None},
            1: {"waiting": False, "start_time": None, "duration": 0.0, "decision_logic": None},
        }

        # 
        self.arm_controllers = [
            ArmController(self, "left"),
            ArmController(self, "right")
        ]
        self.subscription = self.create_subscription(
            CameraInfo,
            '/left_camera/color/camera_info',
            self.camera_info_callback,
            10
        )
        # 1️⃣ 
        self.action_map = {
            "jointTo": self.jointTo,
            "moveTo": self.moveTo,
            "ArcT": self.ArcTo,
            "closeGripper": self.closeGripper,
            "openGripper": self.openGripper,
            "wait": self.wait_time,
        }

        #  Action 
        self.wait_for_arm_controllers()

        # Set
        # self.setup_initial_positions()

        #  ROS2 ()
        self.subscribe_to_topics()
        self.timer = self.create_timer(0.005, self.manage_task_execution)  # ****
         #  ROS 2 , 0.1s 
        self.wait_timer = self.create_timer(0.1, self.wait_callback)

        # ()
        # self.start_grasping_sequence()
        # 
        # self.process_grasp_task()

    def load_camera_poses(self):
        """"""
        try:
            self.right_camera_pose = np.genfromtxt(str(REAL_CONFIG_DIR / 'right_camera_pose.txt'), delimiter=' ')
            self.left_camera_pose = np.genfromtxt(str(REAL_CONFIG_DIR / 'left_camera_pose.txt'), delimiter=' ')
            self.top_rightcamera_pose = np.genfromtxt(str(REAL_CONFIG_DIR / 'top_rightcamera_pose.txt'), delimiter=' ')
            self.top_leftcamera_pose = np.genfromtxt(str(REAL_CONFIG_DIR / 'top_leftcamera_pose.txt'), delimiter=' ')
        except Exception as e:
            self.get_logger().error(f"Error reading file: {e}")

    def wait_for_arm_controllers(self):
        """"""
        for controller in self.arm_controllers:
            controller.wait_for_action_servers()

    def setup_initial_positions(self):
        """Set"""
        """"""
        self.moveTo(1, [280.516,26.289,41.087,3.1416,0.0,1.5808])
        # Set(:)
        # self.moveTo(1, [517.683, -213.007, 60.0, 3.1416, 0.0, 1.5808])
        # self.moveTo(0, [-479.762,-85.60666223,18.0,3.1416,0.0,0.0])
        # self.jointTo(1, [230.416*np.pi/180, 105.559*np.pi/180, 38.741*np.pi/180, 
                        #  93.193*np.pi/180, -123.021*np.pi/180, -229.462*np.pi/180])
        
    def subscribe_to_topics(self):
        """"""
        self.create_subscription(Float64MultiArray, 'right/grasppose', self.subscription_box_callback, 10)
        self.create_subscription(PoseArray, '/right/aruco_poses', self.pose_callback, 10)
        self.create_subscription(Float64MultiArray,'left/grasppose', self.subscription_grasp_callback,10)
    def camera_info_callback(self, msg):
        # Extract camera intrinsics
        self.camera_intrinsics = {
            "fx": msg.k[0],  # K11
            "fy": msg.k[4],  # K22
            "cx": msg.k[2],  # K13
            "cy": msg.k[5]   # K23
        }

        self.get_logger().info(f"Camera intrinsics: {self.camera_intrinsics}")
        
            # Unsubscribe to ensure this runs only once
        self.destroy_subscription(self.subscription)
        self.get_logger().info("Camera info subscription destroyed.")
    def jointTo(self,index, joint,decision_logic=None):
         # Example commands
        arm_command = ArmCommand()
        #
        arm_command.type = arm_command.JOINT_TYPE
        arm_command.joint.joint = joint
        self.pending_decision_logic = (decision_logic, "jointTo")  # 
        self.execute_arm_control(index, arm_command)
        self.get_logger().info(f"{self.arm_controllers[index].name_prefix} : {joint}")
        return True

    def moveTo(self,index, tcp,decision_logic=None):
         # Example commands    
        arm_command = ArmCommand()
        #
        arm_command.type = arm_command.TCP_TYPE
        arm_command.tcp.pos.x = tcp[0]
        arm_command.tcp.pos.y = tcp[1]
        arm_command.tcp.pos.z = tcp[2]

        arm_command.tcp.euler.x = tcp[3]
        arm_command.tcp.euler.y = tcp[4]
        arm_command.tcp.euler.z = tcp[5]
        # 
        self.pending_decision_logic = (decision_logic, "moveTo") 
        self.execute_arm_control(index, arm_command)
        self.get_logger().info(f"{self.arm_controllers[index].name_prefix} tcp: {tcp}")
        return True 

    def ArcTo(self,index,mid_tcp, end_tcp,decision_logic=None):
         # Example commands
        arm_command = ArmCommand()
        #
        arm_command.type = arm_command.TCP_CIRCLE_TYPE
        arm_command.mid_tcp.pos.x = mid_tcp[0]
        arm_command.mid_tcp.pos.y = mid_tcp[1]
        arm_command.mid_tcp.pos.z = mid_tcp[2]
        arm_command.mid_tcp.euler.x = mid_tcp[3]
        arm_command.mid_tcp.euler.y = mid_tcp[4]
        arm_command.mid_tcp.euler.z = mid_tcp[5]
        #
        arm_command.tcp.pos.x = end_tcp[0]
        arm_command.tcp.pos.y = end_tcp[1]
        arm_command.tcp.pos.z = end_tcp[2]
        arm_command.tcp.euler.x = end_tcp[3]
        arm_command.tcp.euler.y = end_tcp[4]
        arm_command.tcp.euler.z = end_tcp[5]
        # 
        self.pending_decision_logic = (decision_logic, "ArcTo")
        self.execute_arm_control(index, arm_command)
        self.get_logger().info(f"{self.arm_controllers[index].name_prefix} : {end_tcp}")
        return True 

    def closeGripper(self, index, force, target_pos=None,decision_logic=None):
        """
        , `force`,
        :param index: 
        :param force: 
        :param target_pos: , ()
        """
        gripper_command = GripperCommand()

        # ,Set
        if target_pos:
            # ,,
            gripper_command.pos = max(0, min(1000, target_pos))  # 
        else:
            gripper_command.pos = 0  # 

        # Set
        gripper_command.force = force
        self.pending_decision_logic = (decision_logic, "closeGripper")
        self.execute_gripper_control(index, gripper_command)
        self.get_logger().info(f"{self.arm_controllers[index].name_prefix} ,: {force}, : {target_pos}")
        return True


    def openGripper(self, index, force,target_pos=None,decision_logic=None):
        """
        ,Set `force`
        :param index: 
        :param force: 
        """
        gripper_command = GripperCommand()
        # ,Set
        if target_pos:
            # ,,
            gripper_command.pos = max(0, max(0, target_pos))  # 
        else:
            gripper_command.pos = 1000  # 
        gripper_command.force = force
        self.pending_decision_logic = (decision_logic, "openGripper")
        self.execute_gripper_control(index, gripper_command)
        self.get_logger().info(f"{self.arm_controllers[index].name_prefix} ,: {force}, : {target_pos}")
        return True
    
    def close_gripper_with_armto(self,index, force=50 ,target_pos=None,decision_logic=None):
        ""","""
        # ,
        controller = self.arm_controllers[index]
        if  controller.arm_feedback.distance_remaining< 0.05:
            self.get_logger().info(f"🦾 ")

            #  target_pos,
            if target_pos is not None:
                # Set(pos),
                gripper_command = GripperCommand(pos=max(0, min(1000, target_pos * 1000)), force=force)
            else:
                # ,
                gripper_command = GripperCommand(pos=0, force=force)

            # 
            self.execute_gripper_control(index, gripper_command)
    def wait_time(self,index=None,params=None,decision_logic=None):
        """  ROS 2  """
        if index not in self.waiting_arms:
            self.get_logger().error(f": {index}")
            return

        controller = self.arm_controllers[index]  # 
        arm = self.waiting_arms[index]
        if arm["waiting"]:
            self.get_logger().warn(f" {controller.name_prefix} ,")
            return
        
        arm["waiting"] = True
        arm["start_time"] = self.get_clock().now().seconds_nanoseconds()[0]
        arm["duration"] = float(params)  # 
        arm["decision_logic"] = decision_logic
        self.get_logger().info(f" {controller.name_prefix}  {params} ...")
        
        return True
    
    def wait_callback(self):
        """  """
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        for index, arm in self.waiting_arms.items():
            if not arm["waiting"]:
                continue

            elapsed_time = current_time - arm["start_time"]
            if elapsed_time >= arm["duration"]:
                controller = self.arm_controllers[index]  # 
                self.get_logger().info(f" {controller.name_prefix}  {arm['duration']} ")
                arm["waiting"] = False  # 

                # 
                # if arm["decision_logic"]:
                #     self.get_logger().info(f"🧠  {index} : {arm['decision_logic'].__name__}")
                #     arm["decision_logic"]()
                #     arm["decision_logic"] = None  # ,

    def execute_arm_control(self, index, msg):
        """
        Execute arm control for the specified controller index.

        Args:  grasp_task_manager.start_grasping_sequence()
            index (int): The index of the arm controller.
            msg (grasp_msgs.msg.ArmCommand): The arm command to execute.

        Returns:
            bool: True if the command was successfully executed, False otherwise.
        """
        # Ensure the index is valid
        if index < 0 or index >= len(self.arm_controllers):
            raise IndexError(f"Invalid arm controller index: {index}")

        poses_vector = [msg]
        return self.arm_controllers[index].execute_arm(poses_vector)

    def execute_gripper_control(self, index, msg):
        """
        Execute gripper control for the specified controller index.

        Args:
            index (int): The index of the arm controller.
            msg (grasp_msgs.msg.GripperCommand): The gripper command to execute.

        Returns:
            bool: True if the command was successfully executed, False otherwise.
        """
        # Ensure the index is valid
        if index < 0 or index >= len(self.arm_controllers):
            raise IndexError(f"Invalid arm controller index: {index}")

        return self.arm_controllers[index].execute_gripper(msg)
    

    def subscription_box_callback(self,grasppose: Float64MultiArray):
        """ : """
        if len(grasppose.data) < 27:
            self.get_logger().warn(",")
            return
        
        # ,
        self.grasp_points = {
            "topleft": list(grasppose.data[0:3]),
            "topcenter": list(grasppose.data[3:6]),
            "topright": list(grasppose.data[6:9]),
            "centerleft": list(grasppose.data[9:12]),
            "center": list(grasppose.data[12:15]),
            "centerright": list(grasppose.data[15:18]),
            "downleft": list(grasppose.data[18:21]),
            "downcenter": list(grasppose.data[21:24]),
            "downright": list(grasppose.data[24:27])
        }

        # 
        self.pose_update_needed = True
        # self.get_logger().info("")

    def subscription_grasp_callback(self,grasp_pose):
        """:"""
        
        # 
        if len(grasp_pose.data) < 5:
            self.get_logger().warn(",")
            return
        
        # ,
        self.grasp_info = {
            "x": grasp_pose.data[0],
            "y": grasp_pose.data[1],
            "z": grasp_pose.data[2],
            "angle": grasp_pose.data[3],
            "width": grasp_pose.data[4],
            "qaulity": grasp_pose.data[5],
            "grasp_depth": grasp_pose.data[6],
        }
        
        # 
        self.grasp_pose_update_needed = True
    def pose_callback(self, pose_array: PoseArray):
        if not pose_array.poses:
            self.get_logger().warn(" PoseArray!")
            return

        for pose in pose_array.poses:
            position = [pose.position.x * 1000, pose.position.y * 1000, pose.position.z * 1000]
            orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

            # 
            rotation_matrix = tf_transformations.quaternion_matrix(orientation)[0:3, 0:3]
            euler_angles = R.from_matrix(rotation_matrix).as_euler('xyz', degrees=True)

            # 
            self.pose_buffer.append(position)

            # 
            if len(self.pose_buffer) == self.pose_buffer.maxlen:
                std_dev = np.std(self.pose_buffer, axis=0)
                max_std = np.max(std_dev)  # 
                # self.get_logger().info(f"Aruco : {std_dev}, : {max_std}")

               # 
                if max_std < self.std_threshold:
                    self.stable_frame_count += 1  # 
                    if self.stable_frame_count >= self.min_stable_frames:  
                        self.aruco_grasp_position = np.mean(self.pose_buffer, axis=0)  # 
                        # self.get_logger().info(f" Aruco : {self.aruco_grasp_position}")
                        self.pose_received_event.set()  # 
                else:
                    self.stable_frame_count = 0  # ,

    def calculate_rotated_point_and_orientation_euler(self,start_pose, center, rotation_axis, theta):
        """
         (x, y, z, r_x, r_y, r_z) Parameters,.

        :param start_pose:  (x, y, z, r_x, r_y, r_z) ( + ,xyz)
        :param center:  (x, y, z)
        :param rotation_axis:  (x, y, z)
        :param theta:  ()
        :return:  (x', y', z', r_x', r_y', r_z')
        """
        # 
        p_start = np.array(start_pose[:3])  #  (x, y, z)
        euler_start = start_pose[3:]  #  (r_x, r_y, r_z)
        p_center = np.array(center)  # 
        axis = np.array(rotation_axis)  # 

        # 
        u = axis / np.linalg.norm(axis)

        #  ( theta)
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        u_x, u_y, u_z = u
        R_axis = np.array([
            [cos_theta + u_x**2 * (1 - cos_theta), u_x * u_y * (1 - cos_theta) - u_z * sin_theta, u_x * u_z * (1 - cos_theta) + u_y * sin_theta],
            [u_y * u_x * (1 - cos_theta) + u_z * sin_theta, cos_theta + u_y**2 * (1 - cos_theta), u_y * u_z * (1 - cos_theta) - u_x * sin_theta],
            [u_z * u_x * (1 - cos_theta) - u_y * sin_theta, u_z * u_y * (1 - cos_theta) + u_x * sin_theta, cos_theta + u_z**2 * (1 - cos_theta)]
        ])

        # 
        p_local = p_start - p_center  # 
        p_rot = R_axis @ p_local  # 
        p_end = p_rot + p_center  # 

        # 
        R_start = R.from_euler('xyz', euler_start).as_matrix()

        # 
        R_end = R_axis @ R_start  # 
        euler_end = R.from_matrix(R_end).as_euler('xyz')  # 
        combined = np.concatenate((p_end, euler_end))
        return combined
    def calculate_rotation_to_target_z_axis(self,start_pose, center, rotation_axis, tolerance=1e-1):
        """
         Z .
        
        :param start_pose:  (x, y, z, r_x, r_y, r_z), (XYZ )
        :param center:  (cx, cy, cz)
        :param rotation_axis:  (ux, uy, uz)
        :param tolerance: 
        :return:  θ ()
        """
        # 
        p_start = np.array(start_pose[:3])  # 
        euler_start = start_pose[3:]  # 
        p_center = np.array(center)  # 
        axis = np.array(rotation_axis)  # 

        # 
        u = axis / np.linalg.norm(axis)

        # 
        target_vector = p_center - p_start  # 
        target_vector = target_vector / np.linalg.norm(target_vector)  # 

        #  Z 
        R_start = R.from_euler('xyz', euler_start).as_matrix()  # 
        z_start = R_start[:, 2]  #  Z 

        # 
        def find_theta(z_start, target_vector, u):
            for theta in np.linspace(0, 2 * np.pi, 1000):  #  [0, 2π] 
                # 
                R_axis = R.from_rotvec(theta * u).as_matrix()
                z_end = R_axis @ z_start  #  Z 

                #  z_end  target_vector 
                if np.abs(np.dot(z_end, target_vector) - 1) < tolerance:
                    return theta
            return None

        # Find tcp_pose = [tcp_position.x, tcp_position.y, tcp_position.z, tcp_euler.x, tcp_euler.y, tcp_euler.z]
        theta = find_theta(z_start, target_vector, u)

        if theta is None:
            raise ValueError(",")

        return theta
   
    def get_targetpose(self, pose, rpy_xyz=(180*np.pi/180, 0*np.pi/180, 90*np.pi/180),is_radians=False, knife=True, mode="target", arm_index=1):
        # 
        depth_to_color = np.array([
            [0.99992, -0.0116565, -0.00493683, 14.8483],
            [0.0116607, 0.999932, 0.000819961, -0.120824],
            [0.00492694, -0.000877462, 0.999987, 7.04899e-02],
            [0., 0., 0., 1.]
        ])
        # 🔥  arm_index 
        cam_pose = self.right_camera_pose if arm_index == 1 else self.left_camera_pose
      #  TCP 
        tcp_position = self.arm_controllers[arm_index].arm_feedback.pose.tcp.pos
        tcp_euler = self.arm_controllers[arm_index].arm_feedback.pose.tcp.euler
        tool_pose = [tcp_position.x, tcp_position.y, tcp_position.z, tcp_euler.x, tcp_euler.y, tcp_euler.z]
        if tool_pose is None:
            self.get_logger().error("TCP pose is not available yet.")
            return None
        # print("tool_pose", tool_pose)
        tool_to_base_translation = pose_to_matrix(tool_pose)
        transformation_matrix = self._create_transformation_matrix(pose, is_radians,rpy_xyz)

        # 
        depth_to_base = np.dot(tool_to_base_translation, cam_pose)
        if mode == "6D":
            deep_cam2tool = np.dot(cam_pose, depth_to_color)
            # rotation_matrix_1 = deep_cam2tool[:3, :3]
            # rot_z_matrix = np.array([[-0.70711, 0.70711, 0], [-0.70711, -0.70711, 0], [0, 0, 1]])  # 45
            # point_to_tool = np.dot(rotation_matrix_1, rot_z_matrix)
            # deep_cam2tool[:3, :3] = point_to_tool
            depth_to_base = np.dot(tool_to_base_translation, deep_cam2tool)
            combined = self._process_target_mode(depth_to_base, transformation_matrix, mode="6D", rpy_xyz=rpy_xyz, knife=knife)
        elif mode == "target":
            combined = self._process_target_mode(depth_to_base, transformation_matrix, mode="target", rpy_xyz=rpy_xyz, knife=knife)
        elif mode == "left_pin":
            combined = self._process_left_pin_mode(depth_to_base, transformation_matrix)
       
        return combined
    
    def _create_transformation_matrix(self, pose, is_radians,rpy_xyz):
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, 3] = pose[:3]
        rotation_angles = rpy_xyz
        if not is_radians:
            rotation_angles = np.radians(rotation_angles)
        rotation_matrix = R.from_euler('xyz', rotation_angles, degrees=False).as_matrix()
        transformation_matrix[:3, :3] = rotation_matrix
        return transformation_matrix

    def _process_target_mode(self, depth_to_base, transformation_matrix, rpy_xyz=None, mode="target", knife=True):
        """
        (6D ).
        
        :param depth_to_base: 
        :param transformation_matrix: 
        :param rpy_xyz:  (roll, pitch, yaw)
        :param mode:  ("6D"  "target")
        :param knife: 
        :return:  [x, y, z, roll, pitch, yaw]
        """
        object_to_base_translation = np.dot(depth_to_base, transformation_matrix)
        current_translation = object_to_base_translation[:3, 3]

        if knife:
            rotation_matrix = R.from_euler('xyz', rpy_xyz, degrees=False).as_matrix()
            knife_offset = np.array([8.7643, -8.7643, 84.23])
            knife_rotation_matrix=knife_offset*rotation_matrix
            current_translation = current_translation - knife_rotation_matrix.sum(axis=1) 
            # print("knife_current_translation",current_translation)

        if mode == "6D":
            euler_angles = R.from_matrix(object_to_base_translation[:3, :3]).as_euler('xyz', degrees=False)
        elif mode == "target":
            euler_angles = np.array(rpy_xyz)
        else:
            self.get_logger().error(f": {mode}")
            return None

        current_translation[2] += 3  # Z 
        return np.concatenate((current_translation, euler_angles))


    def _process_left_pin_mode(self, depth_to_base, transformation_matrix, arm_index=1):
        """
        ,.
        
        :param depth_to_base: 
        :param transformation_matrix: 
        :param arm_index:  (1)  (0)
        :return:  [x, y, z, roll, pitch, yaw]
        """
        # 
        right_camera_pose, left_camera_pose = (self.top_rightcamera_pose, self.top_leftcamera_pose) \
            if arm_index == 1 else (self.top_leftcamera_pose, self.top_rightcamera_pose)

        # 
        object_to_base_translation = self._compute_transformation(
            depth_to_base, transformation_matrix, right_camera_pose, left_camera_pose
        )

        current_translation = object_to_base_translation[:3, 3]
        rpy_xyz = np.array([1.570796327, 0.78539816339, -2.35619449019])
        current_translation += np.array([11.4, 1.04, -2.45])

        return np.concatenate((current_translation, rpy_xyz))


    def _compute_transformation(self, depth_to_base, transformation_matrix, right_camera_pose, left_camera_pose):
        """
        .
        
        :param depth_to_base: 
        :param transformation_matrix: 
        :param right_camera_pose: 
        :param left_camera_pose: 
        :return: 
        """
        right_camera_to_top_translation = np.dot(invert_transform(right_camera_pose), depth_to_base)
        right_camera_to_leftbase_translation = np.dot(left_camera_pose, right_camera_to_top_translation)
        return np.dot(right_camera_to_leftbase_translation, transformation_matrix)
    
    def transform_with_combined(self, combined, theta_x, theta_y, theta_z, L):
        """
        Parameters(combined),,.
        ,.
        
        Parameters:
        - combined: np.ndarray, , [x, y, z, roll, pitch, yaw].
        - theta_x: float, - x ().
        - theta_y: float,  y ().
        - theta_z: float,  z ().
        - L: float, .
         -2.706794133013487, 0.36197585591144077, 1.6778486254083171
          2.77568823928427, -0.3201011841616206, 0.03134225891614126

        
        Returns:
        - new_combined: np.ndarray, , [x, y, z, roll, pitch, yaw].
        """
        # 
        current_translation = combined[:3]
        print("current_translation",current_translation)
        current_rpy = combined[3:]
        print("current_rpy",current_rpy)

        #  pose_and_quaternion_to_matrix 
        # T_tool_to_world = self.pose_and_quaternion_to_matrix(current_translation, current_rpy)
        # R_tool = T_tool_to_world [:3, :3]
        R_tool =R.from_euler('xyz', current_rpy, degrees=False).as_matrix()
        
        # ()
        cos_x, sin_x = np.cos(theta_x), np.sin(theta_x)
        cos_y, sin_y = np.cos(theta_y), np.sin(theta_y)
        cos_z, sin_z = np.cos(theta_z), np.sin(theta_z)
        direction_local = np.array([
            cos_y * cos_z,
            sin_x * sin_y * cos_z + cos_x * sin_z,
            cos_x * sin_y * cos_z - sin_x * sin_z
        ])

        # 
        d_world = R_tool @ direction_local  # 
        d_world /= np.linalg.norm(d_world)  # 
        rpy_xyz=(-2.706794133013487, 0.36197585591144077, 1.6778486254083171)
        # 
        P_end = current_translation + L * d_world
        # P_end[2]=P_end[2]-16
        print(P_end)
        combined = np.concatenate((P_end, rpy_xyz))
        return combined

 
    def openbox_pose(self):
        """ , """
        
        if not self.pose_update_needed or self.grasp_points is None:
            self.get_logger().warn(",")
            return
        
        grasp_points = self.grasp_points
        required_keys = ["topleft", "topright", "centerleft", "centerright", "downleft", "downright"]
        missing_keys = [key for key in required_keys if key not in grasp_points]

        if missing_keys:
            self.get_logger().error(f"❌ : {missing_keys},")
            return

        # 
        RPY_xyz = self.compute_pose(grasp_points["centerleft"], grasp_points["topleft"], grasp_points["centerright"], -135)
        centerleft_point1 = self.get_targetpose(grasp_points["centerleft"], rpy_xyz=RPY_xyz, knife=True,mode="6D")

        #  RPY 
        rpy_adjusted = np.array([170*np.pi/180, 10*np.pi/180, centerleft_point1[5]])

        # 
        key_points = {key: self.get_targetpose(grasp_points[key], rpy_xyz=rpy_adjusted,is_radians=True) for key in required_keys}
        key_points["topcenter"] = (key_points["topleft"] + key_points["topright"]) / 2
        key_points["center"] = (key_points["centerleft"] + key_points["centerright"]) / 2
        key_points["downcenter"] = (key_points["downleft"] + key_points["downright"]) / 2

        # 
        poseleft = self.get_targetpose(grasp_points["centerleft"], mode="left_pin")
        print("poesleft:",poseleft)
        poseleft=self.transform_with_combined(poseleft,np.radians(0),np.radians(0),np.radians(-135),42)
        down_center = key_points["downcenter"] + 1.1 * (key_points["downcenter"] - key_points["center"])
        top_center = key_points["topcenter"] + 1.1 * (key_points["topcenter"] - key_points["center"])
        left_circl = key_points["centerleft"] + (key_points["centerleft"] - key_points["center"])
        right_circl = key_points["centerright"] + (key_points["centerright"] - key_points["center"])

        # 
        distance = np.linalg.norm(key_points["center"][:2] - right_circl[:2])
        print("distance:", distance)

        # 
        centerleft_point1 += np.array([0, 0, 0, 0, 0, -90*np.pi/180])
        rpy_adjusted2 = np.array([170*np.pi/180, 10*np.pi/180, centerleft_point1[5]])
        key_points_shifted = {key: self.get_targetpose(grasp_points[key], rpy_xyz=rpy_adjusted2,is_radians=True,arm_index=1) for key in required_keys}
        key_points_shifted["top_right"] = key_points_shifted["topright"] + 0.97 * (key_points_shifted["topright"] - key_points_shifted["centerright"])
        key_points_shifted["top_left"] = key_points_shifted["topleft"] + 0.97 * (key_points_shifted["topleft"] - key_points_shifted["centerleft"])
        key_points_shifted["down_left"] = key_points_shifted["downleft"] + 0.97 * (key_points_shifted["downleft"] - key_points_shifted["centerleft"])
        key_points_shifted["down_right"] = key_points_shifted["downright"] + 0.97 * (key_points_shifted["downright"] - key_points_shifted["centerright"])

        # 
        top_leftH = key_points_shifted["top_left"] + np.array([0, 0, 100, 0, 0, 0])
        down_centerH = down_center + np.array([0, 0, 100, 0, 0, 0])
        trajectory0 = self.calculate_rotated_point_and_orientation_euler(key_points["center"],right_circl[:3],(key_points["topcenter"] - key_points["center"])[:3],np.pi/4)
        trajectory1= self.calculate_rotated_point_and_orientation_euler(key_points["center"],right_circl[:3],(key_points["topcenter"] - key_points["center"])[:3],np.pi/2)
        trajectory1[3:]=(3.14159,0,1.5708)
        trajectory2= self.calculate_rotated_point_and_orientation_euler(trajectory1,right_circl[:3],(key_points["topcenter"] - key_points["center"])[:3],np.pi/4)
        trajectory3= self.calculate_rotated_point_and_orientation_euler(trajectory1,right_circl[:3],(key_points["topcenter"] - key_points["center"])[:3],np.pi/2)
        
        print(" topcenter_point :", key_points["topcenter"])
        print(" center_point :", key_points["center"])
        print(" downcenter_point :",  key_points["downcenter"])
        print(" poseleft :", poseleft)
        print(" down_center :", down_center)
        print(" top_left :", key_points_shifted["top_left"])
        print(" top_right :", key_points_shifted["top_right"])
        print("left_circle:", left_circl) 
        print("right_circle:", right_circl)
        # 
        posesey = (424.307, -118.863, 60.0, 3.1416, 0.0, 1.5808)
        move_sequence = [
            ("moveTo", key_points_shifted["top_right"]),
            ("moveTo", key_points_shifted["top_left"]),
            ("moveTo", top_leftH),
            ("moveTo", top_center),
            ("moveTo", down_center),
            ("moveTo", down_centerH),
            ("moveTo", key_points_shifted["down_right"]),
            ("moveTo", key_points_shifted["down_left"]),
            ("moveTo", posesey),
        ]

        # self.add_action_to_sequence("jointTo", 1, position=[230.416*np.pi/180, 105.559*np.pi/180, 38.741*np.pi/180, 
        #                  93.193*np.pi/180, -123.021*np.pi/180, -229.462*np.pi/180])
        # self.add_action_to_sequence("moveTo", 0, position=poseleft)
        # self.add_action_to_sequence(
        #     ("moveTo", None), 
        #     (1, None),
        #     decision_logic=(None, None),
        #     kwargs1={'tcp': poseleft},
        # )
        for action, position in move_sequence:
            self.add_action_to_sequence( 
            (action, None), 
            (1, None),
            decision_logic=(None, None),
            kwargs1={'tcp': position},
            )

        self.get_logger().info(f"✅ , {len(move_sequence)}  moveTo ")

    def compute_pose(self,A, B, C,theta):
        rpy=(0,0,0)
        theta = np.radians(theta)
        A = np.array(A[:3])
        B = np.array(B[:3])
        C = np.array(C[:3])

        # y
        AB = B - A
        AC = C - A
        y_axis = np.cross(np.cross(AB, AC), AB)  # y
        x_axis = AB / np.linalg.norm(AB)  # x
        y_axis = y_axis / np.linalg.norm(y_axis)  # 

        # z
        z_axis = np.cross(x_axis, y_axis)
        z_axis = z_axis / np.linalg.norm(z_axis)  # 

        # 
        rotation_matrix = np.vstack([x_axis, y_axis, z_axis]).T
        R_selfZ = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
        ])

        # 
        grasp_rotation_matrix= rotation_matrix @ R_selfZ
        rrr=R.from_matrix(grasp_rotation_matrix)
        euler_angles = rrr.as_euler('xyz', degrees=True)


        return euler_angles
    
    
    def action_finished(self, message: Message):
        """
        .
        Returns True ,False .
        """
        if not message.action_func:
            return True  # Returns

        is_executing = True
        action_name = message.action_func.__name__
        controller = self.arm_controllers[message.index]  # 
    
        if action_name in ["jointTo","moveTo","ArcTo"]:
            if controller.arm_feedback.is_running:  # ,
                return not is_executing
            elif controller.arm_feedback.success:  # 
                # ✅ 
                self.get_logger().info(f"{controller.name_prefix}  {action_name} ")
                is_executing = False
            else:  # 
                self.get_logger().error(f"{controller.name_prefix}  {action_name} ,")
                is_executing = False
                  # 

        # 2️⃣ 
        elif action_name in ["closeGripper", "openGripper"]:
            if controller.gripper_feedback.is_running:  # ,
                return not is_executing
            elif controller.gripper_feedback.success:
                self.get_logger().info(f"{controller.name_prefix}  {action_name} ")
                is_executing = False
            else:  # 
                self.get_logger().error(f"{controller.name_prefix}  {action_name} ,")
                is_executing = False
        elif action_name in ["wait", "wait_time"]:
            arm = self.waiting_arms[message.index]
            if arm["waiting"]:
                return not is_executing

            is_executing = False
            self.get_logger().info(f"{controller.name_prefix}  {action_name} ")

        if message.pending_decision_logic:
            logic, action_name = message.pending_decision_logic
            if logic:
                self.get_logger().info(f"{action_name} ,: {logic.__name__}")
                logic()
            message.pending_decision_logic = None  # ,

            # 3️⃣ ,
        is_executing = False
        # self.next_action()
        return not is_executing


    def check_action_finished(self):
        if self.current_action is None:
            return
        
        # Iterate over,
        ret = True
        for msg in self.current_action.messages:
            if msg.action_func is not None:
                ret &= self.action_finished(msg)
        self.is_executing = not ret
        # ✅ ,
        if not self.is_executing:
            self.next_action()
        

    def add_action_to_sequence(self, action_name, index, decision_logic=None, **kwargs):
        """
        ,.
        action_name  index ,kwargs .
        """
        #  action_name  index  tuple
        action_name = (action_name, None) if isinstance(action_name, str) else action_name
        index = (index, index ^ 1) if isinstance(index, int) else index
        decision_logic = (decision_logic, None) if isinstance(decision_logic, (Callable, type(None))) else decision_logic

        #  kwargs,
        if "kwargs1" in kwargs or "kwargs2" in kwargs:
            kwargs_1 = kwargs.pop("kwargs1", {})
            kwargs_2 = kwargs.pop("kwargs2", {})
        else:
            kwargs_1, kwargs_2 = kwargs, {}

        #  action 
        action_func_1 = self.action_map.get(action_name[0])
        action_func_2 = self.action_map.get(action_name[1]) if action_name[1] else None

        if not action_func_1:
            return self.get_logger().error(f": {action_name[0]}")
        if action_name[1] and not action_func_2:
            return self.get_logger().error(f": {action_name[1]}")

        # 
        msg1 = Message(index[0], action_func_1, decision_logic[0], **kwargs_1)
        msg2 = Message(index[1], action_func_2, decision_logic[1], **kwargs_2)
        messages = [msg1, msg2]
        # if action_func_2:
        #     msg2 = Message(index[1], action_func_2, decision_logic[1], **kwargs_2)
        #     messages.append(msg2)

        # 
        self.action_queue.add_action(Action(messages))

        # 
        for i, name in enumerate(action_name):
            if name:
                params = kwargs_1 if i == 0 else kwargs_2
                self.get_logger().info(f": {name} ,Parameters: {params},: {decision_logic[i]}")
                self.action_info.append((name, index[i], params, decision_logic[i]))
        # # ✅ ,
        # if not self.is_executing:
        #     self.next_action()

    def manage_task_execution(self):
        if not self.initialized:
            self.add_tasks()
            # self.moveTo(1, [6.34710329e+02 ,-3.19229682e+02 ,-1.28288696e+02 ,-3.13797304e+00,2.18560313e-02 ,1.64808736e+00])
            # self.openGripper(1,20,1000)
            self.start_next_task()
            return

        if self.is_executing:
            self.check_action_finished()
            if not self.is_executing and not self.task_queue:  # 
                self.get_logger().info(",")
            return
        if self.action_queue.is_empty():
            self.is_executing = False
            self.start_next_task()  # ,
            # print("self.action_history:",self.action_history)
            return
        self.next_action()

    def next_action(self):
        """4️⃣ """
        self.current_action =  self.action_queue.get_next_action()
        if self.current_action is None:
            self.is_executing = False
            return

        self.current_action.execute()
        self.is_executing = True
        self.action_history.append(self.current_action)  # 
    
    def start_next_task(self):
        ""","""
        if not self.task_queue:  # 
            return
        self.initialized = True
        if self.current_task is None or self.action_queue.is_empty():  #  action_queue 
            self.current_task = self.task_queue.pop(0)  # 
            self.get_logger().info(f": {self.current_task}")
            if self.current_task == "grasp_knife":
                self.grasp_knife()
            elif self.current_task == "open_box":
                self.open_box()
            elif self.current_task == "pick_and_grasp_knife":
                self.pick_and_grasp_knife()
            elif self.current_task =="grasp_object":
                self.grasp_object()
            elif self.current_task == "place_object":
                self.place_object()
            else:
                self.get_logger().warn(f"⚠️ : {self.current_task}")
    
    def add_tasks(self):
        ""","""
        self.task_queue = ["grasp_knife", "open_box","place_object"]
        # self.task_queue =["grasp_object"]
        # self.task_queue =["pick_and_grasp_knife"]
        # self.task_queue =["open_box"]
        # self.task_queue =["grasp_knife","place_object"]
        # self.task_queue =["place_object"]
        self.current_task = None  # 

    def pick_and_grasp_knife(self):
        if not self.arm_controllers[0].feedback_received or not self.arm_controllers[1].feedback_received :
            self.get_logger().info(",")
            #return None
        self.initialized = True
        self.get_logger().info("🔪 ###################################################################")
        self.add_action_to_sequence(
            ("moveTo", "moveTo"), 
            (1, 0), 
            # decision_logic=(self.grasp_knife(), self.grasp_object()), 
            kwargs1={'tcp': [280.516,26.289,41.087,3.1416,0.0,1.5808]},
            kwargs2={'tcp': [-321.146,-506.292,36.00,3.1416,0.0,0.0]}
        )
        self.add_action_to_sequence(
            ("wait", "wait"), 
            (1, 0), 
            decision_logic=(None, self.dul_pose), 
            kwargs1={'params': 1},
            kwargs2={'params': 2}
        )
        # self.add_action_to_sequence(
        #     ("closeGripper", "closeGripper"), 
        #     (1, 0), 
        #     # decision_logic=(self.calculate_target_pose, self.grasp4D_place), 
        #     kwargs1={'force': 50},
        #     kwargs2={'force': 20}
        # )
        
        # self.add_action_to_sequence(
        #     ("openGripper", "openGripper"),
        #     (1, 0), 
        #     # decision_logic=(self.calculate_target_pose, self.grasp4D_place), 
        #     kwargs1={'force': 50},
        #     kwargs2={'force': 20}
        # )
    
        
        return None 
    
    
    def grasp4D_place(self):
        if self.grasp_info["x"]!=0.0:
            grasp_points = self.grasp_info
            grasp_pose=self.get_targetpose([grasp_points["x"],grasp_points["y"],grasp_points["z"]],rpy_xyz=(180*np.pi/180,0*np.pi/180,grasp_points["angle"]),knife=False,arm_index=0)
            # grasp_pose[2]=grasp_pose[2]-8
            grasp_pose[2]=grasp_pose[2]-grasp_points["grasp_depth"]-8
            grasp_pose1=grasp_pose.copy()
            grasp_pose1[2]=grasp_pose1[2]+51
            print("grasp_pose:",grasp_pose,"grasp_depth:",grasp_points["grasp_depth"])
  
            self.add_action_to_sequence("moveTo", 0, position=grasp_pose)
            self.add_action_to_sequence('closeGripper', 0,force=50)
            self.add_action_to_sequence("moveTo", 0, position=grasp_pose1)
            self.add_action_to_sequence('moveTo', 0, position=[-514.992,-154.943,-223.237,-127.122*np.pi/180,34.834*np.pi/180,18.532*np.pi/180])
            self.add_action_to_sequence('openGripper', 0,force=50)
            self.add_action_to_sequence('moveTo', 0, position=[-321.146,-506.292,36.00,3.1416,0.0,0.0])         
   
    def place_object(self):
        """"""
        self.get_logger().info("🔪 ###################################################################")

        self.add_action_to_sequence('moveTo', 1, position=[305.23+10,4.00-10,-145.00,3.1416,0.0,1.5808])
        self.add_action_to_sequence('moveTo', 1, position=[305.23-1,4.00+1,-135.00,3.1416,0.0,1.5808])
        self.add_action_to_sequence('openGripper', 1,force=50)

        self.add_action_to_sequence('moveTo', 1, position=[280.516,26.289,41.087,3.1416,0.0,1.5808])
       
    def grasp_knife(self):
        """"""
        self.get_logger().info("🔪 ###################################################################")
        self.add_action_to_sequence(
            ("moveTo", None), 
            (1, None),
            decision_logic=(None, None),
            kwargs1={'tcp': [280.516,26.289,41.087,3.1416,0.0,1.5808]},
        )
        self.add_action_to_sequence(
            ("wait", None), 
            (1, None),
            decision_logic=(self.calculate_target_pose, None),
            kwargs1={'params': 1},
        )
        # self.add_action_to_sequence('moveTo', 1, position=[280.516,26.289,41.087,3.1416,0.0,1.5808])
        # self.add_action_to_sequence('moveTo', 1, position=[280.516,26.289,61.087,3.1416,0.0,1.5808])
        # self.add_action_to_sequence('wait', 1,params=1,decision_logic=self.calculate_target_pose)
        # graspose=self.get_targetpose(self.aruco_grasp_position,self.aruco_grasp_euler,knife=False,arm_index=1)
        # self.add_action_to_sequence('moveTo', 1, position=[517.683, -213.007, 60.0, 3.1416, 0.0, 1.5808])
        # self.add_action_to_sequence('jointTo', 1, joint_angles=[230.416*np.pi/180, 105.559*np.pi/180, 38.741*np.pi/180, 93.193*np.pi/180, -123.021*np.pi/180, -229.462*np.pi/180])
    
    def grasp_object(self):
        """"""
        self.get_logger().info("🔪 ###################################################################")
        self.add_action_to_sequence('moveTo', 0, position=[-321.146,-506.292,36.00,3.1416,0.0,0.0])
        self.add_action_to_sequence('wait',0,params=1,decision_logic=self.grasp4D_place)
    def open_box(self):
        """"""
        self.get_logger().info("🔪 ###################################################################")
        # self.add_action_to_sequence('moveTo', 1, position=[517.683, -213.007, 60.0, 3.1416, 0.0, 1.5808],decision_logic=self.openbox_pose)
        self.add_action_to_sequence(
            ("moveTo", None), 
            (1, None),
            decision_logic=(None, None),
            kwargs1={'tcp': [424.307, -118.863, 60.0, 3.1416, 0.0, 1.5808]},
        )
        self.add_action_to_sequence(
            ("wait", None), 
            (1, None),
            decision_logic=(self.openbox_pose, None),
            kwargs1={'params': 6},
        )
        # self.add_action_to_sequence('moveTo', 1, position=[424.307, -118.863, 60.0, 3.1416, 0.0, 1.5808])
        # self.add_action_to_sequence('wait',1,params=6,decision_logic=self.openbox_pose)
    def start_grasping_sequence(self):
        """"""
        # 
        self.add_action_to_sequence('closeGripper', 0,force=50)
        self.add_action_to_sequence('jointTo', 1, joint_angles=[230.416*np.pi/180, 105.559*np.pi/180, 38.741*np.pi/180, 93.193*np.pi/180, -123.021*np.pi/180, -229.462*np.pi/180])
        self.add_action_to_sequence('moveTo', 1, position=[517.683, -213.007, 60.0, 3.1416, 0.0, 1.5808])
        # self.calculate_target_pose()
        self.add_action_to_sequence('closeGripper', 1,force=50)
        self.get_logger().info(f"Action queue: {self.action_info}")
    
    def dul_pose(self, feedback_data=None):
        """ Aruco """
        self.get_logger().info(" Aruco ...")
        while not self.pose_received_event.wait(timeout=1):  # 
            self.get_logger().warn("Aruco ,...")
        
        self.get_logger().info(f"Aruco : {self.aruco_grasp_position}")
        graspose = self.get_targetpose(self.aruco_grasp_position, knife=False, arm_index=1)
        self.pose_received_event.clear()  # ,
        # arr[:3] = [a + b for a, b in zip(arr[:3], [x, y, z])]#
        graspose[:3] += np.array([-20,18,-48])
        graspose = [graspose[0], graspose[1], graspose[2], 3.1416, 0.0, 1.5808]
        self.get_logger().info(f"🎯 : {graspose},")
        if self.grasp_info["x"]!=0.0:
            grasp_points = self.grasp_info
            grasp_pose=self.get_targetpose([grasp_points["x"],grasp_points["y"],grasp_points["z"]],rpy_xyz=(180*np.pi/180,0*np.pi/180,grasp_points["angle"]),knife=False,arm_index=0)
            # grasp_pose[2]=grasp_pose[2]-8
            grasp_pose[2]=grasp_pose[2]-grasp_points["grasp_depth"]-8
            grasp_pose1=grasp_pose.copy()
            grasp_pose1[2]=grasp_pose1[2]+51
            print("grasp_pose:",grasp_pose,"grasp_depth:",grasp_points["grasp_depth"])
            self.add_action_to_sequence(
                        ("moveTo", "moveTo"), 
                        (1, 0), 
                        kwargs1={'tcp': graspose},
                        kwargs2={'tcp': grasp_pose}
                    )
            self.add_action_to_sequence(
                ("closeGripper", "closeGripper"), 
                (1, 0), 
                # decision_logic=(self.calculate_target_pose, self.grasp4D_place), 
                kwargs1={'force': 50},
                kwargs2={'force': 50}
            )
            self.add_action_to_sequence(
                ("moveTo", "moveTo"), 
                (1, 0), 
                kwargs1={'tcp': [graspose[0],graspose[1],41.087,3.1416,0.0,1.5808]},
                kwargs2={'tcp': grasp_pose1}
            )
            self.add_action_to_sequence(
                ("moveTo", "moveTo"), 
                (1, 0),
                kwargs1={'tcp': [424.307, -118.863, 60.0, 3.1416, 0.0, 1.5808]},
                kwargs2={'tcp': [-514.992,-154.943,-223.237,-127.122*np.pi/180,34.834*np.pi/180,18.532*np.pi/180]}
            )
            
            # self.add_action_to_sequence("moveTo", 0, position=grasp_pose)
            # self.add_action_to_sequence('closeGripper', 0,force=50)
            # self.add_action_to_sequence("moveTo", 0, position=grasp_pose1)
            # self.add_action_to_sequence('moveTo', 0, position=[-514.992,-154.943,-223.237,-127.122*np.pi/180,34.834*np.pi/180,18.532*np.pi/180])
            self.add_action_to_sequence('openGripper', 0,force=50)
            self.add_action_to_sequence(
                ("wait", "moveTo"), 
                (1, 0),
                decision_logic=(self.openbox_pose, None),
                kwargs1={'params': 10},
                kwargs2={'tcp': [-464.992,-304.943,-223.237,-127.122*np.pi/180,34.834*np.pi/180,18.532*np.pi/180]}
            )
            # self.add_action_to_sequence(
            #     ("wait", "moveTo"), 
            #     (1, 0),
            #     decision_logic=(self.openbox_pose, None),
            #     kwargs1={'params': 10},
            #     kwargs2={'tcp': [-321.146,-506.292,36.00,3.1416,0.0,0.0]}
            # )
            # self.add_action_to_sequence('moveTo', 0, position=[-321.146,-506.292,36.00,3.1416,0.0,0.0])         
    def calculate_target_pose(self, feedback_data=None):
        """ Aruco """
        self.get_logger().info(" Aruco ...")
        while not self.pose_received_event.wait(timeout=1):  # 
            self.get_logger().warn("Aruco ,...")
        
        self.get_logger().info(f"Aruco : {self.aruco_grasp_position}")
        graspose = self.get_targetpose(self.aruco_grasp_position, knife=False, arm_index=1)
        self.pose_received_event.clear()  # ,
        # arr[:3] = [a + b for a, b in zip(arr[:3], [x, y, z])]#
        graspose[:3] += np.array([-20,18,-50])
        graspose = [graspose[0], graspose[1], graspose[2], 3.1416, 0.0, 1.5808]
        self.get_logger().info(f"🎯 : {graspose},")
        self.add_action_to_sequence('moveTo', 1, position=graspose)
        self.add_action_to_sequence('closeGripper', 1,force=50)
        self.add_action_to_sequence('moveTo', 1, position=[graspose[0],graspose[1],41.087,3.1416,0.0,1.5808])
        self.get_logger().info("✅ ,")
