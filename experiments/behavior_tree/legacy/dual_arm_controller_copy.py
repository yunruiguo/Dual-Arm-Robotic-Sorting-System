'''
FilePath: experiments/behavior_tree/legacy/dual_arm_controller_copy.py
Author: Yunrui Guo
Date: 2024-12-26 11:59:14
LastEditors: Please set LastEditors
LastEditTime: 2025-03-13 22:01:37

Descripttion: 
'''
import rclpy
from pathlib import Path
REAL_CONFIG_DIR = Path(__file__).resolve().parents[1] / "test_behavior" / "real"                                  # ROS2 Python library
import numpy as np    
import time                        # library
import math                                   # library
from scipy.spatial.transform import Rotation as R  # 
from std_msgs.msg import Float64MultiArray    # ROS,
from geometry_msgs.msg import PoseArray       # ROS,
import tf_transformations                     # library
from rclpy.node import Node                   # ROS2 
from rclpy.action import ActionClient         # ROS2 Action 
from rclpy.exceptions import ROSInterruptException  # ROSexception
from rclpy.executors import MultiThreadedExecutor     # ()
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup  # ,
from grasp_msgs.msg import ArmCommand, GripperCommand  # ,
from grasp_msgs.action import ArmControl, GripperControl   # ,
from action_msgs.msg import GoalStatus        #  Action 
from rclpy.subscription import Subscription     # ROS2 ()
from grasp_msgs.msg import FeedBackMsg          # 
import logging
from rclpy.executors import SingleThreadedExecutor
# -----------------------------------------------------------------------------
# 
# -----------------------------------------------------------------------------
class ArmFeedbackMsg:
    def __init__(self):
        #  ArmCommand ,(TCP)
        self.pose = ArmCommand()
        self.distance_remaining = 0.0  # ()
        self.is_running = False        # 
        self.success = False           # 

    def reset(self):
        """."""
        self.pose = ArmCommand()       # 
        self.distance_remaining = 0.0    # 
        self.is_running = False          # 
        self.success = False             # 

# -----------------------------------------------------------------------------
# 
# -----------------------------------------------------------------------------
class GripperFeedbackMsg:
    def __init__(self):
        #  GripperCommand ,
        self.pose = GripperCommand()
        self.success = False  # 

    def reset(self):
        """."""
        self.pose = GripperCommand()  # 
        self.distance_remaining = 0.0
        self.is_running = False
        self.success = False

class ArmController:
    def __init__(self, node, name_prefix):
        self.node = node                     # ,Output
        self.name_prefix = name_prefix       # ( "left"  "right")
        self.arm_feedback = ArmFeedbackMsg() # 
        self.gripper_feedback = GripperFeedbackMsg() # 
        self.send_goal_future = None
        self.feedback_received = False

        # ,
        self.arm_cb_group = MutuallyExclusiveCallbackGroup()
        self.gripper_cb_group = MutuallyExclusiveCallbackGroup()
        self.feedback_cb_group = MutuallyExclusiveCallbackGroup()
        self.node.get_logger().info(f"Created arm action for {name_prefix}")

        # Create the subscription with a custom callback group
        self.arm_feedback_sub = self.node.create_subscription(
            FeedBackMsg,                             # 
            f"{name_prefix}/feedback_states",        # ,
            self.arm_feedback_callback,              # 
            1,                                       # 
            callback_group=self.feedback_cb_group    # 
        )
        
        # ,
        try:
            self.arm_action_client = ActionClient(
                node,
                ArmControl, 
                f"{name_prefix}/arm_control",         # 
                callback_group=self.arm_cb_group       # 
            )
            self.node.get_logger().info(f"Created arm action client for {name_prefix}")
        except ROSInterruptException as e:
            self.node.get_logger().error(f"Error creating arm action client: {e}")
            raise

        self.node.get_logger().info(f"Created gripper action for {name_prefix}")
        
        # ,
        try:
            self.gripper_action_client = ActionClient(
                node, 
                GripperControl, 
                f"{name_prefix}/gripper_control",      # 
                callback_group=self.gripper_cb_group     # 
            )
            self.node.get_logger().info(f"Created gripper action client for {name_prefix}")
        except ROSInterruptException as e:
            self.node.get_logger().error(f"Error creating gripper action client: {e}")
            raise
        
        self.node.get_logger().info(f"Created controller for {name_prefix}")


    def wait_for_action_servers(self):
        """ Action ."""
        self.node.get_logger().info(f"{self.name_prefix}")
        while not self.arm_action_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().info(f"{self.name_prefix}...")

        while not self.gripper_action_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().info(f"{self.name_prefix}...")

    def arm_feedback_callback(self, msg):
        """Callback for handling arm feedback."""
        #self.node.get_logger().info(f"Received feedback: {msg}")
        # ,
        # if not self.arm_feedback.is_running:
            # Update TCP position
        self.feedback_received = True
        self.arm_feedback.pose.tcp.pos.x = msg.tcp_pose[0]
        self.arm_feedback.pose.tcp.pos.y = msg.tcp_pose[1]
        self.arm_feedback.pose.tcp.pos.z = msg.tcp_pose[2]

        # Update TCP Euler angles
        self.arm_feedback.pose.tcp.euler.x = msg.tcp_pose[3]
        self.arm_feedback.pose.tcp.euler.y = msg.tcp_pose[4]
        self.arm_feedback.pose.tcp.euler.z = msg.tcp_pose[5]
        # Update joint positions
        self.arm_feedback.pose.joint.joint = list(msg.joint_pose)
        # print(f"joints: {self.arm_feedback.pose.joint.joint}")
        # print(f"tcp: {self.arm_feedback.pose.tcp.pos.x}, {self.arm_feedback.pose.tcp.pos.y}, {self.arm_feedback.pose.tcp.pos.z}")
    def execute_arm(self, poses):
        """Send an action goal to move the arm."""
        self.arm_feedback.reset()  # 
        self.arm_feedback.is_running = False
        goal_msg = ArmControl.Goal()  # 
        goal_msg.poses = poses      # Set

        # 
        if not self.arm_action_client.wait_for_server(timeout_sec=10.0):
            self.node.get_logger().error(f"Action server for {self.name_prefix} arm is unavailable")
            return False
        
        # ,Set
        self.node.get_logger().info(f"Sending {self.name_prefix} arm goal")
        self.send_goal_future = self.arm_action_client.send_goal_async(goal_msg,feedback_callback=self.arm_action_feedback_callback)
        # ✅  Future 
        if self.send_goal_future is None:
            self.node.get_logger().error("❌ Failed to create self.send_goal_future!")
        else:
            self.node.get_logger().info("✅ self.send_goal_future created successfully!")
            self.arm_feedback.is_running = True

        # self.send_goal_future.add_done_callback(self.arm_goal_callback)
        self.send_goal_future.add_done_callback(self.arm_goal_callback)
        return True

    def execute_gripper(self, command):
        """Send an action goal to control the gripper."""
        self.gripper_feedback.reset()  # 
        goal_msg = GripperControl.Goal()  # 
        goal_msg.pose = command      # Set
        # 
        if not self.gripper_action_client.wait_for_server(timeout_sec=10.0):
            self.node.get_logger().error(f"Action server for {self.name_prefix} gripper is unavailable")
            return False
        
        self.gripper_feedback.is_running = True
        # ,Set
        self.node.get_logger().info(f"Sending {self.name_prefix} gripper goal")
        send_goal_future = self.gripper_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.gripper_goal_callback)
        return True
    def arm_action_feedback_callback(self, feedback_msg):
        """Handle feedback from the arm action server."""
        self.arm_feedback.distance_remaining = feedback_msg.feedback.distance_remaining
        # self.node.get_logger().info(f"{self.name_prefix} remaining {feedback_msg.feedback.distance_remaining} steps")
    def arm_goal_callback(self, future):
        """Handle the response from the arm action server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error(f"{self.name_prefix} arm goal was rejected")
            self.arm_feedback.success = False
            self.arm_feedback.is_running = False
            return
        self.arm_feedback.is_running = True
        self.node.get_logger().info(f"{self.name_prefix} arm goal accepted")
        # ,Set
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.arm_result_callback)

    def gripper_goal_callback(self, future):
        """Handle the response from the gripper action server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error(f"{self.name_prefix} gripper goal was rejected")
            self.gripper_feedback.success = False
            self.gripper_feedback.is_running = False
            return
        self.gripper_feedback.is_running = True
        self.node.get_logger().info(f"{self.name_prefix} gripper goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.gripper_result_callback)

    def arm_result_callback(self, future):
        """Handle the result from the arm action server."""
        result = future.result().result
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info(f"{self.name_prefix} arm goal succeeded")
            self.arm_feedback.success = True
        else:
            self.node.get_logger().warn(f"{self.name_prefix} arm goal failed or aborted")
            self.arm_feedback.success = False
        self.arm_feedback.is_running = False

    def gripper_result_callback(self, future):
        """Handle the result from the gripper action server."""
        result = future.result().result
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info(f"{self.name_prefix} gripper goal succeeded")
            self.gripper_feedback.success = True
        else:
            self.node.get_logger().warn(f"{self.name_prefix} gripper goal failed or aborted")
            self.gripper_feedback.success = False
        self.gripper_feedback.is_running = False


class Message:
    """Task class to manage tasks in the task queue."""
    def __init__(self):
        self.index = 0
        self.params = False
        self.action_func = None
        self.decision_logic = None
        self.pending_decision_logic = None
    def add_message(self, index, params, action_func, decision_logic):
        self.index = index
        self.params = params
        self.action_func = action_func
        self.decision_logic = decision_logic
        if decision_logic:
            self.pending_decision_logic = (decision_logic, action_func.__name__)



# class Task:
#     def __init__(self):
#         self.left = Message()
#         self.right = Message()
#         self.done = True
#         self.success = False

#     def add_message(self, msg, index):
#         if index == 0:
#             self.left = msg
#         else:
#             self.right = msg
#     def add_task(self, left, right):
#         self.left = left
#         self.right = right

# class TaskQueue:
#     def __init__(self):
#         self.queue = []

#     def add_message(self, left_msg,right_msg):
#         task = Task()
#         task.add_message(left_msg, 0)
#         task.add_message(right_msg, 1)
#         self.queue.append(task)
#         print("TaskQueue: add task {}", len(self.queue))

#     def get_task(self):
#         if len(self.queue) > 0:
#             return self.queue.pop(0)
#         else:
#             return None
#     def is_empty(self):
#         return len(self.queue) == 0

# class MessageQueue:
#     def __init__(self):
#         self.queue = []
#     def add_task(self, task):
#         self.queue.append(task)

#     def get_task(self):
#         if len(self.queue) > 0:
#             return self.queue.pop(0)
#         else:
#             return None
#     def is_empty(self):
#         return len(self.queue) == 0

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
        self.pose_update_needed = False  # 
        self.aruco_grasp_position =[]
        self.aruco_grasp_euler =[]
        self.task_queue = []  # 
        self.grasp_points = {}
        # 
        self.action_queue = [] 
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
        # self.task_action_queue = TaskQueue()  # 
        # ()
        self.arm_state = {'left': None, 'right': None}

        # 
        self.arm_controllers = [
            ArmController(self, "left"),
            ArmController(self, "right")
        ]
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
        self.timer = self.create_timer(0.005, self.onTimerCallback)  # ****

        # ()
        # self.start_grasping_sequence()
        # 
        # self.process_grasp_task()

    def load_camera_poses(self):
        """"""
        try:
            self.cam_pose = np.genfromtxt(str(REAL_CONFIG_DIR / 'right_camera_pose.txt'), delimiter=' ')
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
        self.create_subscription(Float64MultiArray, 'right/grasppose', self.subscription_callback, 10)
        self.create_subscription(PoseArray, '/right/aruco_poses', self.pose_callback, 10)
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
        self.pending_decision_logic = (decision_logic, "moveTo")  # 
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
        time.sleep(params)
        self.get_logger().info(f"🦾  {params} ")
        self.pending_decision_logic = (decision_logic, "wait")  # 
        # (),
        # if decision_logic:
        #     self.get_logger().info(f"🧠 : {decision_logic.__name__}")
        #     decision_logic()
        
        return True

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
    

    def subscription_callback(self,grasppose: Float64MultiArray):
        """ : """
        if len(grasppose.data) < 27:
            self.get_logger().warn(",")
            # return
        
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
    def pose_callback(self, pose_array: PoseArray):
        if not pose_array.poses:
            print("Warning: Received empty PoseArray!")
            return
        # Iterate overPoseArrayPose4x4
        for pose in pose_array.poses:
            # 
            position = [pose.position.x*1000, pose.position.y*1000, pose.position.z*1000]

            # 
            orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            # print('',orientation)
            # 3x3
            rotation_matrix = tf_transformations.quaternion_matrix(orientation)[0:3, 0:3]
            euler_angles = R.from_matrix(rotation_matrix).as_euler('xyz', degrees=True)
            self.aruco_grasp_position=position
            self.aruco_grasp_euler=euler_angles

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
   
    def get_targetpose(self, pose, rpy_xyz=(180, 0, 90),is_radians=False, knife=True, mode="target", arm_index=1):
        # 
        depth_to_color = np.array([
            [0.99992, -0.0116565, -0.00493683, 0.0148483],
            [0.0116607, 0.999932, 0.000819961, -0.000120824],
            [0.00492694, -0.000877462, 0.999987, 7.04899e-05],
            [0., 0., 0., 1.]
        ])
        
      #  TCP 
        tcp_position = self.arm_controllers[arm_index].arm_feedback.pose.tcp.pos
        tcp_euler = self.arm_controllers[arm_index].arm_feedback.pose.tcp.euler
        tool_pose = [tcp_position.x, tcp_position.y, tcp_position.z, tcp_euler.x, tcp_euler.y, tcp_euler.z]
        if tool_pose is None:
            self.get_logger().error("TCP pose is not available yet.")
            return None
        tool_to_base_translation = self.pose__matrix(tool_pose)
        transformation_matrix = self._create_transformation_matrix(pose, is_radians,rpy_xyz)

        # 
        depth_to_base = np.dot(tool_to_base_translation, self.cam_pose)
        if mode == "6D":
            deep_cam2tool = np.dot(self.cam_pose, depth_to_color)
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

    def _process_target_mode(self, depth_to_base, transformation_matrix, rpy_xyz=None,mode="target",knife=True):
        object_to_base_translation = np.dot(depth_to_base, transformation_matrix)
        current_translation = object_to_base_translation[:3, 3]

        if knife:
            # knife_to_base_translation = self.pose_and_quaternion_to_matrix(current_translation, rpy_xyz)
            # rotation_matrix = knife_to_base_translation[:3, :3]
            rotation_matrix=R.from_euler('xyz', rpy_xyz, degrees=False).as_matrix()
            knife = np.array([8.7643063e+00, -8.7643063e+00, 84.23])
            knife_rotation_matrix = knife * rotation_matrix
            current_translation -= knife_rotation_matrix.sum(axis=1)

        if mode == "6D":
            # 
            rotation_matrix = object_to_base_translation[:3, :3]
            euler_angles = R.from_matrix(rotation_matrix).as_euler('xyz', degrees=False)
        elif mode == "target":
            #  rpy_xyz 
            euler_angles = np.array(rpy_xyz)
        else:
            self.get_logger().error(f": {mode}")
            return None
        current_translation[2]+=3
        combined = np.concatenate((current_translation, euler_angles))
        return combined

    def _process_left_pin_mode(self, depth_to_base, transformation_matrix):
        right_camera_to_top_translation = np.dot(self.inverse_transformation_matrix(self.top_rightcamera_pose), depth_to_base)
        right_camera_to_leftbase_translation = np.dot(self.top_leftcamera_pose, right_camera_to_top_translation)
        object_to_base_translation = np.dot(right_camera_to_leftbase_translation, transformation_matrix)

        current_translation = object_to_base_translation[:3, 3]
        rpy_xyz = np.array((2.77568823928427, -0.3201011841616206, 0.03134225891614126))
        current_translation += np.array([11.4, 1.04, -2.45])

        combined = np.concatenate((current_translation, rpy_xyz))
    def pose__matrix(self,cartesian_pose):
        x, y, z, rx, ry, rz = cartesian_pose

        # 
        rotation = R.from_euler('xyz', [rx, ry, rz])
        rotation_matrix = rotation.as_matrix()

        # 
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 3] = [x, y, z]

        return transformation_matrix
    
    def transform_with_combined(self, combined, theta_x, theta_y, theta_z, L):
        """
        Parameters(combined),,.
        ,.
        
        Parameters:
        - combined: np.ndarray, , [x, y, z, roll, pitch, yaw].
        - theta_x: float,  x ().
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
    def inverse_transformation_matrix(self,T):
        R = T[:3, :3]
        t = T[:3, 3]

        # 
        R_inv = R.T

        # 
        t_inv = -np.dot(R_inv, t)


        # 
        T_inv = np.identity(4)
        T_inv[:3, :3] = R_inv
        T_inv[:3, 3] = t_inv

        return T_inv
    # def openbox_pose(self):
    #     if not self.pose_update_needed or self.grasp_points is None:
    #         self.get_logger().warn(",")
    #         return
    #     grasp_points = self.grasp_points
    #     #  grasp_points 
    #     required_keys = ["topleft", "topright", "centerleft", "centerright", "downleft", "downright"]
    #     missing_keys = [key for key in required_keys if key not in grasp_points]

    #     if missing_keys:
    #         self.get_logger().error(f"❌ : {missing_keys},")
    #         return
    #     RPY_xyz = self.compute_pose(grasp_points["centerleft"], grasp_points["topleft"], grasp_points["centerright"], -135)
    #     print("RPY_xyz:",RPY_xyz)
    #     posesey=(517.683,-213.007,60.00,3.1415926,0.00,1.570796)
    #     centerleft_point1=self.get_targetpose(grasp_points["centerleft"],rpy_xyz=RPY_xyz,knife=True,mode="6D")
    #     print("centerleft_point1:",centerleft_point1)
    #     topleft_point=self.get_targetpose(grasp_points["topleft"],rpy_xyz=np.array((170*np.pi/180,10*np.pi/180,centerleft_point1[5])))
    #     topright_point=self.get_targetpose(grasp_points["topright"],rpy_xyz=np.array((170*np.pi/180,10*np.pi/180,centerleft_point1[5])))
    #     centerright_point=self.get_targetpose(grasp_points["centerright"],rpy_xyz=np.array((170*np.pi/180,10*np.pi/180,centerleft_point1[5])))
    #     centerleft_point=self.get_targetpose(grasp_points["centerleft"],rpy_xyz=np.array((170*np.pi/180,10*np.pi/180,centerleft_point1[5])))
    #     downleft_point=self.get_targetpose(grasp_points["downleft"],rpy_xyz=np.array((170*np.pi/180,10*np.pi/180,centerleft_point1[5])))
    #     downright_point=self.get_targetpose(grasp_points["downright"],rpy_xyz=np.array((170*np.pi/180,10*np.pi/180,centerleft_point1[5])))
    #     topcenter_point=(topleft_point+topright_point)/2
    #     center_point=(centerleft_point+centerright_point)/2
    #     downcenter_point=(downleft_point+downright_point)/2
    #     poseleft=self.get_targetpose(grasp_points["centerleft"],mode="left_pin")
        
    #     # centerright_point=self.transform_with_combined(centerright_point,np.radians(0),np.radians(0),np.radians(137),8)
    #     # result = self.robot.linear_move(centerright_point,0,True,20)

    #     down_center = downcenter_point + 1.1 * (downcenter_point - center_point)
    #     top_center = topcenter_point + 1.1 * (topcenter_point - center_point)

    #     left_circl = centerleft_point+(centerleft_point - center_point) 
    #     right_circl = centerright_point +(centerright_point - center_point) 
    #     centerleft_point1 += np.array([0,0,0,0,0,-90*np.pi/180])

    #     topleft_point2=self.get_targetpose(grasp_points["topleft"],rpy_xyz=np.array((170*np.pi/180,10*np.pi/180,centerleft_point1[5])))
    #     topright_point2=self.get_targetpose(grasp_points["topright"],rpy_xyz=np.array((170*np.pi/180,10*np.pi/180,centerleft_point1[5])))
    #     centerright_point2=self.get_targetpose(grasp_points["centerright"],rpy_xyz=np.array((170*np.pi/180,10*np.pi/180,centerleft_point1[5])))
    #     centerleft_point2=self.get_targetpose(grasp_points["centerleft"],rpy_xyz=np.array((170*np.pi/180,10*np.pi/180,centerleft_point1[5])))
    #     downleft_point2=self.get_targetpose(grasp_points["downleft"],rpy_xyz=np.array((170*np.pi/180,10*np.pi/180,centerleft_point1[5])))
    #     downright_point2=self.get_targetpose(grasp_points["downright"],rpy_xyz=np.array((170*np.pi/180,10*np.pi/180,centerleft_point1[5])))
    #     top_right = topright_point2 +0.97*(topright_point2 - centerright_point2)
    #     top_left = topleft_point2 + 0.97*(topleft_point2 - centerleft_point2)
    #     down_left = downleft_point2 + 0.97*(downleft_point2 - centerleft_point2)
    #     down_right = downright_point2 + 0.97* (downright_point2 - centerright_point2)
                    
    #                     # 
    #     position1 = np.array(center_point[:2])  # [x1, y1, z1]
    #     position2 = np.array(right_circl[:2])  # [x2, y2, z2]

    #     # 
    #     distance = np.linalg.norm(position1 - position2)
    #     print("distance",distance)
    #     # print(" top_center :", top_center)
    #     # print(" topcenter_point :", topcenter_point)
    #     # print(" center_point :", center_point)
    #     # print(" downcenter_point :", downcenter_point)
    #     # print(" poseleft :", poseleft)
    #     # print(" centerleft_point :", centerleft_point)
    #     # print(" pright :", centerright_point)
    #     # print(" down_center :", down_center)
    #     # print(" top_left :", top_left)
    #     # print(" top_right :", top_right)
    #     # print("left_circle:", left_circl) 
    #     # print("right_circle:", right_circl)
    #     top_leftH=top_left+np.array([0,0,100,0,0,0])
    #     down_centerH=down_center+np.array([0,0,100,0,0,0])
    #     print(" top_center :", top_center)
    #     print(" down_center :", down_center)
    #     print(" down_centerH :", down_centerH)
    #     move_sequence = [
    #         ("moveTo", top_right),
    #         ("moveTo", top_left),
    #         ("moveTo", top_leftH),
    #         ("moveTo", top_center),
    #         ("moveTo", down_center),
    #         ("moveTo", down_centerH),
    #         ("moveTo",down_right),
    #         ("moveTo",down_left),
    #         ("moveTo",posesey),
    #     ]
    #       # 
    #     for action, position in move_sequence:
    #         self.add_action_to_sequence(action, 1, position=position)
        
    #     self.get_logger().info(f", {len(move_sequence)}  moveTo ")
    #     # self.add_action_to_sequence("moveTo", 1, position=centerright_point)
    #     # self.add_action_to_sequence("wait", 1 ,params=5)
    #     # self.add_action_to_sequence("moveTo", 1, position=[517.683, -213.007, 60.0, 3.1416, 0.0, 1.5808])
        
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
        centerleft_point1 = self.get_targetpose(grasp_points["centerleft"], rpy_xyz=RPY_xyz, knife=True, mode="6D")

        #  RPY 
        rpy_adjusted = np.array([170*np.pi/180, 10*np.pi/180, centerleft_point1[5]])

        # 
        key_points = {key: self.get_targetpose(grasp_points[key], rpy_xyz=rpy_adjusted) for key in required_keys}
        key_points["topcenter"] = (key_points["topleft"] + key_points["topright"]) / 2
        key_points["center"] = (key_points["centerleft"] + key_points["centerright"]) / 2
        key_points["downcenter"] = (key_points["downleft"] + key_points["downright"]) / 2

        # 
        poseleft = self.get_targetpose(grasp_points["centerleft"], mode="left_pin")
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
        key_points_shifted = {key: self.get_targetpose(grasp_points[key], rpy_xyz=rpy_adjusted2) for key in required_keys}
        key_points_shifted["top_right"] = key_points_shifted["topright"] + 0.97 * (key_points_shifted["topright"] - key_points_shifted["centerright"])
        key_points_shifted["top_left"] = key_points_shifted["topleft"] + 0.97 * (key_points_shifted["topleft"] - key_points_shifted["centerleft"])
        key_points_shifted["down_left"] = key_points_shifted["downleft"] + 0.97 * (key_points_shifted["downleft"] - key_points_shifted["centerleft"])
        key_points_shifted["down_right"] = key_points_shifted["downright"] + 0.97 * (key_points_shifted["downright"] - key_points_shifted["centerright"])

        # 
        top_leftH = key_points_shifted["top_left"] + np.array([0, 0, 100, 0, 0, 0])
        down_centerH = down_center + np.array([0, 0, 100, 0, 0, 0])

        # 
        posesey = (517.683, -213.007, 60.00, 3.1415926, 0.00, 1.570796)
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

        # 
        for action, position in move_sequence:
            self.add_action_to_sequence(action, 1, position=position)

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
    def checkActionFinished(self):
        if self.current_action is None:
            return
        
        action_func, index, params, decision_logic = self.current_action  # 
        action_name = action_func.__name__  # 
        controller = self.arm_controllers[index]  # 
        # left_action_name = self.current_action.left.action_func.__name__
        # right_action_name = self.current_action.right.action_func.__name__

        # 1️⃣ 
        if action_name in ["jointTo","moveTo","ArcTo"]:
            if controller.arm_feedback.is_running:  # ,
                return  
            elif controller.arm_feedback.success:  # 
                    # ✅ 
                self.get_logger().info(f"{controller.name_prefix}  {action_name} ")
            else:  # 
                self.get_logger().error(f"{controller.name_prefix}  {action_name} ,")
                # self.handle_action_failure(action_func, index, params, decision_logic)
                self.is_executing = False
                  # 

        # 2️⃣ 
        elif action_name in ["closeGripper", "openGripper"]:
            if controller.gripper_feedback.is_running:  # ,
                return
            elif controller.gripper_feedback.success:
                self.get_logger().info(f"{controller.name_prefix}  {action_name} ")
                self.is_executing = False
            else:  # 
                self.get_logger().error(f"{controller.name_prefix}  {action_name} ,")
                # self.handle_action_failure(action_func, index, params, decision_logic)
                self.is_executing = False
                # self.handle_action_failure(action_func, index, params, decision_logic)
                 # 
            
        elif action_name in ["wait"]:
            self.get_logger().info(f"{controller.name_prefix}  {action_name} ")

        if self.pending_decision_logic:
            logic, action_name = self.pending_decision_logic
            if logic:
                self.get_logger().info(f"🧠 {action_name} ,: {logic.__name__}")
                logic()
            self.pending_decision_logic = None  # ,

            # 3️⃣ ,
        self.is_executing = False
        # self.next_action()

    def handle_action_failure(self, action_func, index, params, decision_logic, max_retries=3):
        """: or """
        action_name = action_func.__name__
        controller = self.arm_controllers[index]

        # 
        if not hasattr(self, "retry_count"):
            self.retry_count = {}
        key = f"{index}-{action_name}"
        self.retry_count[key] = self.retry_count.get(key, 0) + 1

        # ✅ ,
        if self.retry_count[key] <= max_retries:
            self.get_logger().warn(f"⚠️ {controller.name_prefix}  {action_name} , ({self.retry_count[key]}/{max_retries})")
            self.add_action_to_sequence(action_name, index, *params, decision_logic=decision_logic)  # 
            return

        # ❌ ,
        self.get_logger().error(f"❌ {controller.name_prefix}  {action_name} ,,")
        self.rollback_to_safe_state(index)
        self.is_executing = False  # 

    def rollback_to_safe_state(self, index):
        """"""
        controller = self.arm_controllers[index]

        self.get_logger().warn(f"↩️ {controller.name_prefix} ,")

        # 1️⃣ **,**
        self.add_action_to_sequence("openGripper", index)

        # 2️⃣ ****
        safe_pose = [280, 0, 100, 3.1416, 0.0, 1.5708]  # 
        self.add_action_to_sequence("moveTo", index, position=safe_pose)

        # 3️⃣ ****
        self.action_queue.clear()
        self.get_logger().warn(f"🚫 ,")

    def rollback_arm(self, index):
        """"""
        if self.action_history:
            last_action = self.action_history.pop()  # 
            action_func, index, params, _ = last_action

            if action_func.__name__ in ["moveTo", "jointTo", "ArcTo"]:
                safe_position = [params[0], params[1], params[2] + 50]  #  50mm 
                self.get_logger().info(f"🔄  {safe_position}")
                self.add_action_to_sequence("moveTo", index, position=safe_position)

    def rollback_gripper(self, index):
        """"""
        if self.action_history:
            last_action = self.action_history.pop()  # 
            action_func, index, params, _ = last_action

            if action_func.__name__ == "closeGripper":
                self.get_logger().info("🔄 : ")
                self.add_action_to_sequence("openGripper", index, force=30)
            elif action_func.__name__ == "openGripper":
                self.get_logger().info("🔄 : ")
                self.add_action_to_sequence("closeGripper", index, force=30)

    def add_action_to_sequence(self, action_name, index, decision_logic=None, **kwargs):
        """
        2️⃣ ,Parameters
        """
        if action_name not in self.action_map:
            self.get_logger().error(f": {action_name}")
            return
        
        action_func = self.action_map[action_name]  # 
        params = list(kwargs.values())  # Parameters
        self.action_queue.append((action_func, index, params, decision_logic))
        self.action_info.append((action_name, index, params, decision_logic))
        self.logger.info(f"Added action: {action_name} {index} {params} to the sequence.:{decision_logic}")


    def pick_object(self):
        return None
    
    # def closeGripper(self):
    #     if not self.arm_controllers[0].feedback_received or not self.arm_controllers[1].feedback_received :
    #         return None  
  
    # def openGripper(self):
    #     if not self.arm_controllers[0].feedback_received or not self.arm_controllers[1].feedback_received :
    #         return None
    #     self.add_action_to_sequence('openGripper', 1,force=50)
    def place_object(self):
        """"""
        if not self.arm_controllers[0].feedback_received or not self.arm_controllers[1].feedback_received :
            return None
        self.initialized = True
        self.get_logger().info("🔪 ###################################################################")
        self.add_action_to_sequence('moveTo', 1, position=[305.23+10,4.00-10,-145.00,3.1416,0.0,1.5808])
        self.add_action_to_sequence('moveTo', 1, position=[305.23-1,4.00+1,-145.00,3.1416,0.0,1.5808])
        self.add_action_to_sequence('openGripper', 1,force=50)

        self.add_action_to_sequence('moveTo', 1, position=[280.516,26.289,41.087,3.1416,0.0,1.5808])

    def grasp_knife(self):
        """"""
        if not self.arm_controllers[0].feedback_received or not self.arm_controllers[1].feedback_received :
            return None
        self.initialized = True
        self.get_logger().info("🔪 ###################################################################")
        # self.add_action_to_sequence('openGripper', 1,force=50)
        self.add_action_to_sequence('moveTo', 1, position=[280.516,26.289,41.087,3.1416,0.0,1.5808],decision_logic=self.calculate_target_pose)
        # self.add_action_to_sequence('moveTo', 1, position=[280.516,26.289,61.087,3.1416,0.0,1.5808])
        # self.add_action_to_sequence('wait', 1,params=1)
        # graspose=self.get_targetpose(self.aruco_grasp_position,self.aruco_grasp_euler,knife=False,arm_index=1)
        # self.add_action_to_sequence('moveTo', 1, position=[517.683, -213.007, 60.0, 3.1416, 0.0, 1.5808])
        # self.add_action_to_sequence('jointTo', 1, joint_angles=[230.416*np.pi/180, 105.559*np.pi/180, 38.741*np.pi/180, 93.193*np.pi/180, -123.021*np.pi/180, -229.462*np.pi/180])
    
    def open_box(self):
        """"""
        self.get_logger().info("🔪 ###################################################################")
        # self.add_action_to_sequence('moveTo', 1, position=[517.683, -213.007, 60.0, 3.1416, 0.0, 1.5808],decision_logic=self.openbox_pose)
        self.add_action_to_sequence('moveTo', 1, position=[517.683, -213.007, 60.0, 3.1416, 0.0, 1.5808])
        self.add_action_to_sequence('wait',1,params=6,decision_logic=self.openbox_pose)
    def start_grasping_sequence(self):
        """"""
        # 
        self.add_action_to_sequence('closeGripper', 0,force=50)
        self.add_action_to_sequence('jointTo', 1, joint_angles=[230.416*np.pi/180, 105.559*np.pi/180, 38.741*np.pi/180, 93.193*np.pi/180, -123.021*np.pi/180, -229.462*np.pi/180])
        self.add_action_to_sequence('moveTo', 1, position=[517.683, -213.007, 60.0, 3.1416, 0.0, 1.5808])
        # self.calculate_target_pose()
        self.add_action_to_sequence('closeGripper', 1,force=50)
        self.get_logger().info(f"Action queue: {self.action_info}")
    
    def onTimerCallback(self):
        if not self.initialized:
            self.add_tasks()
            # self.moveTo(1, [6.34710329e+02 ,-3.19229682e+02 ,-1.28288696e+02 ,-3.13797304e+00,2.18560313e-02 ,1.64808736e+00])
            # self.closeGripper(1,20,0)
            self.start_next_task()

        if self.is_executing:
            self.checkActionFinished()
            return
        if not self.action_queue:
            self.is_executing = False
            self.start_next_task()  # ,
            return
        self.next_action()

    def next_action(self):
        """4️⃣ """
        self.current_action =  self.action_queue.pop(0)
        if self.current_action is None:
            self.is_executing = False
            self.get_logger().info(f" ")
            return

        action_func, index, params, decision_logic = self.current_action

        self.is_executing = True
        self.action_history.append(self.current_action)  # 
        action_func(index, *params)
          # ⚡ ,
        if decision_logic:
           self.pending_decision_logic = (decision_logic, action_func.__name__)
        
        # action, *args = self.action_queue.pop(0)
        # self.is_action_in_progress = True
        # self.get_logger().info(f": {action.__name__}")
        # action(*args)
    def add_tasks(self):
        ""","""
        self.task_queue = ["grasp_knife", "open_box"]
        # self.task_queue =["open_box"]
        # self.task_queue =["open_box"]
        # self.task_queue =["place_object"]
        self.current_task = None  # 
    def start_next_task(self):
        ""","""
        if not self.task_queue:  # 
            return
        self.initialized = True
        if self.current_task is None or not self.action_queue:  #  action_queue 
            self.current_task = self.task_queue.pop(0)  # 
            self.get_logger().info(f": {self.current_task}")
            if self.current_task == "grasp_knife":
                self.grasp_knife()
            elif self.current_task == "open_box":
                self.open_box()
            elif self.current_task == "pick_object":
                self.pick_object()
            elif self.current_task == "place_object":
                self.place_object()
            else:
                self.get_logger().warn(f"⚠️ : {self.current_task}")
    
    def calculate_target_pose(self, feedback_data=None):
        """( `moveTo` )"""
        if not self.aruco_grasp_position:
            self.get_logger().warn("Aruco ,")
            return
        time.sleep(1)
        
        graspose = self.get_targetpose(self.aruco_grasp_position,knife=False, arm_index=1)
        # arr[:3] = [a + b for a, b in zip(arr[:3], [x, y, z])]#
        graspose[:3] += np.array([-20,18,-46])
        graspose = [graspose[0], graspose[1], graspose[2], 3.1416, 0.0, 1.5808]
        self.get_logger().info(f"🎯 : {graspose},")
        # ,
        self.add_action_to_sequence('moveTo', 1, position=graspose)
        self.add_action_to_sequence('closeGripper', 1,force=50)
        self.add_action_to_sequence('moveTo', 1, position=[graspose[0],graspose[1],41.087,3.1416,0.0,1.5808])
        # self.add_action_to_sequence('moveTo', 1, position=[graspose[0]+10,graspose[1]-10,graspose[2],3.1416,0.0,1.5808])
        # self.add_action_to_sequence('moveTo', 1, position=[graspose[0]-1,graspose[1]+1,graspose[2],3.1416,0.0,1.5808])
        # self.add_action_to_sequence('openGripper', 1,force=50)

        # self.add_action_to_sequence('moveTo', 1, position=[280.516,26.289,41.087,3.1416,0.0,1.5808])
        self.get_logger().info("✅ ,")
        # 🔹 ( `open_box`)
        # self.add_task_to_queue("open_box")

        
        # graspose[:2] += np.array([10,10])
        # print(" graspose:", graspose)
        # self.add_action_to_sequence('moveTo', 1, position=graspose)
        # self.add_action_to_sequence('moveTo', 1, position=graspose)


    def update_action_queue(self, decision_logic):
        """"""
        if decision_logic:
            next_actions = decision_logic(self.feedback_data)
            if next_actions:
                self.action_queue = next_actions + self.action_queue  # 
    def add_task_to_queue(self, task_name):
        """🔹 ,"""
        self.task_queue.append(task_name)
        self.get_logger().info(f"📝 : {task_name}")


    def stop_grasping_sequence(self):
        """"""
        self.action_queue.clear()  # 
    # def start_grasping_sequence(self):
    #     """"""
    #     self.action_queue = [
    #         (self.jointTo,1, [230.416*np.pi/180, 105.559*np.pi/180, 38.741*np.pi/180, 
    #                      93.193*np.pi/180, -123.021*np.pi/180, -229.462*np.pi/180]),
    #         (self.closeGripper, 1, 50),
    #         (self.moveTo, 1, [517.683, -213.007, 100.0, 3.1416, 0.0, 1.5808]),
    #         (self.openGripper, 1, 50)
    #     ]
    #     self.process_next_action()

def main(args=None):
    # rclpy
    rclpy.init(args=args)

    try:
        #  GraspTaskManager 
        grasp_task_manager = GraspTaskManager()
        # grasp_task_manager.closeGripper(1,20,50)

        #  SingleThreadedExecutor 
        executor = MultiThreadedExecutor()
        executor.add_node(grasp_task_manager)

        # 
        grasp_task_manager.get_logger().info("GraspTaskManager is now running...")
        executor.spin()

    except KeyboardInterrupt:
        grasp_task_manager.get_logger().info("Shutting down GraspTaskManager...")

    finally:
        # 
        grasp_task_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()