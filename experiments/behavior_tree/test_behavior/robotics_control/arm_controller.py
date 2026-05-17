# -----------------------------------------------------------------------------
# 
# -----------------------------------------------------------------------------
import sys
import os
sys.path.append(os.path.abspath(os.path.dirname(__file__)))  # 
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException  # ROSexception
from rclpy.executors import MultiThreadedExecutor     # ()
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup  # ,
from feedback_msgs import ArmFeedbackMsg, GripperFeedbackMsg
from grasp_msgs.action import ArmControl, GripperControl   # ,
from action_msgs.msg import GoalStatus        #  Action 
from rclpy.subscription import Subscription     # ROS2 ()
from grasp_msgs.msg import FeedBackMsg          # 
import logging

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

