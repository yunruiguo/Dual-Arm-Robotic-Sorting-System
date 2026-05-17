'''
FilePath: pose_manager.py
Author: Yunrui Guo
Date: 2024-12-18 15:54:20
LastEditors: Please set LastEditors
LastEditTime: 2024-12-18 17:11:12

Descripttion: 
'''
import rclpy
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
import numpy as np
from rclpy.node import Node

class PoseManager(Node):
    def __init__(self, name='pose_manager'):
        super().__init__(name)
        try:
            self.cam_pose = np.genfromtxt('real/right_camera_pose.txt', delimiter=' ')
            self.top_rightcamera_pose = np.genfromtxt('real/top_rightcamera_pose.txt', delimiter=' ')
            self.top_leftcamera_pose = np.genfromtxt('real/top_leftcamera_pose.txt', delimiter=' ')
        except Exception as e:
            print("Error reading file:", e)
        self.center_point=[0.0,0.0,0.0]
        self.middle_left =[0.0,0.0,0.0]
        self.middle_right=[0.0,0.0,0.0] 
        self.pose_subscriber_left = self.create_subscription(
            Float64MultiArray,
            'right/grasppose',
            self.pose_callback_left,
            10
        )
        self.pose_subscriber_right = self.create_subscription(
            Pose,
            'robot/pose/right_arm',
            self.pose_callback_right,
            10
        )
        self.subscription = self.create_subscription(Pose,
            '/right/aruco_poses',
            self.pose_callback_aruco,
            10  # QoSParameters,Set
        )

    def pose_callback_left(self, msg):
        """Store the latest left arm pose."""
        self.latest_pose = self.convert_pose_to_numpy(msg)

    def pose_callback_right(self, msg):
        """Store the latest right arm pose."""
        self.latest_pose = self.convert_pose_to_numpy(msg)
    def pose_callback_aruco(self, msg):
        """Store the latest right arm pose."""
        self.latest_pose = self.convert_pose_to_numpy(msg)
    def get_latest_pose(self):
        """Get the latest stored pose."""
        return self.latest_pose

    def convert_pose_to_numpy(self, msg: Pose):
        """Convert the Pose message to a numpy array."""
        return np.array([msg.position.x, msg.position.y, msg.position.z,
                         msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
