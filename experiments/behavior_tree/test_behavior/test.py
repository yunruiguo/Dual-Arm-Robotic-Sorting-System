'''
FilePath: test.py
Author: Yunrui Guo
Date: 2025-02-26 16:49:34
LastEditors: Please set LastEditors
LastEditTime: 2025-02-26 19:33:46

Descripttion: 
'''
import rclpy
import numpy as np
import math
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from dual_arm_controller import GraspTaskManager ,ArmController,ArmFeedbackMsg,GripperFeedbackMsg
def main(args=None):
    rclpy.init(args=args)  # 
    node = GraspTaskManager()
    node.closeGripper(1, force=50)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()