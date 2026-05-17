'''
FilePath: behavior_tree_nodes.py
Author: Yunrui Guo
Date: 2024-12-18 15:56:10
LastEditors: Please set LastEditors
LastEditTime: 2024-12-18 16:16:38

Descripttion: 
'''
class GetPoseNode:
    def __init__(self, pose_manager, arm_controller, arm):
        self.pose_manager = pose_manager
        self.arm_controller = arm_controller
        self.arm = arm

    def execute(self):
        """Get the latest pose and send it to the arm controller."""
        latest_pose = self.pose_manager.get_latest_pose()
        if latest_pose is None:
            return 'FAILURE'
        # Apply calibration transformation if needed
        transformed_pose = self.arm_controller.moveTo(self.arm, latest_pose)
        return 'SUCCESS' if transformed_pose else 'FAILURE'
