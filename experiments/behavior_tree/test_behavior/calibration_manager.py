'''
FilePath: calibration_manager.py
Author: Yunrui Guo
Date: 2024-12-18 15:55:50
LastEditors: Please set LastEditors
LastEditTime: 2024-12-18 16:15:40

Descripttion: 
'''
import numpy as np

class CalibrationManager:
    def __init__(self):
        self.calibration_matrix = None

    def load_calibration_matrix(self, filepath):
        """Load calibration matrix from a file (e.g., CSV or JSON)."""
        self.calibration_matrix = np.genfromtxt(filepath, delimiter=',')

    def apply_transformation(self, pose):
        """Apply the calibration transformation to the pose."""
        if self.calibration_matrix is None:
            raise ValueError("Calibration matrix is not loaded")
        # Apply transformation to the pose
        transformed_pose = np.dot(self.calibration_matrix, pose)
        return transformed_pose
