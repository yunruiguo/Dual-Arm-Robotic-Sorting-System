'''
FilePath: behavior_tree.py
Author: Yunrui Guo
Date: 2024-12-18 15:56:24
LastEditors: Please set LastEditors
LastEditTime: 2024-12-19 11:30:17

Descripttion: 
'''
import py_trees
from test_behavior.behavior_tree_nodes import GetPoseNode, MoveToNode, GripperControlNode
from test_behavior.pose_manager import PoseManager
from test_behavior.calibration_manager import CalibrationManager
from test_behavior.arm_controller import ArmController


def create_behavior_tree(pose_manager: PoseManager, calibration_manager: CalibrationManager, arm_controllers: list):
    """
    Create and return a behavior tree.
    Args:
        pose_manager (PoseManager): The PoseManager instance.
        calibration_manager (CalibrationManager): The CalibrationManager instance.
        arm_controllers (list): A list of ArmController instances for left and right arms.
    Returns:
        py_trees.composites.Sequence: The root of the behavior tree.
    """
    
    # Define the behavior tree nodes
    # 1. GetPoseNode: Retrieve the latest pose for the arm
    get_pose_left_node = GetPoseNode(pose_manager, arm_controllers[0], "left")
    get_pose_right_node = GetPoseNode(pose_manager, arm_controllers[1], "right")
    
    # 2. MoveToNode: Move to the retrieved pose
    move_to_left_node = MoveToNode(arm_controllers[0], "left")
    move_to_right_node = MoveToNode(arm_controllers[1], "right")
    
    # 3. GripperControlNode: Open or close the gripper
    open_gripper_left_node = GripperControlNode(arm_controllers[0], "left", "open")
    open_gripper_right_node = GripperControlNode(arm_controllers[1], "right", "open")
    close_gripper_left_node = GripperControlNode(arm_controllers[0], "left", "close")
    close_gripper_right_node = GripperControlNode(arm_controllers[1], "right", "close")

    # Define the sequence of actions
    sequence = py_trees.composites.Sequence(name="GraspSequence")

    # Add the nodes to the sequence
    sequence.add_children([
        get_pose_left_node,      # Get the left arm pose
        move_to_left_node,      # Move the left arm to the pose
        close_gripper_left_node, # Close the left gripper
        get_pose_right_node,    # Get the right arm pose
        move_to_right_node,     # Move the right arm to the pose
        close_gripper_right_node, # Close the right gripper
    ])

    # You could also add branching behavior here if necessary:
    # If the pose for the arm is invalid, you could return FAILURE instead of continuing.
    return sequence


def main():
    # Create PoseManager and CalibrationManager
    pose_manager = PoseManager()
    calibration_manager = CalibrationManager()

    # Create ArmController instances for both arms
    arm_controllers = [
        ArmController("left", pose_manager),
        ArmController("right", pose_manager)
    ]

    # Create the behavior tree
    behavior_tree = create_behavior_tree(pose_manager, calibration_manager, arm_controllers)

    # Execute the behavior tree
    behavior_tree.tick()


if __name__ == "__main__":
    main()