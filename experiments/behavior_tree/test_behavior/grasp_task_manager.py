import rclpy
from rclpy.node import Node
from test_behavior.pose_manager import PoseManager
from test_behavior.calibration_manager import CalibrationManager
from test_behavior.behavior_tree import create_behavior_tree

class GraspTaskManager(Node):
    def __init__(self):
        super().__init__('grasp_task_manager_node')

        # Initialize PoseManager and CalibrationManager
        self.pose_manager = PoseManager()
        self.calibration_manager = CalibrationManager()

        # Create behavior tree
        self.behavior_tree = create_behavior_tree(self.pose_manager, self.calibration_manager)

    def spin(self):
        """Run the behavior tree."""
        while rclpy.ok():
            self.behavior_tree.tick()
            rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    node = GraspTaskManager()
    try:
        node.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down GraspTaskManager.")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
