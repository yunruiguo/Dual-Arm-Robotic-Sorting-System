import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import rclpy                                  # ROS2 Python library
from rclpy.executors import MultiThreadedExecutor
from dual_arm_controller_alt import GraspTaskManager
# from dual_arm_controller_open_box_hp import GraspTaskManager
# from dual_arm_controller_yr_copy import GraspTaskManager

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
