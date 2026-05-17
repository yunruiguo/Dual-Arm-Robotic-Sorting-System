#include <rclcpp/rclcpp.hpp>
#include "grasp_task_manager/grasp_task_manager.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GraspTaskManager>();
  node->init();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
