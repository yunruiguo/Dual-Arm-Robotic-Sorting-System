#ifndef AVERAGE_POSE_STAMPED_ACTION_HPP
#define AVERAGE_POSE_STAMPED_ACTION_HPP
#include <Eigen/Dense>
#include <deque>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "bt_action_node.hpp"
#include "grasp_msgs/action/gripper_control.hpp"

using namespace behavior_tree;
using PoseStamped = geometry_msgs::msg::PoseStamped;

class AveragePoseStampedNode
    : public BtActionNode<grasp_msgs::action::GripperControl> {
 public:
  AveragePoseStampedNode(const std::string& xml_tag_name,
                         const std::string& action_name,
                         const BT::NodeConfiguration& conf)
      : BtActionNode<grasp_msgs::action::GripperControl>(xml_tag_name,
                                                         action_name, conf) {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts(
        {BT::InputPort<PoseStamped>(
             "pose_sample",
             "Vector of PoseStamped samples to compute average pose"),
         BT::InputPort<double>("max_distance",
                               "Maximum allowed distance difference between "
                               "consecutive PoseStamped samples"),
         BT::InputPort<double>("max_rotation",
                               "Maximum allowed rotation difference between "
                               "consecutive PoseStamped samples (in radians)"),
         BT::InputPort<int>("num_samples",
                            "Number of samples to use for averaging"),
         BT::InputPort<bool>(
             "run_continuously",
             "Flag indicating whether to run continuously or not"),
         BT::OutputPort<PoseStamped>(
             "avg_pose", "Output the computed average PoseStamped pose")

        });
  }

  bool isDistanceValid(const PoseStamped& pose1, const PoseStamped& pose2,
                       double max_distance) {
    double distance =
        std::sqrt(std::pow(pose1.pose.position.x - pose2.pose.position.x, 2) +
                  std::pow(pose1.pose.position.y - pose2.pose.position.y, 2) +
                  std::pow(pose1.pose.position.z - pose2.pose.position.z, 2));

    if (distance > max_distance) {
      RCLCPP_WARN(rclcpp::get_logger("AveragePoseStamped"),
                  "Pose sample rejected due to distance threshold: %f > %f",
                  distance, max_distance);
      return false;
    }
    return true;
  }

  bool isRotationValid(const PoseStamped& pose1, const PoseStamped& pose2,
                       double max_rotation) {
    Eigen::Quaterniond q1(pose1.pose.orientation.w, pose1.pose.orientation.x,
                          pose1.pose.orientation.y, pose1.pose.orientation.z);

    Eigen::Quaterniond q2(pose2.pose.orientation.w, pose2.pose.orientation.x,
                          pose2.pose.orientation.y, pose2.pose.orientation.z);

    double angle = 2.0 * std::acos(std::abs(q1.dot(q2)));  // 

    if (angle > max_rotation) {
      RCLCPP_WARN(rclcpp::get_logger("AveragePoseStamped"),
                  "Pose sample rejected due to rotation threshold: %f > %f",
                  angle, max_rotation);
      return false;
    }
    return true;
  }

  PoseStamped calculateAveragePose(const std::deque<PoseStamped>& poses) {
    Eigen::Vector3d translation_sum(0, 0, 0);
    Eigen::Quaterniond rotation_sum(0, 0, 0, 0);

    for (const auto& pose : poses) {
      // 
      translation_sum += Eigen::Vector3d(
          pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

      // 
      Eigen::Quaterniond quat(pose.pose.orientation.w, pose.pose.orientation.x,
                              pose.pose.orientation.y, pose.pose.orientation.z);
      rotation_sum.coeffs() += quat.coeffs();
    }

    translation_sum /= poses.size();
    rotation_sum.normalize();

    // OutputPoseStamped
    PoseStamped avg_pose;
    avg_pose.pose.position.x = translation_sum.x();
    avg_pose.pose.position.y = translation_sum.y();
    avg_pose.pose.position.z = translation_sum.z();
    avg_pose.pose.orientation.w = rotation_sum.w();
    avg_pose.pose.orientation.x = rotation_sum.x();
    avg_pose.pose.orientation.y = rotation_sum.y();
    avg_pose.pose.orientation.z = rotation_sum.z();
    avg_pose.header.stamp = poses.back().header.stamp;  // 

    return avg_pose;
  }

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override {
    PoseStamped pose_sample;
    double max_distance, max_rotation;
    int num_samples;
    bool run_continuously;

    // 
    if (!getInput("pose_sample", pose_sample) ||
        !getInput("max_distance", max_distance) ||
        !getInput("max_rotation", max_rotation) ||
        !getInput("num_samples", num_samples) ||
        !getInput("run_continuously", run_continuously)) {
      throw BT::RuntimeError("Missing required input ports");
    }

    // 
    max_samples_ = num_samples;

    // 
    if (!pose_queue_.empty()) {
      const auto& last_pose = pose_queue_.back();

      // 
      if (!isDistanceValid(pose_sample, last_pose, max_distance) ||
          !isRotationValid(pose_sample, last_pose, max_rotation)) {
        // return BT::NodeStatus::RUNNING;
      }
    }

    // ,
    pose_queue_.push_back(pose_sample);
    if (pose_queue_.size() > static_cast<size_t>(max_samples_)) {
      pose_queue_.pop_front();
    }

    // 
    PoseStamped avg_pose = calculateAveragePose(pose_queue_);
    setOutput("avg_pose", avg_pose);

    // 
    if (run_continuously) {
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      // return BT::NodeStatus::RUNNING;
    }

    // return BT::NodeStatus::SUCCESS;
  }

 private:
  std::deque<PoseStamped> pose_queue_;
  int max_samples_;
};

#endif
