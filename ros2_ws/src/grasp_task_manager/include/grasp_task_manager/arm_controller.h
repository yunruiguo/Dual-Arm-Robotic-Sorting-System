#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <geometry_msgs/msg/pose_array.hpp>
#include <grasp_msgs/msg/feed_back_msg.hpp>
#include <grasp_util/safe_data_class.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "data_filter.h"
#include "grasp_msgs/action/arm_control.hpp"
#include "grasp_msgs/action/gripper_control.hpp"
#include "grasp_task_manager/grasp_common.h"
#include "grasp_util/geometry_utils.hpp"

class ArmController {
 public:
  using GripperCommand = grasp_msgs::action::GripperControl;
  using GoalHandleGripperCommand =
      rclcpp_action::ClientGoalHandle<GripperCommand>;

  using ExecuteGrasp = grasp_msgs::action::ArmControl;
  using GoalHandleExecuteGrasp = rclcpp_action::ClientGoalHandle<ExecuteGrasp>;

  ArmController(rclcpp::Node::SharedPtr node,
                const std::string &arm_action_name,
                const std::string &gripper_action_name,
                const std::string &name_prefix)
      : node_(node),
        arm_execute_state_(-2),
        gripper_execute_state_(-2),
        aruco_pose_filter(6),
        cutbox_pose_filter(5),
        grasp_pose_filter(5) {
    camera_intrinsic.valid = false;
    // 
    arm_cb_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    gripper_cb_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    grasp_cb_group = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    feedback_cb_group = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    camera_info_cb_group = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    aruco_cb_group = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    RCLCPP_INFO(node_->get_logger(), "[[%s]]", name_prefix.c_str());
    // 
    arm_action_client_ = rclcpp_action::create_client<ExecuteGrasp>(
        node_, arm_action_name, arm_cb_group_);
    gripper_action_client_ = rclcpp_action::create_client<GripperCommand>(
        node_, gripper_action_name, gripper_cb_group_);

    auto grasp_sub_opt = rclcpp::SubscriptionOptions();
    grasp_sub_opt.callback_group = this->grasp_cb_group;
    grasp_pose_sub_ =
        node_->create_subscription<std_msgs::msg::Float64MultiArray>(
            name_prefix + "/grasppose", 1,
            std::bind(&ArmController::grasp_pose_callback, this,
                      std::placeholders::_1),
            grasp_sub_opt);

    auto aruco_sub_opt = rclcpp::SubscriptionOptions();
    aruco_sub_opt.callback_group = this->aruco_cb_group;
    aruco_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
        name_prefix + "/aruco_poses", 1,
        std::bind(&ArmController::aruco_pose_callback, this,
                  std::placeholders::_1),
        aruco_sub_opt);

    auto feedback_sub_opt = rclcpp::SubscriptionOptions();
    feedback_sub_opt.callback_group = this->feedback_cb_group;
    arm_feedback_sub_ =
        node_->create_subscription<grasp_msgs::msg::FeedBackMsg>(
            name_prefix + "/feedback_states", 1,
            std::bind(&ArmController::arm_feedback_callback, this,
                      std::placeholders::_1),
            feedback_sub_opt);

    auto camera_sub_opt = rclcpp::SubscriptionOptions();
    camera_sub_opt.callback_group = this->camera_info_cb_group;
    camera_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
        name_prefix + "_camera/color/camera_info", 1,
        std::bind(&ArmController::camera_info_callback, this,
                  std::placeholders::_1),
        camera_sub_opt);

    name_prefix_ = name_prefix;
    current_grasp_pose.get();
    current_grasp_knife_pose.get();
  }

  void setCameraIntrinsic(const camera_intrinsic_t &intrinsic) {
    camera_intrinsic = intrinsic;
  }

  bool executeArm(const std::vector<grasp_msgs::msg::ArmCommand> &poses) {
    arm_feedback.success = false;
    if (arm_execute_state_.load() == -1) {
      RCLCPP_WARN(node_->get_logger(), "[%s]...",
                  name_prefix_.c_str());
      return false;
    }

    arm_feedback.isRunning = false;
    arm_execute_state_.store(-2);
    auto arm_goal_msg = ExecuteGrasp::Goal();
    arm_goal_msg.poses.assign(poses.begin(), poses.end());

    if (!arm_action_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Action server[Arm] not available after waiting");
      return false;
    }
    arm_feedback.isRunning = true;
    arm_execute_state_.store(-1);

    auto send_goal_options =
        rclcpp_action::Client<ExecuteGrasp>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&ArmController::armGoalCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&ArmController::armFeedbackCallback, this,
                  std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(
        &ArmController::armResultCallback, this, std::placeholders::_1);

    RCLCPP_INFO(node_->get_logger(), "[%s]",
                name_prefix_.c_str());
    arm_action_client_->async_send_goal(arm_goal_msg, send_goal_options);
    return true;
  }

  bool executeGripper(const grasp_msgs::msg::GripperCommand &msg) {
    gripper_feedback.success = false;
    if (gripper_execute_state_.load() == -1) {
      RCLCPP_WARN(node_->get_logger(), "[%s]...",
                  name_prefix_.c_str());
      return false;
    }

    gripper_feedback.isRunning = false;
    gripper_execute_state_.store(-2);
    auto gripper_goal_msg = GripperCommand::Goal();
    gripper_goal_msg.pose = msg;

    if (!gripper_action_client_->wait_for_action_server(
            std::chrono::seconds(10))) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Action server[Gripper] not available after waiting");
      return false;
    }
    gripper_feedback.isRunning = true;
    gripper_execute_state_.store(-1);

    auto send_goal_options =
        rclcpp_action::Client<GripperCommand>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(
        &ArmController::gripperGoalCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&ArmController::gripperFeedbackCallback, this,
                  std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(
        &ArmController::gripperResultCallback, this, std::placeholders::_1);

    RCLCPP_INFO(node_->get_logger(), "[%s]",
                name_prefix_.c_str());
    gripper_action_client_->async_send_goal(gripper_goal_msg,
                                            send_goal_options);
    return true;
  }

  void waitForActionServers() {
    RCLCPP_INFO(node_->get_logger(), "[%s]",
                name_prefix_.c_str());
    while (!gripper_action_client_->wait_for_action_server(
               std::chrono::seconds(1)) &&
           rclcpp::ok()) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                           "%s...",
                           name_prefix_.c_str());
    }
    while (
        !arm_action_client_->wait_for_action_server(std::chrono::seconds(1)) &&
        rclcpp::ok()) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                           "%s...", name_prefix_.c_str());
    }
  }

  bool isInProgress() const { return (execute_state_.load() == -1); }

  bool isSuccessed() const { return (execute_state_.load() == 1); }

  bool isFailed() const { return (execute_state_.load() == 0); }

  ArmFeedbackMsg getArmFeedback() const { return arm_feedback; }

  GripperFeedbackMsg getGripperFeedback() const { return gripper_feedback; }

  bool isHasGraspPose() { return current_grasp_pose.check(); }

  grasp_pose_t getCurrentGraspPose() { return current_grasp_pose.get(); }

  bool isHasGraspKnifePose() { return current_grasp_knife_pose.check(); }

  grasp_pose_t getCurrentGraspKnifePose() {
    return current_grasp_knife_pose.get();
  }

  void resetDataFilter() {
    aruco_pose_filter.clear();
    cutbox_pose_filter.clear();
    grasp_pose_filter.clear();
    getCurrentGraspKnifePose();
    getCurrentGraspPose();
  }

 private:
  void armGoalCallback(const GoalHandleExecuteGrasp::SharedPtr &goal_handle) {
    if (!goal_handle) {
      arm_execute_state_.store(0);
      execute_state_.store(0);

      arm_feedback.isRunning = false;
      arm_feedback.success = false;
      RCLCPP_ERROR(node_->get_logger(), "[%s]: ",
                   name_prefix_.c_str());
    } else {
      arm_execute_state_.store(-1);
      execute_state_.store(-1);
      RCLCPP_INFO(node_->get_logger(),
                  "[%s]: , ",
                  name_prefix_.c_str());
    }
  }

  void armFeedbackCallback(
      const GoalHandleExecuteGrasp::SharedPtr,
      const std::shared_ptr<const ExecuteGrasp::Feedback> feedback) {
    // RCLCPP_INFO(node_->get_logger(), "[%s]: : %f",
    //             name_prefix_.c_str(), feedback->distance_remaining);
    arm_feedback.distance_remaining = feedback->distance_remaining;
    arm_feedback.pose = feedback->current_pose;
  }

  void armResultCallback(const GoalHandleExecuteGrasp::WrappedResult &result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(node_->get_logger(), "[%s]", name_prefix_.c_str());
      arm_execute_state_.store(1);
      if (gripper_execute_state_.load() != -1) {
        execute_state_.store(1);
      }
      arm_feedback.success = true;
    } else {
      RCLCPP_WARN(node_->get_logger(), "[%s]",
                  name_prefix_.c_str());
      arm_execute_state_.store(0);
      if (gripper_execute_state_.load() != -1) {
        execute_state_.store(0);
      }
      arm_feedback.success = false;
    }
    arm_feedback.isRunning = false;
  }

  void gripperGoalCallback(
      const GoalHandleGripperCommand::SharedPtr &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "[%s]: ",
                   name_prefix_.c_str());
      gripper_execute_state_.store(0);
      execute_state_.store(0);
      gripper_feedback.isRunning = false;
      gripper_feedback.success = false;
    } else {
      RCLCPP_INFO(node_->get_logger(), "[%s]: , ",
                  name_prefix_.c_str());
      gripper_execute_state_.store(-1);
      execute_state_.store(-1);
    }
  }

  void gripperFeedbackCallback(
      const GoalHandleGripperCommand::SharedPtr,
      const std::shared_ptr<const GripperCommand::Feedback> feedback) {
    // RCLCPP_INFO(node_->get_logger(),
    //             "[%s]: current position %d, : %d, : %d",
    //             name_prefix_.c_str(), feedback->current_pose.pos,
    //             feedback->current_pose.force, feedback->current_pose.state);
    gripper_feedback.pose = feedback->current_pose;
    gripper_feedback.distance_remaining = feedback->distance_remaining;
  }

  void gripperResultCallback(
      const GoalHandleGripperCommand::WrappedResult &result) {
    gripper_execute_state_.store(0);
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(node_->get_logger(), "[%s]: ",
                  name_prefix_.c_str());
      gripper_execute_state_.store(1);
      if (arm_execute_state_.load() != -1) {
        execute_state_.store(1);
      }
      gripper_feedback.success = true;
    } else {
      RCLCPP_ERROR(node_->get_logger(), "[%s]: ",
                   name_prefix_.c_str());
      gripper_feedback.success = false;

      if (arm_execute_state_.load() != -1) {
        execute_state_.store(0);
      }
    }
    gripper_feedback.isRunning = false;
  }

  void grasp_pose_callback(const std_msgs::msg::Float64MultiArray &msg) {
    if (msg.data.size() == 9) {
      grasp_pose_t pose;
      pose.grasp_type = GRASP_CUT_BOX;
      grasp_time[GRASP_CUT_BOX] = node_->get_clock()->now();
      pose.roll = M_PI;
      pose.pitch = 0.0;
      pose.yaw = M_PI / 2.0;
      pose.center_pos = tf2::Vector3(msg.data[0], msg.data[1], msg.data[2]);
      pose.left_pos = tf2::Vector3(msg.data[3], msg.data[4], msg.data[5]);
      pose.right_pos = tf2::Vector3(msg.data[6], msg.data[7], msg.data[8]);
      current_grasp_knife_pose.set(pose);

    } else if (msg.data.size() == 15) {
      grasp_pose_t pose;
      pose.grasp_type = GRASP_CUT_BOX;
      grasp_time[GRASP_CUT_BOX] = node_->get_clock()->now();
      pose.roll = M_PI;
      pose.pitch = 0.0;
      pose.yaw = M_PI / 2.0;
      pose.center_pos = tf2::Vector3(msg.data[0], msg.data[1], msg.data[2]);
      pose.left_pos = tf2::Vector3(msg.data[3], msg.data[4], msg.data[5]);
      pose.right_pos = tf2::Vector3(msg.data[6], msg.data[7], msg.data[8]);
      pose.left_press_pos =
          tf2::Vector3(msg.data[9], msg.data[10], msg.data[11]);
      pose.right_press_pos =
          tf2::Vector3(msg.data[12], msg.data[13], msg.data[14]);
      cutbox_pose_filter.applyMeanFilter(pose);

      grasp_pose_t mean_pose;
      size_t queue_size = 0;
      double std = cutbox_pose_filter.getPositionStdDev(mean_pose, queue_size);
      if (std < 1.5 && queue_size > 2) {
        std::cout << "CutBox stability[" << queue_size << "]: " << std
                  << std::endl;
        current_grasp_knife_pose.set(pose);
      } else {
        std::cout << "Waiting for detection of Box stability: " << std
                  << std::endl;
      }

    } else if (msg.data.size() == 6) {
      if (!camera_intrinsic.valid) {
        RCLCPP_WARN(node_->get_logger(), "[%s]: ",
                    name_prefix_.c_str());
        return;
      }

      double u = msg.data[1];
      double v = msg.data[0];
      double depth = msg.data[2] * 1000;
      double angle = msg.data[3];
      double width = msg.data[4];
      grasp_time[GRASP_BOX] = node_->get_clock()->now();
      grasp_pose_t pose;
      pose.grasp_type = GRASP_BOX;
      pose.center_pos.setX((u - camera_intrinsic.cx) * depth /
                           camera_intrinsic.fx);
      pose.center_pos.setY((v - camera_intrinsic.cy) * depth /
                           camera_intrinsic.fy);
      pose.center_pos.setZ(depth);

      double x1 =
          (u + width - camera_intrinsic.cx) * depth / camera_intrinsic.fx;
      double y1 = (v - camera_intrinsic.cy) * depth / camera_intrinsic.fy;
      double dx = pose.center_pos.x() - x1;
      double dy = pose.center_pos.y() - y1;
      pose.width = std::hypot(dx, dy);
      pose.roll = M_PI;
      pose.pitch = 0.0;
      pose.yaw = angle;
      grasp_pose_filter.applyMeanFilter(pose);
      grasp_pose_t mean_pose;
      size_t queue_size = 0;
      double std = grasp_pose_filter.getPositionStdDev(mean_pose, queue_size);
      if (/*std < 1.5 &&*/ queue_size > 2) {
        std::cout << "Grasp Box stability[" << queue_size << "]: " << std
                  << std::endl;
        current_grasp_pose.set(pose);

      } else {
        std::cout << "Waiting for detection of Grasp Box stability: " << std
                  << std::endl;
      }

      // RCLCPP_INFO(node_->get_logger(), "[%s]:{%f, %f, %f, %f, %f} ",
      //             name_prefix_.c_str(), pose.center_pos.x(),
      //             pose.center_pos.y(), pose.center_pos.z(), pose.width,
      //             pose.angle);
    }
  }

  void aruco_pose_callback(const geometry_msgs::msg::PoseArray &msg) {
    if (msg.poses.size() < 1) {
      return;
    }
    grasp_time[GRASP_KNIFE] = node_->get_clock()->now();
    grasp_pose_t pose;
    pose.grasp_type = GRASP_KNIFE;
    auto aruco_pose = msg.poses.back();
    pose.center_pos.setX(aruco_pose.position.x * 1000);
    pose.center_pos.setY(aruco_pose.position.y * 1000);
    pose.center_pos.setZ(aruco_pose.position.z * 1000);
    pose.roll = arm_feedback.pose.tcp.euler.x;
    pose.pitch = arm_feedback.pose.tcp.euler.y;
    pose.yaw = arm_feedback.pose.tcp.euler.z;
    aruco_pose_filter.applyMeanFilter(pose);
    grasp_pose_t mean_pose;
    size_t queue_size = 0;

    double std = aruco_pose_filter.getPositionStdDev(mean_pose, queue_size);
    if (std < 0.5 && queue_size > 3) {
      std::cout << "Aruco code stability[" << queue_size << "]: " << std
                << std::endl;
      current_grasp_knife_pose.set(mean_pose);
    } else {
      std::cout << "Waiting for detection of Aruco code stability: " << std
                << std::endl;
    }
  }

  void arm_feedback_callback(const grasp_msgs::msg::FeedBackMsg &msg) {
    if (!arm_feedback.isRunning) {
      arm_feedback.pose.tcp.pos.x = msg.tcp_pose[0];
      arm_feedback.pose.tcp.pos.y = msg.tcp_pose[1];
      arm_feedback.pose.tcp.pos.z = msg.tcp_pose[2];
      arm_feedback.pose.tcp.euler.x = msg.tcp_pose[3];
      arm_feedback.pose.tcp.euler.y = msg.tcp_pose[4];
      arm_feedback.pose.tcp.euler.z = msg.tcp_pose[5];
      arm_feedback.pose.joint.joint.assign(msg.joint_pose.begin(),
                                           msg.joint_pose.end());
    }
  }

  void camera_info_callback(const sensor_msgs::msg::CameraInfo &msg) {
    if (camera_intrinsic.valid) {
      return;
    }
    // # Intrinsic camera matrix for the raw (distorted) images.
    // #     [fx  0 cx]
    // # K = [ 0 fy cy]
    // #     [ 0  0  1]
    // # Projects 3D points in the camera coordinate frame to 2D pixel
    // # coordinates using the focal lengths (fx, fy) and principal point
    // # (cx, cy).
    // 	  float64[9]  k # 3x3 row-major matrix

    camera_info = msg;
    camera_intrinsic.fx = msg.k[0];
    camera_intrinsic.cx = msg.k[2];
    camera_intrinsic.fy = msg.k[4];
    camera_intrinsic.cy = msg.k[5];
    camera_intrinsic.valid = true;
  }

  rclcpp::Node::SharedPtr node_;
  std::atomic<int> arm_execute_state_;
  std::atomic<int> gripper_execute_state_;
  std::atomic<int> execute_state_;
  std::string name_prefix_;
  rclcpp::CallbackGroup::SharedPtr arm_cb_group_;
  rclcpp::CallbackGroup::SharedPtr gripper_cb_group_;
  rclcpp::CallbackGroup::SharedPtr grasp_cb_group;
  rclcpp::CallbackGroup::SharedPtr feedback_cb_group;
  rclcpp::CallbackGroup::SharedPtr camera_info_cb_group;
  rclcpp::CallbackGroup::SharedPtr aruco_cb_group;

  rclcpp_action::Client<ExecuteGrasp>::SharedPtr arm_action_client_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_action_client_;
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float64MultiArray>>
      grasp_pose_sub_;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseArray>>
      aruco_pose_sub_;

  std::shared_ptr<rclcpp::Subscription<grasp_msgs::msg::FeedBackMsg>>
      arm_feedback_sub_;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>>
      camera_info_sub_;
  ArmFeedbackMsg arm_feedback;
  GripperFeedbackMsg gripper_feedback;
  sensor_msgs::msg::CameraInfo camera_info;
  camera_intrinsic_t camera_intrinsic;
  grasp_util::safe_class<grasp_pose_t> current_grasp_pose;
  grasp_util::safe_class<grasp_pose_t> current_grasp_knife_pose;
  rclcpp::Time grasp_time[4];
  DataFilter aruco_pose_filter;
  DataFilter cutbox_pose_filter;
  DataFilter grasp_pose_filter;
};

#endif
