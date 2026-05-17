#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "grasp_msgs/action/arm_control.hpp"
#include "grasp_msgs/action/gripper_control.hpp"
#include "grasp_msgs/msg/feed_back_msg.hpp"
#include "grasp_util/paramters_server.hpp"
#include "jaka_controller/common.h"
#include "jaka_controller/robot_controller.h"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace jaka_controller;

void initLog() {
  // Output
  auto consoleSink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  auto consoleLogger = std::make_shared<spdlog::logger>("console", consoleSink);

  // Output
  auto fileSink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
      "jaka_controller.log");
  auto fileLogger = std::make_shared<spdlog::logger>("file", fileSink);

  // 
  spdlog::sinks_init_list sinks = {consoleSink, fileSink};
  auto multiLogger =
      std::make_shared<spdlog::logger>("", sinks.begin(), sinks.end());

  // Set
  spdlog::set_default_logger(multiLogger);

  // Set
  spdlog::set_level(spdlog::level::info);
  spdlog::flush_on(spdlog::level::info);
  spdlog::flush_every(std::chrono::milliseconds(1000));
}

namespace jaka_controller {

class JakaControllerNode : public rclcpp::Node {
 public:
  using ArmCommandAction = grasp_msgs::action::ArmControl;
  using GoalHandleArmCommand =
      rclcpp_action::ServerGoalHandle<ArmCommandAction>;

  using GripperCommandAction = grasp_msgs::action::GripperControl;
  using GoalHandleGripperCommand =
      rclcpp_action::ServerGoalHandle<GripperCommandAction>;

  JakaControllerNode() : rclcpp::Node("jaka_controller_node") {
    loadParamters();
    setupPublishers();
    setupSubscribers();
    setupServices();
    startRobot();
  }

  ~JakaControllerNode() override {}

  void loadParamters() {
    JakaParamters config;
    grasp_util::declare_param(this, "ip", config.robot_ip, config.robot_ip);

    grasp_util::declare_param(this, "name", config.name, config.name);

    grasp_util::declare_param(this, "tool_frame_name", config.tool_frame_name,
                              config.tool_frame_name);

    grasp_util::declare_param(this, "user_frame_id", config.user_frame_id,
                              config.user_frame_id);
    grasp_util::declare_param(this, "tool_frame_id", config.tool_frame_id,
                              config.tool_frame_id);
    grasp_util::declare_param(this, "init_gripper_pos", config.init_gripper_pos,
                              config.init_gripper_pos);
    grasp_util::declare_param(this, "init_gripper_force",
                              config.init_gripper_force,
                              config.init_gripper_force);
    std::vector<double> init_joint{-120 * M_PI / 180, 84 * M_PI / 180,
                                   76 * M_PI / 180,   86 * M_PI / 180,
                                   -130 * M_PI / 180, -222 * M_PI / 180};
    grasp_util::declare_param_vector(this, "init_joint", init_joint,
                                     init_joint);
    for (int i = 0; i < 6; i++) {
      config.init_joint.jVal[i] = init_joint[i];
    }

    std::vector<double> tool_pose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    grasp_util::declare_param_vector(this, "tool_pose", tool_pose, tool_pose);
    config.tool_pose.tran.x = tool_pose[0];
    config.tool_pose.tran.y = tool_pose[1];
    config.tool_pose.tran.z = tool_pose[2];

    config.tool_pose.rpy.rx = tool_pose[3];
    config.tool_pose.rpy.ry = tool_pose[4];
    config.tool_pose.rpy.rz = tool_pose[5];
    jaka_controller.setParamters(config);
  }
  void setupPublishers() {
    robot_feedback_publisher_ =
        this->create_publisher<grasp_msgs::msg::FeedBackMsg>("feedback_states",
                                                             10);
  }
  void setupSubscribers() {}
  void setupServices() {
    this->arm_action_cb_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    this->gripper_action_cb_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    this->arm_control_server_ = rclcpp_action::create_server<ArmCommandAction>(
        this, "arm_control",
        std::bind(&JakaControllerNode::handle_goal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&JakaControllerNode::handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&JakaControllerNode::handle_accepted, this,
                  std::placeholders::_1),
        rcl_action_server_get_default_options(), arm_action_cb_group);

    this->gripper_control_server_ =
        rclcpp_action::create_server<GripperCommandAction>(
            this, "gripper_control",
            std::bind(&JakaControllerNode::handle_gripper_goal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&JakaControllerNode::handle_gripper_cancel, this,
                      std::placeholders::_1),
            std::bind(&JakaControllerNode::handle_gripper_accepted, this,
                      std::placeholders::_1),
            rcl_action_server_get_default_options(), arm_action_cb_group);
  }
  bool startRobot() {
    jaka_controller.start();
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&JakaControllerNode::TimerCallback, this));
    return true;
  }

  void stopRobot() { jaka_controller.stop(); }

  void TimerCallback(void) {
    auto msg = jaka_controller.getFeedbackMsg();
    grasp_msgs::msg::FeedBackMsg feedback;
    feedback.motion_state = msg.state;
    feedback.power_state = msg.powered_on;
    feedback.enable_state = msg.enabled;
    feedback.servo_state = msg.servo_state;
    feedback.collision_state = msg.protective_stop;
    feedback.emergency_state = msg.emergency_stop;
    feedback.error_code = msg.errcode;
    feedback.gripper_state = msg.gripper_state;
    feedback.gripper_pos = msg.gripper_pos;
    feedback.gripper_force = msg.gripper_force;
    for (int i = 0; i < 6; i++) {
      feedback.joint_pose.push_back(msg.joint_pose.jVal[i]);
    }
    feedback.tcp_pose.push_back(msg.tcp_pose.tran.x);
    feedback.tcp_pose.push_back(msg.tcp_pose.tran.y);
    feedback.tcp_pose.push_back(msg.tcp_pose.tran.z);
    feedback.tcp_pose.push_back(msg.tcp_pose.rpy.rx);
    feedback.tcp_pose.push_back(msg.tcp_pose.rpy.ry);
    feedback.tcp_pose.push_back(msg.tcp_pose.rpy.rz);
    robot_feedback_publisher_->publish(feedback);
  }

 private:
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const ArmCommandAction::Goal> goals) {
    for (const auto& goal : goals->poses) {
      if (goal.type == goal.JOINT_TYPE) {
        std::string data = "{";
        for (const auto& joint : goal.joint.joint) {
          data += std::to_string(joint);
          data += ", ";
        }
        data += "}";
        RCLCPP_INFO(this->get_logger(), ": %s", data.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(),
                    ":{%s} {%f, %f, %f} {%f, %f, %f}",
                    goal.tcp.header.frame_id.c_str(), goal.tcp.pos.x,
                    goal.tcp.pos.y, goal.tcp.pos.z, goal.tcp.euler.x,
                    goal.tcp.euler.y, goal.tcp.euler.z);
      }
    }

    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleArmCommand> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void execute_arm(const std::shared_ptr<GoalHandleArmCommand> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "...");
    auto start_time_ = this->get_clock()->now();
    auto goals = goal_handle->get_goal();
    auto result = std::make_shared<grasp_msgs::action::ArmControl::Result>();
    auto feedback =
        std::make_shared<grasp_msgs::action::ArmControl::Feedback>();
    if (goals->poses.size() < 1) {
      RCLCPP_ERROR(this->get_logger(), "");
      result->success = false;
      goal_handle->abort(result);
      return;
    }
    auto feed = jaka_controller.getFeedbackMsg();
    if (!feed.powered_on || !feed.enabled) {
      RCLCPP_ERROR(this->get_logger(), "");
      result->success = false;
      goal_handle->abort(result);
      return;
    }

    CartesianPose target_tcp_pose;
    ArmCommandMsg msg;
    for (size_t i = 0; i < goals->poses.size(); i++) {
      auto goal = goals->poses[i];
      if (goal.type != goal.JOINT_TYPE && goal.type != goal.TCP_TYPE &&
          goal.type != goal.TCP_CIRCLE_TYPE) {
        RCLCPP_ERROR(this->get_logger(), "[%d]", goal.type);
        result->success = false;
        goal_handle->abort(result);
        return;
      }

      if (goal.type == goal.JOINT_TYPE) {
        if (goal.joint.joint.size() != 6) {
          RCLCPP_ERROR(this->get_logger(), "[%lu]",
                       goal.joint.joint.size());
          result->success = false;
          goal_handle->abort(result);
          return;
        }

        msg.joint.ctrl = true;
        for (size_t i = 0; i < goal.joint.joint.size() && i < 6; i++) {
          msg.joint.joint_pose.jVal[i] = goal.joint.joint[i];
        }
        RCLCPP_INFO(this->get_logger(), "");
        if (!jaka_controller.kine_forward(&msg.joint.joint_pose,
                                          target_tcp_pose)) {
          RCLCPP_ERROR(this->get_logger(), "");
        }
      } else if (goal.type == goal.TCP_TYPE) {
        msg.tcp.move_type = LineMove;
        msg.tcp.tcp_pose.tran.x = goal.tcp.pos.x;
        msg.tcp.tcp_pose.tran.y = goal.tcp.pos.y;
        msg.tcp.tcp_pose.tran.z = goal.tcp.pos.z;
        msg.tcp.tcp_pose.rpy.rx = goal.tcp.euler.x;
        msg.tcp.tcp_pose.rpy.ry = goal.tcp.euler.y;
        msg.tcp.tcp_pose.rpy.rz = goal.tcp.euler.z;
        target_tcp_pose = msg.tcp.tcp_pose;
      } else {
        msg.tcp.move_type = CircleMove;
        //
        msg.tcp.mid_pose.tran.x = goal.mid_tcp.pos.x;
        msg.tcp.mid_pose.tran.y = goal.mid_tcp.pos.y;
        msg.tcp.mid_pose.tran.z = goal.mid_tcp.pos.z;
        msg.tcp.mid_pose.rpy.rx = goal.mid_tcp.euler.x;
        msg.tcp.mid_pose.rpy.ry = goal.mid_tcp.euler.y;
        msg.tcp.mid_pose.rpy.rz = goal.mid_tcp.euler.z;pose
        //
        msg.tcp.tcp_pose.tran.x = goal.tcp.pos.x;
        msg.tcp.tcp_pose.tran.y = goal.tcp.pos.y;
        msg.tcp.tcp_pose.tran.z = goal.tcp.pos.z;
        msg.tcp.tcp_pose.rpy.rx = goal.tcp.euler.x;
        msg.tcp.tcp_pose.rpy.ry = goal.tcp.euler.y;
        msg.tcp.tcp_pose.rpy.rz = goal.tcp.euler.z;
        target_tcp_pose = msg.tcp.tcp_pose;
      }
      feedback->target_pose = goal;
      feedback->current_index = i;

      if (!jaka_controller.addArmAction(msg)) {
        RCLCPP_ERROR(this->get_logger(), "");
        result->success = false;
        goal_handle->abort(result);
        return;
      }
      RCLCPP_INFO(this->get_logger(), "...");

      rclcpp::Rate loop_rate(20);  // 20Hz loop rate
      while (rclcpp::ok()) {
        if (goal_handle->is_canceling()) {
          result->success = false;
          goal_handle->canceled(result);
          jaka_controller.stopArmActions();
          return;
        }

        feed = jaka_controller.getFeedbackMsg();

        feedback->current_pose.type = goal.type;
        feedback->current_pose.tcp.pos.x = feed.tcp_pose.tran.x;
        feedback->current_pose.tcp.pos.y = feed.tcp_pose.tran.y;
        feedback->current_pose.tcp.pos.z = feed.tcp_pose.tran.z;

        feedback->current_pose.tcp.euler.x = feed.tcp_pose.rpy.rx;
        feedback->current_pose.tcp.euler.y = feed.tcp_pose.rpy.ry;
        feedback->current_pose.tcp.euler.z = feed.tcp_pose.rpy.rz;
        feedback->current_pose.joint.joint.clear();
        for (const auto& joint : feed.joint_pose.jVal) {
          feedback->current_pose.joint.joint.push_back(joint);
        }
        double dx = feed.tcp_pose.tran.x - target_tcp_pose.tran.x;
        double dy = feed.tcp_pose.tran.y - target_tcp_pose.tran.y;
        double dz = feed.tcp_pose.tran.z - target_tcp_pose.tran.z;
        feedback->distance_remaining = std::hypot(dx, dy, dz);

        feedback->time = this->get_clock()->now() - start_time_;
        goal_handle->publish_feedback(feedback);

        if (!jaka_controller.isArmRunning()) {
          if (jaka_controller.isArmFailed()) {
            result->success = false;
            goal_handle->abort(result);
            RCLCPP_INFO(this->get_logger(), "");
            return;
          }
          break;
        }
        loop_rate.sleep();
      }
    }

    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "");
  }

  void handle_accepted(
      const std::shared_ptr<GoalHandleArmCommand> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "");
    std::thread{std::bind(&JakaControllerNode::execute_arm, this, goal_handle)}
        .detach();
  }

 private:
  rclcpp_action::GoalResponse handle_gripper_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const GripperCommandAction::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), ":{%d, %d, %d}",
                goal->pose.force, goal->pose.pos, goal->pose.state);

    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  rclcpp_action::CancelResponse handle_gripper_cancel(
      const std::shared_ptr<GoalHandleGripperCommand> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void execute_gripper(
      const std::shared_ptr<GoalHandleGripperCommand> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "...");
    auto start_time_ = this->get_clock()->now();
    auto goal = goal_handle->get_goal();
    auto result =
        std::make_shared<grasp_msgs::action::GripperControl::Result>();
    auto feedback =
        std::make_shared<grasp_msgs::action::GripperControl::Feedback>();
    GripperCommandMsg msg;
    msg.gripper.ctrl = true;
    msg.gripper.gripper_pos = goal->pose.pos;
    msg.gripper.force = goal->pose.force;

    if (!jaka_controller.addGripperAction(msg)) {
      RCLCPP_ERROR(this->get_logger(), "");
      result->success = false;
      goal_handle->abort(result);
      return;
    }

    rclcpp::Rate loop_rate(50);  // 20Hz loop rate
    RCLCPP_INFO(this->get_logger(), "...");
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        jaka_controller.stopArmActions();
        return;
      }

      auto feed = jaka_controller.getFeedbackMsg();

      feedback->current_pose.force = feed.gripper_force;
      feedback->current_pose.pos = feed.gripper_pos;
      feedback->current_pose.state = feed.gripper_state;
      feedback->distance_remaining = goal->pose.pos - feed.gripper_pos;
      feedback->time = this->get_clock()->now() - start_time_;
      goal_handle->publish_feedback(feedback);

      if (!jaka_controller.isGripperRunning()) {
        if (jaka_controller.isGripperFailed()) {
          result->success = false;
          goal_handle->abort(result);
          RCLCPP_INFO(this->get_logger(), "");
          return;
        } else {
          result->success = true;
          goal_handle->succeed(result);
        }

        break;
      }

      loop_rate.sleep();
    }
    RCLCPP_INFO(this->get_logger(), "");
  }
  void handle_gripper_accepted(
      const std::shared_ptr<GoalHandleGripperCommand> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "");
    std::thread{
        std::bind(&JakaControllerNode::execute_gripper, this, goal_handle)}
        .detach();
  }

 private:
  RobotController jaka_controller;

  ////////////////////////////////////////////////////////////////////////////
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr arm_action_cb_group;
  rclcpp::CallbackGroup::SharedPtr gripper_action_cb_group;

  rclcpp_action::Server<ArmCommandAction>::SharedPtr arm_control_server_;
  rclcpp_action::Server<GripperCommandAction>::SharedPtr
      gripper_control_server_;

  //
  rclcpp::Publisher<grasp_msgs::msg::FeedBackMsg>::SharedPtr
      robot_feedback_publisher_;
};

}  // namespace jaka_controller

int main(int argc, char* argv[]) {
  common::core::base::init();
  rclcpp::init(argc, argv);
  initLog();
  auto node = std::make_shared<JakaControllerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
