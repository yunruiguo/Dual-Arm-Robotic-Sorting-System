#include "jaka_controller/robot_controller.h"

#include <chrono>

#include "jaka_controller/common.h"
#include "jaka_controller/jaka_controller.hpp"

using namespace jaka_controller;

RobotController::RobotController() : stop_flag_(false) {
  paramters_.tool_frame_name = "TCP_TEST";
  current_gripper_command.reset();
  current_arm_command.reset();
}

//SetParameters
void RobotController::setParamters(const JakaParamters& param) {
  paramters_ = param;
  jaka_controller.setName(paramters_.name);
}

// SetIP
void RobotController::setRobotIP(const std::string& robot_ip) {
  paramters_.robot_ip = robot_ip;
}
//Set
void RobotController::setName(const std::string& name) {
  paramters_.name = name;
}

//Setjoint position
void RobotController::setInitJoint(const JointValue& value) {
  paramters_.init_joint = value;
}

void RobotController::setToolFrameName(const std::string& name) {
  paramters_.tool_frame_name = name;
}

//Setgripper position
void RobotController::setToolPose(const CartesianPose& pos) {
  paramters_.tool_pose = pos;
}

//
bool RobotController::addArmAction(const ArmCommandMsg& msg) {
  if (current_arm_command.isRunning()) {
    spdlog::error("[{}]: , ",
                  paramters_.name);
    return false;
  }
  current_arm_command = msg;
  current_arm_command.running.store(true);
  current_arm_command.executed.store(false);
  current_arm_command.stoped.store(false);
  return true;
}

//
void RobotController::stopArmActions() {
  current_arm_command.stoped.store(true);
}
//

//
bool RobotController::addGripperAction(const GripperCommandMsg& msg) {
  if (current_gripper_command.isRunning()) {
    spdlog::error("[{}]: , ",
                  paramters_.name);
    return false;
  }
  current_gripper_command = msg;
  current_arm_command.failed.store(false);
  current_gripper_command.running.store(true);
  current_gripper_command.executed.store(false);
  current_gripper_command.stoped.store(false);
  return true;
}

//
void RobotController::stopGripperActions() {
  current_gripper_command.stoped.store(true);
}

CartesianPose RobotController::getCurTCPPose() {
  return jaka_controller.getTcpPose();
}

void RobotController::initialize() { jaka_controller.initialize(); }

void RobotController::start() {
  stop_flag_.store(false);
  thread_ = std::thread(&RobotController::runRobot, this);
  spdlog::info("start [{}] controller", paramters_.name);
}

void RobotController::stop() {
  if (!stop_flag_) {
    spdlog::info("stop robot controller");
    stop_flag_ = true;
    if (thread_.joinable()) {
      spdlog::info("{} is joinning...", paramters_.name);
      thread_.join();
    }
    spdlog::info("stopped [{}] controller", paramters_.name);
  }
}

RobotController::~RobotController() {
  spdlog::info("~RobotController");
  fflush(stdout);
  stop();
}

bool RobotController::hasNextAction() {
  if (current_arm_command.isRunning() && !current_arm_command.isExecuted()) {
    spdlog::info("Has Arm Action");
    return true;
  }

  return false;
}

void RobotController::executeArmAction() {
  spdlog::info("[{}]: doing [arm] aciton", paramters_.name);
  start_moving_time = std::chrono::steady_clock::now();
  current_arm_command.executed.store(true);
  if (current_arm_command.isJointCtrl()) {
    if (!jaka_controller.jointMove(&current_arm_command.joint.joint_pose)) {
      current_arm_command.failed.store(true);
      doActionCompleted();
    }

  } else if (current_arm_command.isLineMoveCtrl()) {
    if (!jaka_controller.lineMove(current_arm_command.tcp.tcp_pose)) {
      current_arm_command.failed.store(true);
      doActionCompleted();
    }
  } else if (current_arm_command.isCircleMoveCtrl()) {
    if (!jaka_controller.circleMove(current_arm_command.tcp.mid_pose,
                                    current_arm_command.tcp.tcp_pose)) {
      current_arm_command.failed.store(true);
      doActionCompleted();
    }
  } else {
    doActionCompleted();
  }
}

void RobotController::checkAction() {
  auto now = std::chrono::steady_clock::now();
  auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now - start_moving_time)
                          .count();
  if (milliseconds < 2000) {
    return;
  }

  if (!current_arm_command.isRunning()) {
    return;
  }
  if (current_arm_command.stoped) {
    if (jaka_controller.stopMotion()) {
      doActionCompleted();
    }
    current_arm_command.stoped.store(false);
  }

  if (jaka_controller.isMoveCompleted()) {
    doActionCompleted();
  }
}

void RobotController::doActionCompleted() {
  current_arm_command.running.store(false);
  spdlog::info("[{}]: aciton [arm] completed", paramters_.name);
}

bool RobotController::hasNextGripperAction() {
  if (current_gripper_command.isRunning() &&
      !current_gripper_command.isExecuted()) {
    spdlog::info("Has Gripper Action");
    return true;
  }

  return false;
}

void RobotController::executeGripperAction() {
  spdlog::info("[{}]: doing [gripper] aciton", paramters_.name);
  start_gripper_time = std::chrono::steady_clock::now();
  current_gripper_command.executed.store(true);
  if (current_gripper_command.isGripperCtrl()) {
    if (jaka_controller.setGripperForce(
            current_gripper_command.gripper.force)) {
    }
    if (!jaka_controller.setGripperPos(
            current_gripper_command.gripper.gripper_pos)) {
      current_gripper_command.failed.store(true);
      doGripperActionCompleted();
    }
  } else {
    doGripperActionCompleted();
  }
}
void RobotController::checkGripperAction() {
  auto now = std::chrono::steady_clock::now();
  auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now - start_gripper_time)
                          .count();
  if (milliseconds < 1500) {
    return;
  }
  if (!current_gripper_command.isRunning()) {
    return;
  }
  if (jaka_controller.isGripperCompleted()) {
    doGripperActionCompleted();
  }
}
void RobotController::doGripperActionCompleted() {
  current_gripper_command.running.store(false);
  spdlog::info("[{}]: aciton [gripper] completed", paramters_.name);
}
ate_time_interval = 100;
  block_wait_timeout = 2;
  max_filter_size = 15;
  filter_kp = 0.03;
  feedback_msg.reset();
void RobotController::runRobot() {
  jaka_controller.setRobotIP(paramters_.robot_ip);
  jaka_controller.setName(paramters_.name);
  jaka_controller.setUserFrameID(paramters_.user_frame_id);
  jaka_controller.setToolFrameID(paramters_.tool_frame_id);
  if (!jaka_controller.initRobot()) {
    spdlog::error("Failed to init [{}] robot", paramters_.name);
    return;
  }
  jaka_controller.initGripper();
  jaka_controller.setGripperForce(paramters_.init_gripper_force);
  jaka_controller.setGripperPos(paramters_.init_gripper_pos);
  jaka_controller.waitGripperCompleted();
  jaka_controller.jointMove(&paramtersate_time_interval = 100;
  block_wait_timeout = 2;
  max_filter_size = 15;
  filter_kp = 0.03;
  feedback_msg.reset();_.init_joint);
  jaka_controller.setToolFramePos(paramters_.tool_frame_id,
                                  paramters_.tool_pose,
                                  paramters_.tool_frame_name.c_str());
  int query_count = 0;
  while (!stop_flag_) {
    if (hasNextAction()) {
      executeArmAction();
    }
    if (hasNextGripperAction()) {
      executeGripperAction();
    }
    checkAction();
    checkGripperAction();
    if (query_count > 3) {
      query_count = 0;
      jaka_controller.SpinOnce();
    }
    query_count++;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}
ate_time_interval = 100;
  block_wait_timeout = 2;
  max_filter_size = 15;
  filter_kp = 0.03;
  feedback_msg.reset();