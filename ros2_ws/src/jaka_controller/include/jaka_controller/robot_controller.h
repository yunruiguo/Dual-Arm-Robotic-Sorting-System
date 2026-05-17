#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <atomic>
#include <list>
#include <string>
#include <thread>

#include "jaka_controller/jaka_controller.hpp"

typedef struct JointMsg_ {
  JointValue joint_pose;
  bool ctrl;
  JointMsg_() { ctrl = false; }

  // 
  JointMsg_(const JointMsg_& other)
      : joint_pose(other.joint_pose), ctrl(other.ctrl) {}

  // 
  JointMsg_& operator=(const JointMsg_& other) {
    if (this != &other) {  // 
      joint_pose = other.joint_pose;
      ctrl = other.ctrl;
    }
    return *this;
  }
} JointMsg;

typedef enum {
  NonMove = 0x00,
  LineMove = 0x01,
  CircleMove = 0x02,
} MoveType;

typedef struct CartesianPoseMsg_ {
  CartesianPose tcp_pose;
  CartesianPose mid_pose;
  uint8_t move_type;
  CartesianPoseMsg_() { move_type = NonMove; }

  // 
  CartesianPoseMsg_(const CartesianPoseMsg_& other)
      : tcp_pose(other.tcp_pose),
        mid_pose(other.mid_pose),
        move_type(other.move_type) {}

  // 
  CartesianPoseMsg_& operator=(const CartesianPoseMsg_& other) {
    if (this != &other) {  // 
      tcp_pose = other.tcp_pose;
      mid_pose = other.mid_pose;
      move_type = other.move_type;
    }
    return *this;
  }
} CartesianPoseMsg;

typedef struct GripperPoseMsg_ {
  uint16_t gripper_pos;
  uint16_t force;
  bool ctrl;
  GripperPoseMsg_() {
    ctrl = false;
    force = 20;
  }

  // 
  GripperPoseMsg_(const GripperPoseMsg_& other)
      : gripper_pos(other.gripper_pos), force(other.force), ctrl(other.ctrl) {}

  // 
  GripperPoseMsg_& operator=(const GripperPoseMsg_& other) {
    if (this != &other) {  // 
      gripper_pos = other.gripper_pos;
      force = other.force;
      ctrl = other.ctrl;
    }
    return *this;
  }
} GripperPoseMsg;

typedef struct ArmCommandMsg_ {
  ArmCommandMsg_() { reset(); }
  void reset() {
    tcp.move_type = NonMove;
    joint.ctrl = false;
    running.store(false);
    executed.store(false);
    stoped.store(false);
    failed.store(false);
  }
  bool isJointCtrl() const { return joint.ctrl; }
  bool isLineMoveCtrl() const { return tcp.move_type == LineMove; }
  bool isCircleMoveCtrl() const { return tcp.move_type == CircleMove; }
  bool isRunning() const { return running.load(); }
  bool isExecuted() const { return executed.load(); }
  bool isStoped() const { return stoped.load(); }
  bool isFailed() const { return failed.load(); }

  // 
  ArmCommandMsg_(const ArmCommandMsg_& other)
      : running(other.running.load()),
        executed(other.executed.load()),
        stoped(other.stoped.load()),
        joint(other.joint),
        tcp(other.tcp) {}

  // 
  ArmCommandMsg_& operator=(const ArmCommandMsg_& other) {
    if (this != &other) {  // 
      running.store(other.running.load());
      executed.store(other.executed.load());
      stoped.store(other.stoped.load());
      joint = other.joint;
      tcp = other.tcp;
    }
    return *this;
  }

 public:
  std::atomic<bool> running;
  std::atomic<bool> executed;
  std::atomic<bool> stoped;
  std::atomic<bool> failed;
  JointMsg joint;
  CartesianPoseMsg tcp;
} ArmCommandMsg;

typedef struct GripperCommandMsg_ {
  GripperCommandMsg_() { reset(); }
  void reset() {
    gripper.ctrl = false;
    running.store(false);
    executed.store(false);
    stoped.store(false);
    failed.store(false);
  }
  bool isGripperCtrl() const { return gripper.ctrl; }
  bool isRunning() const { return running.load(); }
  bool isExecuted() const { return executed.load(); }
  bool isStoped() const { return stoped.load(); }
  bool isFailed() const { return failed.load(); }

  // 
  GripperCommandMsg_(const GripperCommandMsg_& other)
      : running(other.running.load()),
        executed(other.executed.load()),
        stoped(other.stoped.load()),
        gripper(other.gripper) {}

  // 
  GripperCommandMsg_& operator=(const GripperCommandMsg_& other) {
    if (this != &other) {  // 
      running.store(other.running.load());
      executed.store(other.executed.load());
      stoped.store(other.stoped.load());
      gripper = other.gripper;
    }
    return *this;
  }

 public:
  std::atomic<bool> running;
  std::atomic<bool> executed;
  std::atomic<bool> stoped;
  std::atomic<bool> failed;
  GripperPoseMsg gripper;
} GripperCommandMsg;

typedef struct JakaParamters_ {
  std::string robot_ip;
  std::string name;
  std::string tool_frame_name;
  JointValue init_joint;
  CartesianPose tool_pose;
  int user_frame_id;
  int tool_frame_id;
  uint16_t init_gripper_pos;
  uint16_t init_gripper_force;
  JakaParamters_() {
    robot_ip.clear();
    name.clear();
    tool_frame_name.clear();
    user_frame_id = 0;
    tool_frame_id = 1;
    init_gripper_pos = 1000;
    init_gripper_force = 20;
  }
} JakaParamters;

class RobotController {
 public:
  // ,IP,ID
  RobotController();

  //SetParameters
  void setParamters(const JakaParamters& param);
  // SetIP
  void setRobotIP(const std::string& robot_ip);
  //Set
  void setName(const std::string& name);

  //Setjoint position
  void setInitJoint(const JointValue& value);

  //Set
  void setToolFrameName(const std::string& name);

  //Setgripper position
  void setToolPose(const CartesianPose& pos);

  //
  bool addArmAction(const ArmCommandMsg& msg);

  //
  void stopArmActions();

  bool isArmRunning() const { return current_arm_command.isRunning(); }
  bool isArmFailed() const { return current_arm_command.isFailed(); }

  //

  //
  bool addGripperAction(const GripperCommandMsg& msg);

  //
  void stopGripperActions();

  bool isGripperRunning() const { return current_gripper_command.isRunning(); }
  bool isGripperFailed() const { return current_gripper_command.isFailed(); }

  bool kine_forward(const JointValue* joint_pos,
                    CartesianPose& cartesian_pose) {
    return jaka_controller.kine_forward(joint_pos, cartesian_pose);
  }

  //TCP position
  CartesianPose getCurTCPPose();

  //
  jaka_controller::FeedbackMsg getFeedbackMsg() const {
    return jaka_controller.getFeedbackMsg();
  }

  //
  void initialize();

  // 
  void start();

  // 
  void stop();

  // ,
  ~RobotController();

 private:
  // 
  void runRobot();
  bool hasNextAction();
  void executeArmAction();
  void checkAction();
  void doActionCompleted();

  //
  bool hasNextGripperAction();
  void executeGripperAction();
  void checkGripperAction();
  void doGripperActionCompleted();

  JakaParamters paramters_;
  std::atomic<bool> stop_flag_{true};
  CartesianPose tcp_pose_;
  std::thread thread_;
  jaka_controller::JakaController jaka_controller;
  std::chrono::steady_clock::time_point start_moving_time;
  std::chrono::steady_clock::time_point start_gripper_time;

  std::mutex mutex;
  ArmCommandMsg current_arm_command;
  GripperCommandMsg current_gripper_command;
  std::atomic<bool> jaka_moving{false};
};

#endif  // ROBOT_CONTROLLER_H
