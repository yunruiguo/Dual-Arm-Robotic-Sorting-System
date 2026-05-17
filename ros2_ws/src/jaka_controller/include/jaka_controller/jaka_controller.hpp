
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/
#ifndef JAKA_TEST_HPP_
#define JAKA_TEST_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#include <chrono>
#include <map>
#include <string>
#include <thread>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "Eigen/StdVector"
#include "jaka_controller/JAKAZuRobot.h"
#include "jaka_controller/bsp_modbus.h"
#include "jaka_controller/conversion.h"
#include "jaka_controller/jkerr.h"
#include "jaka_controller/jktypes.h"
#include "spdlog/spdlog.h"

#define GRIPPER_RS485_CHANNEL 1

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/
namespace jaka_controller {

typedef enum {
  ModBusRTU = 0x00,
  RawRS485 = 0x01,
  TorqueSensor = 0x02,
} RS485TranformerMode;

/*Define gripper message structure*/
typedef struct __GripperInfo_t {
  int initState;     // 0-not initialized,1-initialized successfully,-1-not available
  int gripperState;  //gripper state,0-moving,1-reached target position,2-object grasped,3-object dropped,-1-not available
  int curPos;        //current real-time position,per mille,-1-not available
  int setForce;      //current configured force,percent,-1-not available
} GripperInfo_t;

typedef struct FeedbackMsg_ {
  uint8_t state;           ///arm state
  uint8_t gripper_state;   ///gripper state
  uint16_t gripper_pos;    ///gripper position
  uint16_t gripper_force;  ///gripper force
  JointValue joint_pose;   ///joint position
  CartesianPose tcp_pose;  ///TCP position
  int errcode;  ///< error code when the robot reports a runtime error,0means normal operation,other values indicate abnormal operation
  int powered_on;  ///< robot power status flag,0means not powered on,1means powered on
  int enabled;  ///< robot enable status flag,0means not enabled,1means enabled
  int protective_stop;  ///< whether the robot detected a collision,0means no collision detected,1means collision detected
  int emergency_stop;  ///< whether the robot is in emergency stop,0means no emergency stop,1means emergency stop
  uint8_t servo_state;
  FeedbackMsg_() { reset(); }

  void reset() {
    state = 0;
    gripper_state = 1;
    errcode = 0;
    powered_on = 0;
    enabled = 0;
    protective_stop = 0;
    emergency_stop = 0;
    gripper_pos = 0;
    gripper_force = 0;
    servo_state = 0;
  }
} FeedbackMsg;

class JakaController {
 public:
  JakaController();
  ~JakaController();
  void initialize();
  void loadParamters();
  void setupPublishers();
  void setupSubscribers();
  void setupServices();

  bool initRobot();
  bool initGripper();
  void stopRobot();
  void SpinOnce(void);
  bool checkRobotStatus();
  void servoMoveEnable(bool enable);
  void setToolFramePos(int id, const CartesianPose& pos,
                       const std::string& tool_frame_name);
  bool jointMove(JointValue* joint_pose);
  bool lineMove(const CartesianPose& pos);
  bool circleMove(const CartesianPose& mid_pos, const CartesianPose& end_pos);
  bool ServoMove(const CartesianPose& pos);
  bool ServoJointMove(const JointValue& joint);
  bool stopMotion();
  bool isMoveCompleted();
  bool WaitMoveCompleted(int max_timeout = 15000);
  CartesianPose getTcpPose();

  void setRobotIP(const std::string& ip);
  void setName(const std::string& name) { name_ = name; }
  void setToolFrameID(int id) { m_tool_frame_id = id; }
  void setUserFrameID(int id) { m_user_frame_id = id; }

  static void printCartesianPose(const CartesianPose& tcp_pose) {
    spdlog::info("[{0}, {1}, {2}, {3}, {4}, {5}]", tcp_pose.tran.x,
                 tcp_pose.tran.y, tcp_pose.tran.z, tcp_pose.rpy.rx,
                 tcp_pose.rpy.ry, tcp_pose.rpy.rz);
  }

  bool kine_forward(const JointValue* joint_pos,
                    CartesianPose& cartesian_pose) {
    error_t ret = robot.kine_forward(joint_pos, &cartesian_pose);
    if (ret != ERR_SUCC) {
      rosError("Failed to kine forward: " + errorToString(ret));
      return false;
    }
    return true;
  }

  FeedbackMsg getFeedbackMsg() const { return feedback_msg; }

 private:
  /*********************
   ** Variables
   **********************/
  JAKAZuRobot robot;
  std::string robot_ip;
  float block_wait_timeout;
  float status_data_update_time_interval;
  int max_filter_size;
  float filter_kp;
  int timer_count;
  //
  RobotStatus robotstatus;
  ProgramState last_programstate;
  bool emergency_stop = false;
  bool soft_limited = false;
  bool isConnected = false;
  //
  JointValue joint_pose;
  CartesianPose tcp_pose;
  CartesianPose mid_pose;

  int gripper_rs485_channel = 1;
  int m_user_frame_id = 0;
  int m_tool_frame_id = 1;
  std::string name_;
  bool isServoMode = false;
  std::atomic<bool> stop_flag_{true};

  /*********************
  ** Ros Debugging
  **********************/
  void rosDebug(const std::string& msg) {
    spdlog::debug("[{}]: {}", name_, msg);
  }
  void rosInfo(const std::string& msg) { spdlog::info("[{}]: {}", name_, msg); }
  void rosWarn(const std::string& msg) { spdlog::warn("[{}]: {}", name_, msg); }
  void rosError(const std::string& msg) {
    spdlog::error("[{}]: {}", name_, msg);
  }

  std::string errorToString(errno_t error);

  /*********************
  **  Handles
  **********************/
  void handleToolPosition();
  void handleJointPosition();
  void handleRobotState();
  void updateProgrammState(const ProgramState& state);

 public:
  /*Define signal indices*/
  typedef enum {
    SIGNAL_INIT_STATE = 0x00,    //initialization state
    SIGNAL_GRIPPER_SATE = 0x01,  //gripper state
    SIGNAL_CUR_POS = 0x02,       //current position
    SIGNAL_SET_FORCE = 0x03,     //actual configured force
  } GRIPPER_SIGNAL;
  bool setGripperForce(uint16_t force);
  bool setGripperPos(uint16_t pos);
  bool queryGripperInfo();
  GripperInfo_t getGripperInfo() const;
  void pintGraipperInfo(const GripperInfo_t& info);
  void waitGripperInited(int max_timeout = 15000);
  void waitGripperCompleted(int max_timeout = 15000);
  bool isGripperCompleted();

 private:
  bool setGripperTransformerMode(int channel, const RS485TranformerMode& mode);
  bool initGripperSignInfo();
  bool initGripperType(uint16_t initType);
  bool isGripperSignalState(const SignInfo& info, GRIPPER_SIGNAL sg);

  //Set received signal values
  const SignInfo g_tSignInfo[4] = {
      {"InitState", GRIPPER_RS485_CHANNEL, 0x03, 0x0200, 0, 1},
      {"GripperState", GRIPPER_RS485_CHANNEL, 0x03, 0x0201, 0, 5},
      {"CurPos", GRIPPER_RS485_CHANNEL, 0x03, 0x0202, 0, 5},
      {"SetForce", GRIPPER_RS485_CHANNEL, 0x03, 0x0101, 0, 2}};
  //
  GripperInfo_t gripper_info;

  FeedbackMsg feedback_msg;
};

}  // namespace jaka_controller

#endif /* KOBUKI_ROS_HPP_ */
