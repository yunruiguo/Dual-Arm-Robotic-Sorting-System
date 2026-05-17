#include "jaka_controller/jaka_controller.hpp"

#include "jaka_controller/common.h"

namespace jaka_controller {
JakaController::JakaController() {}

JakaController::~JakaController() {}

void JakaController::initialize() {
  loadParamters();
  setupPublishers();
  setupSubscribers();
  setupServices();
  rosInfo("Initialized");
}

void JakaController::loadParamters() {
  timer_count = 0;
  robot_ip = "192.168.2.11";
  status_data_update_time_interval = 100;
  block_wait_timeout = 2;
  max_filter_size = 15;
  filter_kp = 0.03;
  feedback_msg.reset();
  rosInfo("Loaded Paramters");
}

bool JakaController::initRobot() {
  rosInfo("Initializing......");
  isConnected = false;
  if (robot.login_in(robot_ip.c_str()) != ERR_SUCC) {
    rosError("Failed to login");
    return false;
  }
  if (robot.set_block_wait_timeout(block_wait_timeout) != ERR_SUCC) {
    rosError("Failed to set block wait timeout");
    // return false;
  }

  rosInfo("Power on...");
  if (robot.power_on() != ERR_SUCC) {
    rosError("Failed to power on");
    robotstatus.powered_on = 0;
    robotstatus.enabled = 0;
    return false;
  }
  feedback_msg.powered_on = 1;
  robotstatus.powered_on = 1;
  std::this_thread::sleep_for(std::chrono::microseconds(1000));
  rosInfo("enable robot");
  if (robot.enable_robot() != ERR_SUCC) {
    rosError("Failed to enable robot");
    feedback_msg.enabled = 0;
    robotstatus.enabled = 0;
    return false;
  }
  // Joint-space first-order low-pass filtering in robot servo mode
  // robot.servo_move_use_joint_LPF(2);
  if (robot.servo_speed_foresight(max_filter_size, filter_kp) != ERR_SUCC) {
    rosError("Failed to servo speed foresight");
    // return false;
  }
  stop_flag_.store(false);
  feedback_msg.enabled = 1;
  robotstatus.enabled = 1;
  isConnected = true;
  return true;
}

bool JakaController::initGripper() {
  rosInfo("initGripper");
  bool ret = setGripperTransformerMode(GRIPPER_RS485_CHANNEL, ModBusRTU);
  if (!ret) {
    rosError("Failed to Init Gripper");
    return false;
  }
  ret = initGripperSignInfo();
  if (!ret) {
    rosError("Failed to Init Gripper");
    return false;
  }

  ret = initGripperType(0xA5);
  if (!ret) {
    rosError("Failed to Init Gripper");
    return false;
  }
  //
  waitGripperInited();
  waitGripperCompleted();
  return true;
}

bool JakaController::setGripperTransformerMode(
    int channel, const RS485TranformerMode& mode) {
  int chnMode;
  //
  error_t err = robot.get_rs485_chn_mode(channel, &chnMode);  //
  if (err != ERR_SUCC) {
    rosError("Failed to Get Gripper Channel Model");
    return false;
  }
  if (chnMode == mode) {
    rosInfo("Current Gripper Channel Mode[" + std::to_string(mode) +
            "] successfully");
    return true;
  }
  //Set
  err = robot.set_rs485_chn_mode(channel, mode);  //Set
  if (err == ERR_SUCC) {
    rosInfo("Set Gripper Channel Mode[" + std::to_string(mode) +
            "] successfully");
    return true;
  }
  return false;
}

bool JakaController::initGripperSignInfo() {
  if (!isConnected) {
    rosError("robot is disconnected");
    return false;
  }
  //Set
  bool ret = true;
  for (auto info : g_tSignInfo) {
    error_t err = robot.add_tio_rs_signal(info);
    if (ERR_SUCC != err) {
      ret &= false;
      rosWarn("Failed to Add RS Signal" + std::string(info.sig_name));
    }
  }
  return ret;
}

bool JakaController::initGripperType(uint16_t upServices()initType) {
  if (!isConnected) {
    rosError("robot is disconnected");
    return false;
  }
  uint8_t data[(MODBUS_RESGISTER_ACCESS_MAX_NUM << 1) + 11];
  ModbusAccessInfo_t tAccessInfo = {
      0x00, MODBUS_RTU, GRIPPER_RS485_CHANNEL, MODBUS_WriteSingleRegister,
      0x00, 1};

  tAccessInfo.startingAddress = 0x0100;
  uint16_t commandBytes =
      BSP_MODBUS_GetReadWriteServerCommand(&tAccessInfo, NULL, &initType, data);

  if (0 == commandBytes) {
    rosError("Failed to init gripper type:  invalid arg");
    return false;
  }
  error_t err =
      robot.send_tio_rs_command(GRIPPER_RS485_CHANNEL, data, commandBytes);
  if (err == ERR_SUCC) {
    rosInfo("Set Gripper Type Successfully");
    return true;
  }
  rosError("Failed to set type: " + errorToString(err));
  return false;
}

bool JakaController::isGripperSignalState(const SignInfo& info,
                                          GRIPPER_SIGNAL sg) {
  if (strcmp(info.sig_name, g_tSignInfo[sg].sig_name) == 0 ||
      std::string(info.sig_name).find(g_tSignInfo[sg].sig_name) !=
          std::string::npos) {
    if ((info.sig_type == g_tSignInfo[sg].sig_type) &&
        (info.sig_addr == g_tSignInfo[sg].sig_addr)) {
      return true;
    }
  }
  return false;
}

bool JakaController::queryGripperInfo() {
  errno_t err = ERR_SUCC;
  GripperInfo_t info;
  SignInfo tSignInfo[10] = {};
  int iSignalLen = 10;
  err = robot.get_rs485_signal_info(tSignInfo, &iSignalLen);
  if (err == ERR_SUCC) {
    for (int i = 0; i < iSignalLen && i < 10; i++) {
      if (tSignInfo[i].chn_id == GRIPPER_RS485_CHANNEL) {
        if (isGripperSignalState(tSignInfo[i], SIGNAL_GRIPPER_SATE)) {
          info.gripperState = tSignInfo[i].value;
        } else if (isGripperSignalState(tSignInfo[i], SIGNAL_INIT_STATE)) {
          info.initState = tSignInfo[i].value;
        } else if (isGripperSignalState(tSignInfo[i], SIGNAL_CUR_POS)) {
          info.curPos = tSignInfo[i].value;
        } else if (isGripperSignalState(tSignInfo[i], SIGNAL_SET_FORCE)) {
          info.setForce = tSignInfo[i].value;
        }
      }
    }

    feedback_msg.gripper_state = info.gripperState;
    feedback_msg.gripper_pos = info.curPos;
    feedback_msg.gripper_force = info.setForce;
    pintGraipperInfo(info);
    return true;
  }
  rosError("Failed to get gripper info: " + errorToString(err));
  return false;
}

GripperInfo_t JakaController::getGripperInfo() const { return gripper_info; }

void JakaController::pintGraipperInfo(const GripperInfo_t& info) {
  if (info.gripperState != gripper_info.gripperState) {
    rosInfo("##########################Gripper#########################");
    rosInfo("Gripper State: " + std::to_string(gripper_info.gripperState));
    rosInfo("Gripper initState State: " +
            std::to_string(gripper_info.initState));
    rosInfo("Gripper Pos: " + std::to_string(gripper_info.curPos));
    rosInfo("Gripper Force: " + std::to_string(gripper_info.setForce));
    rosInfo("#########################################################");
  }
  gripper_info = info;
}

bool JakaController::setGripperForce(uint16_t force) {
  if (!isConnected) {
    rosError("robot is disconnected");
    return false;
  }
  uint8_t data[(MODBUS_RESGISTER_ACCESS_MAX_NUM << 1) + 11];
  ModbusAccessInfo_t tAccessInfo = {
      0x00, MODBUS_RTU, GRIPPER_RS485_CHANNEL, MODBUS_WriteSingleRegister,
      0x00, 1};

  tAccessInfo.startingAddress = 0x0101;
  uint16_t commandBytes =
      BSP_MODBUS_GetReadWriteServerCommand(&tAccessInfo, NULL, &force, data);

  if (0 == commandBytes) {
    rosError("Failed to set gripper force:  invalid arg");
    return false;
  }
  error_t err =
      robot.send_tio_rs_command(GRIPPER_RS485_CHANNEL, data, commandBytes);
  if (err == ERR_SUCC) {
    rosInfo("Set Gripper Force Successfully");
    return true;
  }
  rosError("Failed to set force: " + errorToString(err));
  return false;
}
bool JakaController::setGripperPos(uint16_t pos) {
  if (!isConnected) {
    rosError("robot is disconnected");
    return false;
  }
  uint8_t data[(MODBUS_RESGISTER_ACCESS_MAX_NUM << 1) + 11];
  ModbusAccessInfo_t tAccessInfo = {
      0x00, MODBUS_RTU, GRIPPER_RS485_CHANNEL, MODBUS_WriteSingleRegister,
      0x00, 1};

  tAccessInfo.startingAddress = 0x0103;
  uint16_t commandBytes =
      BSP_MODBUS_GetReadWriteServerCommand(&tAccessInfo, NULL, &pos, data);

  if (0 == commandBytes) {
    rosError("Failed to set gripper force:  invalid arg");
    return false;
  }
  error_t err =
      robot.send_tio_rs_command(GRIPPER_RS485_CHANNEL, data, commandBytes);

  if (err == ERR_SUCC) {
    rosInfo("Set Gripper Pos[ " + std::to_string(pos) + "] Successfully");
    return true;
  }
  rosError("Failed to set Pos: " + errorToString(err));
  return false;
}

void JakaController::waitGripperInited(int max_timeout) {
  std::chrono::steady_clock::time_point start =
      std::chrono::steady_clock::now();

  auto dur = std::chrono::steady_clock::now() - start;
  while (!stop_flag_ &&
         std::chrono::duration_cast<std::chrono::milliseconds>(dur).count() <
             max_timeout) {
    if (queryGripperInfo()) {
      if (gripper_info.initState == 1) {
        rosInfo("Gripper Initialized");
        return;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    dur = std::chrono::steady_clock::now() - start;
  }
  rosWarn("Wait Gripper Init Timeout");
}

bool JakaController::isGripperCompleted() {
  // if (queryGripperInfo()) {
  if (feedback_msg.gripper_state > 0) {
    return true;
  }
  // }
  return false;
}

void JakaController::waitGripperCompleted(int max_timeout) {
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  std::chrono::steady_clock::time_point start =
      std::chrono::steady_clock::now();

  auto dur = std::chrono::steady_clock::now() - start;
  while (!stop_flag_ &&
         std::chrono::duration_cast<std::chrono::milliseconds>(dur).count() <
             max_timeout) {
    if (queryGripperInfo()) {
      if (gripper_info.gripperState > 0) {
        rosInfo("Gripper Completed");
        return;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    dur = std::chrono::steady_clock::now() - start;
  }
  rosWarn("Wait Gripper State Timeout");
}

void JakaController::stopRobot() {
  if (isConnected) {
    feedback_msg.enabled = 1;
    feedback_msg.powered_on = 1;
    // robot shutdown sequence
    robot.servo_move_enable(false);
    robot.disable_robot();
    robot.power_off();
    isConnected = false;
    rosInfo("Robot powered off.");
  }
  stop_flag_.store(true);
}

void JakaController::setupPublishers() { rosInfo("Publishers"); }
void JakaController::setupSubscribers() { rosInfo("Subscribered"); }
void JakaController::setupServices() { rosInfo("Services"); }

void JakaController::SpinOnce(void) {
  std::chrono::steady_clock::time_point start =
      std::chrono::steady_clock::now();
  //
  if (checkRobotStatus()) {
    handleToolPosition();
    handleJointPosition();
    handleRobotState();
  }
  std::chrono::steady_clock::time_point gripper_time =
      std::chrono::steady_clock::now();
  queryGripperInfo();
  auto dur = std::chrono::steady_clock::now() - start;
  auto count =
      std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
  if (count > 20) {
    auto dur1 = gripper_time - start;
    auto count1 =
        std::chrono::duration_cast<std::chrono::milliseconds>(dur1).count();
    std::cout << "[" << name_ << "]: [" << count << "]robot: " << count1
              << " gripper:" << count - count1 << std::endl;
    fflush(stdout);
  }
}

bool JakaController::checkRobotStatus() {
  if (!isConnected) {
    return false;
  }

  errno_t ret = robot.get_robot_status(&robotstatus);
  if (ret == ERR_SUCC) {
    static bool success = false;
    if (!success) {
      rosInfo("check robot status successed");
      success = true;
    }
    return true;
  }
  if (!robotstatus.is_socket_connect) {
    rosError("connect error!!!");
  } else {
    rosError("get_robot_status error: " + errorToString(ret));
  }
  return false;
}

void JakaController::servoMoveEnable(bool enable) {
  if (!isConnected) {
    return;
  }
  if (isServoMode == enable) {
    return;
  }
  if (!robotstatus.powered_on) {
    rosError("error occurred: robot is power off");
    return;
  }
  rosInfo("servo_move_enable executing...");
  error_t ret = robot.servo_move_enable(enable);
  if (ret == ERR_SUCC) {
    isServoMode = enable;
    feedback_msg.servo_state = isServoMode;
    rosInfo("servo_move_enable has been executed");
  } else {
    rosError("error occurred:" + errorToString(ret));
  }
}

bool JakaController::isMoveCompleted() {
  BOOL in_pos = feedback_msg.state;
  // robot.is_in_pos(&in_pos);
  return in_pos;
}

bool JakaController::WaitMoveCompleted(int max_timeout) {
  std::chrono::steady_clock::time_point start =
      std::chrono::steady_clock::now();
  BOOL in_pos = false;
  robot.is_in_pos(&in_pos);
  auto dur = std::chrono::steady_clock::now() - start;
  do {
    robot.is_in_pos(&in_pos);
    if (in_pos) {
      rosInfo("Move completed");
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    dur = std::chrono::steady_clock::now() - start;
    if (checkRobotStatus()) {
      handleRobotState();
    }
    queryGripperInfo();

  } while (!stop_flag_ && !in_pos &&
           std::chrono::duration_cast<std::chrono::milliseconds>(dur).count() <
               max_timeout);
  return in_pos;
}

void JakaController::setToolFramePos(int id, const CartesianPose& pos,
                                     const std::string& tool_frame_name) {
  error_t err = ERR_SUCC;
  err = robot.set_user_frame_id(m_user_frame_id);
  if (err != ERR_SUCC) {
    rosError("Failed to set user frame id: " + errorToString(m_user_frame_id));
  }
  err = robot.get_tool_data(id, &tcp_pose);
  if (err != ERR_SUCC) {
    rosError("Failed to get tool pose " + errorToString(err));
  }
  rosInfo("tool pose:");
  printCartesianPose(tcp_pose);
  tcp_pose = pos;
  err = robot.set_tool_data(id, &tcp_pose, tool_frame_name.c_str());
  if (err != ERR_SUCC) {
    rosError("Failed to set tool pose " + errorToString(err));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  err = robot.get_tool_data(id, &tcp_pose);
  if (err != ERR_SUCC) {
    rosError("Failed to get tool pose " + errorToString(err));
  }
  rosInfo("new tool pose:");
  printCartesianPose(tcp_pose);
  int cur_tool_id;
  err = robot.get_tool_id(&cur_tool_id);
  if (err != ERR_SUCC) {
    rosError("Failed to get tool id " + errorToString(err));
  }
  rosInfo("currrent tool id:" + std::to_string(cur_tool_id));
  if (err != ERR_SUCC) {
    rosError("Failed to set tool id " + errorToString(err));
  }
  err = robot.get_tool_id(&cur_tool_id);
  if (err != ERR_SUCC) {
    rosError("Failed to get tool id " + errorToString(err));
  }
  rosInfo("new tool id:" + std::to_string(cur_tool_id));
}

bool JakaController::jointMove(JointValue* joint_pose) {
  if (!isConnected) {
    rosError("robot is disconnected");
    return false;
  }
  servoMoveEnable(false);

  double speed = 0.4;
  double accel = 0.5;
  double tol = 0.5;
  OptionalCond* option_cond = nullptr;

  errno_t ret = robot.joint_move(joint_pose, MoveMode::ABS, false, speed, accel,

                                 tol, option_cond);
  if (ret == ERR_SUCC) {
    rosInfo("joint_move has been executed");
  } else {
    rosError("joint move error occurred:" + errorToString(ret));
    return false;
  }
  return true;
}

bool JakaController::lineMove(const CartesianPose& pos) {
  if (!isConnected) {
    rosError("robot is disconnected");
    return false;
  }
  servoMoveEnable(false);
  tcp_pose = pos;
  rosInfo("line pose:");
  printCartesianPose(tcp_pose);
  auto ret = robot.linear_move(&tcp_pose, MoveMode::ABS, false, 100);
  if (ret != ERR_SUCC) {
    rosError("line move error occurred:" + errorToString(ret));
    return false;
  }
  rosInfo("line_move has been executed");
  return true;
}

bool JakaController::circleMove(const CartesianPose& mid_pos,
                                const CartesianPose& end_pos) {
  if (!isConnected) {
    rosError("robot is disconnected");
    return false;
  }
  servoMoveEnable(false);
  tcp_pose = end_pos;
  mid_pose = mid_pos;
  rosInfo("circular pose:");
  printCartesianPose(feedback_msg.tcp_pose);
  printCartesianPose(mid_pose);
  printCartesianPose(tcp_pose);

  double speed = 20;
  double accel = 5;
  double tol = 0.1;
  OptionalCond* option_cond = nullptr;

  auto ret = robot.circular_move(&tcp_pose, &mid_pose, MoveMode::ABS, false,
                                 speed, accel, tol, option_cond);
  if (ret != ERR_SUCC) {
    rosError("circular move error occurred:" + errorToString(ret));
    return false;
  }
  rosInfo("circular_move has been executed");
  return true;
}

bool JakaController::ServoMove(const CartesianPose& pos) {
  if (!isConnected) {
    rosError("robot is disconnected");
    return false;
  }
  servoMoveEnable(true);
  tcp_pose = pos;
  rosInfo("pose:");
  printCartesianPose(tcp_pose);
  auto ret = robot.servo_p(&tcp_pose, MoveMode::ABS);
  if (ret != ERR_SUCC) {
    rosError(" servo move error occurred:" + errorToString(ret));
    return false;
  }
  rosInfo("servo pose move has been executed");
  return true;
}

bool JakaController::ServoJointMove(const JointValue& joint) {
  if (!isConnected) {
    rosError("robot is disconnected");
    return false;
  }
  servoMoveEnable(true);
  joint_pose = joint;
  auto ret = robot.servo_j(&joint_pose, MoveMode::ABS);
  if (ret != ERR_SUCC) {
    rosError(" servo joint move error occurred:" + errorToString(ret));
    return false;
  }
  rosInfo("servo joint move has been executed");
  return true;
}

bool JakaController::stopMotion() {
  if (!isConnected) {
    rosError("robot is disconnected");
    return false;
  }
  auto ret = robot.motion_abort();
  if (ret != ERR_SUCC) {
    rosError(" stop motion error occurred:" + errorToString(ret));
    return false;
  }
  rosInfo("stop motion has been executed");
  return true;
}

CartesianPose JakaController::getTcpPose() {
  CartesianPose tcp;
  robot.get_tcp_position(&tcp);
  return tcp;
}

void JakaController::setRobotIP(const std::string& ip) { robot_ip = ip; }

void JakaController::handleToolPosition() {}
void JakaController::handleJointPosition() {}
void JakaController::handleRobotState() {
  // ProgramState programstate;
  // robot.get_program_state(&programstate);
  // updateProgrammState(programstate);
  if (robotstatus.emergency_stop && !emergency_stop) {
    rosWarn("Emergency stop pressed!!!");
  } else if (emergency_stop && !robotstatus.emergency_stop) {
    rosInfo("Emergency stop release");
  }
  emergency_stop = robotstatus.emergency_stop;
  if (robotstatus.on_soft_limit && !soft_limited) {
    rosError("Soft Limited");
  } else if (!robotstatus.on_soft_limit && soft_limited) {
    rosInfo("Soft Limit removed");
  }
  soft_limited = robotstatus.on_soft_limit;
  feedback_msg.emergency_stop = robotstatus.emergency_stop;
  feedback_msg.protective_stop = robotstatus.protective_stop;
  feedback_msg.powered_on = robotstatus.powered_on;
  feedback_msg.errcode = robotstatus.errcode;
  feedback_msg.enabled = robotstatus.enabled;
  feedback_msg.state = robotstatus.inpos;
  feedback_msg.tcp_pose.tran.x = robotstatus.cartesiantran_position[0];
  feedback_msg.tcp_pose.tran.y = robotstatus.cartesiantran_position[1];
  feedback_msg.tcp_pose.tran.z = robotstatus.cartesiantran_position[2];

  feedback_msg.tcp_pose.rpy.rx = robotstatus.cartesiantran_position[3];
  feedback_msg.tcp_pose.rpy.ry = robotstatus.cartesiantran_position[4];
  feedback_msg.tcp_pose.rpy.rz = robotstatus.cartesiantran_position[5];

  feedback_msg.joint_pose.jVal[0] = robotstatus.joint_position[0];
  feedback_msg.joint_pose.jVal[1] = robotstatus.joint_position[1];
  feedback_msg.joint_pose.jVal[2] = robotstatus.joint_position[2];
  feedback_msg.joint_pose.jVal[3] = robotstatus.joint_position[3];
  feedback_msg.joint_pose.jVal[4] = robotstatus.joint_position[4];
  feedback_msg.joint_pose.jVal[5] = robotstatus.joint_position[5];

  // robot.get_tcp_position(&feedback_msg.tcp_pose);
  // robot.get_joint_position(&feedback_msg.joint_pose);
}
static std::string programmStateStr(const ProgramState& state) {
  std::string str = "UNKOWN";
  if (state == PROGRAM_IDLE) {
    str = "IDLE";
  } else if (state == PROGRAM_RUNNING) {
    str = "RUNNING";
  } else if (state == PROGRAM_PAUSED) {
    str = "PAUSED";
  }
  return str;
}

void JakaController::updateProgrammState(const ProgramState& state) {
  if (state != last_programstate) {
    spdlog::info("[{}]: robot state from [{}] to [{}]", name_,
                 programmStateStr(last_programstate), programmStateStr(state));
    last_programstate = state;
  }
}

std::string JakaController::errorToString(errno_t error) {
  std::string str = "UNKOWN ERROR";
  switch (error) {
    case 2:
      str = "ERR_FUCTION_CALL_ERROR";
      break;
    case -1:
      str = "ERR_INVALID_HANDLER";
      break;
    case -2:
      str = "ERR_INVALID_PARAMETER";
      break;
    case -3:
      str = "ERR_COMMUNICATION_ERR";
      break;
    case -4:
      str = "ERR_KINE_INVERSE_ERR";
      break;
    case -5:
      str = "ERR_EMERGENCY_PRESSED";
      break;
    case -6:
      str = "ERR_NOT_POWERED";
      break;
    case -7:
      str = "ERR_NOT_ENABLED";
      break;
    case -8:
      str = "ERR_DISABLE_SERVOMODE";
      break;
    case -9:
      str = "ERR_NOT_OFF_ENABLE";
      break;
    case -10:
      str = "ERR_PROGRAM_IS_RUNNING";
      break;
    case -11:
      str = "ERR_CANNOT_OPEN_FILE";
      break;
    case -12:
      str = "ERR_MOTION_ABNORMAL";
      break;
    case -14:
      str = "ERR_FTP_PREFROM";
      break;
    case -15:
      str = "ERR_VALUE_OVERSIZE";
      break;
    default:
      str = std::to_string(error);
      break;
  }
  return str;
}

}  // namespace jaka_controller
