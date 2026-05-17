#ifndef GRASP_TASK_MANAGER_H
#define GRASP_TASK_MANAGER_H

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <grasp_util/safe_data_class.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "grasp_task_manager/arm_controller.h"
#include "grasp_task_manager/grasp_common.h"
#include "kinematics_interface.hpp"

// ()
#define GRASP_OBJECT_ARM (RobotIndex::LEFT)

/// @brief GraspTaskManager:/ROS 2
class GraspTaskManager : public rclcpp::Node {
 public:
  /// @brief 
  /// @param options ROS 2 
  explicit GraspTaskManager(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /**
   * @brief 
   */
  void init();
  /**
   * @brief 
   */
  void initVariant();
  /**
   * @brief Parameters
   */
  void loadParamters();

  /**
   * @brief Set
   */
  void setupSubscribers();
  /**
   * @brief Set
   */
  void setupActions();

 protected:
  /**
   * @brief 
   * @param index 
   * @param poses 
   * @return 
   */
  bool executeArmControl(int index, const grasp_msgs::msg::ArmCommand& poses);
  /**
   * @brief 
   * @param index 
   * @param msg 
   * @return 
   */
  bool executeGripperControl(int index,
                             const grasp_msgs::msg::GripperCommand& msg);
  /**
   * @brief 
   * @param index 
   * @param seconds ()
   * @return 
   */
  bool executeDelay(int index, double senconds);

 private:
  /**
   * @brief 
   */
  void onTimerCallback(void);

  ///////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * @brief 
   * @param index 
   */
  void queryTaskStatus(int index);
  /**
   * @brief 
   * @param index 
   * @return 
   */
  bool isTaskInProgress(int index);
  /**
   * @brief 
   * @param index 
   */
  void handleTaskIdleState(int index);
  /**
   * @brief 
   * @param index 
   */
  void handleTaskExecution(int index);
  /**
   * @brief 
   * @param index 
   */
  void popTaskFromQueue(int index);
  /**
   * @brief 
   * @param index 
   */
  void processTask(int index);
  /**
   * @brief 
   * @param index 
   */
  void executeNextTask(int index);
  /**
   * @brief 
   * @param index 
   */
  void provideTaskFeedback(int index);
  /**
   * @brief 
   * @param index 
   * @return Returns true,otherwiseReturns false
   */
  bool areAllTasksCompleted(int index);
  /**
   * @brief 
   * @param index 
   * @return Returns true,otherwiseReturns false
   */
  bool isTaskFinished(int index);
  /**
   * @brief 
   * @param index 
   * @return Returns true,otherwiseReturns false
   */
  bool isArmMotionCompleted(int index) const;
  /**
   * @brief 
   * @param index 
   * @return Returns true,otherwiseReturns false
   */
  bool isGripperActionCompleted(int index) const;
  /**
   * @brief 
   * @param index 
   * @return Returns true,otherwiseReturns false
   */
  bool isDelayCompleted(int index);
  /**
   * @brief 
   * @param index 
   * @param state 
   */
  void updateTaskStatus(int index, const TaskState& state);

  ///////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * @brief 
   * @param index 
   * @param goal 
   * @return 
   */
  bool isGoalReached(int index, const grasp_msgs::msg::ArmCommand& goal);

  /**
   * @brief joint position
   * @param index 
   * @param goal joint position
   * @return 
   */
  bool areJointsGoalReached(int index, const grasp_msgs::msg::ArmCommand& goal);

  /**
   * @brief 
   * @param index 
   * @return Returns true,otherwiseReturns false
   */
  bool isGraspObjectArm(int index) const;

  ///////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * @brief 
   * @param index 
   * @param type 
   */
  void updateTaskType(int index, const TaskType& type);
  /**
   * @brief 
   * @param index 
   */
  void dispatchTask(int index);
  /**
   * @brief 
   * @param index 
   */
  void waitDispatchTask(int index);
  /**
   * @brief Returns
   * @param index 
   */
  void dispatchHomeTask(int index);
  /**
   * @brief 
   * @param index 
   */
  void dispatchGraspObjectTask(int index);
  /**
   * @brief 
   * @param index 
   */
  void dispatchCutBoxTask(int index);

  /**
   * @brief 
   * @param index 
   */
  void executeCutBoxTask(int index);

  /**
   * @brief 
   * @param index 
   */
  void dispatchPressBoxTask(int index);

  /**
   * @brief 
   * @param index 
   */
  void dispatchGraspKnifeTask(int index);

  /**
   * @brief 
   * @param index 
   */
  void dispatchPlaceKnifeTask(int index);

  /**
   * @brief 
   * @param index 
   */
  void dispatchOpenBoxTask(int index);

  /**
   * @brief 
   * @param index 
   */
  void dispatchLiftBoxTask(int index);

  /**
   * @brief 
   * @param index 
   */
  void dispatchFlippBoxTask(int index);

  /**
   * @brief 
   * @param index 
   */
  void dispatchDropBoxTask(int index);

  /**
   * @brief 
   * @param index 
   */
  void dispatchSortingObjectTask(int index);

  /**
   * @brief 
   * @param index 
   */
  void waitHomeTask(int index);
  /**
   * @brief 
   * @param index 
   */
  void waitObjectTask(int index);
  /**
   * @brief 
   * @param index 
   */
  void waitCutBoxTask(int index);

  /**
   * @brief 
   * @param index 
   */
  void waitPressBoxTask(int index);

  /**
   * @brief 
   * @param index 
   */
  void waitKinfeTask(int index);

  /**
   * @brief 
   * @param index 
   */
  void waitPlaceKnifeTask(int index);

  /**
   * @brief 
   * @param index 
   */
  void waitOpenBoxTask(int index);

  /**
   * @brief 
   * @param index 
   */
  void waitLfitBoxTask(int index);

  /**
   * @brief 
   * @param index 
   */
  void waitFlippBoxTask(int index);

  /**
   * @brief 
   * @param index 
   */
  void waitDropBoxTask(int index);

  /**
   * @brief 
   * @param index 
   */
  void waitSortingObjectTask(int index);
  /**
   * @brief 
   * @param index 
   * @param msgs 
   */
  void performCutBoxOperation(
      int index, const std::vector<grasp_msgs::msg::ArmCommand>& msgs);

  /**
   * @brief 
   * @param index 
   * @param msgs 
   */
  void performGraspBoxOperation(
      int index, const std::vector<grasp_msgs::msg::ArmCommand>& msgs);

  tf2::Transform generateBoxOpeningPose(const tf2::Transform& tool_translation,
                                        double roll, double pitch, double yaw,
                                        double length);

  /**
   * @brief 
   * @param index 
   */
  void moveArmToCutBoxHome(int index);
  /**
   * @brief 
   * @param index 
   */
  void moveArmToGraspBoxHome(int index);

  ///////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * @brief 
   */
  void executeDualGrasp();
  ///////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * @brief 
   * @param index 
   */
  void closeGripper(int index);

  /**
   * @brief 
   * @param index 
   */
  void openGripper(int index);

  /**
   * @brief 
   * @param index 
   * @param command 
   */
  void controlGripper(int index,
                      const grasp_msgs::msg::GripperCommand& command);

  /**
   * @brief 
   * @param index 
   * @param command 
   */
  void moveTo(int index, const grasp_msgs::msg::ArmCommand& command);

  void moveToCloseGripper(int index,
                          const grasp_msgs::msg::ArmCommand& command);

  /**
   * @brief 
   * @param index 
   * @param command 
   */
  void moveToPlace(int index, const grasp_msgs::msg::ArmCommand& command);

  /**
   * @brief 
   * @param index 
   * @param command 
   */
  void moveToGrasp(int index, const grasp_msgs::msg::ArmCommand& command);

  /**
   * @brief 
   * @param index 
   */
  void goHome(int index);

  /**
   * @brief 
   * @param index 
   */
  void goVisonHome(int index);

  /**
   * @brief 
   * @param index 
   */
  void goSortingVisonHome(int index);

  /**
   * @brief 
   * @param index 
   */
  void goVisonBoxHome(int index);

  /**
   * @brief 
   * @param index 
   */
  void goKnifeHome(int index);

  /**
   * @brief 
   * @param index 
   */
  void goKnifeRetreat(int index);

  /**
   * @brief 
   * @param index 
   * @param seconds ()
   */
  void doDelay(int index, double seconds);
  ///////////////////////////////////////////////////////////////////////////////////////////////

  /**
   * @brief 
   * @param index 
   * @param pose 
   * @return 
   */
  std::vector<grasp_msgs::msg::ArmCommand> generateArmCommandsForPose(
      int index, const grasp_pose_t& pose);

  tf2::Transform toTargetBase(int index, int target_index,
                              const tf2::Transform& pose);

  /**
   * @brief 
   * @param index 
   * @param pose 
   * @return 
   */
  std::vector<grasp_msgs::msg::ArmCommand> generateArmPressCommandsForPose(
      int index, int vision_index, const grasp_pose_t& pose);

  /**
   * @brief 
   * @param index 
   * @param pose 
   * @return 
   */
  std::list</*grasp_msgs::msg::ArmCommand*/ task_msg_t>
  generateArmOpenCommandsForPose(int index, int vision_index,
                                 const grasp_pose_t& pose);

  /**
   * @brief 
   * @param position 
   * @param euler_x  X 
   * @param euler_z  Z 
   * @return Returns
   */
  grasp_msgs::msg::ArmCommand createArmMovementCommand(
      const tf2::Vector3& position, double euler_x, double euler_y,
      double euler_z);

  /**
   * @brief createArmArcMovementCommand
   * @param position
   * @return
   */
  grasp_msgs::msg::ArmCommand createArmArcMovementCommand(
      const std::vector<tf2::Transform>& position);

  /**
   * @brief 
   * @param start_pos 
   * @param mid_pos ()
   * @param end_pos 
   * @return Returns
   */
  grasp_msgs::msg::ArmCommand createCircularMovementCommand(
      const grasp_msgs::msg::TCPPose& mid_pos,
      const grasp_msgs::msg::TCPPose& end_pos);

  /**
   * @brief TCP
   * @param index 
   * @param camera2obj 
   * @return TCP
   */
  tf2::Transform transformObjectToTCPFrame(int index,
                                           const tf2::Transform& camera2obj);

  /**
   * @brief ()
   *
   * (),.
   *
   * @param cart
   * ,(,).
   * @return ,.
   */
  grasp_msgs::msg::ArmCommand JntToCart(int index,
                                        const grasp_msgs::msg::TCPPose& cart);

  /**
   * @brief 
   *
   * ,.
   *
   * @param jnt ,.
   * @return ,.
   */
  grasp_msgs::msg::ArmCommand CartToJnt(const grasp_msgs::msg::JointPose& jnt);

  ///////////////////////////////////////////////////////////////////////////////////////////////

  /**
   * @brief 
   */
  std::string arm_action_name_[ARM_NUMBER];

  /**
   * @brief 
   */
  std::string gripper_action_name_[ARM_NUMBER];

  ////////////////////////////////////////////////////////////////////////////////###############

  /**
   * @brief  tf 
   */
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

  /**
   * @brief tf 
   */
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  /**
   * @brief ,
   */
  rclcpp::CallbackGroup::SharedPtr timer_cb_group;

  ///////////////////////////////////////////////////////////////////////////////////////////////

  /**
   * @brief ,
   */
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief 
   */
  std::shared_ptr<ArmController> arm_controller_[ARM_NUMBER];

  /**
   * @brief 
   */
  grasp_msgs::msg::ArmCommand home_pos[ARM_NUMBER];

  /**
   * @brief  TCP 
   */
  grasp_msgs::msg::ArmCommand home_tcp_pos[ARM_NUMBER];

  /**
   * @brief 
   */
  grasp_msgs::msg::ArmCommand cutbox_vision_home[ARM_NUMBER];
  /**
   * @brief 
   */
  grasp_msgs::msg::ArmCommand sorting_vision_home[ARM_NUMBER];
  grasp_msgs::msg::ArmCommand sorting_vision_joint[ARM_NUMBER];

  /**
   * @brief 
   */
  grasp_msgs::msg::ArmCommand graspbox_vision_home[ARM_NUMBER];

  /**
   * @brief 
   */
  grasp_msgs::msg::ArmCommand graspknife_vision_home[ARM_NUMBER];
  grasp_msgs::msg::ArmCommand graspknife_vision_joint[ARM_NUMBER];

  /**
   * @brief 
   */
  grasp_msgs::msg::ArmCommand pickbox_pose[ARM_NUMBER];

  grasp_msgs::msg::ArmCommand knife_retreat_joint[ARM_NUMBER];

  grasp_msgs::msg::ArmCommand sorting_place_pose[ARM_NUMBER];

  grasp_msgs::msg::ArmCommand drop_box_pos[ARM_NUMBER];

  /**
   * @brief 
   */
  grasp_msgs::msg::ArmCommand goal_pose[ARM_NUMBER];

  /**
   * @brief 
   */
  grasp_util::safe_class<task_msg_t> arm_command[ARM_NUMBER];

  /////////////////////////////////////////////////////////////////////////////////////////////////

  /**
   * @brief 
   */
  current_task_t current_task[ARM_NUMBER];

  /**
   * @brief 
   */
  TaskState task_state[ARM_NUMBER];

  /**
   * @brief 
   */
  ArmFeedbackMsg arm_feedback[ARM_NUMBER];

  /**
   * @brief 
   */
  GripperFeedbackMsg gripper_feedback[ARM_NUMBER];

  /**
   * @brief 
   */
  grasp_msgs::msg::GripperCommand open_gripper;

  /**
   * @brief 
   */
  grasp_msgs::msg::GripperCommand close_gripper;

  /**
   * @brief 
   */
  std::list<task_msg_t> task_queue[ARM_NUMBER];

  /**
   * @brief 
   */
  std::vector<grasp_msgs::msg::ArmCommand> place_knife_task_cmd[ARM_NUMBER];

  /**
   * @brief press_box_task_cmd
   */
  std::vector<grasp_msgs::msg::ArmCommand> press_box_task_cmd[ARM_NUMBER];

  /**
   * @brief 
   */
  rclcpp::Time delay_endpoint_time[ARM_NUMBER];

  /**
   * @brief 
   */
  camera_intrinsic_t camera_intrinsic[ARM_NUMBER];

  /**
   * @brief 
   */
  TaskType task_type[ARM_NUMBER];

  /**
   * @brief 
   */
  uint16_t pipeline_state_;

  /**
   * @brief 
   */
  std::unique_ptr<kinematics_interface::KinematicsInterface> kinematics_;

  grasp_pose_t cut_box_pose;

  std::list</*grasp_msgs::msg::ArmCommand*/ task_msg_t>
      open_box_tasks[ARM_NUMBER];
  int sorting_number = 0;

 private:
  /**
   * @brief 
   */
  tf2::Transform tool2camera[ARM_NUMBER];
  /**
   * @brief 
   */
  tf2::Transform object_calib[ARM_NUMBER];
  /**
   * @brief TCP
   */
  tf2::Transform tcp2tool[ARM_NUMBER];
  /**
   * @brief   TCP 
   */
  tf2::Vector3 tcp_calib[ARM_NUMBER];

  tf2::Transform camerabase2top[ARM_NUMBER];

  tf2::Quaternion open_box_qua[ARM_NUMBER];
  tf2::Quaternion circle_vel_qua[ARM_NUMBER];
  tf2::Vector3 euler_rotated[ARM_NUMBER];
};

#endif
