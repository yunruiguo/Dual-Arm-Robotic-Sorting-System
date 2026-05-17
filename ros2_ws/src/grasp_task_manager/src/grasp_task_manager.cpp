#include "grasp_task_manager/grasp_task_manager.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <grasp_util/geometry_utils.hpp>
#include <grasp_util/paramters_server.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "kinematics_interface_kdl.hpp"

#define EPSILON 1e-1

GraspTaskManager::GraspTaskManager(const rclcpp::NodeOptions& options)
    : Node("grasp_task_manager", options) {
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  kinematics_ =
      std::make_unique<kinematics_interface_kdl::KinematicsInterfaceKDL>();
  loadParamters();
}

/**
 * @brief initialize the move group interface
 *  MoveGroupInterface needs the shared point of the Node, but shared_ptr won't
 * be created until after the constructor returns so we need to create the move
 * group interface in a separate function
 * @ref
 * https://robotics.stackexchange.com/questions/96027/getting-a-nodesharedptr-from-this
 */
void GraspTaskManager::init() {
  initVariant();
  setupActions();
  setupSubscribers();
}

void GraspTaskManager::loadParamters() {
  std::string robot_description = "";
  grasp_util::declare_param(this, "robot_description", robot_description,
                            robot_description);

  std::string end_effector_name = "";
  grasp_util::declare_param(this, "end_effector_name", end_effector_name,
                            end_effector_name);

  kinematics_->initialize(robot_description, end_effector_name);
  kinematics_->setEndEffectorOffsetAngle(-0.66155);
  kinematics_->setBaseMountingAngles(-M_PI / 4, 0.0, M_PI / 2);
  for (int i = 0; i < ARM_NUMBER; i++) {
    const std::string idx = IndexToStr(i);
    arm_action_name_[i] = idx + "/arm_control";
    grasp_util::declare_param(this, idx + ".arm_action_name",
                              arm_action_name_[i], arm_action_name_[i]);

    gripper_action_name_[i] = idx + "/gripper_control";

    grasp_util::declare_param(this, idx + ".gripper_action_name",
                              gripper_action_name_[i], gripper_action_name_[i]);

    std::vector<double> tool2camera_vect = {1., 0., 0., 0., 0., 1., 0., 0.,
                                            0., 0., 1., 0., 0., 0., 0., 1.};
    grasp_util::declare_param_vector(this, idx + ".tool2camera",
                                     tool2camera_vect, tool2camera_vect);
    tool2camera[i] = createTransformFromPose(tool2camera_vect);

    std::vector<double> object_calib_vect = {1., 0., 0., 0., 0., 1., 0., 0.,
                                             0., 0., 1., 0., 0., 0., 0., 1.};
    grasp_util::declare_param_vector(this, idx + ".object_calib",
                                     object_calib_vect, object_calib_vect);
    object_calib[i] = createTransformFromPose(object_calib_vect);

    std::vector<double> camerabase2top_vect = {1., 0., 0., 0., 0., 1., 0., 0.,
                                               0., 0., 1., 0., 0., 0., 0., 1.};
    grasp_util::declare_param_vector(this, idx + ".camerabase2top",
                                     camerabase2top_vect, camerabase2top_vect);

    camerabase2top[i] = createTransformFromPose(camerabase2top_vect);

    std::vector<double> tcp2tool_vect{0., 0., 0., 0., 0., 0.};
    grasp_util::declare_param_vector(this, idx + ".tcp2tool", tcp2tool_vect,
                                     tcp2tool_vect);
    tcp2tool[i] = poseToTransformMatrix(tcp2tool_vect);

    std::vector<double> tf_calib_vect = {0., 0., 0.};
    grasp_util::declare_param_vector(this, idx + ".tcp_calib", tf_calib_vect,
                                     tf_calib_vect);
    tcp_calib[i] =
        tf2::Vector3(tf_calib_vect[0], tf_calib_vect[1], tf_calib_vect[2]);

    std::vector<double> camera_intrinsic_vector = {
        516.896423, 0., 318.293701, 0.0, 517.009033, 244.981689, 0.0, 0.0, 1.0};
    grasp_util::declare_param_vector(this, idx + ".camera_intrinsic",
                                     camera_intrinsic_vector,
                                     camera_intrinsic_vector);
    camera_intrinsic[i] = toCameraIntrinsicFromArray(camera_intrinsic_vector);
  }
}

void GraspTaskManager::setupSubscribers() {
  RCLCPP_INFO_ONCE(this->get_logger(), "setupSubscribers...");
  timer_cb_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&GraspTaskManager::onTimerCallback, this), timer_cb_group);
}

void GraspTaskManager::setupActions() {
  for (int i = 0; i < ARM_NUMBER; i++) {
    arm_controller_[i] = std::make_shared<ArmController>(
        this->shared_from_this(), arm_action_name_[i], gripper_action_name_[i],
        IndexToStr(i));
    // arm_controller_[i]->setCameraIntrinsic(camera_intrinsic[i]);
    arm_controller_[i]->waitForActionServers();
  }
}

static tf2::Transform calculateRotatedPose(const tf2::Transform& start_pose,
                                           const tf2::Vector3& center_pose,
                                           tf2::Vector3 axis, double theta) {
  double cos_theta = cos(theta);
  double sin_theta = sin(theta);

  auto norm_axis = axis.normalized();

  double u_x = norm_axis.x();
  double u_y = norm_axis.y();
  double u_z = norm_axis.z();

  tf2::Matrix3x3 R_axis(cos_theta + u_x * u_x * (1 - cos_theta),
                        u_x * u_y * (1 - cos_theta) - u_z * sin_theta,
                        u_x * u_z * (1 - cos_theta) + u_y * sin_theta,
                        u_y * u_x * (1 - cos_theta) + u_z * sin_theta,
                        cos_theta + u_y * u_y * (1 - cos_theta),
                        u_y * u_z * (1 - cos_theta) - u_x * sin_theta,
                        u_z * u_x * (1 - cos_theta) - u_y * sin_theta,
                        u_z * u_y * (1 - cos_theta) + u_x * sin_theta,
                        cos_theta + u_z * u_z * (1 - cos_theta));

  auto pose_local = start_pose.getOrigin() - center_pose;  //# 
  auto pose_rot = R_axis * pose_local;     // # 
  auto pose_end = pose_rot + center_pose;  //# 
  // # 
  auto R_end = R_axis * start_pose.getBasis();

  return tf2::Transform(R_end, pose_end);
}

static std::vector<tf2::Transform> generateCircleTrajectoryWithPose(
    const tf2::Transform& start_pose, const tf2::Vector3& box_center_pos,
    const tf2::Vector3& center, int num_points = 2,
    double arc_angle = -M_PI / 2) {
  // 
  tf2::Vector3 start_position = start_pose.getOrigin();
  tf2::Matrix3x3 rotation_matrix(start_pose.getRotation());

  //  (Z )
  tf2::Vector3 direction = (box_center_pos - center).normalized();

  //  (u  v)
  tf2::Vector3 k = (direction != tf2::Vector3(0, 0, 1)) ? tf2::Vector3(0, 0, 1)
                                                        : tf2::Vector3(0, 1, 0);
  tf2::Vector3 u = direction.cross(k).normalized();
  tf2::Vector3 v = direction.cross(u);

  // , 45deg
  tf2::Matrix3x3 claw_rotation_offset;
  claw_rotation_offset.setRPY(0, 0, 10 * M_PI / 180);  //  Z  45deg

  // 
  std::vector<tf2::Transform> trajectory_poses;

  // 
  double angle_step = arc_angle / (num_points - 1);
  double radius = (start_position - center).length();  // 
  // 
  for (int i = 0; i < num_points; ++i) {
    double angle = i * angle_step;

    // 
    tf2::Vector3 point =
        center + radius * std::cos(angle) * u + radius * std::sin(angle) * v;

    // ( Z )
    tf2::Matrix3x3 rotation_matrix_current;
    rotation_matrix_current.setRPY(0, 0, angle);

    // 
    tf2::Matrix3x3 current_rotation_matrix =
        rotation_matrix * rotation_matrix_current;

    // 
    tf2::Matrix3x3 claw_rotation_matrix =
        current_rotation_matrix * claw_rotation_offset;

    // 
    double current_roll, current_pitch, current_yaw;
    claw_rotation_matrix.getRPY(current_roll, current_pitch, current_yaw);

    //  tf2::Transform 
    tf2::Transform transform;
    tf2::Quaternion q;
    q.setRPY(current_roll, current_pitch, current_yaw);

    transform.setOrigin(point);
    transform.setRotation(q);

    trajectory_poses.push_back(transform);
  }

  return trajectory_poses;
}

tf2::Transform GraspTaskManager::toTargetBase(int index, int target_index,
                                              const tf2::Transform& pose) {
  if (index == target_index) {
    return pose;
  }
  auto top2target = camerabase2top[target_index].inverse() * pose;
  auto current2target = camerabase2top[index] * top2target;
  return current2target;
}

std::vector<grasp_msgs::msg::ArmCommand>
GraspTaskManager::generateArmCommandsForPose(int index,
                                             const grasp_pose_t& pose) {
  grasp_msgs::msg::ArmCommand command;
  std::vector<grasp_msgs::msg::ArmCommand> commands;

  tf2::Quaternion qua;
  qua.setRPY(pose.roll, pose.pitch, pose.yaw);
  tf2::Transform camera2center(qua, pose.center_pos);
  tf2::Transform tcp2center = transformObjectToTCPFrame(index, camera2center);
  tf2::Vector3 d = tcp2center.getOrigin();
  tf2::Vector3 delta = tf2::Vector3(4.5, 5.5, 86.5 - 2);
  tf2::Vector3 delta1 = tf2::Vector3(0.0, 0.0, 0.0);

  if (pose.isCutBox()) {
    tcp2center.setOrigin(tcp2center.getOrigin() + delta);

    tf2::Transform camera2left(qua, pose.left_pos);
    tf2::Transform camera2right(qua, pose.right_pos);

    tf2::Transform camera2leftpress(qua, pose.left_press_pos);
    tf2::Transform camera2rightpress(qua, pose.right_press_pos);

    tf2::Transform tcp2left = transformObjectToTCPFrame(index, camera2left);
    tf2::Transform tcp2right = transformObjectToTCPFrame(index, camera2right);

    tf2::Transform tcp2leftpress =
        transformObjectToTCPFrame(index, camera2leftpress);
    tf2::Transform tcp2rightpress =
        transformObjectToTCPFrame(index, camera2rightpress);

    tcp2left.setOrigin(tcp2left.getOrigin() + delta);
    tcp2right.setOrigin(tcp2right.getOrigin() + delta);

    tcp2leftpress.setOrigin(tcp2leftpress.getOrigin() + delta);
    tcp2rightpress.setOrigin(tcp2rightpress.getOrigin() + delta);
    //  Z 
    double newZ =
        (tcp2leftpress.getOrigin().z() + tcp2rightpress.getOrigin().z()) / 2;
    tcp2center.getOrigin().setZ(newZ);
    tcp2left.getOrigin().setZ(newZ + 2.5);
    tcp2right.getOrigin().setZ(newZ - 1.5);

    //  BA  BC
    tf2::Vector3 vec_BA = tcp2left.getOrigin() - tcp2center.getOrigin();
    tf2::Vector3 vec_BC = tcp2right.getOrigin() - tcp2center.getOrigin();
    //  k
    double k = 1.1;
    //  d  e 
    d = tcp2left.getOrigin() + k * vec_BA;                // 1
    tf2::Vector3 e = tcp2right.getOrigin() + k * vec_BC;  // 3

    ////////////////////////////////////////////////////
    //
    // tcp2left.getOrigin().setZ(newZ + 0.5);
    // tcp2right.getOrigin().setZ(newZ - 0.5);

    //  AD  AE
    tf2::Vector3 vec_AD = d - tcp2left.getOrigin();
    tf2::Vector3 vec_AE = e - tcp2left.getOrigin();

    auto vec_top_left = tcp2rightpress.getOrigin() - tcp2right.getOrigin();
    auto vec_top_right = tcp2leftpress.getOrigin() - tcp2right.getOrigin();

    auto top_left = tcp2right.getOrigin() - 0.87 * vec_top_left;
    auto top_right = tcp2right.getOrigin() - 0.87 * vec_top_right;

    auto vec_down_left = tcp2rightpress.getOrigin() - tcp2left.getOrigin();
    auto vec_down_right = tcp2leftpress.getOrigin() - tcp2left.getOrigin();
    //
    auto down_left = tcp2left.getOrigin() - 0.85 * vec_down_left;
    auto down_right = tcp2left.getOrigin() - 0.85 * vec_down_right;

    //
    // commands.push_back(createArmMovementCommand(top_left, M_PI, 0.0, M_PI));

    // auto offset =  generateBoxOpeningPose(top_right, 0, 0, 137 * M_PI / 180,
    // 8);
    double roll = M_PI, pitch = 0, yaw = 0.0;

    commands.push_back(
        createArmMovementCommand(top_right + delta1, roll, pitch, yaw));
    commands.push_back(
        createArmMovementCommand(top_left + delta1, roll, pitch, yaw));

    //
    commands.push_back(createArmMovementCommand(
        top_left + tf2::Vector3(0.0, 0.0, 20.0), roll, pitch, yaw));

    //
    yaw = M_PI / 2;
    commands.push_back(createArmMovementCommand(e, roll, pitch, yaw));
    commands.push_back(createArmMovementCommand(d, roll, pitch, yaw));
    //
    commands.push_back(createArmMovementCommand(
        d + tf2::Vector3(0.0, 0.0, 20.0), roll, pitch, yaw));

    yaw = 0;
    //
    commands.push_back(
        createArmMovementCommand(down_right + delta1, roll, pitch, yaw));
    commands.push_back(
        createArmMovementCommand(down_left + delta1, roll, pitch, yaw));

    //
    commands.push_back(createArmMovementCommand(
        down_left + tf2::Vector3(0.0, 0.0, 20.0), roll, pitch, yaw));

  } else {
    command = createArmMovementCommand(d, M_PI, 0.0, M_PI / 2);
  }
  if (pose.isGraspBox()) {
    command.tcp.euler.z = pose.yaw + arm_feedback[index].pose.tcp.euler.z;
  } else if (pose.isGraspKnife()) {
    command.tcp.pos.x += 18;
    command.tcp.pos.y -= 11;
    command.tcp.pos.z -= 16;
    // command.tcp.euler = arm_feedback[index].pose.tcp.euler;
    command.tcp.euler.x = 1.600667119026184;
    command.tcp.euler.y = 0.8211413621902466;
    command.tcp.euler.z = 0.7938387989997864;
  }
  commands.push_back(command);

  RCLCPP_INFO(this->get_logger(), "grasp pose: %s [%f, %f, %f]",
              vectorToString(d).c_str(), command.tcp.euler.x,
              command.tcp.euler.y, command.tcp.euler.z);

  return commands;
}

std::vector<grasp_msgs::msg::ArmCommand>
GraspTaskManager::generateArmPressCommandsForPose(int index, int vision_index,
                                                  const grasp_pose_t& pose) {
  grasp_msgs::msg::ArmCommand command;
  std::vector<grasp_msgs::msg::ArmCommand> commands;
  tf2::Quaternion qua;
  qua.setRPY(pose.roll, pose.pitch, pose.yaw);
  tf2::Vector3 delta = tf2::Vector3(11.4, 1.04, 2.45);
  if (pose.isCutBox()) {
    tf2::Transform camera2left(qua, pose.left_press_pos);
    tf2::Transform camera2right(qua, pose.right_press_pos);
    tf2::Transform tcp2press[ARM_NUMBER];
    tcp2press[LEFT] = transformObjectToTCPFrame(vision_index, camera2left);
    tcp2press[RIGHT] = transformObjectToTCPFrame(vision_index, camera2right);
    tf2::Transform self_base2press_pose =
        toTargetBase(index, !index, tcp2press[index]);
    self_base2press_pose.setOrigin(self_base2press_pose.getOrigin() + delta);

    printTransform(self_base2press_pose);

    command = createArmMovementCommand(
        self_base2press_pose.getOrigin() + tf2::Vector3(0.0, 0.0, 15),
        1.5515958566998687, 0.8202967590417055, -2.3736464631660774);
    commands.push_back(command);
    /////////////////////////////////////////////////////////
    grasp_msgs::msg::ArmCommand new_command = command;
    task_msg_t task;
    new_command.type = new_command.TCP_TYPE;
    new_command.tcp = arm_feedback[index].pose.tcp;
    task.arm = new_command;
    task.ctrl_type = ARM_CTRL;
    task.name = "OpenBox";
    open_box_tasks[index].push_front(task);
    //
    press_box_task_cmd[index].push_back(command);

    task.arm = command;
    task.ctrl_type = ARM_CTRL;
    task.name = "OpenBox";
    open_box_tasks[index].push_front(task);
    ///////////////////////////////////////////////////////////
    command = createArmMovementCommand(
        self_base2press_pose.getOrigin() + tf2::Vector3(0.0, 0.0, -1.0),
        1.5515958566998687, 0.8202967590417055, -2.3736464631660774);

    commands.push_back(command);
  }
  return commands;
}
tf2::Transform GraspTaskManager::generateBoxOpeningPose(
    const tf2::Transform& tcp2tool, double roll, double pitch, double yaw,
    double length) {
  // ()
  double cos_roll = std::cos(roll), sin_roll = std::sin(roll);
  double cos_pitch = std::cos(pitch), sin_pitch = std::sin(pitch);
  double cos_yaw = std::cos(yaw), sin_yaw = std::sin(yaw);

  // ()
  tf2::Vector3 local_direction(
      cos_pitch * cos_yaw, sin_roll * sin_pitch * cos_yaw + cos_roll * sin_yaw,
      cos_roll * sin_pitch * cos_yaw - sin_roll * sin_yaw);

  // 
  tf2::Vector3 world_direction =
      tcp2tool.getBasis() * local_direction;  // 
  world_direction.normalize();                // 

  // 
  tf2::Vector3 tool_position = tcp2tool.getOrigin();  // 

  // 
  tf2::Vector3 endpoint_position = tool_position + world_direction * length;

  return tf2::Transform(tcp2tool.getRotation(), endpoint_position);
}

static tf2::Vector3 computeArcPointZ(const tf2::Vector3& center,
                                     const tf2::Vector3& start_point,
                                     double angle_rad) {
  // 
  tf2::Vector3 radius_vector = start_point - center;

  //  XY  Z  angle_rad
  double cos_angle = std::cos(angle_rad);
  double sin_angle = std::sin(angle_rad);
  double x_new = radius_vector.x() * cos_angle - radius_vector.y() * sin_angle;
  double y_new = radius_vector.x() * sin_angle + radius_vector.y() * cos_angle;

  // Returns
  return center + tf2::Vector3(x_new, y_new, radius_vector.z());
}

static double calculateRadius(const tf2::Vector3& center,
                              const tf2::Vector3& start_point) {
  // Euclidean distance,
  return (start_point - center).length();
}

/**
 * @brief 
 * @param index 
 * @param pose 
 * @return 
 */
std::list<task_msg_t> GraspTaskManager::generateArmOpenCommandsForPose(
    int index, int vision_index, const grasp_pose_t& pose) {
  grasp_msgs::msg::ArmCommand command;
  task_msg_t task;
  task.name = "OpenBox";
  std::list<task_msg_t> commands;
  tf2::Quaternion qua;
  qua.setRPY(pose.roll, pose.pitch, pose.yaw);
  tf2::Transform camera2center(qua, pose.center_pos);
  tf2::Transform tcp2center =
      transformObjectToTCPFrame(vision_index, camera2center);
  tf2::Vector3 delta = tf2::Vector3(5, 7, 2);
  if (pose.isCutBox()) {
    ////////////////////////////////////////////////////////
    tcp2center = toTargetBase(index, vision_index, tcp2center);
    /////////////////////////////////////////////////////////////
    tcp2center.setOrigin(tcp2center.getOrigin() + delta);

    std::cout << "generateArmOpenCommandsForPose: " << std::endl;
    printTransform(tcp2center);
    auto box_opening_pose = generateBoxOpeningPose(
        tcp2center, euler_rotated[index].x(), euler_rotated[index].y(),
        euler_rotated[index].z(), 42);

    printTransform(box_opening_pose);

    /////////////////////////////////////////////////////////////////////////////////
    tf2::Matrix3x3 mat(open_box_qua[index]);
    // Extract Euler angles from the matrix
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);  // roll, pitch, yaw in radians

    // auto box_opening_pose_cmd =
    //     toTargetBase(vision_index, index, box_opening_pose);
    command = createArmMovementCommand(box_opening_pose.getOrigin(), roll,
                                       pitch, yaw);
    task.arm = command;
    task.ctrl_type = ARM_CTRL;
    commands.push_back(task);
    //////////////////////////////////////////////////////////////////////////////
    /// \brief camera2left
    tf2::Transform camera2pos[ARM_NUMBER];
    camera2pos[LEFT] = tf2::Transform(qua, pose.left_pos);
    camera2pos[RIGHT] = tf2::Transform(qua, pose.right_pos);

    tf2::Transform tcp2pos[ARM_NUMBER];
    tf2::Vector3 vec_BC[ARM_NUMBER];
    for (int i = 0; i < 2; i++) {
      tcp2pos[i] = transformObjectToTCPFrame(vision_index, camera2pos[index]);
      /////////////////////////////////////////////////////////////
      tcp2pos[i] = toTargetBase(index, vision_index, tcp2pos[i]);
      /////////////////////////////////////////////////////////////
      tcp2pos[i].setOrigin(tcp2pos[i].getOrigin() + delta);

      vec_BC[i] = tcp2pos[i].getOrigin() - tcp2center.getOrigin();
    }

    //
    tf2::Transform camera2press[ARM_NUMBER];

    camera2press[LEFT] = tf2::Transform(qua, pose.left_press_pos);
    camera2press[RIGHT] = tf2::Transform(qua, pose.right_press_pos);

    tf2::Transform tcp2press[ARM_NUMBER];
    double sum_z = 0.0;
    for (int i = 0; i < 2; i++) {
      tcp2press[i] = transformObjectToTCPFrame(vision_index, camera2press[i]);
      /////////////////////////////////////////////////////////////
      tcp2press[i] = toTargetBase(index, vision_index, tcp2press[i]);
      /////////////////////////////////////////////////////////////
      tcp2press[i].setOrigin(tcp2press[i].getOrigin() + delta);

      sum_z += tcp2press[i].getOrigin().z();
    }
    double newZ = sum_z / 2;
    tcp2center.getOrigin().setZ(newZ);
    tcp2pos[LEFT].getOrigin().setZ(newZ + 2.5);
    tcp2pos[RIGHT].getOrigin().setZ(newZ - 1.5);
    // 
    tf2::Vector3 vector2press[ARM_NUMBER];
    tf2::Vector3 circle_center[ARM_NUMBER];

    for (int i = 0; i < 2; i++) {
      vector2press[i] = tcp2press[i].getOrigin() - tcp2center.getOrigin();
      // 
      circle_center[i] = tcp2press[i].getOrigin() + 0.9 * vector2press[i];
    }

    tf2::Transform start_pos;
    start_pos.setOrigin(box_opening_pose.getOrigin());
    start_pos.setRotation(open_box_qua[index]);
    std::vector<tf2::Transform> trajectorys;

    auto mid_pose = calculateRotatedPose(start_pos, circle_center[index],
                                         vec_BC[index], M_PI / 4);
    // auto mid_pose_cmd = toTargetBase(vision_index, index, mid_pose);

    trajectorys.push_back(mid_pose);

    auto end_pose = calculateRotatedPose(start_pos, circle_center[index],
                                         vec_BC[index], M_PI / 2);
    // auto end_pose_cmd = toTargetBase(vision_index, index, end_pose);

    trajectorys.push_back(end_pose);
    trajectorys[1].setRotation(circle_vel_qua[index]);
    printTransform(trajectorys[0]);
    printTransform(trajectorys[1]);
    ///////////////////////////////////////////////////////////////
    command = createArmArcMovementCommand(trajectorys);
    //
    task.ctrl_type = GRIPPER_CTRL;
    task.gripper.force = 20;
    task.gripper.pos = 500;
    commands.push_back(task);

    task.arm = command;
    task.ctrl_type = ARM_CTRL;
    commands.push_back(task);

    ////////////////////////////////////////////////////
    tf2::Transform new_start = end_pose;
    new_start.setRotation(circle_vel_qua[index]);
    trajectorys.clear();
    mid_pose = calculateRotatedPose(new_start, circle_center[index],
                                    vec_BC[index], M_PI / 4);
    // mid_pose_cmd = toTargetBase(vision_index, index, mid_pose);

    trajectorys.push_back(mid_pose);
    end_pose = calculateRotatedPose(new_start, circle_center[index],
                                    vec_BC[index], M_PI / 2);
    // end_pose_cmd = toTargetBase(vision_index, index, end_pose);
    trajectorys.push_back(end_pose);

    // trajectorys[0].setRotation(circle_vel_qua[index]);
    // trajectorys[1].setRotation(circle_vel_qua[index]);
    printTransform(trajectorys[0]);
    printTransform(trajectorys[1]);
    /////////////////////////////////////////////////////////////
    command = createArmArcMovementCommand(trajectorys);
    task.ctrl_type = GRIPPER_CTRL;
    task.gripper.force = 50;
    task.gripper.pos = 0;
    commands.push_back(task);

    task.arm = command;
    task.ctrl_type = ARM_CTRL;
    commands.push_back(task);
  }
  return commands;
}

grasp_msgs::msg::ArmCommand GraspTaskManager::createArmMovementCommand(
    const tf2::Vector3& position, double euler_x, double euler_y,
    double euler_z) {
  grasp_msgs::msg::ArmCommand command;

  command.type = command.TCP_TYPE;
  command.tcp.pos.x = position.getX();
  command.tcp.pos.y = position.getY();
  command.tcp.pos.z = position.getZ();
  command.tcp.euler.x = euler_x;
  command.tcp.euler.y = euler_y;
  command.tcp.euler.z = euler_z;
  return command;
}

grasp_msgs::msg::ArmCommand GraspTaskManager::createArmArcMovementCommand(
    const std::vector<tf2::Transform>& position) {
  grasp_msgs::msg::ArmCommand command;
  command.type = command.TCP_CIRCLE_TYPE;
  command.tcp = fromTF2Transform(position[1]);
  command.mid_tcp = fromTF2Transform(position[0]);
  return command;
}

grasp_msgs::msg::ArmCommand GraspTaskManager::createCircularMovementCommand(
    const grasp_msgs::msg::TCPPose& mid_pos,
    const grasp_msgs::msg::TCPPose& end_pos) {
  grasp_msgs::msg::ArmCommand command;

  command.type = command.TCP_CIRCLE_TYPE;
  command.tcp = end_pos;
  command.mid_tcp = mid_pos;
  return command;
}

tf2::Transform GraspTaskManager::transformObjectToTCPFrame(
    int index, const tf2::Transform& camera2obj) {
  auto end2tool = TCPPoseToTransformMatrix(arm_feedback[index].pose.tcp);
  tf2::Transform tcp2camera = end2tool * tool2camera[index];
  tf2::Transform tcp2obj = tcp2camera * camera2obj * object_calib[index];
  tf2::Vector3 tcp_transform = tcp2obj.getOrigin();
  tcp_transform += tcp_calib[index];
  return tf2::Transform(camera2obj.getRotation(), tcp_transform);
}

/**
 * @brief ()
 *
 * (),.
 *
 * @param cart
 * ,(,).
 * @return ,.
 */
grasp_msgs::msg::ArmCommand GraspTaskManager::JntToCart(
    int index, const grasp_msgs::msg::TCPPose& cart) {
  grasp_msgs::msg::ArmCommand command;
  command.type = command.JOINT_TYPE;
  auto joints = arm_feedback[index].pose.joint.joint;
  if (!joints.size()) {
    joints.resize(6, 0);
  }
  Eigen::Matrix<double, Eigen::Dynamic, 1> eigen_matrix(joints.size());
  auto init_eigen_matrix = stdVectorToEigen(joints);
  Eigen::Isometry3d desired_pose = toIsometry3d<grasp_msgs::msg::TCPPose>(cart);
  if (!kinematics_->calculateInverseKinematics(desired_pose, eigen_matrix,
                                               init_eigen_matrix)) {
    throw std::runtime_error("Inverse Kinematics failed.");
  }
  command.joint.joint = eigenToStdVector(eigen_matrix);

  return command;
}

/**
 * @brief 
 *
 * ,.
 *
 * @param jnt ,.
 * @return ,.
 */
grasp_msgs::msg::ArmCommand GraspTaskManager::CartToJnt(
    const grasp_msgs::msg::JointPose& jnt) {
  grasp_msgs::msg::ArmCommand command;
  command.type = command.TCP_TYPE;
  auto eigen_matrix = stdVectorToEigen(jnt.joint);
  Eigen::Isometry3d transform;
  if (!kinematics_->calculateForwardKinematics(eigen_matrix, "", transform)) {
    throw std::runtime_error("Forward Kinematics failed.");
  }
  command.tcp = fromIsometry3d<grasp_msgs::msg::TCPPose>(transform);
  return command;
}

void GraspTaskManager::initVariant() {
  for (int i = 0; i < ARM_NUMBER; i++) {
    home_pos[i].type = home_pos[i].JOINT_TYPE;
    home_pos[i].joint.joint.assign(std::begin(home_joint[i]),
                                   std::end(home_joint[i]));
    //
    home_tcp_pos[i].type = home_tcp_pos[i].TCP_TYPE;
    home_tcp_pos[i].tcp = toTCPPoseFromArray(home_tcp[i]);
    ////////////////////////////////////////////////////////////////
    //
    cutbox_vision_home[i].type = cutbox_vision_home[i].TCP_TYPE;
    cutbox_vision_home[i].tcp = toTCPPoseFromArray(cut_box_vision_pose[i]);

    ////////////////////////////////////////////////////////////////
    //
    sorting_vision_home[i].type = sorting_vision_home[i].TCP_TYPE;
    sorting_vision_home[i].tcp = toTCPPoseFromArray(sorting_vision_pose[i]);

    //
    sorting_vision_joint[i].type = sorting_vision_joint[i].JOINT_TYPE;
    sorting_vision_joint[i].joint.joint.assign(std::begin(sorting_joint[i]),
                                               std::end(sorting_joint[i]));

    //
    graspbox_vision_home[i].type = graspbox_vision_home[i].TCP_TYPE;
    graspbox_vision_home[i].tcp = toTCPPoseFromArray(grasp_box_vision_pose[i]);

    //
    sorting_place_pose[i].type = sorting_place_pose[i].TCP_TYPE;
    sorting_place_pose[i].tcp =
        toTCPPoseFromArray(sorting_place_vision_pose[i]);

    //

    pickbox_pose[i].type = pickbox_pose[i].TCP_TYPE;
    pickbox_pose[i].tcp = toTCPPoseFromArray(pick_box_pose[i]);

    drop_box_pos[i].type = drop_box_pos[i].TCP_TYPE;
    drop_box_pos[i].tcp = toTCPPoseFromArray(drop_box_pose[i]);

    //
    graspknife_vision_home[i].type = graspknife_vision_home[i].TCP_TYPE;
    graspknife_vision_home[i].tcp =
        toTCPPoseFromArray(grasp_knife_vision_pose[i]);

    graspknife_vision_joint[i].type = graspknife_vision_joint[i].JOINT_TYPE;
    graspknife_vision_joint[i].joint.joint.assign(
        std::begin(grasp_knife_vision_joint[i]),
        std::end(grasp_knife_vision_joint[i]));
    printArray(grasp_knife_vision_joint[i], 6);

    knife_retreat_joint[i].type = knife_retreat_joint[i].JOINT_TYPE;
    knife_retreat_joint[i].joint.joint.assign(
        std::begin(grasp_knife_retreat_joint[i]),
        std::end(grasp_knife_retreat_joint[i]));

    task_state[i] = TASK_IDLE;
    delay_endpoint_time[i] = this->get_clock()->now();
    arm_command[i].get();
    task_queue[i].clear();
    task_type[i] = TASK_WAIT_DISPATCH;
    goal_pose[i] = home_tcp_pos[i];
    if (!isGraspObjectArm(i)) {
      goal_pose[i] = home_pos[i];
    }
  }

  open_box_qua[LEFT].setRPY(2.77568823928427, -0.3201011841616206,
                            0.03134225891614126);
  open_box_qua[RIGHT].setRPY(-2.706794133013487, 0.36197585591144077,
                             1.6778486254083171);

  circle_vel_qua[LEFT].setRPY(M_PI, 0.0, 0.0);
  circle_vel_qua[RIGHT].setRPY(M_PI, 0.0, M_PI / 2);
  euler_rotated[LEFT] = {0.0, 0.0, M_PI / 4};
  euler_rotated[RIGHT] = {0.0, 0.0, -135 * M_PI / 180};

  pipeline_state_ = InitState;
  open_gripper.pos = 1000;
  open_gripper.force = 50;
  close_gripper.pos = 0;
  close_gripper.force = 100;
  executeDualGrasp();
}

void GraspTaskManager::onTimerCallback() {
  RCLCPP_INFO_ONCE(this->get_logger(), "onTimerCallback...");
  for (int i = 0; i < ARM_NUMBER; i++) {
    queryTaskStatus(i);
  }
  for (int i = 0; i < ARM_NUMBER; i++) {
    processTask(i);
  }
  for (int i = 0; i < ARM_NUMBER; i++) {
    provideTaskFeedback(i);
  }
}

bool GraspTaskManager::isGoalReached(int index,
                                     const grasp_msgs::msg::ArmCommand& goal) {
  double distance = euclidean_distance(arm_feedback[index].pose.tcp, goal.tcp);
  if (distance < EPSILON) {
    return true;
  }
  return false;
}

bool GraspTaskManager::areJointsGoalReached(
    int index, const grasp_msgs::msg::ArmCommand& goal) {
  // 
  if (arm_feedback[index].pose.joint.joint.size() != goal.joint.joint.size() ||
      goal.joint.joint.size() < 1) {
    return false;  // Returns false 
  }

  // Iterate over,
  for (size_t i = 0; i < goal.joint.joint.size(); ++i) {
    double delta =
        arm_feedback[index].pose.joint.joint[i] - goal.joint.joint[i];
    if (fabs(delta) > EPSILON) {
      return false;  // ,Returns false
    }
  }

  return true;  // ,Returns true
}

bool GraspTaskManager::isGraspObjectArm(int index) const {
  if (index == GRASP_OBJECT_ARM) {
    return true;
  }
  return false;
}

void GraspTaskManager::waitHomeTask(int index) {
  if (isGraspObjectArm(index)) {
    if (pipeline_state_ == InitState) {
      updateTaskType(index, TASK_GRASP_OBJECT);
      updateTaskType(!index, TASK_GRASP_KNIFE);
    } else {
      if (pipeline_state_ & DropBoxState) {
        updateTaskType(index, TASK_SORTING_OBJECT);
      } else {
        updateTaskType(index, TASK_WAIT_DISPATCH);
      }
    }
  } else {
    if (pipeline_state_ & ObjectState) {  //, 
      if (pipeline_state_ & KnifeState) {
        if (pipeline_state_ & DropBoxState) {
          updateTaskType(index, TASK_SORTING_OBJECT);
        } else {
          updateTaskType(index, TASK_CUT_BOX);
        }
      } else {
        updateTaskType(index, TASK_GRASP_KNIFE);
      }
    } else {
      updateTaskType(index, TASK_WAIT_DISPATCH);
    }
  }
}
void GraspTaskManager::waitObjectTask(int index) {
  pipeline_state_ |= ObjectState;
  updateTaskType(index, TASK_WAIT_DISPATCH);
  if (pipeline_state_ & KnifeState) {  //, 
    updateTaskType(!index, TASK_CUT_BOX);
  }
}
void GraspTaskManager::waitCutBoxTask(int index) {
  pipeline_state_ |= CutBoxState;
  updateTaskType(index, TASK_PLACE_KNIFE);
}

void GraspTaskManager::waitPressBoxTask(int index) {
  pipeline_state_ |= PressBoxState;
  updateTaskType(index, TASK_WAIT_DISPATCH);
  //
  executeCutBoxTask(!index);
}

void GraspTaskManager::waitKinfeTask(int index) {
  pipeline_state_ |= KnifeState;
  if (pipeline_state_ & ObjectState) {  //, 
    updateTaskType(index, TASK_CUT_BOX);
  } else {
    updateTaskType(index, TASK_WAIT_DISPATCH);
  }
}

void GraspTaskManager::waitPlaceKnifeTask(int index) {
  // pipeline_state_ = InitState;
  // updateTaskType(index, TASK_WAIT_DISPATCH);
  updateTaskType(index, TASK_OPEN_BOX);
}

void GraspTaskManager::waitOpenBoxTask(int index) {
  open_box_tasks[index].clear();
  if (pipeline_state_ & OpenBoxState) {
    pipeline_state_ |= SOpenBoxState;
    updateTaskType(index, TASK_LIFT_BOX);
    updateTaskType(!index, TASK_LIFT_BOX);
  } else {
    updateTaskType(index, TASK_WAIT_DISPATCH);
    updateTaskType(!index, TASK_OPEN_BOX);
  }
  pipeline_state_ |= OpenBoxState;
}

void GraspTaskManager::waitLfitBoxTask(int index) {
  if (pipeline_state_ & LiftBoxState) {
    pipeline_state_ |= SLiftBoxState;
    updateTaskType(index, TASK_FLIPP_BOX);
    updateTaskType(!index, TASK_FLIPP_BOX);
  } else {
    updateTaskType(index, TASK_WAIT_DISPATCH);
  }
  pipeline_state_ |= LiftBoxState;
}

void GraspTaskManager::waitFlippBoxTask(int index) {
  if (pipeline_state_ & FlippBoxState) {
    pipeline_state_ |= SFlippBoxState;
    updateTaskType(index, TASK_DROP_BOX);
    updateTaskType(!index, TASK_DROP_BOX);
  } else {
    updateTaskType(index, TASK_WAIT_DISPATCH);
  }
  pipeline_state_ |= FlippBoxState;
}

void GraspTaskManager::waitDropBoxTask(int index) {
  pipeline_state_ |= DropBoxState;
  if (index == LEFT) {
    updateTaskType(index, TASK_SORTING_OBJECT);
    sorting_number = 0;
  } else {
    updateTaskType(index, TASK_WAIT_DISPATCH);
  }
}

void GraspTaskManager::waitSortingObjectTask(int index) {
  pipeline_state_ |= SortingObjectState;
  updateTaskType(index, TASK_SORTING_OBJECT);
}

void GraspTaskManager::executeDualGrasp() {
  pipeline_state_ = InitState;
  for (int i = 0; i < ARM_NUMBER; i++) {
    updateTaskType(i, TASK_GO_HOME);
  }
}

void GraspTaskManager::dispatchHomeTask(int index) {
  moveTo(index, goal_pose[index]);
  doDelay(index, 3.0);
  updateTaskType(index, TASK_WAIT_HOME);
}

void GraspTaskManager::dispatchGraspObjectTask(int index) {
  if (isGoalReached(index, graspbox_vision_home[index])) {
    if (arm_controller_[index]->isHasGraspPose()) {
      auto msg = arm_controller_[index]->getCurrentGraspPose();
      if (!msg.isGraspBox()) {
        return;
      }
      grasp_msgs::msg::GripperCommand command;
      command.force = 50;
      command.pos = (1.0 - (95 - msg.width) / 95.0) * 1000;
      controlGripper(index, command);

      auto tasks = generateArmCommandsForPose(index, msg);
      performCutBoxOperation(index, tasks);
      closeGripper(index);
      auto pose = tasks.back();
      //50
      pose.tcp.pos.z += 50;
      moveTo(index, pose);
      moveToPlace(index, pickbox_pose[index]);
      auto retreat_pose = pickbox_pose[index];
      retreat_pose.tcp.pos.x += 150;
      retreat_pose.tcp.pos.y -= 150;
      moveTo(index, retreat_pose);
      //, 
      // goal_pose[index] = graspbox_vision_home[index];
      // moveTo(index, goal_pose[index]);
      updateTaskType(index, TASK_WAIT_GRASP_OBJECT);
    }
  } else {
    goal_pose[index] = graspbox_vision_home[index];
    updateTaskType(index, TASK_GO_HOME);
  }
}

void GraspTaskManager::dispatchCutBoxTask(int index) {
  if (isGoalReached(index, cutbox_vision_home[index])) {
    if (arm_controller_[index]->isHasGraspKnifePose()) {
      cut_box_pose = arm_controller_[index]->getCurrentGraspKnifePose();
      if (!cut_box_pose.isCutBox()) {
        return;
      }
      open_box_tasks[index] =
          generateArmOpenCommandsForPose(index, index, cut_box_pose);
      open_box_tasks[!index] =
          generateArmOpenCommandsForPose(!index, index, cut_box_pose);

      //, 
      updateTaskType(index, TASK_WAIT_DISPATCH);
      //
      updateTaskType(!index, TASK_PRESS_BOX);
    }
  } else {
    goal_pose[index] = cutbox_vision_home[index];
    updateTaskType(index, TASK_GO_HOME);
  }
}

void GraspTaskManager::executeCutBoxTask(int index) {
  closeGripper(index);
  auto tasks = generateArmCommandsForPose(index, cut_box_pose);
  performCutBoxOperation(index, tasks);
  goal_pose[index] = graspknife_vision_home[index];
  moveTo(index, goal_pose[index]);
  updateTaskType(index, TASK_WAIT_CUT_BOX);
}

void GraspTaskManager::dispatchPressBoxTask(int index) {
  press_box_task_cmd[index].clear();
  auto tasks = generateArmPressCommandsForPose(index, !index, cut_box_pose);
  for (const auto& task : tasks) {
    moveTo(index, task);
  }
  doDelay(index, 1.0);
  updateTaskType(index, TASK_WAIT_PRESS_BOX);
}

/**
 * @brief 
 * @param index 
 */
void GraspTaskManager::dispatchGraspKnifeTask(int index) {
  if (isGoalReached(index, graspknife_vision_home[index])) {
    if (arm_controller_[index]->isHasGraspKnifePose()) {
      auto msg = arm_controller_[index]->getCurrentGraspKnifePose();
      if (!msg.isGraspKnife()) {
        return;
      }
      place_knife_task_cmd[index].clear();
      // x=-60, y=90
      goKnifeRetreat(index);
      place_knife_task_cmd[index].push_back(knife_retreat_joint[index]);
      openGripper(index);
      //
      auto tasks = generateArmCommandsForPose(index, msg);
      performCutBoxOperation(index, tasks);
      //
      closeGripper(index);

      //
      auto cmd = tasks.back();
      cmd.tcp.pos.x -= 60;
      cmd.tcp.pos.y += 90;
      moveTo(index, cmd);

      //
      cmd.tcp.pos.z += 5;
      place_knife_task_cmd[index].push_back(cmd);
      //
      auto down = tasks.back();
      for (auto task : tasks) {
        down = task;
        down.tcp.pos.z += 5;
        place_knife_task_cmd[index].push_back(down);
      }
      down.tcp.pos.z -= 5;
      place_knife_task_cmd[index].push_back(down);

      cmd.tcp.pos.z -= 5;
      place_knife_task_cmd[index].push_back(cmd);
      //
      cmd.tcp.pos.z += 100;
      moveTo(index, cmd);
      // place_knife_task_cmd[index].push_back(cmd);
      //
      goal_pose[index] = graspknife_vision_joint[index];
      moveTo(index, goal_pose[index]);
      place_knife_task_cmd[index].push_back(goal_pose[index]);
      updateTaskType(index, TASK_WAIT_GRASP_KNIFE);
    }
  } else {
    goal_pose[index] = graspknife_vision_joint[index];
    updateTaskType(index, TASK_GO_HOME);
  }
}

/**
 * @brief 
 * @param index 
 */
void GraspTaskManager::dispatchPlaceKnifeTask(int index) {
  if (isGoalReached(index, graspknife_vision_home[index])) {
    if (pipeline_state_ & CutBoxState) {
      for (size_t i = 0; i < place_knife_task_cmd[index].size(); i++) {
        if (i == place_knife_task_cmd[index].size() - 2) {
          openGripper(index);
        }
        moveTo(index, place_knife_task_cmd[index][i]);
      }
      updateTaskType(index, TASK_WAIT_PLACE_KNIFE);
    }
    place_knife_task_cmd[index].clear();
  }
}

/**
 * @brief 
 * @param index 
 */
void GraspTaskManager::dispatchOpenBoxTask(int index) {
  if (open_box_tasks[index].empty()) {
    updateTaskType(index, TASK_WAIT_DISPATCH);
    return;
  }

  for (const auto& task : open_box_tasks[index]) {
    task_queue[index].push_back(task);
  }

  doDelay(index, 1.0);
  updateTaskType(index, TASK_WAIT_OPEN_BOX);
}

/**
 * @brief 
 * @param index 
 */
void GraspTaskManager::dispatchLiftBoxTask(int index) {
  grasp_msgs::msg::ArmCommand command;
  command.type = command.TCP_TYPE;
  command.tcp = arm_feedback[index].pose.tcp;
  command.tcp.pos.z += 100;
  moveTo(index, command);
  doDelay(index, 1.0);
  updateTaskType(index, TASK_WAIT_LIFT_BOX);
}

/**
 * @brief 
 * @param index 
 */
void GraspTaskManager::dispatchFlippBoxTask(int index) {
  grasp_msgs::msg::ArmCommand command;
  command.type = command.JOINT_TYPE;
  command.joint = arm_feedback[index].pose.joint;
  double delta = M_PI;
  if (index == LEFT) {
    delta *= -1.0;
  }
  command.joint.joint[5] = command.joint.joint[5] + delta;
  moveTo(index, command);
  if (index == LEFT) {
    openGripper(index);
  }
  doDelay(index, 1.0);
  updateTaskType(index, TASK_WAIT_FLIPP_BOX);
}

void GraspTaskManager::dispatchDropBoxTask(int index) {
  if (index == LEFT) {
    // goSortingVisonHome(index);
    moveTo(index, sorting_vision_joint[index]);
  } else {
    moveTo(index, drop_box_pos[index]);
    openGripper(index);
    moveTo(index, graspknife_vision_home[index]);
    goal_pose[index] = graspknife_vision_home[index];
  }

  updateTaskType(index, TASK_WAIT_DROP_BOX);
}

void GraspTaskManager::dispatchSortingObjectTask(int index) {
  if (isGoalReached(index, sorting_vision_home[index])) {
    if (arm_controller_[index]->isHasGraspPose()) {
      auto msg = arm_controller_[index]->getCurrentGraspPose();
      if (!msg.isGraspBox()) {
        return;
      }
      grasp_msgs::msg::GripperCommand command;
      command.force = 50;
      command.pos = (1.0 - (95 - msg.width) / 95.0) * 1000;
      controlGripper(index, command);
      auto tasks = generateArmCommandsForPose(index, msg);
      performCutBoxOperation(index, tasks);
      closeGripper(index);
      auto pose = tasks.back();
      //50
      pose.tcp.pos.z += 50;
      moveTo(index, pose);
      auto place_pose = sorting_place_pose[index];
      if (sorting_number % 2 == 0) {
        place_pose.tcp.pos.x += sorting_number * 30;
        place_pose.tcp.pos.y += sorting_number * 30;
      } else {
        place_pose.tcp.pos.x -= sorting_number * 30;
        place_pose.tcp.pos.y -= sorting_number * 30;
      }
      place_pose.tcp.pos.z = tasks.back().tcp.pos.z + 5;
      sorting_number++;
      moveToPlace(index, place_pose);
      auto retreat_pose = place_pose;
      retreat_pose.tcp.pos.z += 50;
      moveTo(index, retreat_pose);
      goSortingVisonHome(index);
      updateTaskType(index, TASK_WAIT_SORTING_OBJECT);
    }
  } else {
    goal_pose[index] = sorting_vision_home[index];
    updateTaskType(index, TASK_GO_HOME);
  }
}

bool GraspTaskManager::isTaskInProgress(int index) {
  if (task_state[index] != TASK_IDLE) {
    return true;
  }
  return false;
}
void GraspTaskManager::updateTaskType(int index, const TaskType& type) {
  if (type != task_type[index]) {
    RCLCPP_INFO(this->get_logger(), "[%s][%s][%s]",
                IndexToString(index).c_str(),
                TaskTypeToStr(task_type[index]).c_str(),
                TaskTypeToStr(type).c_str());
    task_type[index] = type;
    if (arm_controller_[index]) {
      arm_controller_[index]->resetDataFilter();
    }
  }
}

void GraspTaskManager::waitDispatchTask(int index) {}

void GraspTaskManager::dispatchTask(int index) {
  switch (task_type[index]) {
    case TASK_WAIT_DISPATCH:
      waitDispatchTask(index);
      break;
    case TASK_GO_HOME:
      dispatchHomeTask(index);
      break;
    case TASK_GRASP_OBJECT:
      dispatchGraspObjectTask(index);
      break;
    case TASK_GRASP_KNIFE:
      dispatchGraspKnifeTask(index);
      break;
    case TASK_CUT_BOX:
      dispatchCutBoxTask(index);
      break;
    case TASK_PRESS_BOX:
      dispatchPressBoxTask(index);
      break;
    case TASK_PLACE_KNIFE:
      dispatchPlaceKnifeTask(index);
      break;
    case TASK_OPEN_BOX:
      dispatchOpenBoxTask(index);
      break;
    case TASK_LIFT_BOX:
      dispatchLiftBoxTask(index);
      break;
    case TASK_FLIPP_BOX:
      dispatchFlippBoxTask(index);
      break;
    case TASK_DROP_BOX:
      dispatchDropBoxTask(index);
      break;
    case TASK_SORTING_OBJECT:
      dispatchSortingObjectTask(index);
      break;
    case TASK_WAIT_HOME:
      waitHomeTask(index);
      break;
    case TASK_WAIT_GRASP_OBJECT:
      waitObjectTask(index);
      break;
    case TASK_WAIT_GRASP_KNIFE:
      waitKinfeTask(index);
      break;
    case TASK_WAIT_CUT_BOX:
      waitCutBoxTask(index);
      break;
    case TASK_WAIT_PRESS_BOX:
      waitPressBoxTask(index);
      break;
    case TASK_WAIT_PLACE_KNIFE:
      waitPlaceKnifeTask(index);
      break;
    case TASK_WAIT_OPEN_BOX:
      waitOpenBoxTask(index);
      break;
    case TASK_WAIT_LIFT_BOX:
      waitLfitBoxTask(index);
      break;
    case TASK_WAIT_FLIPP_BOX:
      waitFlippBoxTask(index);
      break;
    case TASK_WAIT_DROP_BOX:
      waitDropBoxTask(index);
      break;
    case TASK_WAIT_SORTING_OBJECT:
      waitSortingObjectTask(index);
      break;
    default:
      break;
  }
}

void GraspTaskManager::queryTaskStatus(int index) {
  if (isTaskInProgress(index)) {
    return;
  }

  if (areAllTasksCompleted(index)) {
    dispatchTask(index);
  }

  executeNextTask(index);
}

void GraspTaskManager::executeNextTask(int index) {
  if (!task_queue[index].empty()) {
    if (isTaskFinished(index)) {
      auto task = task_queue[index].front();
      arm_command[index].set(task);
    }
  }
}

void GraspTaskManager::provideTaskFeedback(int index) {
  arm_feedback[index] = arm_controller_[index]->getArmFeedback();
  gripper_feedback[index] = arm_controller_[index]->getGripperFeedback();
}

bool GraspTaskManager::areAllTasksCompleted(int index) {
  if (isTaskFinished(index) && task_queue[index].empty()) {
    return true;
  }

  return false;
}

bool GraspTaskManager::isTaskFinished(int index) {
  if (isArmMotionCompleted(index) && isGripperActionCompleted(index) &&
      isDelayCompleted(index)) {
    if (!arm_command[index].check()) return true;
  }
  return false;
}

bool GraspTaskManager::isArmMotionCompleted(int index) const {
  return !arm_feedback[index].isRunning;
}

bool GraspTaskManager::isGripperActionCompleted(int index) const {
  return !gripper_feedback[index].isRunning;
}

bool GraspTaskManager::isDelayCompleted(int index) {
  return this->get_clock()->now() > delay_endpoint_time[index];
}

void GraspTaskManager::handleTaskIdleState(int index) {
  // 
  if (arm_command[index].check()) {
    bool taskShouldBePopped = true;
    auto taskCommand = arm_command[index].get();
    RCLCPP_INFO(this->get_logger(),
                "[%s]ArmCtrl: %d, GripperCtrl: %d, DelayCtrl: %d",
                IndexToStr(index).c_str(), taskCommand.isArmCtrl(),
                taskCommand.isGripperCtrl(), taskCommand.isDelayCtrl());
    if (taskCommand.isArmCtrl()) {
      if (taskCommand.arm.type == taskCommand.arm.JOINT_TYPE) {
        RCLCPP_INFO(this->get_logger(), "[%s]ArmType: %d, Joint Size: %lu",
                    IndexToStr(index).c_str(), taskCommand.arm.type,
                    taskCommand.arm.joint.joint.size());
      }
    }

    // 
    current_task[index].task = taskCommand;
    current_task[index].completed = false;
    current_task[index].success = false;

    // 
    if (taskCommand.isGripperCtrl()) {
      if (!executeGripperControl(index, taskCommand.gripper)) {
        taskShouldBePopped &= false;  // ,
      }
    }

    // 
    if (taskCommand.isArmCtrl()) {
      if (!executeArmControl(index, taskCommand.arm)) {
        taskShouldBePopped &= false;  // ,
      }
    }

    // 
    if (taskCommand.isDelayCtrl()) {
      if (!executeDelay(index, taskCommand.delay_time)) {
        taskShouldBePopped &= false;  // ,
      }
    }

    // ,
    if (taskShouldBePopped) {
      popTaskFromQueue(index);
    }
  }
}

void GraspTaskManager::popTaskFromQueue(int index) {
  if (!task_queue[index].empty()) {
    task_queue[index].pop_front();
    RCLCPP_INFO(this->get_logger(), "[%s]: %lu",
                IndexToStr(index).c_str(), task_queue[index].size());
  }
  updateTaskStatus(index, TASK_EXECUTING);
}

void GraspTaskManager::handleTaskExecution(int index) {
  if (isTaskFinished(index)) {
    current_task[index].completed = true;
    current_task[index].success = arm_controller_[index]->isSuccessed();
    if (!current_task[index].success) {
      RCLCPP_ERROR(this->get_logger(), "[%s]: [%s]",
                   IndexToStr(index).c_str(),
                   current_task[index].task.name.c_str());
    }
    updateTaskStatus(index, TASK_IDLE);
  }
}

void GraspTaskManager::updateTaskStatus(int index, const TaskState& state) {
  if (state != task_state[index]) {
    RCLCPP_WARN(this->get_logger(), "[%s][%d][%d]",
                IndexToString(index).c_str(), task_state[index], state);
    task_state[index] = state;
    if (arm_controller_[index]) {
      arm_controller_[index]->resetDataFilter();
    }
  }
}

void GraspTaskManager::processTask(int index) {
  RCLCPP_INFO_ONCE(this->get_logger(), "processTask...");
  switch (task_state[index]) {
    case TASK_IDLE:
      handleTaskIdleState(index);
      break;
    case TASK_EXECUTING:
      handleTaskExecution(index);
      break;
    default:
      break;
  }
}

void GraspTaskManager::performCutBoxOperation(
    int index, const std::vector<grasp_msgs::msg::ArmCommand>& msgs) {
  for (const auto& msg : msgs) {
    moveTo(index, msg);
  }
}

void GraspTaskManager::performGraspBoxOperation(
    int index, const std::vector<grasp_msgs::msg::ArmCommand>& msgs) {
  for (const auto& msg : msgs) {
    moveTo(index, msg);
  }
  doDelay(index, 1.0);
  moveArmToCutBoxHome(index);
}

void GraspTaskManager::moveArmToCutBoxHome(int index) {
  goVisonHome(index);
  doDelay(index, 5.0);
}

void GraspTaskManager::moveArmToGraspBoxHome(int index) {
  goVisonBoxHome(index);
  doDelay(index, 5.0);
}

void GraspTaskManager::closeGripper(int index) {
  task_msg_t task;
  task.name = "closeGripper";
  task.ctrl_type = GRIPPER_CTRL;
  task.gripper = close_gripper;
  task_queue[index].push_back(task);
}
void GraspTaskManager::openGripper(int index) {
  task_msg_t task;
  task.name = "openGripper";
  task.ctrl_type = GRIPPER_CTRL;
  task.gripper = open_gripper;
  task_queue[index].push_back(task);
}

void GraspTaskManager::controlGripper(
    int index, const grasp_msgs::msg::GripperCommand& command) {
  task_msg_t task;
  task.name = "controlGripper";

  task.ctrl_type = GRIPPER_CTRL;
  task.gripper = command;
  task_queue[index].push_back(task);
}

void GraspTaskManager::moveTo(int index,
                              const grasp_msgs::msg::ArmCommand& command) {
  task_msg_t task;
  task.name = "moveTo";
  task.ctrl_type = ARM_CTRL;
  task.arm = command;
  task_queue[index].push_back(task);
}

void GraspTaskManager::moveToCloseGripper(
    int index, const grasp_msgs::msg::ArmCommand& command) {
  task_msg_t task;
  task.name = "moveToCloseGripper";
  task.ctrl_type = ARM_GRIPPER_CTRL;
  task.arm = command;
  task.gripper = close_gripper;
  task_queue[index].push_back(task);
}

void GraspTaskManager::moveToPlace(int index,
                                   const grasp_msgs::msg::ArmCommand& command) {
  task_msg_t task;
  task.name = "moveToPlace[ARM]";
  task.ctrl_type = ARM_CTRL;
  task.arm = command;
  task_queue[index].push_back(task);

  task.name = "moveToPlace[GRIPPER]";
  task.ctrl_type = GRIPPER_CTRL;
  task.gripper = open_gripper;
  task_queue[index].push_back(task);
}

void GraspTaskManager::moveToGrasp(int index,
                                   const grasp_msgs::msg::ArmCommand& command) {
  task_msg_t task;
  task.name = "moveToGrasp[ARM]";

  task.ctrl_type = ARM_CTRL;
  task.arm = command;
  task_queue[index].push_back(task);
  task.name = "moveToGrasp[GRIPPER]";

  task.ctrl_type = GRIPPER_CTRL;
  task.gripper = close_gripper;
  task_queue[index].push_back(task);
}

void GraspTaskManager::goHome(int index) {
  task_msg_t task;
  task.name = "goHome";
  task.ctrl_type = ARM_CTRL;
  task.arm = home_pos[index];
  task_queue[index].push_back(task);
}

void GraspTaskManager::goVisonHome(int index) {
  task_msg_t task;
  task.ctrl_type = ARM_CTRL;
  task.arm = cutbox_vision_home[index];
  task_queue[index].push_back(task);
}

void GraspTaskManager::goSortingVisonHome(int index) {
  task_msg_t task;
  task.ctrl_type = ARM_CTRL;
  task.arm = sorting_vision_home[index];
  task_queue[index].push_back(task);
}

void GraspTaskManager::goVisonBoxHome(int index) {
  task_msg_t task;
  task.name = "goVisonBoxHome";

  task.ctrl_type = ARM_CTRL;
  task.arm = graspbox_vision_home[index];
  task_queue[index].push_back(task);
}

void GraspTaskManager::goKnifeHome(int index) {
  task_msg_t task;
  task.name = "goKnifeHome";
  task.ctrl_type = ARM_CTRL;
  task.arm = graspknife_vision_joint[index];
  task_queue[index].push_back(task);
}

void GraspTaskManager::goKnifeRetreat(int index) {
  task_msg_t task;
  task.name = "goKnifeRetreat";
  task.ctrl_type = ARM_CTRL;
  task.arm = knife_retreat_joint[index];
  task_queue[index].push_back(task);
}

void GraspTaskManager::doDelay(int index, double seconds) {
  task_msg_t task;
  task.name = "doDelay";
  task.ctrl_type = DELAY_CTRL;
  task.delay_time = seconds;
  task_queue[index].push_back(task);
}

bool GraspTaskManager::executeArmControl(
    int index, const grasp_msgs::msg::ArmCommand& poses) {
  std::vector<grasp_msgs::msg::ArmCommand> poses_vector;
  poses_vector.push_back(poses);
  return arm_controller_[index]->executeArm(poses_vector);
}
bool GraspTaskManager::executeGripperControl(
    int index, const grasp_msgs::msg::GripperCommand& msg) {
  return arm_controller_[index]->executeGripper(msg);
}

bool GraspTaskManager::executeDelay(int index, double senconds) {
  delay_endpoint_time[index] =
      this->get_clock()->now() + rclcpp::Duration::from_seconds(senconds);
  return true;
}
