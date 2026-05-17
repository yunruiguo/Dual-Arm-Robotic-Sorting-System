#ifndef GRASP_COMMON_H
#define GRASP_COMMON_H

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>  // for Eigen::Isometry3d
#include <grasp_msgs/msg/arm_command.hpp>
#include <grasp_msgs/msg/gripper_command.hpp>
#include <grasp_util/safe_data_class.hpp>

#define ARM_NUMBER 2

typedef struct ArmFeedbackMsg_ {
  grasp_msgs::msg::ArmCommand pose;
  float distance_remaining;
  bool isRunning;
  bool success;
  ArmFeedbackMsg_() { reset(); }

  void reset() {
    distance_remaining = 0.0;
    isRunning = false;
    success = false;
  }
} ArmFeedbackMsg;

typedef struct GripperFeedbackMsg_ {
  grasp_msgs::msg::GripperCommand pose;
  float distance_remaining;
  bool isRunning;
  bool success;
  GripperFeedbackMsg_() { reset(); }

  void reset() {
    distance_remaining = 0.0;
    isRunning = false;
    success = false;
  }
} GripperFeedbackMsg;

typedef enum {
  GRASP_UNDEFINED = 0x00,
  GRASP_BOX = 0x01,
  GRASP_CUT_BOX = 0x02,
  GRASP_KNIFE = 0x03,
} GraspType;

typedef struct grasp_pose_ {
  tf2::Vector3 center_pos;
  tf2::Vector3 left_pos;
  tf2::Vector3 right_pos;
  tf2::Vector3 left_press_pos;
  tf2::Vector3 right_press_pos;

  double width;
  double roll;
  double pitch;
  double yaw;
  GraspType grasp_type;
  bool isGraspBox() const { return grasp_type == GRASP_BOX; }
  bool isCutBox() const { return grasp_type == GRASP_CUT_BOX; }
  bool isGraspKnife() const { return grasp_type == GRASP_KNIFE; }
  grasp_pose_() { grasp_type = GRASP_UNDEFINED; }

} grasp_pose_t;

typedef struct {
  int index;
  grasp_msgs::msg::ArmCommand command;
} CtrlCommand;

typedef struct {
  CtrlCommand ctrl[ARM_NUMBER];
} ArmCtrlCommand;

typedef enum {
  ARM_CTRL = 0x01,
  GRIPPER_CTRL = 0x02,
  ARM_GRIPPER_CTRL = 0x03,
  DELAY_CTRL = 0x04,
} CtrlType;

typedef enum {
  TASK_WAIT_DISPATCH = 0x00,
  TASK_GO_HOME = 0x01,
  TASK_GRASP_OBJECT = 0x02,
  TASK_GRASP_KNIFE = 0x03,
  TASK_CUT_BOX = 0x04,
  TASK_PRESS_BOX = 0x05,
  TASK_PLACE_KNIFE = 0x06,
  TASK_OPEN_BOX = 0x07,
  TASK_LIFT_BOX = 0x08,
  TASK_FLIPP_BOX = 0x09,
  TASK_DROP_BOX = 0x0A,
  TASK_SORTING_OBJECT = 0x0B,

  TASK_WAIT_HOME = 0x10,
  TASK_WAIT_GRASP_OBJECT = 0x11,
  TASK_WAIT_GRASP_KNIFE = 0x12,
  TASK_WAIT_CUT_BOX = 0x13,
  TASK_WAIT_PRESS_BOX = 0x14,
  TASK_WAIT_PLACE_KNIFE = 0x15,
  TASK_WAIT_OPEN_BOX = 0x16,
  TASK_WAIT_LIFT_BOX = 0x17,
  TASK_WAIT_FLIPP_BOX = 0x18,
  TASK_WAIT_DROP_BOX = 0x19,
  TASK_WAIT_SORTING_OBJECT = 0x1A,

} TaskType;

typedef enum {
  InitState = 0x0000,
  ObjectState = 0x0001,
  KnifeState = 0x0002,
  CutBoxState = 0x0004,
  PressBoxState = 0x0008,
  PlaceKnifeState = 0x0010,
  OpenBoxState = 0x0020,
  SOpenBoxState = 0x0040,
  LiftBoxState = 0x0080,
  SLiftBoxState = 0x0100,
  FlippBoxState = 0x0200,
  SFlippBoxState = 0x0400,
  DropBoxState = 0x0800,
  SortingObjectState = 0x1000,
} PipelineState;

typedef struct TASK_MSG {
  std::string name;
  grasp_msgs::msg::ArmCommand arm;
  grasp_msgs::msg::GripperCommand gripper;
  double delay_time;
  CtrlType ctrl_type;
  bool isArmCtrl() const { return ctrl_type & ARM_CTRL; }
  bool isGripperCtrl() const { return ctrl_type & GRIPPER_CTRL; }
  bool isDelayCtrl() const { return ctrl_type & DELAY_CTRL; }
  TASK_MSG() { ctrl_type = ARM_CTRL; }
} task_msg_t;

typedef struct {
  task_msg_t task;
  bool completed;
  bool success;
} current_task_t;

const std::string task_type_name[27] = {
    "WaitDispatch",
    "goHome",
    "graspObject",
    "graspKnife",
    "cutBox",
    "PressBox",
    "PlaceKnife",
    "OpenBox",
    "LiftBox",
    "FlippBox",
    "DropBox",
    "SortingObject"
    "",
    "",
    "",
    "",
    "",
    "WaitHome",
    "WaitObject",
    "WaitKnife",
    "WaitCutBox",
    "WaitPressBox",
    "WaitPlaceKnife",
    "WaitOpenBox",
    "WaitLiftBox",
    "WaitFlippBox",
    "WaitDropBox",
    "WaitSortingObject",

};

typedef enum {
  TASK_IDLE = 0,
  TASK_EXECUTING = 1,
} TaskState;

typedef enum {
  LEFT = 0,
  RIGHT = 1,
} RobotIndex;

typedef struct {
  bool valid;
  double fx;
  double fy;
  double cx;
  double cy;
} camera_intrinsic_t;

// const float home_joint[2][6] = {
//     {-51.271 * M_PI / 180, 78.136 * M_PI / 180, -48.332 * M_PI / 180,
//      92.215 * M_PI / 180, 123.477 * M_PI / 180, -221.409 * M_PI / 180},
//     {-120 * M_PI / 180, 84 * M_PI / 180, 76 * M_PI / 180, 86 * M_PI / 180,
//      -130 * M_PI / 180, -222 * M_PI / 180}};

const float home_joint[2][6] = {
    {-1.5334371899265402, 1.6282614084505451, -0.9312822455289568,
     0.9111497088136412, -3.9276956813275476, 1.6236051848759416},
    {230.416 * M_PI / 180, 105.559 * M_PI / 180, 38.741 * M_PI / 180,
     93.193 * M_PI / 180, -123.021 * M_PI / 180, -229.462 * M_PI / 180}};

// const double home_tcp[2][6] = {
//     {-266.311, -738.428, -79.614, -179.991 * M_PI / 180, -0.006 * M_PI / 180,
//      -0.001 * M_PI / 180},
//     {230.808, -641.856, -57.170, -177.014 * M_PI / 180, 0.285 * M_PI / 180,
//      94.386 * M_PI / 180}};

const double home_tcp[2][6] = {
    {-320.978, -569.607, 82.384, 3.1416, 0, 0},  // grasp object
    {264.8658142089844, -759.2849731445312, -68.98348236083984,
     3.141572952270508, -8.742425052332692e-06,
     1.5707879066467285}};  // grasp knife

const double target_tcp[2][6] = {
    {-409.0, -666.0, 0.0, -180 * M_PI / 180, 0.0, -90 * M_PI / 180},
    {400.0, -600.0, 0.0, -180.0 * M_PI / 180, 0.0, 90 * M_PI / 180}};

const double cut_box_vision_pose[2][6] = {
    {-594.516, -324.794, 2.291, 3.1416, 0, 0.0},
    {517.683, -213.007, 60, 3.1416, 0, 1.5808}};

const double sorting_vision_pose[2][6] = {
    {-594.0, -324.0, 2.29, 3.1416, 0, 0.0},
    {517.683, -213.007, 60, 3.1416, 0, 1.5808}};

const double sorting_joint[2][6] = {
    {-100.193 * M_PI / 180, 62.105 * M_PI / 180, -56.327 * M_PI / 180,
     74.276 * M_PI / 180, -225.897 * M_PI / 180, 75.733 * M_PI / 180},
    {517.683, -213.007, 60, 3.1416, 0, 1.5808}};

const double grasp_box_vision_pose[2][6] = {
    {-320.978, -569.607, 82.384, 3.1416, 0, 0},
    {320.978, -569.607, 82.384, 3.1416, 0, 90 * M_PI / 180}};

const double sorting_place_vision_pose[2][6] = {
    {-320.978, -569.607, 82.384, 3.1416, 0, 0},
    {320.978, -569.607, 82.384, 3.1416, 0, 90 * M_PI / 180}};

const double drop_box_pose[2][6] = {
    {-320.978, -569.607, 82.384, 3.1416, 0, 0},
    {-192.4608154296875, -734.35546875, -108.88797760009766,
     -3.1163370609283447, 0.027088064700365067, 0.7445746064186096}};

//
const double grasp_knife_vision_pose[2][6] = {
    {-267.48, -602.835, 89.862, 3.1416, 0, 0},
    {264.8658142089844, -759.2849731445312, -68.98348236083984,
     3.141572952270508, -8.742425052332692e-06, 1.5707879066467285}};

const double grasp_knife_vision_joint[2][6] = {
    {239.742 * M_PI / 180, 84.156 * M_PI / 180, 78.854 * M_PI / 180,
     80.246 * M_PI / 180, -127.645 * M_PI / 180, -219.523 * M_PI / 180},
    {230.416 * M_PI / 180, 105.559 * M_PI / 180, 38.741 * M_PI / 180,
     93.193 * M_PI / 180, -123.021 * M_PI / 180, -229.462 * M_PI / 180}};

//
const double grasp_knife_retreat_joint[2][6] = {
    {203.044 * M_PI / 180, 106.140 * M_PI / 180, 110.841 * M_PI / 180,
     109.827 * M_PI / 180, -295.966 * M_PI / 180, -65.594 * M_PI / 180},
    {193.873 * M_PI / 180, 105.250 * M_PI / 180, 104.976 * M_PI / 180,
     114.087 * M_PI / 180, -304.202 * M_PI / 180, -57.817 * M_PI / 180}};

const double pick_box_pose[2][6] = {
    {-582, -258.271, -223.237, -127.122 * M_PI / 180, 34.834 * M_PI / 180,
     18.532 * M_PI / 180},
    {-582, -258.271, -223.237, -127.122 * M_PI / 180, 34.834 * M_PI / 180,
     18.532 * M_PI / 180}};

const double test_joint[2][6] = {
    {-202.771 * M_PI / 180, 88.415 * M_PI / 180, 98.134 * M_PI / 180,
     37.065 * M_PI / 180, -82.896 * M_PI / 180, -161.047 * M_PI / 180},
    {-202.771 * M_PI / 180, 88.415 * M_PI / 180, 98.134 * M_PI / 180,
     37.065 * M_PI / 180, -82.896 * M_PI / 180, -161.047 * M_PI / 180}};

inline std::string IndexToString(int index) {
  std::string name = "";
  if (index == 1) {
    name = "";
  }
  return name;
}

inline std::string IndexToStr(int index) {
  std::string name = "left";
  if (index == 1) {
    name = "right";
  }
  return name;
}

inline std::string TaskTypeToStr(TaskType type) {
  std::string name = "UnKown";
  if ((type > TASK_SORTING_OBJECT && type < TASK_WAIT_HOME) ||
      type < TASK_WAIT_DISPATCH || type > TASK_WAIT_SORTING_OBJECT) {
    return name;
  }
  name = task_type_name[int(type)];

  return name;
}

inline grasp_msgs::msg::TCPPose toTCPPoseFromArray(const double value[6]) {
  grasp_msgs::msg::TCPPose tcp;
  tcp.pos.x = value[0];
  tcp.pos.y = value[1];
  tcp.pos.z = value[2];
  tcp.euler.x = value[3];
  tcp.euler.y = value[4];
  tcp.euler.z = value[5];
  return tcp;
}

inline camera_intrinsic_t toCameraIntrinsicFromArray(
    const std::vector<double>& values) {
  camera_intrinsic_t intrinsic;
  intrinsic.fx = values[0];
  intrinsic.cx = values[2];
  intrinsic.fy = values[4];
  intrinsic.cy = values[5];

  return intrinsic;
}

inline std::string vectorToString(const tf2::Vector3& vector) {
  std::string data;
  data = "[" + std::to_string(vector.x()) + ", " + std::to_string(vector.y()) +
         ", " + std::to_string(vector.z()) + "]";
  return data;
}

inline double euclidean_distance(const grasp_msgs::msg::TCPPose& pos1,
                                 const grasp_msgs::msg::TCPPose& pos2) {
  double dx = pos1.pos.x - pos2.pos.x;
  double dy = pos1.pos.y - pos2.pos.y;
  double dz = pos1.pos.z - pos2.pos.z;

  return std::hypot(dx, dy, dz);
}

inline bool getTransform(
    const std::string& target_frame, const std::string& source_frame,
    tf2_ros::Buffer& tf_buffer,
    geometry_msgs::msg::TransformStamped& transform_stamped) {
  try {
    transform_stamped = tf_buffer.lookupTransform(target_frame, source_frame,
                                                  tf2::TimePointZero,
                                                  tf2::durationFromSec(1.0));
    return true;
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(rclcpp::get_logger("grasp_decision_node"),
                "exception[%s][%s]: %s", source_frame.c_str(),
                target_frame.c_str(), ex.what());
  }
  return false;  // Returnsfalse
}

inline tf2::Transform createTransformFromPose(
    const std::vector<double>& grasppose) {
  if (grasppose.size() != 16) {
    tf2::Transform transform;
    transform.setIdentity();
    return transform;
  }
  // ( 3 )
  tf2::Vector3 position(grasppose[3], grasppose[7], grasppose[11]);

  // (3x3 , 0  8 )
  tf2::Matrix3x3 rotation(grasppose[0], grasppose[1], grasppose[2],
                          grasppose[4], grasppose[5], grasppose[6],
                          grasppose[8], grasppose[9], grasppose[10]);

  // Create the transform object
  tf2::Transform transform(rotation, position);

  return transform;
}

inline tf2::Transform poseToTransformMatrix(
    const std::vector<double>& cartesian_pose) {
  // Extract position (x, y, z) and rotation (rx, ry, rz) from the input
  double x = cartesian_pose[0];
  double y = cartesian_pose[1];
  double z = cartesian_pose[2];
  double rx = cartesian_pose[3];
  double ry = cartesian_pose[4];
  double rz = cartesian_pose[5];

  // Create the rotation matrix using Euler angles (roll, pitch, yaw)
  tf2::Matrix3x3 rotation;
  rotation.setRPY(rx, ry, rz);  // Set rotation from Euler angles (rx, ry, rz)

  // Create the translation vector
  tf2::Vector3 translation(x, y, z);

  // Create the transformation matrix (translation + rotation)
  tf2::Transform transform(rotation, translation);

  return transform;
}

inline tf2::Transform TCPPoseToTransformMatrix(
    const grasp_msgs::msg::TCPPose& cartesian_pose) {
  // Create the rotation matrix using Euler angles (roll, pitch, yaw)
  tf2::Matrix3x3 rotation;
  rotation.setRPY(
      cartesian_pose.euler.x, cartesian_pose.euler.y,
      cartesian_pose.euler.z);  // Set rotation from Euler angles (rx, ry, rz)

  // Create the translation vector
  tf2::Vector3 translation(cartesian_pose.pos.x, cartesian_pose.pos.y,
                           cartesian_pose.pos.z);

  // Create the transformation matrix (translation + rotation)
  tf2::Transform transform(rotation, translation);

  return transform;
}

inline std::vector<double> transformToPoseAndEulerAngles(
    const tf2::Transform& transform) {
  std::vector<double> cartesian_pose(6);

  // Extract translation (x, y, z)
  tf2::Vector3 translation = transform.getOrigin();
  cartesian_pose[0] = translation.x();
  cartesian_pose[1] = translation.y();
  cartesian_pose[2] = translation.z();

  // Extract rotation matrix and convert to Euler angles (roll, pitch, yaw)
  tf2::Matrix3x3 rotation = transform.getBasis();
  double roll, pitch, yaw;
  rotation.getRPY(roll, pitch, yaw);

  cartesian_pose[3] = roll;
  cartesian_pose[4] = pitch;
  cartesian_pose[5] = yaw;

  return cartesian_pose;
}

// Function to convert Eigen::Isometry3d to tf2::Transform
inline tf2::Transform eigenToTF2(const Eigen::Isometry3d& eigen_transform) {
  // Extract translation from Eigen::Isometry3d
  tf2::Vector3 tf2_translation(eigen_transform.translation().x(),
                               eigen_transform.translation().y(),
                               eigen_transform.translation().z());

  // Extract rotation from Eigen::Isometry3d (convert Eigen quaternion to tf2
  // quaternion)
  Eigen::Quaterniond eigen_rotation(eigen_transform.rotation());
  tf2::Quaternion tf2_rotation(eigen_rotation.x(), eigen_rotation.y(),
                               eigen_rotation.z(), eigen_rotation.w());

  // Create a tf2::Transform and set the translation and rotation
  tf2::Transform tf2_transform;
  tf2_transform.setOrigin(tf2_translation);
  tf2_transform.setRotation(tf2_rotation);

  return tf2_transform;
}

inline std::vector<float> eigenToStdVector(
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& eigen_matrix) {
  // std::vector<float>
  std::vector<float> result(eigen_matrix.size());

  // Eigenstd::vector
  for (int i = 0; i < eigen_matrix.size(); ++i) {
    result[i] = static_cast<float>(eigen_matrix(i));  // float
  }

  return result;
}

inline Eigen::Matrix<double, Eigen::Dynamic, 1> stdVectorToEigen(
    const std::vector<float>& vec) {
  // Eigen::Matrix,std::vector
  Eigen::Matrix<double, Eigen::Dynamic, 1> eigen_matrix(vec.size());

  // std::vectorEigen::Matrix
  for (size_t i = 0; i < vec.size(); ++i) {
    eigen_matrix(i) = static_cast<double>(vec[i]);  // double
  }

  return eigen_matrix;
}

template <class CommandType>
Eigen::Isometry3d toIsometry3d(const CommandType& command) {
  // Eigen::Isometry3d
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  // Set (translation)
  transform.translation() =
      Eigen::Vector3d(command.pos.x, command.pos.y, command.pos.z);

  // Set (rotation) 
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix =
      Eigen::AngleAxisd(command.euler.x, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(command.euler.y, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(command.euler.z, Eigen::Vector3d::UnitZ());

  transform.rotate(rotation_matrix);

  return transform;
}

template <class CommandType>
CommandType fromIsometry3d(const Eigen::Isometry3d& transform) {
  CommandType command;
  // 
  command.pos.x = transform.translation().x();
  command.pos.y = transform.translation().y();
  command.pos.z = transform.translation().z();

  //  (ZYX)
  Eigen::Vector3d euler_angles =
      transform.rotation().eulerAngles(0, 1, 2);  // ZYX
  command.euler.x = euler_angles[0];              // Roll
  command.euler.y = euler_angles[1];              // Pitch
  command.euler.z = euler_angles[2];              // Yaw
  return command;
}

inline grasp_msgs::msg::TCPPose fromTF2Transform(
    const tf2::Transform& transform) {
  grasp_msgs::msg::TCPPose pose;
  pose.pos.x = transform.getOrigin().getX();
  pose.pos.y = transform.getOrigin().getY();
  pose.pos.z = transform.getOrigin().getZ();
  // (Quaternion)
  tf2::Quaternion rotation = transform.getRotation();

  //  Matrix3x3  Quaternion 
  tf2::Matrix3x3 mat(rotation);

  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  pose.euler.x = roll;
  pose.euler.y = pitch;
  pose.euler.z = yaw;
  return pose;
}

inline void printTransform(const tf2::Transform& transform) {
  // (Quaternion)
  tf2::Quaternion rotation = transform.getRotation();

  //  Matrix3x3  Quaternion 
  tf2::Matrix3x3 mat(rotation);

  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);

  // (Vector3)
  tf2::Vector3 translation = transform.getOrigin();

  // ()
  std::cout << "Rotation (Euler angles): "
            << "Roll = " << roll * 180 / M_PI << ", "
            << "Pitch = " << pitch * 180 / M_PI << ", "
            << "Yaw = " << yaw * 180 / M_PI << std::endl;

  std::cout << "Translation: "
            << "x = " << translation.x() << ", "
            << "y = " << translation.y() << ", "
            << "z = " << translation.z() << std::endl;
}

// :
template <typename T>
void printArray(const T* array, int size) {
  std::cout << "Array values: ";
  for (int i = 0; i < size; ++i) {
    std::cout << array[i] << " ";
  }
  std::cout << std::endl;
}

#endif
