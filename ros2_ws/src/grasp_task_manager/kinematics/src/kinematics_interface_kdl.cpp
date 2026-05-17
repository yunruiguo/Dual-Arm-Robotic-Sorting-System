#include "kinematics_interface_kdl.hpp"

#include <stdio.h>

#include <iostream>
namespace kinematics_interface_kdl {

bool KinematicsInterfaceKDL::initialize(const std::string& robot_description,
                                        const std::string& end_effector_name) {
  // track initialization plugin
  initialized = true;
  end_effector_offset_ = 0.0;
  base_transform_ = KDL::Frame::Identity();
  robot_description_local = robot_description;
  end_effector_name_ = end_effector_name;
  // create kinematic chain
  KDL::Tree robot_tree;
  if (!kdl_parser::treeFromFile(robot_description_local, robot_tree)) {
    std::cerr << "Failed to parse robot description into KDL tree!"
              << std::endl;
    return false;
  }
  root_name_ = robot_tree.getRootSegment()->first;
  // ,
  if (end_effector_name_.empty()) {
    for (const auto& segment : robot_tree.getSegments()) {
      if (segment.second.children.empty()) {  // 
        end_effector_name_ = segment.first;
      }
    }
  }

  std::cout << "end_effector_name: " << end_effector_name_ << std::endl;

  if (!robot_tree.getChain(root_name_, end_effector_name_, chain_)) {
    std::cerr << "Failed to find chain from root: " << root_name_
              << " to end effector: " << end_effector_name_ << std::endl;
    return false;
  }
  // create map from link names to their index
  for (size_t i = 0; i < chain_.getNrOfSegments(); ++i) {
    link_name_map_[chain_.getSegment(i).getName()] = i + 1;
  }
  // allocate dynamics memory
  num_joints_ = chain_.getNrOfJoints();
  q_ = KDL::JntArray(num_joints_);
  // create KDL solvers
  fk_pos_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
  ik_pos_solver_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(chain_);
  return true;
}

/**
 * \brief Set.
 * \param[in] roll X(:)
 * \param[in] pitch Y(:)
 * \param[in] yaw Z(:)
 */
void KinematicsInterfaceKDL::setBaseMountingAngles(double roll, double pitch,
                                                   double yaw) {
  //@ Z  (KDL::Rotation::RotZ(yaw))
  //@ Y  (KDL::Rotation::RotY(pitch))
  //@ X  (KDL::Rotation::RotX(roll))
  KDL::Rotation rotation = KDL::Rotation::RotZ(yaw) *
                           KDL::Rotation::RotY(pitch) *
                           KDL::Rotation::RotX(roll);

  base_transform_ = KDL::Frame(rotation, base_transform_.p);  // 

  // 
  double roll_out, pitch_out, yaw_out;
  base_transform_.M.GetRPY(roll_out, pitch_out, yaw_out);

  fprintf(stdout, "(Euler angles): Roll = %f, Pitch = %f, Yaw = %f\r\n",
          roll_out * 180 / M_PI, pitch_out * 180 / M_PI, yaw_out * 180 / M_PI);
}

/**
 * \brief Set.
 * \param[in] x X(:)
 * \param[in] y Y(:)
 * \param[in] z Z(:)
 */
void KinematicsInterfaceKDL::setBaseMountingPosition(double x, double y,
                                                     double z) {
  // Set
  KDL::Vector translation(x, y, z);
  base_transform_ = KDL::Frame(base_transform_.M, translation);
}

void KinematicsInterfaceKDL::setEndEffectorOffsetAngle(double angle) {
  end_effector_offset_ = angle;
}

/**
 * @brief ,.
 * @param joint_pos joint position (Eigen::MatrixXd).
 * @param transform Output (Eigen::Isometry3d).
 * @return Returns true,otherwiseReturns false.
 */
bool KinematicsInterfaceKDL::calculateForwardKinematics(
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& joint_pos,
    const std::string& link_name, Eigen::Isometry3d& transform) {
  std::string link = link_name;
  if (link_name.empty()) {
    link = end_effector_name_;
  }

  // verify inputs
  if (!verify_initialized() || !verify_joint_vector(joint_pos) ||
      !verify_link_name(link)) {
    return false;
  }

  //  Eigen::Vector  KDL::JntArray
  q_.data = joint_pos;
  q_(num_joints_ - 1) += end_effector_offset_;

  transform = Eigen::Isometry3d::Identity();
  // 
  if (fk_pos_solver_->JntToCart(q_, frame_, link_name_map_[link]) < 0) {
    std::cerr << "Failed to calculate forward kinematics!" << std::endl;
    return false;
  }
  transform = Eigen::Isometry3d::Identity();
  KDL::Frame base = base_transform_.Inverse() * frame_;

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      transform.linear()(i, j) = base.M(i, j);
    }
    transform.translation()(i) = base.p(i);
  }

  return true;
}

/**
 * @brief ,.
 * @param desired_pose  (Eigen::Isometry3d).
 * @param joint_pos Outputjoint position (Eigen::MatrixXd).
 * @param initial_guess joint position (Eigen::MatrixXd).
 * @return Returns true,otherwiseReturns false.
 */
bool KinematicsInterfaceKDL::calculateInverseKinematics(
    const Eigen::Isometry3d& desired_pose,
    Eigen::Matrix<double, Eigen::Dynamic, 1>& joint_pos,
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& initial_guess) {
  // verify inputs
  if (!verify_initialized() || !verify_joint_vector(initial_guess)) {
    return false;
  }
  // 
  KDL::JntArray kdl_joint_pos(chain_.getNrOfJoints());
  KDL::JntArray kdl_initial_guess(chain_.getNrOfJoints());
  for (int i = 0; i < initial_guess.size(); ++i) {
    kdl_initial_guess(i) = initial_guess[i];
  }

  KDL::Frame kdl_desired_frame;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      kdl_desired_frame.M(i, j) = desired_pose.linear()(i, j);
    }
    kdl_desired_frame.p(i) = desired_pose.translation()(i);
  }

  KDL::Frame transformed_target =
      base_transform_ * kdl_desired_frame;  // Inverse

  // 
  if (ik_pos_solver_->CartToJnt(kdl_initial_guess, transformed_target, q_) <
      0) {
    std::cerr << "Failed to calculate inverse kinematics!" << std::endl;
    return false;
  }
  q_(num_joints_ - 1) -= end_effector_offset_;
  joint_pos = q_.data;

  return true;
}

bool KinematicsInterfaceKDL::verify_link_name(const std::string& link_name) {
  if (link_name == root_name_) {
    return true;
  }
  if (link_name_map_.count(link_name) == 0) {
    std::cerr << "Invalid link name: " << link_name << std::endl;
    std::cerr << "Available links are:";
    for (size_t i = 0; i < chain_.getNrOfSegments(); ++i) {
      std::cerr << " " << chain_.getSegment(i).getName();
    }

    std::cerr << std::endl;
    return false;
  }
  return true;
}

bool KinematicsInterfaceKDL::verify_joint_vector(
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& joint_pose) const {
  if (static_cast<size_t>(joint_pose.size()) != num_joints_) {
    std::cerr << "Invalid joint vector size: " << joint_pose.size()
              << " (expected: " << num_joints_ << ")" << std::endl;
    return false;
  }
  return true;
}

bool KinematicsInterfaceKDL::verify_initialized() const {
  // check if interface is initialized
  if (!initialized) {
    std::cerr
        << "KDL Kinematics Plugin is not initialized. Please call initialize()."
        << std::endl;
    return false;
  }
  return true;
}

}  // namespace kinematics_interface_kdl
