#ifndef KINEMATICS_INTERFACE_KDL__KINEMATICS_INTERFACE_KDL_HPP_
#define KINEMATICS_INTERFACE_KDL__KINEMATICS_INTERFACE_KDL_HPP_

#include <kdl/chainiksolverpos_lma.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/LU"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainfksolvervel_recursive.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/treejnttojacsolver.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "kinematics_interface.hpp"

namespace kinematics_interface_kdl {
class KinematicsInterfaceKDL
    : public kinematics_interface::KinematicsInterface {
 public:
  /**
   * \brief ..
   * \param[in] robot_description  URDF 
   * \param[in] end_effector_name 
   * \return initialized successfullyReturns true,otherwiseReturns false
   */
  bool initialize(const std::string &robot_description,
                  const std::string &end_effector_name) override;

  /**
   * \brief Set.
   * \param[in] roll X(:)
   * \param[in] pitch Y(:)
   * \param[in] yaw Z(:)
   */
  void setBaseMountingAngles(double roll, double pitch, double yaw) override;

  /**
   * \brief Set.
   * \param[in] x X(:)
   * \param[in] y Y(:)
   * \param[in] z Z(:)
   */
  void setBaseMountingPosition(double x, double y, double z) override;

  /**
   * \brief Set
   * \param[in] angle (:)
   */
  void setEndEffectorOffsetAngle(double angle) override;

  /**
   * @brief ,.
   * @param joint_pos joint position (Eigen::MatrixXd).
   * @param link_name .
   * @param transform Output (Eigen::Isometry3d).
   * @return Returns true,otherwiseReturns false.
   */
  bool calculateForwardKinematics(
      const Eigen::Matrix<double, Eigen::Dynamic, 1> &joint_pos,
      const std::string &link_name, Eigen::Isometry3d &transform) override;

  /**
   * @brief ,.
   * @param desired_pose  (Eigen::Isometry3d).
   * @param joint_pos Outputjoint position (Eigen::MatrixXd).
   * @param initial_guess joint position (Eigen::MatrixXd).
   * @return Returns true,otherwiseReturns false.
   */
  bool calculateInverseKinematics(
      const Eigen::Isometry3d &desired_pose,
      Eigen::Matrix<double, Eigen::Dynamic, 1> &joint_pos,
      const Eigen::Matrix<double, Eigen::Dynamic, 1> &initial_guess) override;

 private:
  /**
   * @brief .
   * @return Returns true,otherwiseReturns false.
   */
  bool verify_initialized() const;

  /**
   * @brief .
   * @param link_name .
   * @return Returns true,otherwiseReturns false.
   */
  bool verify_link_name(const std::string &link_name);

  /**
   * @brief .
   * @param joint_pose joint position.
   * @return Returns true,otherwiseReturns false.
   */
  bool verify_joint_vector(
      const Eigen::Matrix<double, Eigen::Dynamic, 1> &joint_pose) const;

  /**
   * @brief ,.
   */
  bool initialized = false;

  /**
   * @brief .
   */
  std::string root_name_;

  /**
   * @brief  URDF .
   */
  std::string robot_description_local;

  /**
   * @brief .
   */
  size_t num_joints_;

  /**
   * @brief  KDL .
   */
  KDL::Chain chain_;

  /**
   * @brief KDL .
   */
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;

  /**
   * @brief KDL .
   */
  std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_pos_solver_;

  /**
   * @brief KDL .
   */
  KDL::JntArray q_;

  /**
   * @brief KDL .
   */
  KDL::Frame frame_;

  /**
   * @brief KDL 
   */
  KDL::Frame base_transform_;

  /**
   * @brief 
   */
  double end_effector_offset_;

  /**
   * @brief ,Find.
   */
  std::unordered_map<std::string, int> link_name_map_;
};

}  // namespace kinematics_interface_kdl

#endif  // KINEMATICS_INTERFACE_KDL__KINEMATICS_INTERFACE_KDL_HPP_
