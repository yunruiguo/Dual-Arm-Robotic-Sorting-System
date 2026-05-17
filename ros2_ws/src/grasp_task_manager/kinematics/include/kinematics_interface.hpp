#ifndef KINEMATICS_INTERFACE__KINEMATICS_INTERFACE_HPP_
#define KINEMATICS_INTERFACE__KINEMATICS_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/LU"

namespace kinematics_interface {
class KinematicsInterface {
 public:
  KinematicsInterface() = default;

  virtual ~KinematicsInterface() = default;

  /**
   * \brief ..
   * \param[in] robot_description  URDF 
   * \param[in] end_effector_name 
   * \return initialized successfullyReturns true,otherwiseReturns false
   */
  virtual bool initialize(const std::string &robot_description,
                          const std::string &end_effector_name) = 0;

  /**
   * \brief Set.
   * \param[in] roll X(:)
   * \param[in] pitch Y(:)
   * \param[in] yaw Z(:)
   */
  virtual void setBaseMountingAngles(double roll, double pitch, double yaw) = 0;

  /**
   * \brief Set.
   * \param[in] x X(:)
   * \param[in] y Y(:)
   * \param[in] z Z(:)
   */
  virtual void setBaseMountingPosition(double x, double y, double z) = 0;

  /**
   * \brief Set
   * \param[in] angle (:)
   */
  virtual void setEndEffectorOffsetAngle(double angle) = 0;

  /**
   * @brief ,.
   * @param joint_pos joint position (Eigen::MatrixXd).
   * @param link_name .
   * @param transform Output (Eigen::Isometry3d).
   * @return Returns true,otherwiseReturns false.
   */
  virtual bool calculateForwardKinematics(
      const Eigen::Matrix<double, Eigen::Dynamic, 1> &joint_pos,
      const std::string &link_name, Eigen::Isometry3d &transform) = 0;

  /**
   * @brief ,.
   * @param desired_pose  (Eigen::Isometry3d).
   * @param joint_pos Outputjoint position (Eigen::MatrixXd).
   * @param initial_guess joint position (Eigen::MatrixXd).
   * @return Returns true,otherwiseReturns false.
   */
  virtual bool calculateInverseKinematics(
      const Eigen::Isometry3d &desired_pose,
      Eigen::Matrix<double, Eigen::Dynamic, 1> &joint_pos,
      const Eigen::Matrix<double, Eigen::Dynamic, 1> &initial_guess) = 0;

 protected:
  std::string end_effector_name_;
};

}  // namespace kinematics_interface

#endif  // KINEMATICS_INTERFACE__KINEMATICS_INTERFACE_HPP_
