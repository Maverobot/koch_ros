#pragma once

#include <array>
#include <string>

#include <koch_controllers/kinematics/screw_theory.h>

// Only the fourth joint's local y offset is synchronized with the active description.
// All other POE approximations and the gripper-tip convention remain unchanged.
class Kinematics
{
public:
  Kinematics(const std::string & robot_description, const std::string & fourth_joint);

  Eigen::Matrix4d forwardKinematics(const Eigen::Matrix<double, 6, 1> & q) const;
  Eigen::Matrix<double, 6, 6> spaceJacobian(const Eigen::Matrix<double, 6, 1> & q) const;

private:
  Eigen::Matrix4d motionByJoint(size_t index, double theta) const;

  std::array<Eigen::Vector3d, 6> v_;
  Eigen::Matrix4d m_;
};
