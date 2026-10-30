#include <koch_controllers/kinematics/kinematics.h>

#include <cmath>
#include <stdexcept>
#include <urdf/model.hpp>

namespace
{
const std::array<Eigen::Vector3d, 6> omega = {
  Eigen::Vector3d{0, 0, 1}, Eigen::Vector3d{-1, 0, 0}, Eigen::Vector3d{1, 0, 0},
  Eigen::Vector3d{1, 0, 0}, Eigen::Vector3d{0, 0, 1}, Eigen::Vector3d{0, 1, 0},
};
}

Kinematics::Kinematics(
  const std::string & robot_description, const std::string & fourth_joint)
{
  urdf::Model model;
  if (!model.initString(robot_description))
  {
    throw std::invalid_argument("Missing or invalid robot description for follower kinematics");
  }
  const auto joint = model.getJoint(fourth_joint);
  if (!joint)
  {
    throw std::invalid_argument("Missing configured fourth joint: " + fourth_joint);
  }
  const double forearm_y = joint->parent_to_joint_origin_transform.position.y;
  if (!std::isfinite(forearm_y) || forearm_y <= 0.0)
  {
    throw std::invalid_argument("Fourth joint local y must be finite and positive: " + fourth_joint);
  }
  const std::array<Eigen::Vector3d, 6> points_on_axis = {
    Eigen::Vector3d{0, 0, 0.039},
    Eigen::Vector3d{0, 0, 0.0563},
    Eigen::Vector3d{0, 0.014791, 0.164647},
    Eigen::Vector3d{0, 0.014791, 0.164647 + forearm_y},
    Eigen::Vector3d{0, 0.014791, 0.164647 + forearm_y + 0.045},
    Eigen::Vector3d{0, 0.014791, 0.164647 + forearm_y + 0.045 + 0.01315},
  };
  for (size_t i = 0; i < v_.size(); ++i)
  {
    v_[i] = omega[i].cross(-points_on_axis[i]);
  }
  m_ = Eigen::Matrix4d::Identity();
  m_(1, 3) = 0.014791;
  m_(2, 3) = 0.164647 + forearm_y + 0.045 + 0.01315 + 0.052;
}

Eigen::Matrix4d Kinematics::motionByJoint(size_t index, double theta) const
{
  return screw_motion(omega.at(index), v_.at(index), theta);
}

Eigen::Matrix4d Kinematics::forwardKinematics(const Eigen::Matrix<double, 6, 1> & q) const
{
  Eigen::Matrix<double, 6, 1> q_0;
  q_0 << 0, 0, M_PI_2, 0, 0, 0;

  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 6, 1> delta_q = q - q_0;
  for (int i = 0; i < 6; i++)
  {
    transform *= motionByJoint(i, delta_q[i]);
  }

  return transform * m_;
};

Eigen::Matrix<double, 6, 6> adjoint_representation(const Eigen::Matrix4d & T)
{
  Eigen::Matrix3d R = T.topLeftCorner(3, 3);
  Eigen::Vector3d p = T.topRightCorner(3, 1);
  Eigen::Matrix3d p_skew = skew_symmetric(p);

  Eigen::Matrix<double, 6, 6> AdT;
  AdT.topLeftCorner(3, 3) = R;
  AdT.topRightCorner(3, 3) = Eigen::Matrix3d::Zero();
  AdT.bottomLeftCorner(3, 3) = p_skew * R;
  AdT.bottomRightCorner(3, 3) = R;

  return AdT;
}

Eigen::Matrix<double, 6, 6> Kinematics::spaceJacobian(
  const Eigen::Matrix<double, 6, 1> & q) const
{
  Eigen::Matrix<double, 6, 6> Js;
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

  Eigen::Matrix<double, 6, 1> q_0;
  q_0 << 0, 0, M_PI_2, 0, 0, 0;
  Eigen::Matrix<double, 6, 1> delta_q = q - q_0;

  for (int i = 0; i < 6; i++)
  {
    if (i == 0)
    {
      Js.col(0) = Eigen::Matrix<double, 6, 1>(
        omega.at(0)(0), omega.at(0)(1), omega.at(0)(2), v_.at(0)(0), v_.at(0)(1), v_.at(0)(2));
    }
    else
    {
      T *= motionByJoint(i - 1, delta_q[i - 1]);
      Eigen::Matrix<double, 6, 6> AdT = adjoint_representation(T);
      Eigen::Matrix<double, 6, 1> screw_axis(
        omega.at(i)(0), omega.at(i)(1), omega.at(i)(2), v_.at(i)(0), v_.at(i)(1), v_.at(i)(2));
      Js.col(i) = AdT * screw_axis;
    }
  }

  return Js;
}
