#include <koch_controllers/kinematics/kinematics.h>

Eigen::Matrix4d motionByJoint(size_t index, double theta)
{
  return screw_motion(omega.at(index), v.at(index), theta);
}

Eigen::Matrix4d forwardKinematics(const Eigen::Matrix<double, 6, 1> & q)
{
  Eigen::Matrix<double, 6, 1> q_0;
  q_0 << 0, 0, M_PI_2, 0, 0, 0;

  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 6, 1> delta_q = q - q_0;
  for (int i = 0; i < 6; i++)
  {
    transform *= screw_motion(omega.at(i), v.at(i), delta_q[i]);
  }

  // Transformation O_T_gripper_tip at q_0
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  m(0, 3) = 0.0;
  m(1, 3) = 0.014791;
  m(2, 3) = 0.164647 + 0.090467 + 0.045 + 0.01315 + 0.052;

  return transform * m;
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

Eigen::Matrix<double, 6, 6> spaceJacobian(const Eigen::Matrix<double, 6, 1> & q)
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
        omega.at(0)(0), omega.at(0)(1), omega.at(0)(2), v.at(0)(0), v.at(0)(1), v.at(0)(2));
    }
    else
    {
      T *= motionByJoint(i - 1, delta_q[i - 1]);
      Eigen::Matrix<double, 6, 6> AdT = adjoint_representation(T);
      Eigen::Matrix<double, 6, 1> screw_axis(
        omega.at(i)(0), omega.at(i)(1), omega.at(i)(2), v.at(i)(0), v.at(i)(1), v.at(i)(2));
      Js.col(i) = AdT * screw_axis;
    }
  }

  return Js;
}
