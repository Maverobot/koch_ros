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
