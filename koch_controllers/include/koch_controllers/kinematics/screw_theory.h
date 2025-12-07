#pragma once

#include <Eigen/Eigen>

inline Eigen::Matrix3d skew_symmetric(Eigen::Vector3d v)
{
  Eigen::Matrix3d m;
  m << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
  return m;
}

/**
 * Computes the rotational matrix from an angle-axis expression.
 *
 * See Proposition 3.11 in Modern Robotics, Mechanics, Planning, and Control.
 *
 * @param      omega The rotation axis
 * @param      theta The angle around the rotation axis
 *
 * @return     returns the equivalent rotation matrix
 */
inline Eigen::Matrix3d rodrigues_formula(Eigen::Vector3d omega, double theta)
{
  Eigen::Matrix3d omega_skew_symmetric = skew_symmetric(omega);
  return Eigen::Matrix3d::Identity() + sin(theta) * omega_skew_symmetric +
         (1 - cos(theta)) * omega_skew_symmetric * omega_skew_symmetric;
}

/**
 * Computes the homogeneous transformation (S_T_B) from an screw motion.
 *
 * S: space frame (usually a fixed frame). B: body frame.
 *
 * See Proposition 3.25 in Modern Robotics, Mechanics, Planning, and Control.
 *
 * @param      omega The rotational velocity in S frame. \f$\omega = \hat{s}\dot{\theta}\f$
 * @param      v The translational velocity in S frame.
 *             \fv = -\hat{s}\dot{\theta}\times q + h\hat{s}\dot{\theta}\f$
 * @param      theta The "angle" around the screw axis [omega; v].
 *
 * @return     returns the equivalent rotation matrix
 */
inline Eigen::Matrix4d screw_motion(Eigen::Vector3d omega, Eigen::Vector3d v, double theta)
{
  Eigen::Matrix3d omega_skew_symmetric = skew_symmetric(omega);
  Eigen::Matrix4d homogeneous_transformation;
  homogeneous_transformation.topLeftCorner(3, 3) = rodrigues_formula(omega, theta);
  homogeneous_transformation.topRightCorner(3, 1) =
    (Eigen::Matrix3d::Identity() * theta + (1 - cos(theta)) * omega_skew_symmetric +
     (theta - sin(theta)) * omega_skew_symmetric * omega_skew_symmetric) *
    v;
  homogeneous_transformation.row(3) << 0, 0, 0, 1;
  return homogeneous_transformation;
}
