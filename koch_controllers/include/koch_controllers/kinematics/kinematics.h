
#include <koch_controllers/kinematics/screw_theory.h>

static const std::array<Eigen::Vector3d, 6> omega = {
  // 1/-1: the joint angle increases/decreases when the joint is rotated per right-hand rule
  Eigen::Vector3d{0, 0, 1}, Eigen::Vector3d{-1, 0, 0}, Eigen::Vector3d{1, 0, 0},
  Eigen::Vector3d{1, 0, 0}, Eigen::Vector3d{0, 0, 1},  Eigen::Vector3d{0, 1, 0},
};

static const std::array<Eigen::Vector3d, 6> points_on_axis = {
  Eigen::Vector3d{0, 0, 0.039},                                         // a point on joint 1 axis
  Eigen::Vector3d{0, 0, 0.0563},                                        // a point on joint 2 axis
  Eigen::Vector3d{0, 0.014791, 0.164647},                               // a point on joint 3 axis
  Eigen::Vector3d{0, 0.014791, 0.164647 + 0.090467},                    // a point on joint 4 axis
  Eigen::Vector3d{0, 0.014791, 0.164647 + 0.090467 + 0.045},            // a point on joint 5 axis
  Eigen::Vector3d{0, 0.014791, 0.164647 + 0.090467 + 0.045 + 0.01315},  // a point on joint 6 axis
};

static const std::array<Eigen::Vector3d, 6> v = {
  omega.at(0).cross(-points_on_axis.at(0)), omega.at(1).cross(-points_on_axis.at(1)),
  omega.at(2).cross(-points_on_axis.at(2)), omega.at(3).cross(-points_on_axis.at(3)),
  omega.at(4).cross(-points_on_axis.at(4)), omega.at(5).cross(-points_on_axis.at(5)),
};

Eigen::Matrix4d motionByJoint(size_t index, double theta);

Eigen::Matrix4d forwardKinematics(const Eigen::Matrix<double, 6, 1> & q);
