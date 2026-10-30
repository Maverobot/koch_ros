#include <gtest/gtest.h>
#include <koch_controllers/kinematics/kinematics.h>

#include <string>

namespace
{
// A renamed fourth joint ensures geometry follows the configured joint, not a literal name.
std::string description(const std::string & length)
{
  return "<robot name='test'><link name='base'/><link name='tip'/>"
         "<joint name='configured_fourth' type='revolute'><parent link='base'/>"
         "<child link='tip'/><origin xyz='0.000125 " + length +
         " 0.002747'/><axis xyz='1 0 0'/><limit lower='-3' upper='3' effort='1' "
         "velocity='1'/></joint></robot>";
}

Eigen::Matrix<double, 6, 1> sample()
{
  Eigen::Matrix<double, 6, 1> q;
  q << .2, -.4, .7, -.3, .5, .1;
  return q;
}
}

TEST(Kinematics, FrozenDescriptionPreservesBaseline)
{
  const Kinematics model(description("0.090467"), "configured_fourth");
  // Captured from e2291be at this nonzero joint vector (not a URDF endpoint oracle).
  Eigen::Matrix4d expected;
  expected << .80162606425332061, -.59493907773589716, -.058678334097785112,
    -.013576790311572651, .44068901125710658, .52174317455857155, .7304637261074487,
    .089908118960761008, -.40396639521990574, -.6114176588750968, .68042604149806818,
    .31957534930470249, 0, 0, 0, 1;
  EXPECT_TRUE(model.forwardKinematics(sample()).isApprox(expected, 1e-13));
  Eigen::Matrix<double, 6, 6> jacobian;
  jacobian << 0, -.98006657784124163, .98006657784124163, .98006657784124163,
    -.13841425570643057, -.59493907773589716,
    0, -.19866933079506122, .19866933079506122, .19866933079506122,
    .68281896038890899, .52174317455857155,
    1, 0, 0, 0, .71735609089952268, -.6114176588750968,
    0, .011185083323761946, -.032155442200762017, -.048173128435942847,
    -.15680450066334717, -.18002311352457936,
    0, -.055177748332461904, .15862777646934173, .23764550341604507,
    -.031785845897386769, -.17551312434443722,
    0, 0, .028568895971818681, -.012466584345189023, 0, .025400003146573452;
  EXPECT_TRUE(model.spaceJacobian(sample()).isApprox(jacobian, 1e-13));
}

TEST(Kinematics, DescriptionLengthAndInstanceIsolation)
{
  const Kinematics old_model(description("0.090467"), "configured_fourth");
  const auto old_fk = old_model.forwardKinematics(sample());
  const auto old_jacobian = old_model.spaceJacobian(sample());
  const Kinematics new_model(description("0.100467"), "configured_fourth");
  Eigen::Matrix<double, 6, 1> reference;
  reference << 0, 0, M_PI_2, 0, 0, 0;
  EXPECT_NEAR(old_model.forwardKinematics(reference)(2, 3), .365264, 1e-14);
  EXPECT_NEAR(new_model.forwardKinematics(reference)(2, 3), .375264, 1e-14);
  for (int i = 0; i < 4; ++i)
  {
    const auto delta = (new_model.forwardKinematics(sample()) - old_fk).eval();
    const auto q = sample();
    const double angle = q[2] - q[1];
    const Eigen::Vector3d expected_delta = .01 * Eigen::Vector3d(
      -std::sin(q[0]) * std::cos(angle), std::cos(q[0]) * std::cos(angle), std::sin(angle));
    EXPECT_TRUE((delta.topRightCorner<3, 1>().isApprox(expected_delta, 1e-13)));
    EXPECT_TRUE((delta.topLeftCorner<3, 3>().isZero(1e-14)));
    EXPECT_FALSE(new_model.spaceJacobian(sample()).isApprox(old_jacobian, 1e-6));
    EXPECT_TRUE(old_model.forwardKinematics(sample()).isApprox(old_fk, 1e-14));
    EXPECT_TRUE(old_model.spaceJacobian(sample()).isApprox(old_jacobian, 1e-14));
  }
}

TEST(Kinematics, SpaceJacobianMatchesSpatialTwistFiniteDifference)
{
  for (const auto & length : {"0.090467", "0.100467"})
  {
    const Kinematics model(description(length), "configured_fourth");
    for (const auto & q : {sample(), Eigen::Matrix<double, 6, 1>::Zero().eval()})
    {
      const auto transform = model.forwardKinematics(q);
      const auto jacobian = model.spaceJacobian(q);
      for (int i = 0; i < 6; ++i)
      {
        auto plus = q; auto minus = q;
        plus[i] += 1e-6; minus[i] -= 1e-6;
        // Spatial twist is dT * T^-1, NOT [angular velocity; tip velocity].
        const Eigen::Matrix4d twist =
          (model.forwardKinematics(plus) - model.forwardKinematics(minus)) / 2e-6 *
          transform.inverse();
        Eigen::Matrix<double, 6, 1> numerical;
        numerical << twist(2, 1), twist(0, 2), twist(1, 0), twist(0, 3), twist(1, 3),
          twist(2, 3);
        EXPECT_TRUE(numerical.isApprox(jacobian.col(i), 1e-8));
      }
    }
  }
}

TEST(Kinematics, InvalidDescriptionNeverFallsBack)
{
  EXPECT_THROW(Kinematics("", "configured_fourth"), std::invalid_argument);
  EXPECT_THROW(Kinematics("not xml", "configured_fourth"), std::invalid_argument);
  EXPECT_THROW(Kinematics(description("0.1"), "missing"), std::invalid_argument);
  for (const auto & length : {"0", "-0.1", "nan", "inf", "bad"})
  {
    EXPECT_THROW(Kinematics(description(length), "configured_fourth"), std::invalid_argument);
  }
}
