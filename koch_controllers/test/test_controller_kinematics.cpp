#include <gtest/gtest.h>
#include <koch_controllers/claw_machine_controller.h>
#include <koch_controllers/leader_follower_controller.h>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <chrono>
#include <thread>

namespace
{
const std::vector<std::string> joints = {"one", "two", "three", "four", "five", "gripper"};
const std::vector<std::string> leaders = {"l1", "l2", "l3", "l4", "l5", "l6"};

std::string description(const std::string & length)
{
  return "<robot name='test'><link name='base'/><link name='tip'/>"
         "<joint name='four' type='revolute'><parent link='base'/><child link='tip'/>"
         "<origin xyz='0 " + length + " 0.002747'/><axis xyz='1 0 0'/>"
         "<limit lower='-3' upper='3' effort='1' velocity='1'/></joint></robot>";
}

template<class Controller>
void initialize(Controller & controller, const std::string & name, const std::string & urdf)
{
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = name;
  params.robot_description = urdf;
  params.update_rate = 80;
  params.controller_manager_update_rate = 80;
  ASSERT_EQ(controller.init(params), controller_interface::return_type::OK);
}

void set_joints(koch_controllers::ClawMachineController & controller)
{
  controller.get_node()->set_parameter(rclcpp::Parameter("joints", joints));
}

void set_joints(koch_controllers::LeaderFollowerController & controller)
{
  controller.get_node()->set_parameters({rclcpp::Parameter("follower_joints", joints),
    rclcpp::Parameter("leader_joints", leaders),
    rclcpp::Parameter("leader_to_follower_offset", std::vector<double>(6, 0)),
    rclcpp::Parameter("leader_to_follower_scale", std::vector<double>(6, 1))});
}

class ControllerKinematics : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

template<class Controller>
void check_configuration()
{
  for (const auto & length : {"0.090467", "0.100467", "0", "-0.1", "nan"})
  {
    Controller controller;
    initialize(controller, "configuration_test", description(length));
    set_joints(controller);
    const bool valid = std::string(length) == "0.090467" || std::string(length) == "0.100467";
    EXPECT_EQ(controller.configure().label(), valid ? "inactive" : "unconfigured");
  }
  for (const auto & urdf : {std::string(""), std::string("invalid"),
      std::string("<robot name='test'><link name='base'/></robot>")})
  {
    Controller controller;
    initialize(controller, "missing_description_test", urdf);
    set_joints(controller);
    EXPECT_NE(controller.configure().label(), "inactive");
  }
}

TEST_F(ControllerKinematics, BothControllersUseTheirReceivedDescriptionAtConfigure)
{
  check_configuration<koch_controllers::LeaderFollowerController>();
  check_configuration<koch_controllers::ClawMachineController>();
}

struct ClawSession
{
  koch_controllers::ClawMachineController controller;
  std::array<double, 6> q{.2, -.4, .7, -.3, .5, .1};
  std::array<double, 6> velocity{};
  std::array<double, 6> command{};
  std::vector<hardware_interface::StateInterface::SharedPtr> states;
  std::vector<hardware_interface::CommandInterface::SharedPtr> commands;

  void start(const std::string & name, const std::string & urdf)
  {
    initialize(controller, name, urdf);
    set_joints(controller);
    ASSERT_EQ(controller.configure().label(), "inactive");
    states.reserve(12);
    commands.reserve(6);
    for (size_t i = 0; i < 6; ++i)
    {
      states.push_back(std::make_shared<hardware_interface::StateInterface>(
        joints[i], "position", &q[i]));
      states.push_back(std::make_shared<hardware_interface::StateInterface>(
        joints[i], "velocity", &velocity[i]));
      commands.push_back(std::make_shared<hardware_interface::CommandInterface>(
        joints[i], "position", &command[i]));
    }
    std::vector<hardware_interface::LoanedStateInterface> loaned_states;
    std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
    for (auto & state : states) { loaned_states.emplace_back(state); }
    for (auto & command_interface : commands) { loaned_commands.emplace_back(command_interface); }
    controller.assign_interfaces(std::move(loaned_commands), std::move(loaned_states));
    ASSERT_EQ(controller.get_node()->activate().label(), "active");
  }

  ~ClawSession() { controller.release_interfaces(); }
};

// Independent damped least squares expression, rather than the production SVD helper.
Eigen::MatrixXd damped_inverse(const Eigen::MatrixXd & matrix)
{
  return matrix.transpose() *
         (matrix * matrix.transpose() + .0001 *
         Eigen::MatrixXd::Identity(matrix.rows(), matrix.rows())).inverse();
}

TEST_F(ControllerKinematics, ClawUpdateUsesInstanceJacobianThroughPublicInterfaces)
{
  ClawSession old_session, new_session;
  old_session.start("old_claw", description("0.090467"));
  new_session.start("new_claw", description("0.100467"));
  auto node = std::make_shared<rclcpp::Node>("claw_command_test");
  auto publisher = node->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(old_session.controller.get_node()->get_node_base_interface());
  executor.add_node(new_session.controller.get_node()->get_node_base_interface());
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (publisher->get_subscription_count() < 2 && std::chrono::steady_clock::now() < deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(publisher->get_subscription_count(), 2u);
  geometry_msgs::msg::TwistStamped message;
  message.header.stamp = node->now();
  message.twist.linear.x = .02;
  publisher->publish(message);
  // Spin for delivery without advancing controller updates or changing the measured joints.
  executor.spin_some(std::chrono::milliseconds(100));
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  executor.spin_some();
  for (auto * session : {&old_session, &new_session, &old_session})
  {
    const bool old = session == &old_session;
    const Kinematics model(description(old ? "0.090467" : "0.100467"), "four");
    Eigen::Map<Eigen::Matrix<double, 6, 1>> q(session->q.data());
    const auto jacobian = model.spaceJacobian(q);
    Eigen::Matrix<double, 5, 5> reduced;
    reduced << jacobian.row(0).head<5>(), jacobian.row(1).head<5>(),
      jacobian.row(3).head<5>(), jacobian.row(4).head<5>(), jacobian.row(5).head<5>();
    Eigen::Matrix<double, 5, 1> desired;
    desired << 0, 0, 0, .02, 0;
    const Eigen::VectorXd primary = damped_inverse(reduced) * desired;
    const Eigen::MatrixXd nullspace = Eigen::Matrix<double, 5, 5>::Identity() -
      damped_inverse(reduced) * reduced;
    const Eigen::MatrixXd z_row = jacobian.row(2).head<5>();
    const Eigen::VectorXd dq = primary - nullspace * damped_inverse(z_row * nullspace) *
      z_row * primary;
    const auto before = session->command;
    ASSERT_EQ(session->controller.update(rclcpp::Time(message.header.stamp),
      rclcpp::Duration::from_seconds(.0125)), controller_interface::return_type::OK);
    for (int i = 0; i < 5; ++i)
    {
      const double start = before[5] == 0 ? q[i] : before[i];
      EXPECT_NEAR(session->command[i], start + .0125 * dq[i], 1e-12);
    }
    EXPECT_DOUBLE_EQ(session->command[5], 1.46);
  }
  EXPECT_GT(std::abs((old_session.command[2] - old_session.q[2]) / 2 -
    (new_session.command[2] - new_session.q[2])), 1e-8);
}
}
