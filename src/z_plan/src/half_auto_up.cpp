/** 
 * @file half_auto_up.cpp
 * @brief  交互式测试：先做一个很小的上移，再按 W 每次沿该方向移动 1cm，末端朝向保持不变
 * @date    2026-03-31
*/

#include <algorithm>
#include <iostream>
#include <iterator>
#include <memory>
#include <string>
#include <thread>

#include <termios.h>
#include <unistd.h>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "utils.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("half_auto_up");

namespace
{
class RawTerminalGuard
{
public:
  RawTerminalGuard()
  {
    enabled_ = (tcgetattr(STDIN_FILENO, &old_termios_) == 0);
    if (enabled_) {
      termios raw = old_termios_;
      raw.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
      raw.c_cc[VMIN] = 1;
      raw.c_cc[VTIME] = 0;
      tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }
  }

  ~RawTerminalGuard()
  {
    if (enabled_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
    }
  }

  RawTerminalGuard(const RawTerminalGuard&) = delete;
  RawTerminalGuard& operator=(const RawTerminalGuard&) = delete;

private:
  termios old_termios_{};
  bool enabled_{false};
};

moveit_msgs::msg::RobotTrajectory plan_cartesian_step(
    const geometry_msgs::msg::Pose& current_pose,
    const Eigen::Vector3d& direction,
    double distance,
    const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group)
{
  geometry_msgs::msg::Pose target_pose = current_pose;
  const Eigen::Vector3d direction_normalized = direction.normalized();
  target_pose.position.x += direction_normalized.x() * distance;
  target_pose.position.y += direction_normalized.y() * distance;
  target_pose.position.z += direction_normalized.z() * distance;
  std::vector<geometry_msgs::msg::Pose> waypoints = {target_pose};

  moveit_msgs::msg::RobotTrajectory trajectory;
  //const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = 0.0;
  int attempts = 0;
  const int maxtries = 100;

  while (fraction < 1.0 && attempts < maxtries) {
    fraction = move_group->computeCartesianPath(waypoints, eef_step, trajectory);
    ++attempts;
  }

  if (fraction < 1.0) {
    RCLCPP_WARN(LOGGER, "Cartesian path only reached fraction %.2f after %d attempts", fraction, attempts);
  }

  return trajectory;
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("half_auto_up", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "arm";
  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node, PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group =
      move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "base_link", "z_plan_robo_arm_tutorial",
                                                      move_group->getRobotModel());
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Half_Auto_Up_Test", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group->getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group->getEndEffectorLink().c_str());
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group->getJointModelGroupNames().begin(), move_group->getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
  std::cout << std::endl;

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move to reset joint state");

  std::vector<double> zero_joint_values = {0.0, -0.9, 0.0, 3.4, -1, -0.5};
  move_group->setJointValueTarget(zero_joint_values);

  moveit::planning_interface::MoveGroupInterface::Plan zero_plan;
  const bool zero_success = (move_group->plan(zero_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!zero_success) {
    RCLCPP_ERROR(LOGGER, "Failed to plan to zero joint state.");
    rclcpp::shutdown();
    return 1;
  }

  visual_tools.publishText(text_pose, "Move_To_Zero", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(zero_plan.trajectory, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute zero-joint motion");

  move_group->execute(zero_plan);

  const geometry_msgs::msg::Pose start_pose = move_group->getCurrentPose().pose;
  const Eigen::Vector3d seed_direction(0.0, 0.0, 1.0);
  const double seed_distance = 0.01;

  moveit_msgs::msg::RobotTrajectory seed_trajectory =
      plan_cartesian_step(start_pose, seed_direction, seed_distance, move_group);

  if (seed_trajectory.joint_trajectory.points.empty()) {
    RCLCPP_ERROR(LOGGER, "Failed to compute the initial seed motion.");
    rclcpp::shutdown();
    return 1;
  }

  geometry_msgs::msg::Pose seed_pose = start_pose;
  seed_pose.position.x += seed_direction.x() * seed_distance;
  seed_pose.position.y += seed_direction.y() * seed_distance;
  seed_pose.position.z += seed_direction.z() * seed_distance;
  Eigen::Vector3d motion_direction(
      seed_pose.position.x - start_pose.position.x,
      seed_pose.position.y - start_pose.position.y,
      seed_pose.position.z - start_pose.position.z);
  if (motion_direction.norm() < 1e-9) {
    RCLCPP_ERROR(LOGGER, "Seed motion produced a near-zero direction vector.");
    rclcpp::shutdown();
    return 1;
  }
  motion_direction.normalize();

  RCLCPP_INFO(LOGGER, "Seed direction: [%.3f, %.3f, %.3f]", motion_direction.x(), motion_direction.y(), motion_direction.z());

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Seed_Up_Motion", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(seed_trajectory, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute seed motion");

  moveit::planning_interface::MoveGroupInterface::Plan seed_plan;
  seed_plan.trajectory = seed_trajectory;
  move_group->execute(seed_plan);

  RawTerminalGuard guard;
  RCLCPP_INFO(LOGGER, "Interactive mode ready. Press 'w' to move 1cm, 'q' to quit.");

  size_t step_index = 0;
  while (rclcpp::ok()) {
    std::cout << "[w] move 1cm, [q] quit > " << std::flush;
    const int ch = std::getchar();

    if (ch == 'q' || ch == 'Q') {
      RCLCPP_INFO(LOGGER, "Quit requested.");
      break;
    }

    if (ch != 'w' && ch != 'W') {
      continue;
    }

    move_group->setStartStateToCurrentState();
    const geometry_msgs::msg::Pose current_pose = move_group->getCurrentPose().pose;

    moveit_msgs::msg::RobotTrajectory step_trajectory =
        plan_cartesian_step(current_pose, motion_direction, 0.01, move_group);

    if (step_trajectory.joint_trajectory.points.empty()) {
      RCLCPP_WARN(LOGGER, "Step %zu planning failed, skipped.", step_index + 1);
      continue;
    }

    auto trajectory_dict = extractTrajectory(step_trajectory);
    printTrajectory(trajectory_dict);

    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Half_Auto_Up_Step", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(step_trajectory, joint_model_group);
    visual_tools.trigger();

    moveit::planning_interface::MoveGroupInterface::Plan step_plan;
    step_plan.trajectory = step_trajectory;
    move_group->execute(step_plan);

    ++step_index;
    const auto end_pose = move_group->getCurrentPose().pose;
    RCLCPP_INFO(LOGGER, "Step %zu complete. Current pose: [%.3f, %.3f, %.3f]",
                step_index,
                end_pose.position.x,
                end_pose.position.y,
                end_pose.position.z);
  }

  rclcpp::shutdown();
  return 0;
}