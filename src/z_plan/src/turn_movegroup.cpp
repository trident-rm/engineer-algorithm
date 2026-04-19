/** 
 * @file turn_movegroup.cpp
 * @brief  半自动TURN模式半圆规划
 * @author  Grace
 * @date    2026-03-31
*/

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "utils.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("z_plan_movegroup_robo_arm");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("z_plan_movegroup_robo_arm", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
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

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Z_Plan_RoboArm_Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();


  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group->getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group->getEndEffectorLink().c_str());

  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group->getJointModelGroupNames().begin(), move_group->getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move to reset joint state");

  std::vector<double> zero_joint_values = {0.0, -0.9, 0.0, 3.4, -1, -0.5};
  move_group->setJointValueTarget(zero_joint_values);

  moveit::planning_interface::MoveGroupInterface::Plan zero_plan;
  bool zero_success = (move_group->plan(zero_plan) == moveit::core::MoveItErrorCode::SUCCESS);
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

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to plan half-circle trajectory");

  const std::string ee_link = "gripper";
  const geometry_msgs::msg::Pose start_pose = move_group->getCurrentPose(ee_link).pose;
  const double radius = 0.1;
  const int num_points = 40;
  const Eigen::Quaterniond start_quat(
    start_pose.orientation.w,
    start_pose.orientation.x,
    start_pose.orientation.y,
    start_pose.orientation.z);

  // gripper 在世界系下的 x/y 轴方向
  const Eigen::Vector3d x_axis_world = (start_quat * Eigen::Vector3d::UnitX()).normalized();
  const Eigen::Vector3d y_axis_world = (start_quat * Eigen::Vector3d::UnitY()).normalized();

  const Eigen::Vector3d normal_direction = y_axis_world;
  const Eigen::Vector3d e_hat_direction = x_axis_world;
  const Eigen::Vector3d start_point(start_pose.position.x, start_pose.position.y, start_pose.position.z);
  const Eigen::Vector3d center = start_point - radius * e_hat_direction;

  moveit_msgs::msg::RobotTrajectory trajectory =
      plan_half_circle_turn(center, start_pose, radius, num_points, normal_direction, e_hat_direction, move_group);

  if (trajectory.joint_trajectory.points.empty()) {
    RCLCPP_ERROR(LOGGER, "Half-circle trajectory planning failed: empty trajectory.");
    rclcpp::shutdown();
    return 1;
  }

  auto trajectory_dict = extractTrajectory(trajectory);
  printTrajectory(trajectory_dict);

  RCLCPP_INFO(LOGGER, "Visualizing half-circle Cartesian path in base_link xz plane (radius=%.3fm)", radius);
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Half_Circle_Path", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(trajectory, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute half-circle path");

  moveit::planning_interface::MoveGroupInterface::Plan half_circle_plan;
  half_circle_plan.trajectory = trajectory;
  move_group->execute(half_circle_plan);
  
  visual_tools.prompt("Press 'next' to finish the demo");

  RCLCPP_INFO(LOGGER, "Demo complete!");
  rclcpp::shutdown();
  return 0;
}
