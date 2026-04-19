/** 
 * @file turn_retreat.cpp
 * @brief  测试 utils.cpp 中的 retreat 笛卡尔撤退轨迹规划
 * @author  Grace
 * @date    2026-03-31
*/

#include <iterator>
#include <memory>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "utils.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("turn_retreat");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("turn_retreat", node_options);

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
  visual_tools.publishText(text_pose, "TURN_Retreat_Test", rvt::WHITE, rvt::XLARGE);
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

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to plan retreat trajectory");

  const double retreat_distance = -0.2;  // 沿末端 z 轴撤退 20 cm
  RCLCPP_INFO(LOGGER, "Using retreat_distance = %.3f m", retreat_distance);

  const geometry_msgs::msg::Pose current_pose = move_group->getCurrentPose().pose;
  moveit_msgs::msg::RobotTrajectory trajectory =
      plan_retreat(current_pose, retreat_distance, move_group);

  if (trajectory.joint_trajectory.points.empty()) {
    RCLCPP_ERROR(LOGGER, "Retreat trajectory planning failed: empty trajectory.");
    rclcpp::shutdown();
    return 1;
  }

  auto trajectory_dict = extractTrajectory(trajectory);
  printTrajectory(trajectory_dict);

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Retreat_Path", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(trajectory, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute retreat path");

  moveit::planning_interface::MoveGroupInterface::Plan retreat_plan;
  retreat_plan.trajectory = trajectory;
  move_group->execute(retreat_plan);

  visual_tools.prompt("Press 'next' to finish the demo");

  RCLCPP_INFO(LOGGER, "Retreat test complete!");
  rclcpp::shutdown();
  return 0;
}