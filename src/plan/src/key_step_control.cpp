/** 
 * @file key_step_control.cpp
 * @brief  交互式测试：先移动到指定位置，再按rvizgui里的next按钮每次沿该方向移动 1cm，末端朝向保持不变
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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("key_step_control");

static const std::string planning_type = "LEFT";

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("key_step_control", node_options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();
  static const std::string PLANNING_GROUP = "arm";
  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, PLANNING_GROUP);
  move_group->setEndEffectorLink("gripper");  // 明确设置末端执行器为 gripper
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group =
      move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  namespace rvt = rviz_visual_tools;  
  moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link", "key_step_control",
                                                      move_group->getRobotModel());
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move to reset joint state");
  std::vector<double> zero_joint_values = {-1.55, -0.77, 0.67, 0.0, 1.37, -0.05};
  move_group->setJointValueTarget(zero_joint_values);
  moveit::planning_interface::MoveGroupInterface::Plan zero_plan;
  bool zero_success = false;
  while(!zero_success) {
    zero_success = (move_group->plan(zero_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!zero_success) {
      RCLCPP_WARN(LOGGER, "Failed to plan to zero joint state Retrying...");
    }
  }
  move_group->execute(zero_plan);
  while(true){
      visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the gripper by 1cm in corresponding direction");
      geometry_msgs::msg::Pose current_pose = move_group->getCurrentPose().pose;
      geometry_msgs::msg::Pose target_pose = current_pose;
      if (planning_type == "UP"){
        target_pose.position.z += 0.01;
      } else if (planning_type == "DOWN"){
        target_pose.position.z -= 0.01; 
      } else if (planning_type == "LEFT"){
        target_pose.position.y += 0.01; 
      } else if (planning_type == "RIGHT"){
        target_pose.position.y -= 0.01; 
      } else if (planning_type == "FORWARD"){
        target_pose.position.x += 0.01; 
      } else if (planning_type == "BACKWARD"){
        target_pose.position.x -= 0.01; 
      } 
      move_group->setPoseTarget(target_pose);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success = false;
      while(!success) {
        success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (!success) {
          RCLCPP_WARN(LOGGER, "Failed to plan motion. Retrying..."); 
        }
      }
      visual_tools.publishTrajectoryLine(plan.trajectory, joint_model_group);
      move_group->execute(plan);
    }

  rclcpp::shutdown();
  return 0;
}