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
  move_group->setEndEffectorLink("gripper");  // 明确设置末端执行器为 gripper

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const moveit::core::JointModelGroup* joint_model_group =
      move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "base_link", "turn_plan",
                                                      move_group->getRobotModel());

  visual_tools.deleteAllMarkers();

  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Z_Plan_RoboArm_Demo", rvt::BLACK, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();


  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group->getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group->getEndEffectorLink().c_str());

  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  for (const auto& group_name : move_group->getJointModelGroupNames()) {
    RCLCPP_INFO(LOGGER, " - %s", group_name.c_str());
  }

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

  visual_tools.publishText(text_pose, "Move_To_Zero", rvt::BLACK, rvt::XLARGE);
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

  // 生成并可视化 waypoint（用于验证输入）
  visual_tools.deleteAllMarkers();
  RCLCPP_INFO(LOGGER, "Start pose: [%.4f, %.4f, %.4f]", 
    start_pose.position.x, start_pose.position.y, start_pose.position.z);
  
  const double angle_range = -M_PI/6;
  Eigen::Vector3d start_pos(
      start_pose.position.x,
      start_pose.position.y,
      start_pose.position.z);

  Eigen::Vector3d world_center = start_pos - x_axis_world * radius;
  Eigen::Vector3d radius_vec0 = start_pos - world_center;

  RCLCPP_INFO(LOGGER, "Half-circle center: [%.4f, %.4f, %.4f]", 
    world_center.x(), world_center.y(), world_center.z());
  RCLCPP_INFO(LOGGER, "Radius: %.4f, Angle range: %.4f rad, Num points: %d",
    radius, angle_range, num_points);

  // 绘制圆心
  Eigen::Isometry3d center_marker = Eigen::Isometry3d::Identity();
  center_marker.translation() = world_center;
  visual_tools.publishSphere(center_marker, rvt::RED, rvt::MEDIUM, "center");

  // 绘制所有 waypoint
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(start_pose);
  
  // 绘制起始点
  Eigen::Isometry3d start_marker;
  start_marker.translation() = start_pos;
  start_marker.linear() = start_quat.toRotationMatrix();
  visual_tools.publishSphere(start_marker, rvt::GREEN, rvt::MEDIUM, "start_point");
  // 绘制起始点朝向坐标系
  visual_tools.publishAxisLabeled(start_marker, "start_axis", rvt::SMALL);

  for (int i = 1; i <= num_points; ++i) {
    double angle = angle_range * static_cast<double>(i) / static_cast<double>(num_points);
    Eigen::AngleAxisd aa(angle, y_axis_world);
    Eigen::Vector3d world_point = world_center + aa * radius_vec0;

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = world_point.x();
    target_pose.position.y = world_point.y();
    target_pose.position.z = world_point.z();

    // 姿态半圆：末端（gripper）x 轴绕 y_axis_world 旋转
    Eigen::Vector3d x_target = (aa * x_axis_world).normalized();
    Eigen::Vector3d y_target = y_axis_world;
    Eigen::Vector3d z_target = x_target.cross(y_target).normalized();

    Eigen::Matrix3d target_rot;
    target_rot.col(0) = x_target;
    target_rot.col(1) = y_target;
    target_rot.col(2) = z_target;

    Eigen::Quaterniond target_quat(target_rot);
    target_quat.normalize();
    target_pose.orientation.x = target_quat.x();
    target_pose.orientation.y = target_quat.y();
    target_pose.orientation.z = target_quat.z();
    target_pose.orientation.w = target_quat.w();
    waypoints.push_back(target_pose);

    // 绘制 waypoint

    Eigen::Isometry3d waypoint_marker;
    waypoint_marker.translation() = world_point;
    waypoint_marker.linear() = target_rot;
    if (i == num_points) {
        visual_tools.publishSphere(waypoint_marker, rvt::BLUE, rvt::MEDIUM, "waypoint_" + std::to_string(i));
        // 绘制终点朝向坐标系
        visual_tools.publishAxisLabeled(waypoint_marker, "waypoint_axis_" + std::to_string(i), rvt::SMALL);
      } else if (i % 8 == 0) {
        visual_tools.publishSphere(waypoint_marker, rvt::YELLOW, rvt::SMALL, "waypoint_" + std::to_string(i));
        // 绘制中间waypoint的朝向坐标系（每8个点绘制一个）
        visual_tools.publishAxisLabeled(waypoint_marker, "waypoint_axis_" + std::to_string(i), rvt::SMALL);
      } else {
        visual_tools.publishSphere(waypoint_marker, rvt::YELLOW, rvt::SMALL, "waypoint_" + std::to_string(i));
      }
  
  }

  visual_tools.publishText(text_pose, "Waypoints_Preview", rvt::BLACK, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' to visualize waypoints and proceed to trajectory planning");

  const Eigen::Vector3d normal_direction = y_axis_world;
  const Eigen::Vector3d e_hat_direction = x_axis_world;
  const Eigen::Vector3d start_point(start_pose.position.x, start_pose.position.y, start_pose.position.z);

  moveit_msgs::msg::RobotTrajectory trajectory =
      plan_half_circle_turn(start_pose, radius, num_points, normal_direction, e_hat_direction, move_group);

  if (trajectory.joint_trajectory.points.empty()) {
    RCLCPP_ERROR(LOGGER, "Half-circle trajectory planning failed: empty trajectory.");
    rclcpp::shutdown();
    return 1;
  }

  auto trajectory_dict = extractTrajectory(trajectory);
  printTrajectory(trajectory_dict);

  RCLCPP_INFO(LOGGER, "Visualizing half-circle Cartesian path in base_link xz plane (radius=%.3fm)", radius);
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Half_Circle_Path", rvt::BLACK, rvt::XLARGE);
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
