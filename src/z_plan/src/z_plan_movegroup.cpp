#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "trajectory_utils.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("z_plan_movegroup");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("z_plan_movegroup", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "panda_link0", "z_plan_tutorial",
                                                      move_group.getRobotModel());

  visual_tools.deleteAllMarkers();

  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Z_Plan_Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();


  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));


  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.5;
  move_group.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  RCLCPP_INFO(LOGGER, "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose_Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute");


  RCLCPP_INFO(LOGGER, "Executing plan 1 (moving to pose goal)...");
  move_group.execute(my_plan);
  
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue with Cartesian path");

  // Cartesian Paths
  geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;
  
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(current_pose);

  // Move down along Z axis by 0.1m
  geometry_msgs::msg::Pose target_pose2 = current_pose;
  target_pose2.position.z -= 0.1;
  waypoints.push_back(target_pose2);

  // We want the Cartesian path to be interpolated at a resolution of 1 cm,
  // which is why we will specify 0.01 as the max step in Cartesian translation.
  const double eef_step = 0.01;
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory);

  // 提取并打印轨迹数据
  auto trajectory_dict = extractTrajectory(trajectory);
  printTrajectory(trajectory_dict);

  RCLCPP_INFO(LOGGER, "Visualizing Cartesian path (%.2f%% achieved)", fraction * 100.0);

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Cartesian_Path", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute Cartesian path");


  if (fraction > 0.99) {
    RCLCPP_INFO(LOGGER, "Executing Cartesian path...");
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory = trajectory;
    move_group.execute(cartesian_plan);
  } else {
    RCLCPP_WARN(LOGGER, "Cartesian path planning only achieved %.2f%% of the path", fraction * 100.0);
  }

  // END_TUTORIAL
  visual_tools.prompt("Press 'next' to finish the demo");

  RCLCPP_INFO(LOGGER, "Demo complete!");
  rclcpp::shutdown();
  return 0;
}
