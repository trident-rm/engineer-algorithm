#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "utils.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("z_plan_movegroup");
static const geometry_msgs::msg::Pose energy_unit_1_pose = []() {
    Eigen::Matrix3d rot;
    rot << 0, 0, 1,
           0, 1, 0,
          -1, 0, 0;

          
    Eigen::Quaterniond q(rot);
    q.normalize();
    
    geometry_msgs::msg::Pose p;
    p.position.x = 0.3;
    p.position.y = 0.0;
    p.position.z = 0.525;
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();
    return p;
}();

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

  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
 
  move_group->setEndEffectorLink("gripper");
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "base_link", "z_plan_tutorial",
                                                      move_group.getRobotModel());

  visual_tools.deleteAllMarkers();

  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Z_Plan_Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  geometry_msgs::msg::Pose target_pose = energy_unit_1_pose;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
    
  moveit::core::MoveItErrorCode rc = moveit::core::MoveItErrorCode::PLANNING_FAILED;
  auto start_state = std::make_shared<moveit::core::RobotState>(mg_->getRobotModel());
  start_state.setJointGroupPositions(joint_model_group, {0.0,0.0,0.0,0.0,0.0,0.0})
  move_group->setStartState(*start_state)
  const double SEARCH_RADIUS = 0.15;  // 15cm范围
  const int MAX_ATTEMPTS = 10;
  for (int attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
            move_group->setPoseTarget(target_pose, move_group->getEndEffectorLink());
            rc = move_group->plan(plan);
            
            if (rc == moveit::core::MoveItErrorCode::SUCCESS) {
                std::ofstream outfile("trajectory.msg")
                outfile<<plan.trajectory;
                RCLCPP_INFO(LOGGER, "Plan to energy_unit_%d succeeded at attempt %d", unit_id, attempt + 1);
                break;
            }
            
            // 如果失败，在目标的x、y坐标范围内尝试邻近位置
            if (attempt < MAX_ATTEMPTS - 1) {
                // 使用网格搜索：在x、y方向各尝试多个偏移 (3x3网格 = 9种位置)
                int grid_index = attempt % 9;
                double offset_x = ((grid_index % 3) - 1) * SEARCH_RADIUS / 2;  // [-0.075, 0, 0.075]
                double offset_y = ((grid_index / 3) - 1) * SEARCH_RADIUS / 2;  // [-0.075, 0, 0.075]
                
                target_pose.position.x = original_target.position.x + offset_x;
                target_pose.position.y = original_target.position.y + offset_y;
                target_pose.position.z = original_target.position.z;
                target_pose.orientation = original_target.orientation;
                
                RCLCPP_WARN(LOGGER, "Plan to energy_unit_%d failed at attempt %d, trying nearby position (offset: %.3f, %.3f)", 
                    unit_id, attempt + 1, offset_x, offset_y);
            }
        }
  
  

  visual_tools.publishTrajectoryLine(plan.trajectory;.trajectory, joint_model_group);
  visual_tools.trigger();
  
  rclcpp::shutdown();
  return 0;
}
