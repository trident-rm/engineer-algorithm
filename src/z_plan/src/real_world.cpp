#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Geometry>
#include <chrono>
#include <thread>
#include <map>
#include <vector>
#include "trajectory_utils.hpp"
#include "usb_data.hpp" 

static const rclcpp::Logger LOGGER = rclcpp::get_logger("z_plan_real_world");

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("z_plan_real_world", node_options);

    auto serial_node = std::make_shared<SerialRxNode>();
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    executor.add_node(serial_node); 
    std::thread([&executor]() { executor.spin(); }).detach();
    
    // 启动 SerialRxNode 的串口接收线程
    std::thread([serial_node]() { 
        serial_node->receive_data(); 
    }).detach();

    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    moveit::core::RobotModelConstPtr robot_model = move_group.getRobotModel();
    const std::string end_effector_link = move_group.getEndEffectorLink();

    const std::vector<std::string>& joint_names = joint_model_group->getJointModelNames();
    
    auto joint_state_sub = move_group_node->create_subscription<sensor_msgs::msg::JointState>(
        "/usb_joint_states", 10,
        [&](const sensor_msgs::msg::JointState::SharedPtr msg) {
            if(msg->position.size() >= 6){
                std::vector<float> joint_angles(msg->position.begin(), msg->position.begin() + 6);
                geometry_msgs::msg::Pose current_end_effector_pose = compute_end_effector_pose(
                    joint_angles, robot_model, joint_model_group, end_effector_link);
                
                RCLCPP_INFO(LOGGER, "Current end effector pose - Position: [%.3f, %.3f, %.3f], Orientation: [%.3f, %.3f, %.3f, %.3f]",
                    current_end_effector_pose.position.x,
                    current_end_effector_pose.position.y,
                    current_end_effector_pose.position.z,
                    current_end_effector_pose.orientation.x,
                    current_end_effector_pose.orientation.y,
                    current_end_effector_pose.orientation.z,
                    current_end_effector_pose.orientation.w);
                
                std::vector<geometry_msgs::msg::Pose> waypoints;
                waypoints.push_back(current_end_effector_pose);
                geometry_msgs::msg::Pose target_end_effector_pose = current_end_effector_pose;
                target_end_effector_pose.position.z -= 0.1;
                waypoints.push_back(target_end_effector_pose);
                const double eef_step = 0.01;
                moveit_msgs::msg::RobotTrajectory trajectory;
                double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory);
                
                if(fraction < 0.9){
                    RCLCPP_WARN(LOGGER, "Failed to compute Cartesian path, fraction: %.2f", fraction);
                    return; 
                }
                
                RCLCPP_INFO(LOGGER, "Computed Cartesian path with fraction: %.2f", fraction);
                auto trajectory_dict = extractTrajectory(trajectory);
                printTrajectory(trajectory_dict);
                std::vector<std::vector<float>> trajectory_array = trajectory_2d_array(
                    trajectory_dict, joint_names);
                
                RCLCPP_INFO(LOGGER, "Trajectory converted to 2D array with %zu time points", trajectory_array.size());

                for(const auto& joint_angles : trajectory_array){
                    if(joint_angles.size() == 6){
                        serial_node->send_joint_angles(joint_angles, 0);
                        std::this_thread::sleep_for(std::chrono::milliseconds(2));
                    } else {
                        RCLCPP_WARN(LOGGER, "Invalid joint angles size: %zu, expected 6", joint_angles.size());
                    }
                }
                RCLCPP_INFO(LOGGER, "Finished sending trajectory to serial port");
            }
        });

    RCLCPP_INFO(LOGGER, "Waiting for joint states and computing Cartesian path...");
    while(rclcpp::ok()){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    rclcpp::shutdown();
    return 0;
}