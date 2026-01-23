#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Geometry>
#include <moveit/robot_state/conversions.hpp>
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

    const std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();
    // publisher for displaying robot state in RViz
    auto display_pub = move_group_node->create_publisher<moveit_msgs::msg::DisplayRobotState>("/display_robot_state", 10);
    
    auto joint_state_sub = move_group_node->create_subscription<sensor_msgs::msg::JointState>(
        "/usb_joint_states", 10,
        [&](const sensor_msgs::msg::JointState::SharedPtr msg) {
            if (!msg) {
                RCLCPP_WARN(LOGGER, "Received null JointState");
                return;
            }
            if (msg->position.size() < joint_names.size()) {
                RCLCPP_WARN(LOGGER, "Received JointState with %zu positions but %zu active joints expected", msg->position.size(), joint_names.size());
                return;
            }

            // Build a RobotState from the received joint angles (map by active joint order)
            float original_value2 = msg->position[1];
            float original_value5 = msg->position[4];
            msg->position[1] = -original_value2;
            msg->position[4] = 6.28-original_value5;

            moveit::core::RobotState start_state(robot_model);
            start_state.setToDefaultValues();
            for (size_t i = 0; i < joint_names.size(); ++i) {
                double v = static_cast<double>(msg->position[i]);
                start_state.setJointPositions(joint_names[i], &v);
            }
            start_state.update();

            // Set this as the MoveGroup start state so planning uses the received joint angles
            move_group.setStartState(start_state);

            // Publish robot state to RViz
            moveit_msgs::msg::RobotState robot_state_msg;
            moveit::core::robotStateToRobotStateMsg(start_state, robot_state_msg);
            moveit_msgs::msg::DisplayRobotState display_msg;
            display_msg.state = robot_state_msg;
            display_pub->publish(display_msg);
        Eigen::Isometry3d ee_transform = start_state.getGlobalLinkTransform(end_effector_link);
        geometry_msgs::msg::Pose current_end_effector_pose;
        // Manual conversion from Eigen::Isometry3d to geometry_msgs::msg::Pose
        Eigen::Vector3d trans = ee_transform.translation();
        Eigen::Quaterniond q(ee_transform.rotation());
        current_end_effector_pose.position.x = trans.x();
        current_end_effector_pose.position.y = trans.y();
        current_end_effector_pose.position.z = trans.z();
        current_end_effector_pose.orientation.x = q.x();
        current_end_effector_pose.orientation.y = q.y();
        current_end_effector_pose.orientation.z = q.z();
        current_end_effector_pose.orientation.w = q.w();
            
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
            target_end_effector_pose.position.z += 0.1;
            waypoints.push_back(target_end_effector_pose);

            const double eef_step = 0.01;
            moveit_msgs::msg::RobotTrajectory trajectory;
            double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory);

            if (fraction < 0.9) {
                RCLCPP_WARN(LOGGER, "Failed to compute Cartesian path, fraction: %.2f", fraction);
                return;
            }

            RCLCPP_INFO(LOGGER, "Computed Cartesian path with fraction: %.2f", fraction);
            auto trajectory_dict = extractTrajectory(trajectory);
            printTrajectory(trajectory_dict);
            std::vector<std::vector<float>> trajectory_array = trajectory_2d_array(trajectory_dict, joint_names);

            RCLCPP_INFO(LOGGER, "Trajectory converted to 2D array with %zu time points", trajectory_array.size());

            for (const auto &ja : trajectory_array) {
                if (ja.size() == joint_names.size()) {
                    // Before sending to serial, invert joint2 (index 1) if present
                    std::vector<float> send_angles = ja;
                    if (joint_names.size() > 1) {
                        send_angles[1] = -send_angles[1];
                    }
                    serial_node->send_joint_angles(send_angles, 0);
                    RCLCPP_INFO(LOGGER, "Sent joint angles to serial");
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                } else {
                    RCLCPP_WARN(LOGGER, "Invalid joint angles size: %zu, expected %zu", ja.size(), joint_names.size());
                }
            }
        });

    while(rclcpp::ok()){
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        RCLCPP_INFO(LOGGER, "Waiting for joint states and computing Cartesian path...");
    }

    rclcpp::shutdown();
    return 0;
}