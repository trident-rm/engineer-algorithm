#include "trajectory_utils.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Geometry>

std::map<std::string, std::vector<double>> extractTrajectory(
    const moveit_msgs::msg::RobotTrajectory& robot_trajectory)
{
    std::map<std::string, std::vector<double>> result;
    
    const auto& joint_traj = robot_trajectory.joint_trajectory;
    const auto& joint_names = joint_traj.joint_names;
    const auto& points = joint_traj.points;
    
    if (joint_names.empty() || points.empty()) {
        return result;
    }
    
    for (const auto& name : joint_names) {
        result[name] = std::vector<double>();
        result[name].reserve(points.size());
    }

    result["time_from_start"] = std::vector<double>();
    result["time_from_start"].reserve(points.size());
    
    for (const auto& point : points) {
        for (size_t i = 0; i < joint_names.size(); ++i) {
            if (i < point.positions.size()) {
                result[joint_names[i]].push_back(point.positions[i]);
            }
        }
        double time_seconds = point.time_from_start.sec + 
                             point.time_from_start.nanosec * 1e-9;
        result["time_from_start"].push_back(time_seconds);
    }
    
    return result;
}

std::map<std::string, std::vector<double>> extractAcceleration(
    const trajectory_msgs::msg::JointTrajectory& joint_trajectory)
{
    std::map<std::string, std::vector<double>> result;
    
    const auto& joint_names = joint_trajectory.joint_names;
    const auto& points = joint_trajectory.points;
    if (joint_names.empty() || points.empty()) {
        return result;
    }
    for (const auto& name : joint_names) {
        result[name] = std::vector<double>();
        result[name].reserve(points.size());
    }
    for (const auto& point : points) {
        for (size_t i = 0; i < joint_names.size(); ++i) {
            if (i < point.accelerations.size()) {
                result[joint_names[i]].push_back(point.accelerations[i]);
            }
        }
    }
    return result;
}



void printTrajectory(const std::map<std::string, std::vector<double>>& trajectory_dict)
{
    static const rclcpp::Logger LOGGER = rclcpp::get_logger("trajectory_utils");
    
    RCLCPP_INFO(LOGGER, "========================================");
    RCLCPP_INFO(LOGGER, "         Trajectory Data");
    RCLCPP_INFO(LOGGER, "========================================");
    
    auto time_it = trajectory_dict.find("time_from_start");
    if (time_it != trajectory_dict.end()) {
        const auto& times = time_it->second;

        RCLCPP_INFO(LOGGER, "⏱️  Time Information:");
        RCLCPP_INFO(LOGGER, "   Total duration: %.3f seconds", 
                    times.empty() ? 0.0 : times.back());
        RCLCPP_INFO(LOGGER, "   Number of waypoints: %zu", times.size());
        
        if (times.size() > 0) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(3);
            oss << "   Time points: [";
            for (size_t i = 0; i < times.size(); ++i) {
                oss << times[i] << "s";
                if (i < times.size() - 1) {
                    oss << ", ";
                }
            }
            oss << "]";
            RCLCPP_INFO(LOGGER, "%s", oss.str().c_str());
        }
    }
    RCLCPP_INFO(LOGGER, "🔧 Joint Positions:");
    for (const auto& [joint_name, positions] : trajectory_dict) {
        if (joint_name == "time_from_start") {
            continue;
        }

        RCLCPP_INFO(LOGGER, "   %s:", joint_name.c_str());
        RCLCPP_INFO(LOGGER, "     Points: %zu", positions.size());
        if (!positions.empty()) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(4);
            oss << "     [";
            
            // 打印所有点
            for (size_t i = 0; i < positions.size(); ++i) {
                oss << positions[i];
                if (i < positions.size() - 1) {
                    oss << ", ";
                }
            }
            oss << "]";
            RCLCPP_INFO(LOGGER, "%s", oss.str().c_str());
        }
    }
    
    RCLCPP_INFO(LOGGER, "========================================");
}

// 计算末端相对于base_link的位姿
geometry_msgs::msg::Pose compute_end_effector_pose(
    const std::vector<float>& joint_angles,
    const moveit::core::RobotModelConstPtr& robot_model,
    const moveit::core::JointModelGroup* joint_model_group,
    const std::string& end_effector_link)
{
    moveit::core::RobotStatePtr robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
    const std::vector<std::string>& joint_names = joint_model_group->getJointModelNames();

    if(joint_names.size() == 6 && joint_angles.size() == 6){
        for(size_t i = 0; i < 6; ++i){
            double angle = static_cast<double>(joint_angles[i]);
            robot_state->setJointPositions(joint_names[i], &angle);
        }
    } else {
        RCLCPP_WARN(LOGGER, "Joint count mismatch: expected 6, got %zu joints and %zu angles", 
            joint_names.size(), joint_angles.size());
        return geometry_msgs::msg::Pose(); 
    }
    
    robot_state->update();
    const Eigen::Isometry3d& end_effector_state = robot_state->getGlobalLinkTransform(end_effector_link);
    
    geometry_msgs::msg::Pose pose;
    pose.position.x = end_effector_state.translation().x();
    pose.position.y = end_effector_state.translation().y();
    pose.position.z = end_effector_state.translation().z();
    
    Eigen::Quaterniond quat(end_effector_state.rotation());
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();
    
    return pose;
}

std::vector<std::vector<float>> trajectory_2d_array(
    const std::map<std::string, std::vector<double>>& trajectory_dict,
    const std::vector<std::string>& joint_names)
{
    std::vector<std::vector<float>> result;
    if(trajectory_dict.empty() || joint_names.empty()){
        return result;
    }

    size_t num_points = trajectory_dict[joint_names[0]].size();
    result.reserve(num_points);
    for(size_t t = 0; t < num_points; ++t){
        std::vector<float> joint_angles_at_time(6, 0.0f);
        for(int i = 0; i < 6; ++i){
            joint_angles_at_time[i] = static_cast<float>(trajectory_dict[joint_names[i]][t]);
        }
        result.push_back(joint_angles_at_time);
    }
    return result;
}


