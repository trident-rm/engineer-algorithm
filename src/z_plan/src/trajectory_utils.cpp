#include "z_plan/trajectory_utils.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <rclcpp/rclcpp.hpp>

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
    
    // 为每个关节初始化数组
    for (const auto& name : joint_names) {
        result[name] = std::vector<double>();
        result[name].reserve(points.size());
    }
    
    // 为 time_from_start 初始化数组
    result["time_from_start"] = std::vector<double>();
    result["time_from_start"].reserve(points.size());
    
    // 提取每个点的位置和时间
    for (const auto& point : points) {
        // 提取关节位置
        for (size_t i = 0; i < joint_names.size(); ++i) {
            if (i < point.positions.size()) {
                result[joint_names[i]].push_back(point.positions[i]);
            }
        }
        
        // 提取时间戳 (duration 转换为秒)
        // duration 类型: builtin_interfaces::msg::Duration
        //   - int32 sec
        //   - uint32 nanosec
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

/*
std::map<std::string, std::vector<double>> calculate_current(std::map<std::string, std::vector<double>> acceleration){
    for (const auto& joint : acceleration) {
        std::string joint_name = joint.first;
        std::vector<double> acc_values = joint.second; 
           
    //get acceleration
    //calculate torque using zhonglibuchangfa
    //calculate current


}
*/


