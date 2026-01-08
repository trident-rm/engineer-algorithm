#ifndef Z_PLAN_TRAJECTORY_UTILS_HPP
#define Z_PLAN_TRAJECTORY_UTILS_HPP

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <map>
#include <vector>
#include <string>

std::map<std::string, std::vector<double>> extractTrajectory(
    const moveit_msgs::msg::RobotTrajectory& robot_trajectory);


void printTrajectory(const std::map<std::string, std::vector<double>>& trajectory_dict);

#endif
