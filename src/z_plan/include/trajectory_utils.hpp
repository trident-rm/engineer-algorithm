#ifndef Z_PLAN_TRAJECTORY_UTILS_HPP
#define Z_PLAN_TRAJECTORY_UTILS_HPP

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <map>
#include <vector>
#include <string>
#include <moveit/robot_model/robot_model.hpp>

std::map<std::string, std::vector<double>> extractTrajectory(
    const moveit_msgs::msg::RobotTrajectory& robot_trajectory);

geometry_msgs::msg::Pose compute_end_effector_pose(
    const std::vector<float>& joint_angles,
    const moveit::core::RobotModelConstPtr& robot_model,
    const moveit::core::JointModelGroup* joint_model_group,
    const std::string& end_effector_link);

std::vector<std::vector<float>> trajectory_2d_array(
    const std::map<std::string, std::vector<double>>& trajectory_dict,
    const std::vector<std::string>& joint_names);

void printTrajectory(const std::map<std::string, std::vector<double>>& trajectory_dict);

#endif
