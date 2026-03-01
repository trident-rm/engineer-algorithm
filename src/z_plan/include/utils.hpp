#ifndef Z_PLAN_UTILS_HPP
#define Z_PLAN_UTILS_HPP

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <map>
#include <vector>
#include <string>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <functional>

/**
 * @brief  从 JointTrajectory 中提取加速度信息
 * @param  joint_trajectory  关节轨迹消息
 * @retval std::map<std::string, std::vector<double>> 关节名到加速度序列的映射
 */
std::map<std::string, std::vector<double>> extractAcceleration(
    const trajectory_msgs::msg::JointTrajectory& joint_trajectory);

/**
 * @brief  从 RobotTrajectory 中提取位置和时间信息
 * @param  robot_trajectory  机器人轨迹消息
 * @retval std::map<std::string, std::vector<double>> 关节名到位置序列的映射（包含 "time_from_start"）
 */
std::map<std::string, std::vector<double>> extractTrajectory(
    const moveit_msgs::msg::RobotTrajectory& robot_trajectory);

/**
 * @brief  根据关节角度计算末端执行器相对于 base_link 的位姿
 * @param  joint_angles  关节角度数组
 * @param  robot_model  机器人模型
 * @param  joint_model_group  关节模型组
 * @param  end_effector_link  末端执行器连杆名称
 * @retval geometry_msgs::msg::Pose 末端执行器位姿
 */
geometry_msgs::msg::Pose compute_end_effector_pose(
    const std::vector<float>& joint_angles,
    const moveit::core::RobotModelConstPtr& robot_model,
    const moveit::core::JointModelGroup* joint_model_group,
    const std::string& end_effector_link);

/**
 * @brief  将轨迹字典转换为 2D 数组形式
 * @param  trajectory_dict  轨迹字典（关节名 -> 位置序列）
 * @param  joint_names  关节名称列表
 * @retval std::vector<std::vector<float>> 2D 数组，每行对应一个时刻的所有关节角度
 */
std::vector<std::vector<float>> trajectory_2d_array(
    const std::map<std::string, std::vector<double>>& trajectory_dict,
    const std::vector<std::string>& joint_names);

/**
 * @brief  打印轨迹信息到日志
 * @param  trajectory_dict  轨迹字典（关节名 -> 位置序列）
 * @retval None
 */
void printTrajectory(const std::map<std::string, std::vector<double>>& trajectory_dict);

/**
 * @brief  创建重力补偿计算闭包（使用 Pinocchio）
 * @param  robot_model  MoveIt 机器人模型
 * @retval std::function<std::vector<float>(const std::vector<double>&)> 
 *         返回一个函数，输入关节角度（double），输出重力补偿力矩（float）
 */
std::function<std::vector<float>(const std::vector<double>&)> make_gravity_closure(
    const moveit::core::RobotModelConstPtr& robot_model);

/**
 * @brief  计算沿指定方向移动指定距离后的目标位姿
 * @param  current_pose  当前位姿
 * @param  direction  移动方向的单位向量
 * @param  distance  移动距离（米）
 * @retval geometry_msgs::msg::Pose 目标位姿
 */
geometry_msgs::msg::Pose compute_target_pose_along_direction(
    const geometry_msgs::msg::Pose& current_pose,
    const Eigen::Vector3d& direction,
    double distance);

/**
 * @brief  计算沿轨迹路径的 e_hat 方向（指向科技核心）
 * @param  path  轨迹点的 2D 数组，每个点包含 [x, y, z] 坐标（相对于 base_link）
 *               轨迹沿 -e_hat 方向（远离科技核心）
 * @retval Eigen::Vector3d e_hat 方向单位向量（指向科技核心）
 */
Eigen::Vector3d compute_e_hat_direction(const std::vector<std::vector<double>>& path);

/**
 * @brief  对控制器输出的关节角度施加 e_hat 方向约束
 * @note   通过 FK 计算末端位姿，消除 e_hat 方向上的位移分量，然后通过 IK 求解新的关节角度
 * @param  controller_joint_angle  控制器输出的 6 个关节角度（弧度）
 * @param  reference_pose  参考位姿（用于确定约束平面）
 * @param  e_hat  e_hat 方向单位向量（指向科技核心）
 * @param  robot_model  MoveIt 机器人模型
 * @param  joint_model_group  关节模型组
 * @param  end_effector_link  末端执行器连杆名称
 * @param  joint_names  关节名称列表
 * @retval std::vector<std::vector<double>> 约束后的 6 个关节角度的数组（弧度）
 */
std::vector<std::vector<double>> e_hat_direction_constraint(
    const std::vector<double>& controller_joint_angle,
    const geometry_msgs::msg::Pose& reference_pose,
    const Eigen::Vector3d& e_hat,
    const moveit::core::RobotModelConstPtr& robot_model,
    const moveit::core::JointModelGroup* joint_model_group,
    const std::string& end_effector_link,
    const std::vector<std::string>& joint_names,
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group);

#endif
