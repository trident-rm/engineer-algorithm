#ifndef Z_PLAN_UTILS_HPP
#define Z_PLAN_UTILS_HPP

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <map>
#include <vector>
#include <string>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <functional>
#include <Eigen/Dense>

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
 * @brief  对控制器末端位置施加 e_hat 方向约束，并让末端朝向完全跟随 controller
 *
 * 算法（与 test_constraint.cpp::constrained_step() 一致）：
 *   1. FK(controller_angles)  → controller_ee
 *   2. FK(robo_arm_angles)    → current_ee
 *   3. constrained_target = reference_position + (controller_ee - reference_position) 投影到 e_hat 垂直平面
 *   4. velocity = constrained_target - current_ee（再次去掉 e_hat 分量，双重保险）
 *   5. 死区（< DEAD_ZONE）直接返回 robo_arm_angles（不动）
 *   6. 限速（velocity.norm() ≤ MAX_SPEED）
 *   7. J = 完整 Jacobian (6×n)；SVD 计算条件数
 *   8. 自适应阻尼：cond ∈ [COND_LOW, COND_HIGH] → λ ∈ [λ_min, λ_max]
 *   9. J_pinv = Jᵀ(JJᵀ + λ²I)⁻¹；Δq = J_pinv · [velocity, omega] · dt
 *  10. 等比缩放 Δq（MAX_JOINT_DELTA = 0.1 rad/cycle）
 *  11. 关节限位裕度等比缩放
 *
 * @param  robo_arm_angles         当前机械臂关节角度 (rad)
 * @param  controller_angles       控制器关节角度 (rad)
 * @param  reference_position      模式切换瞬间机械臂末端位置 (m)，作为约束平面原点
 * @param  controller_ee_reference 模式切换瞬间控制器末端位置 (m)，作为位移原点（切换瞬间=0，避免跳变）
 * @param  constrain_direction     约束方向单位向量（此方向上末端不可运动）
 * @param  robot_model             MoveIt 机器人模型
 * @param  joint_model_group       关节模型组
 * @param  end_effector_link       末端执行器连杆名称
 * @param  joint_names             关节名称列表
 * @param  dt                      控制增益（等效时间步长），默认 0.05
 * @param  arm_state               可选：外部已 update() 的机械臂 RobotState；非 nullptr 时跳过内部 FK 重建，
 *                                 节省约 2/3 的重复 FK 开销（每周期在 control_callback 统一更新一次）
 * @param  desired_ee_orientation  兼容参数（当前实现固定跟随 controller 朝向，不使用）
 * @param  change_ee_orientation_allowed 兼容参数（true: 允许末端朝向变化；false: 锁定末端朝向）
 * @retval std::vector<double>     约束后一步目标关节角度（死区时返回当前角度，失败时返回空）
 */
std::vector<double> direction_constraint(
    const std::vector<double>& robo_arm_angles,
    const std::vector<double>& controller_angles,
    const Eigen::Vector3d& reference_position,
    const Eigen::Vector3d& controller_ee_reference,
    const Eigen::Vector3d& constrain_direction,
    const moveit::core::RobotModelConstPtr& robot_model,
    const moveit::core::JointModelGroup* joint_model_group,
    const std::string& end_effector_link,
    const std::vector<std::string>& joint_names,
    double dt = 0.05,
    moveit::core::RobotState* arm_state = nullptr,
    const Eigen::Quaterniond* desired_ee_orientation = nullptr,
    const bool change_ee_orientation_allowed = false);

/**
 * @brief  TURN 模式半圆轨迹规划（笛卡尔路径）
 * @param  center           半圆圆心（base_link 坐标系）
 * @param  start_pose       起始末端位姿
 * @param  radius           半径（米）
 * @param  num_points       离散点数量
 * @param  up_direction     半圆平面内“上”方向单位向量
 * @param  e_hat_direction  半圆平面内起始径向单位向量
 * @param  move_group       MoveGroup 接口
 * @retval moveit_msgs::msg::RobotTrajectory 规划结果
 */
moveit_msgs::msg::RobotTrajectory plan_half_circle_turn(
    geometry_msgs::msg::Pose start_pose,
    double radius,
    int num_points,
    Eigen::Vector3d up_direction,
    Eigen::Vector3d e_hat_direction,
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group);

/**
 * @brief  计算末端沿自身 z 轴方向撤退的笛卡尔路径
 * @param  current_pose      当前末端位姿
 * @param  retreat_distance  撤退距离（米，通常为负值）
 * @param  move_group        MoveGroup 接口
 * @retval moveit_msgs::msg::RobotTrajectory 规划结果
 */
moveit_msgs::msg::RobotTrajectory plan_retreat(
    geometry_msgs::msg::Pose current_pose,
    double retreat_distance,
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group);

/**
 * @brief  URDF 角度 → 下位机角度转换
 * @param  urdf_angles  URDF 坐标系下的关节角度
 * @retval std::vector<float> 硬件坐标系下的关节角度
 */
std::vector<float> convert_urdf_to_hardware(const std::vector<float>& urdf_angles);
std::vector<float> convert_urdf_to_hardware(const std::vector<double>& urdf_angles);

/**
 * @brief  下位机角度 → URDF 角度转换
 * @param  hardware_angles  硬件坐标系下的关节角度
 * @retval std::vector<double> URDF 坐标系下的关节角度
 */
std::vector<double> convert_hardware_to_urdf(const std::vector<float>& hardware_angles);
std::vector<double> convert_hardware_to_urdf(const std::vector<double>& hardware_angles);

#endif
