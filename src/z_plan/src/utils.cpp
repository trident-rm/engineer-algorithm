#include <pinocchio/fwd.hpp>
#include "utils.hpp"
#include <iostream>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Geometry>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <urdf_model/pose.h>
#include <fstream>
#include <map>
#include <string>
#include <vector>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/force.hpp>
#include <pinocchio/algorithm/rnea.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("utils");

/**
 * @brief  根据时间戳和 position 计算加速度
 * @param  t_d  包含各关节位置和时间戳的字典
 * @param  joint_names  关节名称列表
 * @retval std::map<std::string, std::vector<double>>  包含各关节加速度的字典
 */
std::map<std::string, std::vector<double>> calculate_acceleration(const std::map<std::string, std::vector<double>> t_d, const std::vector<std::string>& joint_names){
    std::map<std::string, std::vector<double>> result;
    for (size_t i = 0; i < joint_names.size(); ++i) {
        std::vector<double> accelerations;
        for (size_t j = 1; j < t_d.at(joint_names[i]).size(); ++j) {
            double acceleration = (t_d.at(joint_names[i])[j] - t_d.at(joint_names[i])[j - 1]) / (t_d.at("time_from_start")[j] - t_d.at("time_from_start")[j - 1]);
            accelerations.push_back(acceleration);
        }
        result[joint_names[i] + "_acceleration"] = accelerations;
    }
    return result;
}

/**
 * @brief  从 MoveIt 轨迹消息中提取关节位置和时间信息
 * @param  robot_trajectory  MoveIt 机器人轨迹消息
 * @retval std::map<std::string, std::vector<double>>  包含各关节位置和时间戳的字典
 */
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
            } else {
                result[joint_names[i]].push_back(0.0);
            }
        }
        double time_seconds = point.time_from_start.sec + 
                             point.time_from_start.nanosec * 1e-9;
        result["time_from_start"].push_back(time_seconds);
    }
    
    return result;
}

/**
 * @brief  从 MoveIt 轨迹消息中提取关节加速度信息
 * @param  joint_trajectory  关节轨迹消息
 * @retval std::map<std::string, std::vector<double>>  包含各关节加速度的字典
 */
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


/**
 * @brief  打印轨迹信息（包含时间戳和各关节位置）
 * @param  trajectory_dict  包含轨迹数据的字典
 * @retval None
 */
void printTrajectory(const std::map<std::string, std::vector<double>>& trajectory_dict)
{
    static const rclcpp::Logger LOGGER = rclcpp::get_logger("utils");
    
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
}

/**
 * @brief  计算末端执行器相对于 base_link 的位姿
 * @param  joint_angles  关节角度列表
 * @param  robot_model  机器人模型指针
 * @param  joint_model_group  关节模型组指针
 * @param  end_effector_link  末端执行器 link 名称
 * @retval geometry_msgs::msg::Pose  末端执行器位姿
 */
geometry_msgs::msg::Pose compute_end_effector_pose(
    const std::vector<float>& joint_angles,
    const moveit::core::RobotModelConstPtr& robot_model,
    const moveit::core::JointModelGroup* joint_model_group,
    const std::string& end_effector_link)
{
    auto logger = rclcpp::get_logger("utils");
    moveit::core::RobotStatePtr robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
    // 使用活动（可驱动）关节列表，忽略 virtual_joint 等非驱动关节
    const std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();

    if (joint_names.size() == joint_angles.size()) {
        for (size_t i = 0; i < joint_names.size(); ++i) {
            double angle = static_cast<double>(joint_angles[i]);
            robot_state->setJointPositions(joint_names[i], &angle);
        }
    } else {
        RCLCPP_WARN(logger, "Joint count mismatch: expected %zu, got %zu joints and %zu angles", 
            joint_names.size(), joint_names.size(), joint_angles.size());
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

/**
 * @brief  将轨迹字典转换为 2D 数组形式
 * @param  trajectory_dict  包含各关节轨迹的字典
 * @param  joint_names  关节名称列表
 * @retval std::vector<std::vector<float>>  2D 数组，每个元素对应每一时刻所有关节的角度
 */
std::vector<std::vector<float>> trajectory_2d_array(
    const std::map<std::string, std::vector<double>>& trajectory_dict,
    const std::vector<std::string>& joint_names)
{
    std::vector<std::vector<float>> result;
    if(trajectory_dict.empty() || joint_names.empty()){
        return result;
    }

    size_t num_points = trajectory_dict.at(joint_names[0]).size();
    result.reserve(num_points);
    for(size_t t = 0; t < num_points; ++t){
        std::vector<float> joint_angles_at_time(joint_names.size(), 0.0f);
        for(size_t i = 0; i < joint_names.size(); ++i){
            joint_angles_at_time[i] = static_cast<float>(trajectory_dict.at(joint_names[i])[t]);
        }
        result.push_back(joint_angles_at_time);
    }
    return result;
}

/**
 * @brief  创建重力补偿计算闭包工厂函数
 * @note   从 MoveIt RobotModel 构建 Pinocchio 模型，返回一个可调用对象
 * @param  robot_model  MoveIt 机器人模型常量指针
 * @retval std::function<std::vector<float>(const std::vector<double>&)>  
 *         可调用对象，输入关节角度（double），返回重力补偿力矩（float）
 */
std::function<std::vector<float>(const std::vector<double>&)> make_gravity_closure(const moveit::core::RobotModelConstPtr& robot_model) {
    urdf::ModelInterfaceSharedPtr urdf_model = robot_model->getURDF();
    if (!urdf_model) {
        RCLCPP_ERROR(LOGGER, "Failed to obtain URDF from MoveIt RobotModel");
        return {};
    }
    RCLCPP_INFO(LOGGER, "make_gravity_closure: building model from URDF");

    pinocchio::Model model;
    try {
        pinocchio::urdf::buildModel(urdf_model, model);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(LOGGER, "pinocchio::urdf::buildModel exception: %s", e.what());
        return {};
    }
    
    // 打印所有关节名称以验证
    std::ostringstream joint_list;
    for (size_t i = 0; i < model.names.size(); ++i) {
        joint_list << model.names[i];
        if (i < model.names.size() - 1) joint_list << ", ";
    }
    RCLCPP_INFO(LOGGER, "Pinocchio model built: nq=%d, nv=%d, joints=[%s]", 
                model.nq, model.nv, joint_list.str().c_str());

    // move model into heap and capture shared_ptr in closure
    auto model_ptr = std::make_shared<pinocchio::Model>(std::move(model));

    return [model_ptr](const std::vector<double>& angles) -> std::vector<float> {
        //RCLCPP_INFO(LOGGER, "gravity closure called: input angles size=%zu, model nv=%d", angles.size(), model_ptr->nv);

        pinocchio::Data data(*model_ptr);

        Eigen::VectorXd q = Eigen::VectorXd::Map(angles.data(), static_cast<Eigen::Index>(angles.size()));
        Eigen::VectorXd v = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(model_ptr->nv));
        Eigen::VectorXd a = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(model_ptr->nv));

        Eigen::VectorXd tau = pinocchio::rnea(*model_ptr, data, q, v, a);

        std::vector<float> tau_f;
        // model.names includes base; collect torques for actuated joints
        for (size_t i = 0; i < model_ptr->names.size(); ++i) {
            if (i > 0 && i <= static_cast<size_t>(angles.size())) {
                tau_f.push_back(static_cast<float>(tau[static_cast<Eigen::Index>(i - 1)]));
            }
        }
        RCLCPP_INFO(LOGGER, "gravity closure returning %zu torques", tau_f.size());
        return tau_f;
    };
}

/**
 * @brief  计算沿指定方向移动指定距离后的目标位姿
 * @param  current_pose  当前位姿
 * @param  direction  移动方向的单位向量
 * @param  distance  移动距离（米）
 * @retval geometry_msgs::msg::Pose  目标位姿
 */
geometry_msgs::msg::Pose compute_target_pose_along_direction(
    const geometry_msgs::msg::Pose& current_pose,
    const Eigen::Vector3d& direction,
    double distance)
{
    geometry_msgs::msg::Pose target_pose = current_pose;
    
    // 沿方向向量移动指定距离
    Eigen::Vector3d direction_normalized = direction.normalized();
    target_pose.position.x += direction_normalized.x() * distance;
    target_pose.position.y += direction_normalized.y() * distance;
    target_pose.position.z += direction_normalized.z() * distance;
    
    return target_pose;
}

/**
 * @brief  对控制器输出的关节角度施加 e_hat 方向约束
 * @note   通过 FK 计算末端位姿，消除 e_hat 方向上的位移分量，然后通过 IK 求解新的关节角度
 * @param  controller_joint_angle  控制器输出的 6 个关节角度（弧度）
 * @param  reference_pose  参考位姿（用于确定约束平面）
 * @param  e_hat  e_hat 方向单位向量（指离科技核心）
 * @param  robot_model  MoveIt 机器人模型
 * @param  joint_model_group  关节模型组
 * @param  end_effector_link  末端执行器link名称
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
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group)
{
    moveit::core::RobotStatePtr robot_state = std::make_shared<moveit::core::RobotState>(robot_model);

    for (size_t i = 0; i < joint_names.size(); ++i) {
        robot_state->setJointPositions(joint_names[i], &controller_joint_angle[i]);
    }
    robot_state->update();
    
    const Eigen::Isometry3d& controller_ee_transform = robot_state->getGlobalLinkTransform(end_effector_link);
    Eigen::Vector3d controller_position = controller_ee_transform.translation();
    
    // 约束平面上的参考点
    Eigen::Vector3d reference_position(reference_pose.position.x,
                                       reference_pose.position.y,
                                       reference_pose.position.z);
    
    // 计算控制器输出位置在 e_hat 方向上的分量
    Eigen::Vector3d displacement = controller_position - reference_position;
    double e_hat_component = displacement.dot(e_hat); 
    
    //消除 e_hat 方向的分量，得到约束后的目标位置
    Eigen::Vector3d constrained_position = controller_position - e_hat_component * e_hat;
    
    RCLCPP_DEBUG(LOGGER, "Constrained position: [%.4f, %.4f, %.4f]", 
                 constrained_position.x(), constrained_position.y(), constrained_position.z());
    
    // 构造约束后的目标位姿（位置修正，姿态保持不变）
    geometry_msgs::msg::Pose constrained_pose;
    constrained_pose.position.x = constrained_position.x();
    constrained_pose.position.y = constrained_position.y();
    constrained_pose.position.z = constrained_position.z();
    constrained_pose.orientation = reference_pose.orientation; 


    std::vector<geometry_msgs::msg::Pose> waypoints;
    //waypoints.push_back(current_end_effector_pose);
    waypoints.push_back(constrained_pose);

    // 设置目标容差以提高 Cartesian 路径规划的成功率
    move_group->setGoalPositionTolerance(0.005);      // 5mm 位置容差
    move_group->setGoalOrientationTolerance(0.05);   // 0.05 弧度姿态容差

    const double eef_step = 0.01;
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group->computeCartesianPath(waypoints, eef_step, trajectory);
    if (fraction < 0.99) {
        RCLCPP_WARN(LOGGER, "Only %.2f%% of the path was planned successfully. Returning original angles.", fraction * 100.0);
        return {controller_joint_angle}; 
    }
    
    // 将 trajectory 转换为二维数组：每个元素是一帧的6个关节角度
    std::map<std::string, std::vector<double>> trajectory_dict = extractTrajectory(trajectory);
    std::vector<std::vector<float>> trajectory_2d = trajectory_2d_array(trajectory_dict, joint_names);
    
    // 转换为 double 类型的二维数组
    std::vector<std::vector<double>> result;
    result.reserve(trajectory_2d.size());
    for (const auto& frame : trajectory_2d) {
        std::vector<double> frame_double(frame.begin(), frame.end());
        result.push_back(frame_double);
    }
    
    RCLCPP_INFO(LOGGER, "Applied e_hat constraint: removed %.4f m displacement along e_hat, generated %zu trajectory frames", 
                e_hat_component, result.size());
    
    return result;
    
    /* 使用 MoveIt IK 求解约束后的关节角度
    从当前控制器角度作为 IK 的种子状态
    robot_state->setToDefaultValues();
    for (size_t i = 0; i < joint_names.size(); ++i) {
        robot_state->setJointPositions(joint_names[i], &controller_joint_angle[i]);
    }
    
    // 转换为 Eigen::Isometry3d
    Eigen::Isometry3d target_transform = Eigen::Isometry3d::Identity();
    target_transform.translation() = constrained_position;
    
    Eigen::Quaterniond target_quat(constrained_pose.orientation.w,
                                    constrained_pose.orientation.x,
                                    constrained_pose.orientation.y,
                                    constrained_pose.orientation.z);
    target_transform.linear() = target_quat.toRotationMatrix();
    
    // 调用 IK 求解
    double timeout = 0.1; 
    bool found_ik = robot_state->setFromIK(joint_model_group, target_transform, end_effector_link, timeout);
    
    if (!found_ik) {
        RCLCPP_WARN(LOGGER, "IK solution not found, using original controller angles");
        return controller_joint_angle;  // IK 失败，返回原值
    }
    
    // 提取 IK 结果
    std::vector<double> constrained_joint_angle(joint_names.size());
    for (size_t i = 0; i < joint_names.size(); ++i) {
        constrained_joint_angle[i] = *robot_state->getJointPositions(joint_names[i]);
    }
    */
}

