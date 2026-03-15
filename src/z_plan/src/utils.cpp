#include <pinocchio/fwd.hpp>
#include "utils.hpp"
#include <iostream>
#include <iomanip>
#include <memory>
#include <moveit/robot_state/robot_state.hpp>
#include <Eigen/Dense>
#include <Eigen/SVD>

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
        RCLCPP_DEBUG(LOGGER, "gravity closure returning %zu torques", tau_f.size());
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
 * @brief  对控制器末端位置施加 e_hat 方向约束，用雅可比伪逆（自适应阻尼最小二乘）求一步关节增量
 * @note   算法与 test_constraint.cpp::constrained_step() 一致
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
    double dt,
    moveit::core::RobotState* arm_state,
    const Eigen::Quaterniond* desired_ee_orientation,
    const bool change_link6_orientation_allowed)
{
    // 兼容旧接口：当前实现始终“朝向完全跟随 controller”
    (void)desired_ee_orientation;
    (void)change_link6_orientation_allowed;

    // ── Step 1: FK(controller) → controller_ee ────────────────────────────────
    moveit::core::RobotState controller_state(robot_model);
    for (size_t i = 0; i < joint_names.size(); ++i)
        controller_state.setJointPositions(joint_names[i], &controller_angles[i]);
    controller_state.update();
    Eigen::Vector3d controller_ee =
        controller_state.getGlobalLinkTransform(end_effector_link).translation();

    // ── Step 2: FK(robo_arm) → current_ee ────────────────────────────────────
    // 若外部传入已 update() 的 arm_state 则直接复用
    std::unique_ptr<moveit::core::RobotState> local_state;
    if (!arm_state) {
        local_state = std::make_unique<moveit::core::RobotState>(robot_model);
        for (size_t i = 0; i < joint_names.size(); ++i)
            local_state->setJointPositions(joint_names[i], &robo_arm_angles[i]);
        local_state->update();
        arm_state = local_state.get();
    }
    Eigen::Vector3d current_ee =
        arm_state->getGlobalLinkTransform(end_effector_link).translation();

    // ── Step 3: 相对位移投影到约束平面，得到约束目标 ─────────────────────────
    // displacement 以模式切换时控制器 EE 位置（controller_ee_reference）为原点：
    //   切换瞬间 controller_ee == controller_ee_reference → displacement=0，避免跳变。
    // reference_position（机械臂切换时 EE）作为约束平面的空间原点。
    // 
    // 约束平面容差机制：允许在约束方向上有一定的自由度（tolerance），缓冲模式切换时的冲击
    const double CONSTRAINT_TOLERANCE = 0.03;  // 30mm tolerance
    Eigen::Vector3d constrain_direction_normalized = constrain_direction.normalized();
    Eigen::Vector3d displacement = controller_ee - controller_ee_reference;
    double constrain_component = displacement.dot(constrain_direction_normalized);
    
    Eigen::Vector3d constrained_target;
    if (std::abs(constrain_component) > CONSTRAINT_TOLERANCE) {
        // 超过 tolerance：应用约束，但保留 tolerance 范围内的移动
        double sign = (constrain_component > 0) ? 1.0 : -1.0;
        double constrain_excess = std::abs(constrain_component) - CONSTRAINT_TOLERANCE;
        constrained_target = reference_position + 
                           (displacement - (constrain_component - sign * constrain_excess) * constrain_direction_normalized);
    } else {
        // 在 tolerance 范围内：直接跟随controller，不施加约束
        constrained_target = reference_position + displacement;
    }

    // ── Step 4: 速度方向
    // 平面内跟踪分量（去除 constrain_direction 分量，保证目标只在平面内）
    Eigen::Vector3d velocity = constrained_target - current_ee;
    //velocity = velocity - velocity.dot(constrain_direction_normalized) * constrain_direction_normalized;

    // 平面漂移修正：若 current_ee 因雅可比数值误差偏出约束平面，
    // 则沿 constrain_direction 方向施加回平面弹簧力（该分量不参与下方投影，直接叠加）。
    // gain=2.0：每周期消除 ~18% 漂移量，比跟踪增益稍快，可在数个周期内将 EE 拉回平面。
    
    double drift_along_constraint = (current_ee - reference_position).dot(constrain_direction_normalized);
    const double DRIFT_CORRECTION_GAIN = 2.0;
    velocity -= DRIFT_CORRECTION_GAIN * drift_along_constraint * constrain_direction_normalized;
    

    // ── Step 4b: 朝向完全跟随 controller（世界坐标角速度误差）────────────────
    Eigen::Matrix3d current_rot =
        arm_state->getGlobalLinkTransform(end_effector_link).rotation();
    Eigen::Quaterniond current_q(current_rot);

    Eigen::Matrix3d ctrl_rot =
        controller_state.getGlobalLinkTransform(end_effector_link).rotation();
    Eigen::Quaterniond ctrl_q(ctrl_rot);

    Eigen::Quaterniond q_err = ctrl_q * current_q.inverse();
    if (q_err.w() < 0) q_err.coeffs() = -q_err.coeffs();  // 取短弧
    Eigen::Vector3d omega = 2.0 * q_err.vec();

    const double MAX_ANGULAR_SPEED = 1.0;  // rad/s
    double omega_norm = omega.norm();
    if (omega_norm > MAX_ANGULAR_SPEED) {
        omega *= (MAX_ANGULAR_SPEED / omega_norm);
    }

    // ── Step 5: 死区（位置+朝向都很小时不动）────────────────────────────────
    double position_error = velocity.norm();
    double orient_error = omega.norm();
    const double DEAD_ZONE = 0.0005;   // m
    const double ORIENT_DEAD_ZONE = 0.01;  // rad
    if (position_error < DEAD_ZONE && orient_error < ORIENT_DEAD_ZONE) {
        return robo_arm_angles;
    }

    // ── Step 6: 限速（max 0.45 m/s）──────────────────────────────────────────
    const double MAX_SPEED = 0.45;   // m/s
    if (position_error > MAX_SPEED) {
        velocity *= (MAX_SPEED / position_error);
    }

    // ── Step 7: 雅可比矩阵（位置+朝向，使用完整 6 × n）──────────────────────
    Eigen::MatrixXd full_jacobian;
    Eigen::Vector3d ref_pt(0.0, 0.0, 0.0);
    if (!arm_state->getJacobian(
            joint_model_group,
            arm_state->getLinkModel(end_effector_link),
            ref_pt, full_jacobian)) {
        RCLCPP_ERROR(LOGGER, "direction_constraint: failed to compute Jacobian");
        return {};
    }

    Eigen::MatrixXd J = full_jacobian;   // 6 × n
    Eigen::VectorXd vel_cmd(6);
    vel_cmd << velocity, omega;

    // ── Step 8: SVD → 条件数 → 自适应阻尼 ───────────────────────────────────
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
    const Eigen::VectorXd& sv = svd.singularValues();
    double condition_number = (sv(sv.size() - 1) > 1e-10)
                              ? sv(0) / sv(sv.size() - 1)
                              : 1e6;

    const double DAMPING_MIN = 0.005;
    const double DAMPING_MAX = 0.1;
    const double COND_LOW    = 50.0;
    const double COND_HIGH   = 200.0;
    double damping;
    if (condition_number <= COND_LOW) {
        damping = DAMPING_MIN;
    } else if (condition_number >= COND_HIGH) {
        damping = DAMPING_MAX;
    } else {
        double t = (condition_number - COND_LOW) / (COND_HIGH - COND_LOW);
        damping = DAMPING_MIN + t * (DAMPING_MAX - DAMPING_MIN);
    }

    // ── Step 9: J_pinv = Jᵀ(JJᵀ + λ²I)⁻¹；Δq = J_pinv · vel_cmd · dt ──────
    int jrows = static_cast<int>(J.rows());
    Eigen::MatrixXd JJt = J * J.transpose()
                        + (damping * damping) * Eigen::MatrixXd::Identity(jrows, jrows);
    Eigen::MatrixXd J_pinv = J.transpose() * JJt.inverse();   // n × jrows
    Eigen::VectorXd delta_q = J_pinv * vel_cmd * dt;

    // ── Step 9b: 关节空间牵引（投影到零空间，避免干扰主任务）────────────────
    {
        const double JOINT_SYNC_GAIN = 0.3;   // rad/s per rad error
        int n = static_cast<int>(robo_arm_angles.size());
        Eigen::VectorXd delta_q_sync = Eigen::VectorXd::Zero(n);
        for (int i = 0; i < n; ++i) {
            double joint_err = controller_angles[i] - robo_arm_angles[i];
            delta_q_sync(i) = JOINT_SYNC_GAIN * joint_err * dt;
        }

        Eigen::MatrixXd N = Eigen::MatrixXd::Identity(n, n) - J_pinv * J;
        delta_q += N * delta_q_sync;
    }

    // ── Step 10: 关节角变化量等比缩放（硬限幅 0.1 rad/cycle）────────────────
    const double MAX_JOINT_DELTA = 0.1;   // rad
    double max_dq = delta_q.cwiseAbs().maxCoeff();
    if (max_dq > MAX_JOINT_DELTA) {
        delta_q *= (MAX_JOINT_DELTA / max_dq);
    }

    // ── Step 11: 关节限位裕度等比缩放（MARGIN = 0.05 rad）───────────────────
    const double JOINT_LIMIT_MARGIN = 0.05;   // rad
    const auto& joint_models = joint_model_group->getActiveJointModels();
    double limit_scale = 1.0;
    for (size_t i = 0; i < joint_models.size(); ++i) {
        const auto& bounds = joint_models[i]->getVariableBounds();
        if (bounds.empty()) continue;
        double target = robo_arm_angles[i] + delta_q(static_cast<Eigen::Index>(i));
        double lo = bounds[0].min_position_ + JOINT_LIMIT_MARGIN;
        double hi = bounds[0].max_position_ - JOINT_LIMIT_MARGIN;
        if (target < lo && delta_q(static_cast<Eigen::Index>(i)) < 0) {
            double allowed = robo_arm_angles[i] - lo;
            if (allowed > 0 && std::abs(delta_q(static_cast<Eigen::Index>(i))) > 1e-6)
                limit_scale = std::min(limit_scale,
                    allowed / std::abs(delta_q(static_cast<Eigen::Index>(i))));
            else
                limit_scale = 0.0;
        } else if (target > hi && delta_q(static_cast<Eigen::Index>(i)) > 0) {
            double allowed = hi - robo_arm_angles[i];
            if (allowed > 0 && std::abs(delta_q(static_cast<Eigen::Index>(i))) > 1e-6)
                limit_scale = std::min(limit_scale,
                    allowed / std::abs(delta_q(static_cast<Eigen::Index>(i))));
            else
                limit_scale = 0.0;
        }
    }
    if (limit_scale < 1.0) {
        delta_q *= std::max(limit_scale, 0.0);
    }

    // ── 构造目标关节角度 ─────────────────────────────────────────────────────
    std::vector<double> result(robo_arm_angles.size());
    for (size_t i = 0; i < robo_arm_angles.size(); ++i) {
        result[i] = robo_arm_angles[i] + delta_q(static_cast<Eigen::Index>(i));
    }

    RCLCPP_DEBUG(LOGGER,
        "direction_constraint: pos_err=%.4fm orient_err=%.4frad cond=%.1f λ=%.4f dt=%.3fs",
        position_error, orient_error, condition_number, damping, dt);

    return result;
}
