/**
 ******************************************************************************
 * @file    real_world_servo.cpp
 * @brief   基于 MoveIt Servo 的实时控制节点
 * @details 
 *          替代 real_world_new.cpp 中基于 computeCartesianPath + IK 的方案。
 *          核心改进：
 *          - 使用 MoveIt Servo 通过雅可比矩阵直接计算关节速度增量
 *          - 不做完整 IK 求解，避免解的多值性和跳变问题
 *          - UP 模式：发送 TwistStamped 并零化 e_hat 方向分量实现平面约束
 *          - ROLL 模式：类似 UP，零化特定方向分量
 *          - TURN 模式：保留 computeCartesianPath 圆弧规划（预规划轨迹）
 * 
 *          控制流程：
 *          1. controller 关节角 -> FK 得到末端目标位置
 *          2. 计算当前末端到目标的位移 delta
 *          3. 投影到约束平面（消除 e_hat 分量）
 *          4. 将 delta 转为 TwistStamped 发送给 Servo
 *          5. Servo 内部用雅可比矩阵解算关节速度，输出平滑的关节角增量
 *          6. 订阅 Servo 输出的关节轨迹，发送给下位机
 * 
 * @note    Dependencies:
 *          - utils.hpp
 *          - usb_data.hpp
 *          - moveit_servo
 ******************************************************************************
 */

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_msgs/msg/int8.hpp>
#include <Eigen/Geometry>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <cmath>
#include "utils.hpp"
#include "usb_data.hpp"
#include <moveit_visual_tools/moveit_visual_tools.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("real_world_servo");

// 控制状态枚举（与 real_world_new.cpp 一致）
enum ControlMode { 
    NO = 0, 
    UP = 1, 
    TURN = 2, 
    ROLL = 3, 
    RESET = 4, 
};

/**
 * @brief 基于 MoveIt Servo 的机械臂实时控制器
 * 
 * 核心区别于 real_world_new.cpp 中的 RobotController：
 * - UP/ROLL 模式不再调用 computeCartesianPath + IK
 * - 而是计算末端位移增量 → 投影到约束平面 → 发送 TwistStamped → Servo 用雅可比矩阵求解
 * - 关节角输出连续平滑，不存在 IK 跳变问题
 */
class ServoRobotController {
public:
    ServoRobotController(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<SerialRxNode> serial_node,
        const std::string& planning_group = "arm")
        : node_(node),
          serial_node_(serial_node),
          robo_filter_(6, 0.3),
          controller_filter_(6, 0.3),
          e_hat_direction_(1, 0, 0),
          up_direction_(0, 0, 1),
          normal_direction_(0, 1, 0),
          last_control_mode_(NO)
    {
        RCLCPP_INFO(LOGGER, "Initializing ServoRobotController...");
        
        // 初始化 MoveIt
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node_, planning_group);
        
        // 等待机器人模型加载
        int max_retries = 10;
        for (int i = 0; i < max_retries; ++i) {
            robot_model_ = move_group_->getRobotModel();
            if (robot_model_) {
                RCLCPP_INFO(LOGGER, "Robot model loaded successfully");
                break;
            }
            RCLCPP_WARN(LOGGER, "Robot model not ready, retrying... (%d/%d)", i + 1, max_retries);
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
        
        if (!robot_model_) {
            throw std::runtime_error("Failed to get robot model");
        }
        
        joint_model_group_ = robot_model_->getJointModelGroup(planning_group);
        end_effector_link_ = move_group_->getEndEffectorLink();
        joint_names_ = joint_model_group_->getActiveJointModelNames();
        
        RCLCPP_INFO(LOGGER, "Planning group: %s, End effector: %s, Joints: %zu",
                    planning_group.c_str(), end_effector_link_.c_str(), joint_names_.size());
        
        rclcpp::sleep_for(std::chrono::seconds(2));
        
        // 初始化重力补偿
        compute_gravity_compensation_ = make_gravity_closure(robot_model_);
        
        // ================================================================
        // MoveIt Servo 集成 — 发布 TwistStamped 指令, 订阅关节轨迹输出
        // ================================================================
        
        // 发布 Twist 指令给 Servo（替代 computeCartesianPath + IK）
        twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/servo_node/delta_twist_cmds", 10);
        
        // 订阅 Servo 输出的关节轨迹（增量关节角命令）
        joint_cmd_sub_ = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/servo_node/joint_trajectory_cmds", 10,
            std::bind(&ServoRobotController::servo_joint_cmd_callback, this, std::placeholders::_1));
        
        // Servo 状态监听
        servo_status_sub_ = node_->create_subscription<std_msgs::msg::Int8>(
            "/servo_node/status", 10,
            [this](const std_msgs::msg::Int8::SharedPtr msg) {
                servo_status_ = msg->data;
                if (msg->data != 0) {
                    RCLCPP_WARN_THROTTLE(LOGGER, *node_->get_clock(), 2000,
                        "Servo status: %d (0=OK, -1=halt, 1=degrade)", msg->data);
                }
            });
        
        // 初始化可视化
        visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
            node_, "base_link", "servo_markers", robot_model_);
        visual_tools_->deleteAllMarkers();
        
        marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
            "servo_text_markers", 10);
        
        is_initialized_ = true;
        RCLCPP_INFO(LOGGER, "ServoRobotController initialized with MoveIt Servo integration");
    }

    /**
     * @brief 启动控制循环（50ms 定时器 = 20Hz）
     */
    void start_control_loop() {
        control_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&ServoRobotController::control_callback, this));
        RCLCPP_INFO(LOGGER, "Control loop started (50ms cycle)");
    }

private:
    // ================================================================
    // Servo 输出回调 — 接收 Servo 计算的关节角并发送给下位机
    // ================================================================
    
    /**
     * @brief Servo 关节轨迹输出回调
     * 
     * MoveIt Servo 接收 TwistStamped → 用雅可比矩阵求解 → 输出 JointTrajectory
     * 这里接收输出，转换后发送给下位机
     */
    void servo_joint_cmd_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
        if (msg->points.empty()) return;
        
        // 取最新的一帧
        const auto& point = msg->points.back();
        
        if (point.positions.size() != 6) {
            RCLCPP_WARN(LOGGER, "Servo output has %zu joints, expected 6", point.positions.size());
            return;
        }
        
        // 存储 Servo 输出的关节角（线程安全）
        {
            std::lock_guard<std::mutex> lock(servo_output_mutex_);
            servo_output_angles_.assign(point.positions.begin(), point.positions.end());
            servo_output_valid_ = true;
        }
        
        // 转换为硬件角度并发送
        std::vector<double> joint_angles(point.positions.begin(), point.positions.end());
        std::vector<float> send_angles = convert_urdf_to_hardware(joint_angles);
        
        // 计算重力补偿
        std::vector<float> torques = compute_gravity_compensation_(joint_angles);
        serial_node_->send_roboarm_with_controller(torques, send_angles);
    }

    // ================================================================
    // 控制循环主逻辑
    // ================================================================
    
    void control_callback() {
        auto robo_arm_angles_raw = serial_node_->get_robo_arm_angles();
        auto controller_angles_raw = serial_node_->get_controller_angles();
        uint8_t control_mode = serial_node_->get_control_mode();
        
        // 日志（每秒一次）
        log_counter_++;
        if (log_counter_ >= 20) {
            log_counter_ = 0;
            RCLCPP_INFO(LOGGER, "Mode: %d | Servo status: %d", control_mode, servo_status_.load());
        }
        
        if (robo_arm_angles_raw.size() != 6) {
            RCLCPP_DEBUG(LOGGER, "Waiting for robo_arm data...");
            return;
        }
        
        // 角度滤波和转换
        std::vector<double> robo_arm_angles = robo_filter_.filter(robo_arm_angles_raw);
        robo_arm_angles[1] = -robo_arm_angles[1];
        robo_arm_angles[4] = 6.28 - robo_arm_angles[4];
        
        std::vector<double> controller_angles = controller_filter_.filter(controller_angles_raw);
        controller_angles[1] = -controller_angles[1];
        controller_angles[4] = 6.28 - controller_angles[4];
        
        // 重力补偿
        latest_torques_ = compute_gravity_compensation_(robo_arm_angles);
        
        // 模式切换处理
        handle_mode_transition(control_mode, robo_arm_angles);
        
        // 可视化当前状态
        visualize_current_state(robo_arm_angles);
        
        // 根据控制模式处理
        switch (control_mode) {
            case NO:
                handle_mode_no(controller_angles);
                break;
            case UP:
                handle_mode_up_servo(robo_arm_angles, controller_angles);
                break;
            case TURN:
                handle_mode_turn(robo_arm_angles);
                break;
            case ROLL:
                handle_mode_roll_servo(robo_arm_angles, controller_angles);
                break;
            case RESET:
                handle_mode_reset();
                break;
        }
        
        visual_tools_->trigger();
    }

    // ================================================================
    // 模式切换处理
    // ================================================================
    
    void handle_mode_transition(uint8_t control_mode, const std::vector<double>& robo_arm_angles) {
        if (control_mode == last_control_mode_) return;
        
        RCLCPP_INFO(LOGGER, "Control mode changed: %d -> %d", last_control_mode_, control_mode);
        
        if (control_mode == UP) {
            // 记录参考位姿和 e_hat 方向
            reference_pose_ = compute_end_effector_pose(robo_arm_angles);
            reference_pose_initialized_ = true;
            
            Eigen::Quaterniond quat(
                reference_pose_.orientation.w,
                reference_pose_.orientation.x,
                reference_pose_.orientation.y,
                reference_pose_.orientation.z
            );
            e_hat_direction_ = (quat * Eigen::Vector3d(1, 0, 0)).normalized();
            
            RCLCPP_INFO(LOGGER, "UP mode: e_hat=[%.3f, %.3f, %.3f], EE=[%.3f, %.3f, %.3f]",
                       e_hat_direction_.x(), e_hat_direction_.y(), e_hat_direction_.z(),
                       reference_pose_.position.x, reference_pose_.position.y, reference_pose_.position.z);
        }
        
        if (control_mode == TURN) {
            up_end_pose_ = compute_end_effector_pose(robo_arm_angles);
            up_direction_ = (Eigen::Vector3d(up_end_pose_.position.x, up_end_pose_.position.y, up_end_pose_.position.z) - 
                             Eigen::Vector3d(reference_pose_.position.x, reference_pose_.position.y, reference_pose_.position.z)).normalized();
            normal_direction_ = e_hat_direction_.cross(up_direction_).normalized();
            
            RCLCPP_INFO(LOGGER, "TURN mode: up=[%.3f, %.3f, %.3f], normal=[%.3f, %.3f, %.3f]",
                       up_direction_.x(), up_direction_.y(), up_direction_.z(),
                       normal_direction_.x(), normal_direction_.y(), normal_direction_.z());
        }
        
        last_control_mode_ = control_mode;
    }

    // ================================================================
    // NO 模式 — 透传 controller 角度
    // ================================================================
    
    void handle_mode_no(const std::vector<double>& controller_angles) {
        std::vector<float> send_angles = convert_urdf_to_hardware(controller_angles);
        serial_node_->send_roboarm_with_controller(latest_torques_, send_angles);
    }

    // ================================================================
    // UP 模式 — 使用 MoveIt Servo（核心改进！）
    // ================================================================
    
    /**
     * @brief UP 模式的 Servo 实现
     * 
     * 对比 real_world_new.cpp 中的 handle_mode_up：
     * 
     * 旧方案（问题大）：
     *   controller FK → 约束投影 → computeCartesianPath(内部做IK) → 多帧轨迹 → 状态机逐帧发送
     *   问题：IK解多值跳变 + 规划延迟高 + 状态机复杂
     * 
     * 新方案（Servo）：
     *   controller FK → 计算目标位置 → 与当前位置求差 → 投影到约束平面（消除e_hat分量）
     *   → 转为 TwistStamped → 发给 Servo → Servo 用雅可比矩阵求解关节速度 → 输出平滑关节角
     *   优势：无IK跳变 + 低延迟(~20ms) + 代码简洁
     */
    void handle_mode_up_servo(
        const std::vector<double>& robo_arm_angles,
        const std::vector<double>& controller_angles)
    {
        if (controller_angles.size() != 6 || robo_arm_angles.size() != 6) return;
        if (!reference_pose_initialized_) return;
        
        // Step 1: FK 计算 controller 的目标末端位置
        geometry_msgs::msg::Pose controller_pose = compute_end_effector_pose(controller_angles);
        Eigen::Vector3d controller_position(
            controller_pose.position.x,
            controller_pose.position.y,
            controller_pose.position.z);
        
        // Step 2: FK 计算当前机械臂的实际末端位置
        geometry_msgs::msg::Pose current_pose = compute_end_effector_pose(robo_arm_angles);
        Eigen::Vector3d current_position(
            current_pose.position.x,
            current_pose.position.y,
            current_pose.position.z);
        
        // Step 3: 计算位移增量
        Eigen::Vector3d reference_position(
            reference_pose_.position.x,
            reference_pose_.position.y,
            reference_pose_.position.z);
        
        // controller 相对于参考点的位移
        Eigen::Vector3d displacement = controller_position - reference_position;
        
        // Step 4: 投影到约束平面 — 消除 e_hat 方向的分量（核心约束！）
        Eigen::Vector3d e_hat = e_hat_direction_.normalized();
        double e_hat_component = displacement.dot(e_hat);
        Eigen::Vector3d constrained_displacement = displacement - e_hat_component * e_hat;
        
        // 约束后的目标位置
        Eigen::Vector3d target_position = reference_position + constrained_displacement;
        
        // Step 5: 计算当前位置到约束目标的速度指令
        Eigen::Vector3d delta = target_position - current_position;
        
        // 同时也消除当前速度中的 e_hat 分量（双重保险）
        delta = delta - delta.dot(e_hat) * e_hat;
        
        // Step 6: 限制速度幅度（安全限速）
        double max_linear_speed = 0.15;  // m/s，可根据实际调整
        double delta_norm = delta.norm();
        if (delta_norm > max_linear_speed) {
            delta = delta * (max_linear_speed / delta_norm);
        }
        
        // Step 7: 如果距离很近，停止发送（死区 1mm）
        if (delta_norm < 0.001) {
            send_zero_twist();
            return;
        }
        
        // Step 8: 构造并发布 TwistStamped
        geometry_msgs::msg::TwistStamped twist_cmd;
        twist_cmd.header.stamp = node_->now();
        twist_cmd.header.frame_id = "base_link";  // 在世界坐标系中指定速度方向
        
        // 线速度 = 位移增量（Servo 会按配置的频率和缩放因子处理）
        twist_cmd.twist.linear.x = delta.x();
        twist_cmd.twist.linear.y = delta.y();
        twist_cmd.twist.linear.z = delta.z();
        
        // 角速度为零 — 保持末端姿态不变
        twist_cmd.twist.angular.x = 0.0;
        twist_cmd.twist.angular.y = 0.0;
        twist_cmd.twist.angular.z = 0.0;
        
        twist_pub_->publish(twist_cmd);
        
        // 可视化
        visualize_constraint_plane(current_position);
        visualize_target_point(target_position);
        
        RCLCPP_DEBUG(LOGGER, "UP Servo: delta=[%.4f, %.4f, %.4f] norm=%.4f e_hat_removed=%.4f",
                    delta.x(), delta.y(), delta.z(), delta_norm, e_hat_component);
    }

    // ================================================================
    // ROLL 模式 — 使用 Servo（消除特定方向分量）
    // ================================================================
    
    /**
     * @brief ROLL 模式：类似 UP，但消除的方向不同
     * 
     * 与 UP 的区别：UP 消除 e_hat 分量，ROLL 消除 normal 方向或其他自定义方向
     */
    void handle_mode_roll_servo(
        const std::vector<double>& robo_arm_angles,
        const std::vector<double>& controller_angles)
    {
        if (controller_angles.size() != 6 || robo_arm_angles.size() != 6) return;
        if (!reference_pose_initialized_) return;
        
        // FK
        geometry_msgs::msg::Pose controller_pose = compute_end_effector_pose(controller_angles);
        geometry_msgs::msg::Pose current_pose = compute_end_effector_pose(robo_arm_angles);
        
        Eigen::Vector3d controller_position(controller_pose.position.x, controller_pose.position.y, controller_pose.position.z);
        Eigen::Vector3d current_position(current_pose.position.x, current_pose.position.y, current_pose.position.z);
        Eigen::Vector3d reference_position(reference_pose_.position.x, reference_pose_.position.y, reference_pose_.position.z);
        
        // 消除 e_hat 方向的分量（和 UP 一致，你也可以改成消除其他方向）
        Eigen::Vector3d displacement = controller_position - reference_position;
        Eigen::Vector3d e_hat = e_hat_direction_.normalized();
        displacement = displacement - displacement.dot(e_hat) * e_hat;
        
        Eigen::Vector3d target_position = reference_position + displacement;
        Eigen::Vector3d delta = target_position - current_position;
        delta = delta - delta.dot(e_hat) * e_hat;
        
        // 限速
        double max_speed = 0.15;
        if (delta.norm() > max_speed) delta = delta.normalized() * max_speed;
        if (delta.norm() < 0.001) { send_zero_twist(); return; }
        
        // 发送 Twist — ROLL 模式可以添加角速度来做绕轴旋转
        geometry_msgs::msg::TwistStamped twist_cmd;
        twist_cmd.header.stamp = node_->now();
        twist_cmd.header.frame_id = "base_link";
        twist_cmd.twist.linear.x = delta.x();
        twist_cmd.twist.linear.y = delta.y();
        twist_cmd.twist.linear.z = delta.z();
        // 如果需要绕 e_hat 轴旋转，可以在这里设置角速度
        // twist_cmd.twist.angular.x = e_hat.x() * roll_speed;
        // twist_cmd.twist.angular.y = e_hat.y() * roll_speed;
        // twist_cmd.twist.angular.z = e_hat.z() * roll_speed;
        
        twist_pub_->publish(twist_cmd);
    }

    // ================================================================
    // TURN 模式 — 保留 computeCartesianPath（预规划圆弧轨迹）
    // ================================================================
    
    /**
     * @brief TURN 模式仍然使用 computeCartesianPath 做预规划
     * 因为圆弧轨迹是一次性规划-执行，不需要实时调整，所以 IK 跳变问题不明显
     */
    void handle_mode_turn(const std::vector<double>& robo_arm_angles) {
        if (!reference_pose_initialized_) return;
        
        Eigen::Vector3d end_position(up_end_pose_.position.x, up_end_pose_.position.y, up_end_pose_.position.z);
        Eigen::Vector3d circle_center = end_position - e_hat_direction_ * 0.1;
        
        std::vector<geometry_msgs::msg::Pose> waypoints;
        const int num_points = 30;
        const double angle_range = M_PI / 2.0;
        const double radius = 0.1;
        
        for (int i = 0; i <= num_points; ++i) {
            double angle = angle_range * i / num_points;
            Eigen::Vector3d point = circle_center + 
                radius * (std::sin(angle) * up_direction_ + std::cos(angle) * e_hat_direction_);
            
            geometry_msgs::msg::Pose waypoint;
            waypoint.position.x = point.x();
            waypoint.position.y = point.y();
            waypoint.position.z = point.z();
            waypoint.orientation = up_end_pose_.orientation;
            waypoints.push_back(waypoint);
        }
        
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = move_group_->computeCartesianPath(waypoints, 0.01, trajectory);
        
        RCLCPP_INFO(LOGGER, "TURN: Cartesian path %.2f%% achieved", fraction * 100.0);
        
        if (fraction > 0.8) {
            auto trajectory_dict = extractTrajectory(trajectory);
            for (const auto& point : trajectory_dict) {
                std::vector<double> joint_angles = point.second;
                if (joint_angles.size() != 6) continue;
                
                std::vector<float> torques = compute_gravity_compensation_(joint_angles);
                std::vector<float> send_angles = convert_urdf_to_hardware(joint_angles);
                serial_node_->send_roboarm_with_controller(torques, send_angles);
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        }
    }

    void handle_mode_reset() {
        RCLCPP_DEBUG(LOGGER, "Mode RESET: not implemented yet");
    }

    // ================================================================
    // 辅助方法
    // ================================================================
    
    /**
     * @brief 发送零速度 Twist（停止运动）
     */
    void send_zero_twist() {
        geometry_msgs::msg::TwistStamped twist_cmd;
        twist_cmd.header.stamp = node_->now();
        twist_cmd.header.frame_id = "base_link";
        // 所有分量为0 — 停止
        twist_pub_->publish(twist_cmd);
    }

    /**
     * @brief 计算末端执行器位姿（FK）
     */
    geometry_msgs::msg::Pose compute_end_effector_pose(const std::vector<double>& joint_angles) {
        moveit::core::RobotState robot_state(robot_model_);
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            robot_state.setJointPositions(joint_names_[i], &joint_angles[i]);
        }
        robot_state.update();
        
        Eigen::Isometry3d ee_transform = robot_state.getGlobalLinkTransform(end_effector_link_);
        
        geometry_msgs::msg::Pose pose;
        pose.position.x = ee_transform.translation().x();
        pose.position.y = ee_transform.translation().y();
        pose.position.z = ee_transform.translation().z();
        
        Eigen::Quaterniond quat(ee_transform.rotation());
        pose.orientation.x = quat.x();
        pose.orientation.y = quat.y();
        pose.orientation.z = quat.z();
        pose.orientation.w = quat.w();
        
        return pose;
    }

    /**
     * @brief URDF 角度 → 下位机角度转换
     */
    std::vector<float> convert_urdf_to_hardware(const std::vector<double>& urdf_angles) {
        std::vector<float> hw(urdf_angles.begin(), urdf_angles.end());
        hw[1] = -hw[1];
        hw[4] = 6.28f - hw[4];
        return hw;
    }

    // ================================================================
    // 可视化
    // ================================================================
    
    void visualize_current_state(const std::vector<double>& joint_angles) {
        if (!is_initialized_ || !robot_model_) return;
        
        moveit::core::RobotState robot_state(robot_model_);
        for (size_t i = 0; i < joint_names_.size() && i < joint_angles.size(); ++i) {
            robot_state.setJointPositions(joint_names_[i], &joint_angles[i]);
        }
        robot_state.update();
        visual_tools_->publishRobotState(robot_state, rviz_visual_tools::GREEN);
    }

    void visualize_constraint_plane(const Eigen::Vector3d& position) {
        // 发布 e_hat 方向的箭头
        Eigen::Vector3d e_hat_end = position + e_hat_direction_ * 0.15;
        visual_tools_->publishArrow(
            visual_tools_->convertPoint(position),
            visual_tools_->convertPoint(e_hat_end),
            rviz_visual_tools::RED, rviz_visual_tools::MEDIUM);
    }

    void visualize_target_point(const Eigen::Vector3d& target) {
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = target.x();
        target_pose.position.y = target.y();
        target_pose.position.z = target.z();
        target_pose.orientation.w = 1.0;
        visual_tools_->publishSphere(target_pose, rviz_visual_tools::CYAN, rviz_visual_tools::SMALL);
    }

    // ================================================================
    // 成员变量
    // ================================================================
    
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<SerialRxNode> serial_node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    const moveit::core::JointModelGroup* joint_model_group_;
    moveit::core::RobotModelConstPtr robot_model_;
    std::string end_effector_link_;
    std::vector<std::string> joint_names_;
    
    // 滤波器
    AngleFilter robo_filter_;
    AngleFilter controller_filter_;
    
    // 重力补偿
    std::function<std::vector<float>(const std::vector<double>&)> compute_gravity_compensation_;
    std::vector<float> latest_torques_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // Servo 相关
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr servo_status_sub_;
    std::atomic<int8_t> servo_status_{0};
    
    // Servo 输出（线程安全）
    std::mutex servo_output_mutex_;
    std::vector<double> servo_output_angles_;
    bool servo_output_valid_ = false;
    
    // 可视化
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    
    // 坐标轴方向
    Eigen::Vector3d e_hat_direction_;
    Eigen::Vector3d up_direction_;
    Eigen::Vector3d normal_direction_;
    
    // 参考位姿
    geometry_msgs::msg::Pose reference_pose_;
    bool reference_pose_initialized_ = false;
    geometry_msgs::msg::Pose up_end_pose_;
    
    // 模式
    uint8_t last_control_mode_;
    
    // 日志
    int log_counter_ = 0;
    bool is_initialized_ = false;
};


// ================================================================
// main
// ================================================================

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("real_world_servo", node_options);

    auto serial_node = std::make_shared<SerialRxNode>();
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(serial_node);
    
    std::atomic<bool> should_stop{false};
    std::thread executor_thread([&executor, &should_stop]() {
        while (!should_stop && rclcpp::ok()) {
            executor.spin_some(std::chrono::milliseconds(10));
        }
    });
    
    std::thread receive_thread([serial_node]() {
        serial_node->receive_data();
    });

    RCLCPP_INFO(rclcpp::get_logger("main"), "Waiting for MoveGroup & Servo to initialize...");
    rclcpp::sleep_for(std::chrono::seconds(5));
    
    auto controller = std::make_shared<ServoRobotController>(node, serial_node);
    controller->start_control_loop();

    RCLCPP_INFO(rclcpp::get_logger("main"), "Servo system ready!");
    
    while (rclcpp::ok()) {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    
    should_stop = true;
    if (executor_thread.joinable()) executor_thread.join();
    if (receive_thread.joinable()) receive_thread.join();
    
    rclcpp::shutdown();
    return 0;
}
