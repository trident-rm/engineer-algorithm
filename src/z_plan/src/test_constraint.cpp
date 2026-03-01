/**
 ******************************************************************************
 * @file    test_constraint.cpp
 * @brief   约束控制测试环境 — 验证雅可比伪逆 + e_hat 平面约束
 * @details
 *          不依赖串口/硬件，纯仿真测试。
 *          模拟 controller 带动末端在约束平面内运动（画圆/随机），
 *          验证末端是否被约束在 ⊥ e_hat 平面内、关节角是否连续无跳变。
 * 
 *          RViz 可视化内容：
 *          - 绿色机械臂：当前状态（被约束后的）
 *          - 紫色机械臂：controller 目标（未约束的）
 *          - 橙色圆盘：约束平面
 *          - 红色箭头：e_hat 方向
 *          - 青色球：约束后目标点
 *          - 黄色轨迹：末端实际运动轨迹
 *          - 品红轨迹：约束后目标轨迹
 * 
 *          终端输出：
 *          - e_hat 方向漂移量（验证约束是否生效）
 *          - 关节角变化量（验证无跳变）
 *          - 末端位置误差
 * 
 *          测试模式（通过 ros2 param 切换）：
 *          - circle：controller 在球面上绕圆，部分在约束平面外
 *          - random：controller 随机运动
 *          - manual：通过 /test_controller_joints topic 手动设置
 * 
 * @usage   ros2 launch z_plan test_constraint.launch.py
 *          # 或
 *          ros2 launch z_plan test_constraint.launch.py test_mode:=random
 ******************************************************************************
 */

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Geometry>
#include <chrono>
#include <thread>
#include <atomic>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include "utils.hpp"
#include <moveit_visual_tools/moveit_visual_tools.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("test_constraint");

/**
 * @brief 约束控制测试节点
 */
class ConstraintTestNode {
public:
    ConstraintTestNode(rclcpp::Node::SharedPtr node,
                       const std::string& planning_group = "arm")
        : node_(node)
    {
        RCLCPP_INFO(LOGGER, "=== Constraint Control Test Environment ===");
        
        // 读取测试模式参数
        node_->declare_parameter<std::string>("test_mode", "circle");
        node_->get_parameter("test_mode", test_mode_);
        RCLCPP_INFO(LOGGER, "Test mode: %s", test_mode_.c_str());
        
        // 初始化 MoveIt
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node_, planning_group);
        
        for (int i = 0; i < 10; ++i) {
            robot_model_ = move_group_->getRobotModel();
            if (robot_model_) break;
            RCLCPP_WARN(LOGGER, "Waiting for robot model... (%d/10)", i + 1);
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
        if (!robot_model_) throw std::runtime_error("Failed to get robot model");
        
        joint_model_group_ = robot_model_->getJointModelGroup(planning_group);
        end_effector_link_ = move_group_->getEndEffectorLink();
        joint_names_ = joint_model_group_->getActiveJointModelNames();
        
        RCLCPP_INFO(LOGGER, "Robot: %zu joints, EE: %s", joint_names_.size(), end_effector_link_.c_str());
        
        // 初始位姿（关节角）
        // 选一个远离奇异点的中间位姿
        current_joint_angles_ = {0.0, 1.5, -0.5, 1.5, 1.5, 0.0};
        controller_joint_angles_ = current_joint_angles_;
        
        // 计算初始末端位姿作为参考
        reference_pose_ = compute_ee_pose(current_joint_angles_);
        
        // 计算 e_hat 方向 = link6 的 x 轴
        Eigen::Quaterniond quat(
            reference_pose_.orientation.w,
            reference_pose_.orientation.x,
            reference_pose_.orientation.y,
            reference_pose_.orientation.z);
        e_hat_direction_ = (quat * Eigen::Vector3d(1, 0, 0)).normalized();
        
        reference_position_ = Eigen::Vector3d(
            reference_pose_.position.x,
            reference_pose_.position.y,
            reference_pose_.position.z);
        
        RCLCPP_INFO(LOGGER, "Reference EE: [%.4f, %.4f, %.4f]",
                   reference_position_.x(), reference_position_.y(), reference_position_.z());
        RCLCPP_INFO(LOGGER, "e_hat direction: [%.4f, %.4f, %.4f]",
                   e_hat_direction_.x(), e_hat_direction_.y(), e_hat_direction_.z());
        
        // 可视化工具
        visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
            node_, "base_link", "test_constraint_markers", robot_model_);
        visual_tools_->deleteAllMarkers();
        
        controller_visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
            node_, "base_link", "test_controller_markers", robot_model_);
        controller_visual_tools_->deleteAllMarkers();
        
        marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
            "test_constraint_text", 10);
        
        // joint_states 发布（给 robot_state_publisher 用）
        joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);
        
        // manual 模式订阅
        manual_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/test_controller_joints", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                if (msg->position.size() == 6) {
                    controller_joint_angles_.assign(msg->position.begin(), msg->position.end());
                    RCLCPP_INFO(LOGGER, "Manual controller input received");
                }
            });
        
        RCLCPP_INFO(LOGGER, "Initialization complete. Starting test...");
        RCLCPP_INFO(LOGGER, "");
        RCLCPP_INFO(LOGGER, "=== Legend ===");
        RCLCPP_INFO(LOGGER, "  Green arm  = Current state (constrained)");
        RCLCPP_INFO(LOGGER, "  Purple arm = Controller target (unconstrained)");
        RCLCPP_INFO(LOGGER, "  Orange disk = Constraint plane (⊥ e_hat)");
        RCLCPP_INFO(LOGGER, "  Red arrow  = e_hat direction");
        RCLCPP_INFO(LOGGER, "  Cyan sphere = Constrained target point");
        RCLCPP_INFO(LOGGER, "  Yellow line = Actual EE trajectory");
        RCLCPP_INFO(LOGGER, "  Magenta line = Target trajectory");
        RCLCPP_INFO(LOGGER, "==============");
    }
    
    void start() {
        timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&ConstraintTestNode::control_loop, this));
    }

private:
    // ================================================================
    // 主控制循环 — 50ms
    // ================================================================
    void control_loop() {
        step_count_++;
        
        // 1. 更新 controller（模拟目标输入）
        update_controller_target();
        
        // 2. 执行约束控制（雅可比方案）
        auto result = constrained_step(current_joint_angles_, controller_joint_angles_);
        
        // 3. 更新当前关节角（仿真：直接设为计算结果，相当于完美执行）
        if (!result.target_angles.empty()) {
            current_joint_angles_ = result.target_angles;
        }
        
        // 4. 发布 joint_states（让 robot_state_publisher 更新 TF）
        publish_joint_states(current_joint_angles_);
        
        // 5. 可视化
        visualize_all(result);
        
        // 6. 日志（每秒一次）
        if (step_count_ % 20 == 0) {
            print_metrics(result);
        }
    }
    
    // ================================================================
    // 约束控制步骤的返回结构
    // ================================================================
    struct StepResult {
        std::vector<double> target_angles;     // 约束后的关节角
        Eigen::Vector3d current_ee;            // 当前末端位置
        Eigen::Vector3d controller_ee;         // controller（未约束）末端位置
        Eigen::Vector3d constrained_target;    // 约束后的目标位置
        Eigen::Vector3d velocity;              // 发送给雅可比的速度
        double e_hat_drift;                    // e_hat 方向偏移量
        double position_error;                 // 到约束目标的距离
        double max_joint_delta;                // 最大关节角变化量
        double condition_number;               // 雅可比条件数
        bool near_singularity;
    };
    
    // ================================================================
    // 核心约束控制（从 real_world_new.cpp 提取）
    // ================================================================
    StepResult constrained_step(
        const std::vector<double>& robo_arm_angles,
        const std::vector<double>& controller_angles)
    {
        StepResult result;
        result.near_singularity = false;
        result.condition_number = 0;
        result.max_joint_delta = 0;
        
        // FK
        auto controller_pose = compute_ee_pose(controller_angles);
        auto current_pose = compute_ee_pose(robo_arm_angles);
        
        result.controller_ee = Eigen::Vector3d(
            controller_pose.position.x,  controller_pose.position.y,  controller_pose.position.z);
        result.current_ee = Eigen::Vector3d(
            current_pose.position.x, current_pose.position.y, current_pose.position.z);
        
        // 平面约束：消除 e_hat 分量
        Eigen::Vector3d displacement = result.controller_ee - reference_position_;
        Eigen::Vector3d e_hat = e_hat_direction_.normalized();
        double e_hat_component = displacement.dot(e_hat);
        Eigen::Vector3d constrained_displacement = displacement - e_hat_component * e_hat;
        
        result.constrained_target = reference_position_ + constrained_displacement;
        
        // 速度
        result.velocity = result.constrained_target - result.current_ee;
        result.velocity = result.velocity - result.velocity.dot(e_hat) * e_hat;
        
        // 计算 e_hat 漂移（当前末端相对于参考，在 e_hat 方向的偏移）
        Eigen::Vector3d current_disp = result.current_ee - reference_position_;
        result.e_hat_drift = std::abs(current_disp.dot(e_hat));
        
        result.position_error = result.velocity.norm();
        
        // 死区
        if (result.position_error < 0.0005) {
            result.target_angles = robo_arm_angles;
            return result;
        }
        
        // 限速
        const double max_speed = 0.15;
        if (result.position_error > max_speed) {
            result.velocity = result.velocity * (max_speed / result.position_error);
        }
        
        // 雅可比伪逆
        const double dt = 0.05;
        
        moveit::core::RobotState robot_state(robot_model_);
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            robot_state.setJointPositions(joint_names_[i], &robo_arm_angles[i]);
        }
        robot_state.update();
        
        Eigen::MatrixXd full_jacobian;
        Eigen::Vector3d ref_pt(0, 0, 0);
        if (!robot_state.getJacobian(joint_model_group_,
                robot_state.getLinkModel(end_effector_link_), ref_pt, full_jacobian)) {
            RCLCPP_ERROR(LOGGER, "Jacobian computation failed!");
            result.target_angles = robo_arm_angles;
            return result;
        }
        
        Eigen::MatrixXd J_lin = full_jacobian.topRows(3);  // 3×n
        
        // 条件数检查
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_lin);
        auto sv = svd.singularValues();
        result.condition_number = sv(0) / sv(sv.size() - 1);
        result.near_singularity = result.condition_number > 100.0;
        
        // 阻尼伪逆
        const double damping = 0.005;
        Eigen::MatrixXd JJt = J_lin * J_lin.transpose();
        JJt += damping * damping * Eigen::Matrix3d::Identity();
        Eigen::MatrixXd J_pinv = J_lin.transpose() * JJt.inverse();
        
        Eigen::VectorXd delta_q = J_pinv * result.velocity * dt;
        
        // 构造目标角度
        result.target_angles.resize(robo_arm_angles.size());
        for (size_t i = 0; i < robo_arm_angles.size(); ++i) {
            result.target_angles[i] = robo_arm_angles[i] + delta_q(i);
            double dq = std::abs(delta_q(i));
            if (dq > result.max_joint_delta) result.max_joint_delta = dq;
        }
        
        // 关节限位 clamp
        const auto& joint_models = joint_model_group_->getActiveJointModels();
        for (size_t i = 0; i < joint_models.size(); ++i) {
            const auto& bounds = joint_models[i]->getVariableBounds();
            if (!bounds.empty()) {
                result.target_angles[i] = std::clamp(
                    result.target_angles[i],
                    bounds[0].min_position_, bounds[0].max_position_);
            }
        }
        
        return result;
    }
    
    // ================================================================
    // Controller 目标生成（模拟 controller 输入）
    // ================================================================
    void update_controller_target() {
        if (test_mode_ == "manual") {
            return;  // 通过 topic 手动设置
        }
        
        double t = step_count_ * 0.05;  // 时间(s)
        
        if (test_mode_ == "circle") {
            // 在关节空间做小幅正弦扰动，使末端画出椭圆/圆形轨迹
            // 关键：扰动会产生 e_hat 方向分量，用来测试约束是否生效
            double amp = 0.3;   // 扰动幅度 (rad)
            double freq = 0.5;  // 频率 (Hz)
            
            controller_joint_angles_ = {
                current_joint_angles_[0] + amp * std::sin(2 * M_PI * freq * t),
                current_joint_angles_[1] + amp * 0.5 * std::cos(2 * M_PI * freq * t),
                current_joint_angles_[2] + amp * 0.3 * std::sin(2 * M_PI * freq * t + 1.0),
                current_joint_angles_[3],  // 保持wrist不动，减少奇异风险
                current_joint_angles_[4],
                current_joint_angles_[5]
            };
        }
        else if (test_mode_ == "random") {
            // 每 2 秒切换一次随机目标
            if (step_count_ % 40 == 0) {
                srand(step_count_);
                for (int i = 0; i < 3; ++i) {  // 只扰动前3个关节
                    double noise = ((rand() % 200) - 100) / 100.0 * 0.2;  // ±0.2 rad
                    controller_joint_angles_[i] = current_joint_angles_[i] + noise;
                }
            }
        }
    }
    
    // ================================================================
    // 可视化
    // ================================================================
    void visualize_all(const StepResult& result) {
        // 当前机械臂（绿色）
        {
            moveit::core::RobotState rs(robot_model_);
            for (size_t i = 0; i < joint_names_.size(); ++i)
                rs.setJointPositions(joint_names_[i], &current_joint_angles_[i]);
            rs.update();
            visual_tools_->publishRobotState(rs, rviz_visual_tools::GREEN);
        }
        
        // Controller 机械臂（紫色）
        {
            moveit::core::RobotState rs(robot_model_);
            for (size_t i = 0; i < joint_names_.size(); ++i)
                rs.setJointPositions(joint_names_[i], &controller_joint_angles_[i]);
            rs.update();
            controller_visual_tools_->publishRobotState(rs, rviz_visual_tools::PURPLE);
        }
        
        // e_hat 箭头（红色）
        Eigen::Vector3d arrow_end = result.current_ee + e_hat_direction_ * 0.15;
        visual_tools_->publishArrow(
            visual_tools_->convertPoint(result.current_ee),
            visual_tools_->convertPoint(arrow_end),
            rviz_visual_tools::RED, rviz_visual_tools::MEDIUM);
        
        // 约束后目标点（青色球）
        {
            geometry_msgs::msg::Pose tp;
            tp.position.x = result.constrained_target.x();
            tp.position.y = result.constrained_target.y();
            tp.position.z = result.constrained_target.z();
            tp.orientation.w = 1.0;
            visual_tools_->publishSphere(tp, rviz_visual_tools::CYAN, rviz_visual_tools::SMALL);
        }
        
        // Controller 目标点（紫色球，可能在平面外）
        {
            geometry_msgs::msg::Pose cp;
            cp.position.x = result.controller_ee.x();
            cp.position.y = result.controller_ee.y();
            cp.position.z = result.controller_ee.z();
            cp.orientation.w = 1.0;
            visual_tools_->publishSphere(cp, rviz_visual_tools::PURPLE, rviz_visual_tools::SMALL);
        }
        
        // 约束平面（橙色圆盘）
        visualize_constraint_plane(result.current_ee);
        
        // 轨迹记录和显示
        actual_trajectory_.push_back(result.current_ee);
        target_trajectory_.push_back(result.constrained_target);
        
        // 限制轨迹长度
        const size_t max_trail = 200;
        if (actual_trajectory_.size() > max_trail) {
            actual_trajectory_.erase(actual_trajectory_.begin());
            target_trajectory_.erase(target_trajectory_.begin());
        }
        
        // 画轨迹线
        if (actual_trajectory_.size() > 2) {
            publish_trajectory_line(actual_trajectory_, "actual_trail", 10,
                                    1.0, 1.0, 0.0, 0.8);  // 黄色
            publish_trajectory_line(target_trajectory_, "target_trail", 11,
                                    1.0, 0.0, 1.0, 0.5);  // 品红色
        }
        
        // 文本信息
        publish_info_text(result);
        
        visual_tools_->trigger();
        controller_visual_tools_->trigger();
    }
    
    void visualize_constraint_plane(const Eigen::Vector3d& center) {
        Eigen::Vector3d e_hat = e_hat_direction_.normalized();
        Eigen::Vector3d arb = (std::abs(e_hat.x()) < 0.9) ? 
            Eigen::Vector3d(1,0,0) : Eigen::Vector3d(0,1,0);
        Eigen::Vector3d u = e_hat.cross(arb).normalized();
        Eigen::Vector3d v = e_hat.cross(u).normalized();
        
        double radius = 0.10;
        
        // 圆环
        visualization_msgs::msg::Marker ring;
        ring.header.frame_id = "base_link";
        ring.header.stamp = node_->now();
        ring.ns = "constraint_ring";
        ring.id = 1;
        ring.type = visualization_msgs::msg::Marker::LINE_STRIP;
        ring.action = visualization_msgs::msg::Marker::ADD;
        ring.scale.x = 0.003;
        ring.color.r = 1.0; ring.color.g = 0.5; ring.color.b = 0.0; ring.color.a = 0.7;
        ring.lifetime = rclcpp::Duration(0, 0);
        
        for (int i = 0; i <= 32; ++i) {
            double angle = 2.0 * M_PI * i / 32;
            Eigen::Vector3d pt = center + radius * (std::cos(angle) * u + std::sin(angle) * v);
            geometry_msgs::msg::Point p;
            p.x = pt.x(); p.y = pt.y(); p.z = pt.z();
            ring.points.push_back(p);
        }
        marker_pub_->publish(ring);
        
        // 半透明圆盘
        visualization_msgs::msg::Marker disk;
        disk.header = ring.header;
        disk.ns = "constraint_disk";
        disk.id = 2;
        disk.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        disk.action = visualization_msgs::msg::Marker::ADD;
        disk.scale.x = disk.scale.y = disk.scale.z = 1.0;
        disk.color.r = 1.0; disk.color.g = 0.7; disk.color.b = 0.0; disk.color.a = 0.2;
        disk.lifetime = rclcpp::Duration(0, 0);
        
        for (int i = 0; i < 24; ++i) {
            double a1 = 2.0 * M_PI * i / 24;
            double a2 = 2.0 * M_PI * (i+1) / 24;
            geometry_msgs::msg::Point c, p1, p2;
            c.x = center.x(); c.y = center.y(); c.z = center.z();
            auto ep1 = center + radius * (std::cos(a1)*u + std::sin(a1)*v);
            auto ep2 = center + radius * (std::cos(a2)*u + std::sin(a2)*v);
            p1.x = ep1.x(); p1.y = ep1.y(); p1.z = ep1.z();
            p2.x = ep2.x(); p2.y = ep2.y(); p2.z = ep2.z();
            disk.points.push_back(c);
            disk.points.push_back(p1);
            disk.points.push_back(p2);
        }
        marker_pub_->publish(disk);
    }
    
    void publish_trajectory_line(
        const std::vector<Eigen::Vector3d>& points,
        const std::string& ns, int id,
        float r, float g, float b, float a)
    {
        visualization_msgs::msg::Marker line;
        line.header.frame_id = "base_link";
        line.header.stamp = node_->now();
        line.ns = ns;
        line.id = id;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.002;
        line.color.r = r; line.color.g = g; line.color.b = b; line.color.a = a;
        line.lifetime = rclcpp::Duration(0, 0);
        
        for (const auto& pt : points) {
            geometry_msgs::msg::Point p;
            p.x = pt.x(); p.y = pt.y(); p.z = pt.z();
            line.points.push_back(p);
        }
        marker_pub_->publish(line);
    }
    
    void publish_info_text(const StepResult& result) {
        visualization_msgs::msg::Marker text;
        text.header.frame_id = "base_link";
        text.header.stamp = node_->now();
        text.ns = "test_info";
        text.id = 0;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.pose.position.x = 0.0;
        text.pose.position.y = 0.0;
        text.pose.position.z = 0.55;
        text.pose.orientation.w = 1.0;
        text.scale.z = 0.04;
        text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0; text.color.a = 1.0;
        text.lifetime = rclcpp::Duration(0, 0);
        
        char buf[512];
        snprintf(buf, sizeof(buf),
            "Constraint Test [%s] t=%.1fs\n"
            "e_hat drift: %.4f mm %s\n"
            "pos error: %.4f mm\n"
            "max dq: %.4f rad\n"
            "cond#: %.1f %s",
            test_mode_.c_str(), step_count_ * 0.05,
            result.e_hat_drift * 1000.0,
            (result.e_hat_drift < 0.001) ? "OK" : "DRIFT!",
            result.position_error * 1000.0,
            result.max_joint_delta,
            result.condition_number,
            result.near_singularity ? "SINGULAR!" : "");
        text.text = buf;
        
        marker_pub_->publish(text);
    }
    
    // ================================================================
    // 工具方法
    // ================================================================
    
    geometry_msgs::msg::Pose compute_ee_pose(const std::vector<double>& joint_angles) {
        moveit::core::RobotState rs(robot_model_);
        for (size_t i = 0; i < joint_names_.size(); ++i)
            rs.setJointPositions(joint_names_[i], &joint_angles[i]);
        rs.update();
        
        Eigen::Isometry3d tf = rs.getGlobalLinkTransform(end_effector_link_);
        geometry_msgs::msg::Pose pose;
        pose.position.x = tf.translation().x();
        pose.position.y = tf.translation().y();
        pose.position.z = tf.translation().z();
        Eigen::Quaterniond q(tf.rotation());
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        return pose;
    }
    
    void publish_joint_states(const std::vector<double>& angles) {
        sensor_msgs::msg::JointState js;
        js.header.stamp = node_->now();
        js.name = joint_names_;
        js.position.assign(angles.begin(), angles.end());
        joint_state_pub_->publish(js);
    }
    
    void print_metrics(const StepResult& result) {
        RCLCPP_INFO(LOGGER, "--- t=%.1fs ---", step_count_ * 0.05);
        RCLCPP_INFO(LOGGER, "  e_hat drift:    %.4f mm  %s",
                   result.e_hat_drift * 1000.0,
                   (result.e_hat_drift < 0.5) ? "[PASS]" : "[FAIL: CONSTRAINT VIOLATED]");
        RCLCPP_INFO(LOGGER, "  position error: %.4f mm", result.position_error * 1000.0);
        RCLCPP_INFO(LOGGER, "  max joint Δ:    %.5f rad (%.3f deg)",
                   result.max_joint_delta, result.max_joint_delta * 180.0 / M_PI);
        RCLCPP_INFO(LOGGER, "  Jacobian cond#: %.1f %s", result.condition_number,
                   result.near_singularity ? "[WARN: NEAR SINGULARITY]" : "[OK]");
        RCLCPP_INFO(LOGGER, "  EE pos:  [%.4f, %.4f, %.4f]",
                   result.current_ee.x(), result.current_ee.y(), result.current_ee.z());
        RCLCPP_INFO(LOGGER, "  Target:  [%.4f, %.4f, %.4f]",
                   result.constrained_target.x(), result.constrained_target.y(), result.constrained_target.z());
        
        // 检测跳变（两次之间关节角变化过大）
        if (result.max_joint_delta > 0.2) {
            RCLCPP_WARN(LOGGER, "  [WARN] Large joint delta detected! Possible jump.");
        }
    }

    // ================================================================
    // 成员变量
    // ================================================================
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    const moveit::core::JointModelGroup* joint_model_group_;
    moveit::core::RobotModelConstPtr robot_model_;
    std::string end_effector_link_;
    std::vector<std::string> joint_names_;
    
    // 状态
    std::vector<double> current_joint_angles_;
    std::vector<double> controller_joint_angles_;
    
    // 约束参考
    geometry_msgs::msg::Pose reference_pose_;
    Eigen::Vector3d reference_position_;
    Eigen::Vector3d e_hat_direction_;
    
    // 可视化
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
    moveit_visual_tools::MoveItVisualToolsPtr controller_visual_tools_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr manual_sub_;
    
    // 轨迹记录
    std::vector<Eigen::Vector3d> actual_trajectory_;
    std::vector<Eigen::Vector3d> target_trajectory_;
    
    // 控制
    rclcpp::TimerBase::SharedPtr timer_;
    int step_count_ = 0;
    std::string test_mode_;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions opts;
    opts.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("test_constraint", opts);
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    
    std::atomic<bool> should_stop{false};
    std::thread exec_thread([&executor, &should_stop]() {
        while (!should_stop && rclcpp::ok())
            executor.spin_some(std::chrono::milliseconds(10));
    });
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Waiting for MoveGroup...");
    rclcpp::sleep_for(std::chrono::seconds(5));
    
    auto test_node = std::make_shared<ConstraintTestNode>(node);
    test_node->start();
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Test running. Press Ctrl+C to stop.");
    
    while (rclcpp::ok()) {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    
    should_stop = true;
    if (exec_thread.joinable()) exec_thread.join();
    rclcpp::shutdown();
    return 0;
}
