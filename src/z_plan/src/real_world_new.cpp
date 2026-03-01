/**
 ******************************************************************************
 * @file    real_world_new.cpp
 * @brief   Real-time control node for the robotic arm
 * @details
 *          receive robo_arm and controller joint angles from SerialRxNode
 *          process angles with filtering and transformations to corresponding urdf angle
 *          compute gravity compensation torques
 *          UP mode:e hat direction constraint
 *          TURN mode: plan and execute a half-circle trajectory around normal_direction
 *          ROLL mode: similar to UP mode, eliminate e_hat direction component
 *          visualize current state and planned trajectories in RViz
 * @note    Dependencies:
 *          - utils.hpp
 *          - usb_data.hpp
 ******************************************************************************
 */

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Geometry>
#include <chrono>
#include <thread>
#include <atomic>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include "utils.hpp"
#include "usb_data.hpp"
#include <moveit_visual_tools/moveit_visual_tools.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("real_world_new");

// 控制状态枚举
enum ControlMode { 
    NO = 0, 
    UP = 1, 
    TURN = 2, 
    ROLL = 3, 
    RESET = 4, 
};

// UP模式现在使用雅可比伪逆方案，不再需要状态机

/**
 * @brief 机械臂实时控制器类
 * 
 * - 从串口节点获取机械臂和控制器角度
 * - 角度滤波和坐标转换
 * - 重力补偿计算
 * - 不同控制模式的处理（NO/UP/TURN/ROLL/RESET）
 */
class RobotController {
public:
    RobotController(
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
        RCLCPP_INFO(LOGGER, "Initializing RobotController...");
        
        // 初始化 MoveIt
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node_, planning_group);
        
        // 等待机器人模型加载 (带重试机制)
        int max_retries = 10;
        int retry_count = 0;
        while (retry_count < max_retries) {
            robot_model_ = move_group_->getRobotModel();
            if (robot_model_) {
                RCLCPP_INFO(LOGGER, "Robot model loaded successfully");
                break;
            }
            RCLCPP_WARN(LOGGER, "Robot model not ready, retrying... (%d/%d)", 
                        retry_count + 1, max_retries);
            rclcpp::sleep_for(std::chrono::seconds(1));
            retry_count++;
        }
        
        if (!robot_model_) {
            RCLCPP_ERROR(LOGGER, "Failed to get robot model after %d retries!", max_retries);
            throw std::runtime_error("Failed to get robot model");
        }
        
        joint_model_group_ = robot_model_->getJointModelGroup(planning_group);
        if (!joint_model_group_) {
            RCLCPP_ERROR(LOGGER, "Failed to get joint model group: %s", planning_group.c_str());
            throw std::runtime_error("Failed to get joint model group");
        }
        
        end_effector_link_ = move_group_->getEndEffectorLink();
        joint_names_ = joint_model_group_->getActiveJointModelNames();
        
        RCLCPP_INFO(LOGGER, "Planning group: %s, End effector: %s", 
                    planning_group.c_str(), end_effector_link_.c_str());
        RCLCPP_INFO(LOGGER, "Joint names: %zu joints", joint_names_.size());
        
        // 等待机器人状态可用
        RCLCPP_INFO(LOGGER, "Waiting for robot state...");
        rclcpp::sleep_for(std::chrono::seconds(2));
        
        // 初始化重力补偿
        compute_gravity_compensation_ = make_gravity_closure(robot_model_);
        
        // 初始化 RESET 位置
        reset_pose_ = compute_end_effector_pose({0, 2.65, -0.86, 3.14, 3.14, 0});
        
        // 初始化 RViz 可视化工具（用于当前机械臂状态）
        visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
            node_, "base_link", "real_world_markers", robot_model_);
        visual_tools_->deleteAllMarkers();
        visual_tools_->loadRemoteControl();
        
        // 初始化第二个可视化工具（用于控制器状态）
        controller_visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
            node_, "base_link", "controller_markers", robot_model_);
        controller_visual_tools_->deleteAllMarkers();
        
        // 初始化标题文本标记（使用不同的话题名避免冲突）
        marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("real_world_text_markers", 10);
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = "base_link";
        text_marker.header.stamp = node_->now();
        text_marker.ns = "real_world_control";
        text_marker.id = 0;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose.position.x = 0.0;
        text_marker.pose.position.y = 0.0;
        text_marker.pose.position.z = 0.5;
        text_marker.pose.orientation.w = 1.0;
        text_marker.scale.z = 0.1;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        text_marker.text = "Real World Control";
        text_marker.lifetime = rclcpp::Duration(0, 0);
        marker_pub_->publish(text_marker);
        
        visual_tools_->trigger();
        
        // 标记初始化完成
        is_initialized_ = true;
        
        RCLCPP_INFO(LOGGER, "RobotController initialized with RViz visualization");
    }
    
    /**
     * @brief 启动控制循环（50ms 定时器）
     */
    void start_control_loop() {
        control_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&RobotController::control_callback, this));
        RCLCPP_INFO(LOGGER, "Control loop started (50ms cycle)");
    }
    
    /**
     * @brief 可视化当前机械臂状态
     */
    void visualize_current_state(const std::vector<double>& joint_angles) {
        if (!is_initialized_ || !robot_model_) {
            return;
        }
        
        moveit::core::RobotState robot_state(robot_model_);
        
        for (size_t i = 0; i < joint_names_.size() && i < joint_angles.size(); ++i) {
            robot_state.setJointPositions(joint_names_[i], &joint_angles[i]);
        }
        robot_state.update();
        
        visual_tools_->publishRobotState(robot_state, rviz_visual_tools::GREEN);
    }
    
    /**
     * @brief 可视化控制器状态（紫色）
     */
    void visualize_controller_state(const std::vector<double>& joint_angles) {
        if (!is_initialized_ || !robot_model_ || joint_angles.size() != joint_names_.size()) {
            return;
        }
        
        moveit::core::RobotState robot_state(robot_model_);
        
        for (size_t i = 0; i < joint_names_.size() && i < joint_angles.size(); ++i) {
            robot_state.setJointPositions(joint_names_[i], &joint_angles[i]);
        }
        robot_state.update();
        
        controller_visual_tools_->publishRobotState(robot_state, rviz_visual_tools::PURPLE);
    }
    
    /**
     * @brief 可视化坐标系方向
     */
    void visualize_ehat_direction(const Eigen::Vector3d& position) {
        // 显示 e_hat 方向（红色箭头）
        Eigen::Vector3d e_hat_end = position + e_hat_direction_ * 0.2;
        visual_tools_->publishArrow(
            visual_tools_->convertPoint(position),
            visual_tools_->convertPoint(e_hat_end),
            rviz_visual_tools::RED, rviz_visual_tools::MEDIUM);
        
        geometry_msgs::msg::Pose text_pose;
        text_pose.position = visual_tools_->convertPoint(e_hat_end);
        text_pose.orientation.w = 1.0;
        visual_tools_->publishText(
            text_pose,
            "e_hat", rviz_visual_tools::RED, rviz_visual_tools::SMALL, false);
        
        // 画出与 ehat 垂直的平面（末端允许运动的平面）
        visualize_perpendicular_plane(position);
    }
    
    /**
     * @brief 可视化与 ehat 垂直的平面（末端允许运动的平面）
     */
    void visualize_perpendicular_plane(const Eigen::Vector3d& position) {
        // 计算两个与 e_hat 垂直的正交向量
        Eigen::Vector3d e_hat_normalized = e_hat_direction_.normalized();
        
        // 找一个不与 e_hat 平行的向量
        Eigen::Vector3d arbitrary;
        if (std::abs(e_hat_normalized.x()) < 0.9) {
            arbitrary = Eigen::Vector3d(1, 0, 0);
        } else {
            arbitrary = Eigen::Vector3d(0, 1, 0);
        }
        
        // 通过叉乘得到两个正交向量
        Eigen::Vector3d u = e_hat_normalized.cross(arbitrary).normalized();
        Eigen::Vector3d v = e_hat_normalized.cross(u).normalized();
        
        // 创建一个圆形平面
        visualization_msgs::msg::Marker plane_marker;
        plane_marker.header.frame_id = "base_link";
        plane_marker.header.stamp = node_->now();
        plane_marker.ns = "constraint_plane";
        plane_marker.id = 1;
        plane_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        plane_marker.action = visualization_msgs::msg::Marker::ADD;
        plane_marker.scale.x = 0.003;  // 线宽
        plane_marker.color.r = 1.0;
        plane_marker.color.g = 0.5;
        plane_marker.color.b = 0.0;
        plane_marker.color.a = 0.6;
        plane_marker.lifetime = rclcpp::Duration(0, 0);
        
        // 画一个圆形表示平面
        double radius = 0.08;  // 平面半径
        int segments = 32;
        for (int i = 0; i <= segments; ++i) {
            double angle = 2.0 * M_PI * i / segments;
            Eigen::Vector3d point = position + radius * (std::cos(angle) * u + std::sin(angle) * v);
            
            geometry_msgs::msg::Point p;
            p.x = point.x();
            p.y = point.y();
            p.z = point.z();
            plane_marker.points.push_back(p);
        }
        
        marker_pub_->publish(plane_marker);
        
        // 添加一个半透明的圆盘（使用三角形列表）
        visualization_msgs::msg::Marker disk_marker;
        disk_marker.header.frame_id = "base_link";
        disk_marker.header.stamp = node_->now();
        disk_marker.ns = "constraint_plane_disk";
        disk_marker.id = 2;
        disk_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        disk_marker.action = visualization_msgs::msg::Marker::ADD;
        disk_marker.scale.x = 1.0;
        disk_marker.scale.y = 1.0;
        disk_marker.scale.z = 1.0;
        disk_marker.color.r = 1.0;
        disk_marker.color.g = 0.7;
        disk_marker.color.b = 0.0;
        disk_marker.color.a = 0.3;  // 半透明
        disk_marker.lifetime = rclcpp::Duration(0, 0);
        
        // 创建扇形三角形来填充圆盘
        int disk_segments = 24;
        for (int i = 0; i < disk_segments; ++i) {
            double angle1 = 2.0 * M_PI * i / disk_segments;
            double angle2 = 2.0 * M_PI * (i + 1) / disk_segments;
            
            // 中心点
            geometry_msgs::msg::Point center;
            center.x = position.x();
            center.y = position.y();
            center.z = position.z();
            
            // 第一个边缘点
            Eigen::Vector3d point1 = position + radius * (std::cos(angle1) * u + std::sin(angle1) * v);
            geometry_msgs::msg::Point p1;
            p1.x = point1.x();
            p1.y = point1.y();
            p1.z = point1.z();
            
            // 第二个边缘点
            Eigen::Vector3d point2 = position + radius * (std::cos(angle2) * u + std::sin(angle2) * v);
            geometry_msgs::msg::Point p2;
            p2.x = point2.x();
            p2.y = point2.y();
            p2.z = point2.z();
            
            disk_marker.points.push_back(center);
            disk_marker.points.push_back(p1);
            disk_marker.points.push_back(p2);
        }
        
        marker_pub_->publish(disk_marker);
    }

    void visualize_up_direction(const Eigen::Vector3d& position) {
        Eigen::Vector3d up_end = position + up_direction_ * 0.2;
        visual_tools_->publishArrow(
            visual_tools_->convertPoint(position),
            visual_tools_->convertPoint(up_end),
            rviz_visual_tools::GREEN, rviz_visual_tools::MEDIUM);
        
        geometry_msgs::msg::Pose text_pose;
        text_pose.position = visual_tools_->convertPoint(up_end);
        text_pose.orientation.w = 1.0;
        visual_tools_->publishText(
            text_pose,
            "up", rviz_visual_tools::GREEN, rviz_visual_tools::SMALL, false);     
    }

    void visualize_normal_direction(const Eigen::Vector3d& position) {
        Eigen::Vector3d normal_end = position + normal_direction_ * 0.2;
        visual_tools_->publishArrow(
            visual_tools_->convertPoint(position),
            visual_tools_->convertPoint(normal_end),
            rviz_visual_tools::BLUE, rviz_visual_tools::MEDIUM);
        
        geometry_msgs::msg::Pose text_pose;
        text_pose.position = visual_tools_->convertPoint(normal_end);
        text_pose.orientation.w = 1.0;
        visual_tools_->publishText(
            text_pose,
            "normal", rviz_visual_tools::BLUE, rviz_visual_tools::SMALL, false);     
    }

    /**
     * @brief 在 RViz 中显示参考位姿信息
     */
    void visualize_reference_pose_info() {
        if (!reference_pose_initialized_) {
            RCLCPP_WARN(LOGGER, "Reference pose not initialized, skipping visualization");
            return;
        }
        
        visualization_msgs::msg::Marker info_marker;
        info_marker.header.frame_id = "base_link";
        info_marker.header.stamp = node_->now();
        info_marker.ns = "reference_pose_info";
        info_marker.id = 1;
        info_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        info_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // 在末端执行器附近显示
        info_marker.pose.position.x = reference_pose_.position.x + 0.1;
        info_marker.pose.position.y = reference_pose_.position.y + 0.1;
        info_marker.pose.position.z = reference_pose_.position.z + 0.15;
        info_marker.pose.orientation.x = 0.0;
        info_marker.pose.orientation.y = 0.0;
        info_marker.pose.orientation.z = 0.0;
        info_marker.pose.orientation.w = 1.0;
        
        // 确保 scale 设置完整
        info_marker.scale.x = 1.0;
        info_marker.scale.y = 1.0;
        info_marker.scale.z = 0.05;  // 文字大小
        
        info_marker.color.r = 1.0;
        info_marker.color.g = 1.0;
        info_marker.color.b = 0.0;  // 黄色
        info_marker.color.a = 1.0;
        
        // 构建显示文本
        char text[512];
        snprintf(text, sizeof(text),
                "UP Mode Reference:\n"
                "e_hat: [%.3f, %.3f, %.3f]\n"
                "EE pos: [%.3f, %.3f, %.3f]",
                e_hat_direction_.x(), e_hat_direction_.y(), e_hat_direction_.z(),
                reference_pose_.position.x, reference_pose_.position.y, reference_pose_.position.z);
        info_marker.text = text;
        
        info_marker.lifetime = rclcpp::Duration(0, 0);  // 持续显示
        
        marker_pub_->publish(info_marker);
        
        RCLCPP_DEBUG(LOGGER, "Published reference pose info marker at [%.3f, %.3f, %.3f]",
                    info_marker.pose.position.x, info_marker.pose.position.y, info_marker.pose.position.z);
    }   

private:
    /**
     * @brief 控制循环回调函数
     */
    void control_callback() {
        auto robo_arm_angles_raw = serial_node_->get_robo_arm_angles();
        auto controller_angles_raw = serial_node_->get_controller_angles();
        uint8_t control_mode = serial_node_->get_control_mode();
        
        // 打印收到的原始数据（每秒打印一次，即每20个循环）
        log_counter_++;
        if (log_counter_ >= 20) {
            log_counter_ = 0;
            RCLCPP_INFO(LOGGER, "=== Callback Data ===");
            RCLCPP_INFO(LOGGER, "Control Mode: %d", control_mode);
            if (robo_arm_angles_raw.size() == 6) {
                RCLCPP_INFO(LOGGER, "Robo Arm Angles: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                    robo_arm_angles_raw[0], robo_arm_angles_raw[1], robo_arm_angles_raw[2],
                    robo_arm_angles_raw[3], robo_arm_angles_raw[4], robo_arm_angles_raw[5]);
            } else {
                RCLCPP_INFO(LOGGER, "Robo Arm Angles: size=%zu (invalid)", robo_arm_angles_raw.size());
            }
            if (controller_angles_raw.size() == 6) {
                RCLCPP_INFO(LOGGER, "Controller Angles: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                    controller_angles_raw[0], controller_angles_raw[1], controller_angles_raw[2],
                    controller_angles_raw[3], controller_angles_raw[4], controller_angles_raw[5]);
            }
            RCLCPP_INFO(LOGGER, "====================");
        }
        
        if (robo_arm_angles_raw.size() != 6) {
            RCLCPP_DEBUG(LOGGER, "Waiting for robo_arm data...");
            return;
        }
        
        //角度滤波和转换
        std::vector<double> robo_arm_angles = robo_filter_.filter(robo_arm_angles_raw);
        robo_arm_angles[1] = -robo_arm_angles[1];
        robo_arm_angles[4] = 6.28 - robo_arm_angles[4];
        std::vector<double> controller_angles = controller_filter_.filter(controller_angles_raw);
        controller_angles[1] = -controller_angles[1];
        controller_angles[4] = 6.28 - controller_angles[4];


        
        // 计算重力补偿力矩
        latest_torques_ = compute_gravity_compensation_(robo_arm_angles);
        
        // 检测模式切换并更新坐标系
        if (control_mode != last_control_mode_) {
            RCLCPP_INFO(LOGGER, "Control mode changed: %d -> %d", last_control_mode_, control_mode);
            
            if (control_mode == UP) {
                reference_pose_ = compute_end_effector_pose(robo_arm_angles);
                reference_pose_initialized_ = true;
                
                // e_hat_direction 是 link6 的 x 轴方向在世界坐标系中的表示
                Eigen::Quaterniond quat(
                    reference_pose_.orientation.w,
                    reference_pose_.orientation.x,
                    reference_pose_.orientation.y,
                    reference_pose_.orientation.z
                );
                e_hat_direction_ = quat * Eigen::Vector3d(1, 0, 0);  // link6 的 x 轴
                
                RCLCPP_INFO(LOGGER, "UP mode entered:");
                RCLCPP_INFO(LOGGER, "  e_hat direction: [%.3f, %.3f, %.3f]",
                           e_hat_direction_.x(), e_hat_direction_.y(), e_hat_direction_.z());
                RCLCPP_INFO(LOGGER, "  EE position: [%.3f, %.3f, %.3f]",
                           reference_pose_.position.x, reference_pose_.position.y, reference_pose_.position.z());
                RCLCPP_INFO(LOGGER, "  Using Jacobian-based velocity-to-angle mapping");
                
                // 在 RViz 中显示参考位姿信息
                visualize_reference_pose_info();
            }

            if (control_mode==TURN){
                up_end_pose_ = compute_end_effector_pose(robo_arm_angles);
                up_direction_ = (Eigen::Vector3d(up_end_pose_.position.x, up_end_pose_.position.y, up_end_pose_.position.z) - 
                                 Eigen::Vector3d(reference_pose_.position.x, reference_pose_.position.y, reference_pose_.position.z)).normalized();
                normal_direction_ = e_hat_direction_.cross(up_direction_).normalized();
                RCLCPP_INFO(LOGGER, "TURN mode entered:");
                RCLCPP_INFO(LOGGER, "  up direction: [%.3f, %.3f, %.3f]",
                           up_direction_.x(), up_direction_.y(), up_direction_.z());
                RCLCPP_INFO(LOGGER, "  normal direction: [%.3f, %.3f, %.3f]",
                           normal_direction_.x(), normal_direction_.y(), normal_direction_.z());
            }
            
            last_control_mode_ = control_mode;
        }
        
        //可视化当前机械臂状态
        visualize_current_state(robo_arm_angles);
        
        //可视化控制器状态
        if (controller_angles_raw.size() == 6) {
            visualize_controller_state(controller_angles);
        }
        
        //可视化坐标系方向（在末端执行器位置）
        geometry_msgs::msg::Pose current_pose = compute_end_effector_pose(robo_arm_angles);
        Eigen::Vector3d ee_position(current_pose.position.x, 
                                    current_pose.position.y, 
                                    current_pose.position.z);
        
        // 根据控制模式处理
        switch (control_mode) {
            case NO:
                handle_mode_no(controller_angles);
                break;
            case UP:
                visualize_ehat_direction(ee_position);
                visualize_reference_pose_info();  // 持续显示参考信息
                handle_mode_up(robo_arm_angles, controller_angles);
                //plan_and_execute_random_trajectory_in_plane(robo_arm_angles);
                break;
            case TURN:
                visualize_ehat_direction(ee_position);
                visualize_up_direction(ee_position);
                handle_mode_turn();
                break;
            case ROLL:
                handle_mode_roll(robo_arm_angles, controller_angles);
                break;
            case RESET:
                handle_mode_reset();
                break;
        }
        
        // 统一触发所有可视化更新
        visual_tools_->trigger();
        controller_visual_tools_->trigger();
    }
    
    /**
     * @brief 处理 NO 模式
     */
    void handle_mode_no(const std::vector<double>& controller_angles) {
        RCLCPP_DEBUG(LOGGER, "Mode NO: not controlled by NUC");
        std::vector<float> send_angles = convert_urdf_to_hardware(controller_angles);
        serial_node_->send_roboarm_with_controller(latest_torques_, send_angles);
    }
    
    /**
     * @brief 处理 UP 模式 — 雅可比伪逆速度→角度映射
     * 
     * 替换旧的 computeCartesianPath + IK + 状态机方案：
     *   旧：controller FK → 投影 → computeCartesianPath(IK) → 多帧轨迹 → 状态机
     *   新：controller FK → 投影 → 位移差 → J⁺·v·dt → 角度增量 → 直接发送
     * 
     * 优势：
     *   - 无 IK 多值解跳变（雅可比给出唯一连续解）
     *   - 无状态机，每个控制周期独立计算
     *   - 延迟从 ~100ms 降到 ~1ms
     */
    void handle_mode_up(
        const std::vector<double>& robo_arm_angles,
        const std::vector<double>& controller_angles)
    {
        if (controller_angles.size() != 6 || robo_arm_angles.size() != 6) {
            RCLCPP_WARN(LOGGER, "Mode UP: waiting for valid data");
            return;
        }
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
        
        // Step 3: controller 相对于参考点的位移，投影到约束平面（消除 e_hat 分量）
        Eigen::Vector3d reference_position(
            reference_pose_.position.x,
            reference_pose_.position.y,
            reference_pose_.position.z);
        
        Eigen::Vector3d displacement = controller_position - reference_position;
        Eigen::Vector3d e_hat = e_hat_direction_.normalized();
        double e_hat_component = displacement.dot(e_hat);
        Eigen::Vector3d constrained_displacement = displacement - e_hat_component * e_hat;
        
        // 约束后的目标位置
        Eigen::Vector3d target_position = reference_position + constrained_displacement;
        
        // Step 4: 当前位置到约束目标的速度方向（同样消除 e_hat 分量，双重保险）
        Eigen::Vector3d velocity = target_position - current_position;
        velocity = velocity - velocity.dot(e_hat) * e_hat;
        
        // Step 5: 死区检查（距离 < 1mm 时不动）
        double dist = velocity.norm();
        if (dist < 0.001) {
            // 已经到达目标，维持当前角度
            std::vector<float> send_angles = convert_urdf_to_hardware(robo_arm_angles);
            serial_node_->send_roboarm_with_controller(latest_torques_, send_angles);
            return;
        }
        
        // Step 6: 限速（最大线速度 0.15 m/s）
        const double max_linear_speed = 0.15;
        if (dist > max_linear_speed) {
            velocity = velocity * (max_linear_speed / dist);
        }
        
        // Step 7: 用雅可比伪逆把笛卡尔速度映射为关节角增量
        //   Δq = J⁺ · v · dt
        //   q_new = q_current + Δq
        const double dt = 0.05;  // 控制周期 50ms
        std::vector<double> target_joint_angles = jacobian_velocity_to_joint_angles(
            robo_arm_angles, velocity, dt);
        
        if (target_joint_angles.empty()) {
            // 雅可比计算失败（奇异点附近），维持当前角度
            RCLCPP_WARN(LOGGER, "UP mode: Jacobian computation failed (near singularity)");
            std::vector<float> send_angles = convert_urdf_to_hardware(robo_arm_angles);
            serial_node_->send_roboarm_with_controller(latest_torques_, send_angles);
            return;
        }
        
        // Step 8: 发送目标角度
        std::vector<float> send_angles = convert_urdf_to_hardware(target_joint_angles);
        serial_node_->send_roboarm_with_controller(latest_torques_, send_angles);
        
        RCLCPP_DEBUG(LOGGER, "UP Jacobian: v=[%.4f,%.4f,%.4f] norm=%.4f e_hat_rm=%.4f",
                    velocity.x(), velocity.y(), velocity.z(), dist, e_hat_component);
    }
    
    /**
     * @brief 用雅可比伪逆将笛卡尔线速度映射为目标关节角度
     * 
     * 核心公式：
     *   J = ∂x/∂q  (6×n 雅可比矩阵，前3行是线速度，后3行是角速度)
     *   J_linear = J 的前3行 (3×n)
     *   Δq = J_linear⁺ · v · dt
     *   q_target = q_current + Δq
     * 
     * @param current_joint_angles  当前关节角度 (rad)
     * @param linear_velocity       期望的笛卡尔线速度 (m/s)
     * @param dt                    时间步长 (s)
     * @return 目标关节角度，失败时返回空 vector
     */
    std::vector<double> jacobian_velocity_to_joint_angles(
        const std::vector<double>& current_joint_angles,
        const Eigen::Vector3d& linear_velocity,
        double dt)
    {
        // 构建当前 RobotState
        moveit::core::RobotState robot_state(robot_model_);
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            robot_state.setJointPositions(joint_names_[i], &current_joint_angles[i]);
        }
        robot_state.update();
        
        // 获取雅可比矩阵 (6×n)，参考点为末端执行器
        Eigen::MatrixXd full_jacobian;
        Eigen::Vector3d reference_point(0.0, 0.0, 0.0);  // 末端连杆坐标系原点
        if (!robot_state.getJacobian(
                joint_model_group_,
                robot_state.getLinkModel(end_effector_link_),
                reference_point,
                full_jacobian)) {
            RCLCPP_ERROR(LOGGER, "Failed to compute Jacobian");
            return {};
        }
        
        // 只取线速度部分 (前3行) → J_linear: 3×n
        Eigen::MatrixXd J_linear = full_jacobian.topRows(3);
        
        // 计算伪逆: J⁺ = Jᵀ(JJᵀ)⁻¹
        // 加阻尼因子避免奇异点附近求逆不稳定
        const double damping = 0.005;  // 阻尼因子
        Eigen::MatrixXd JJt = J_linear * J_linear.transpose();
        JJt += damping * damping * Eigen::Matrix3d::Identity();  // 阻尼最小二乘
        
        // 检查是否接近奇异（条件数过大）
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_linear);
        double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
        if (cond > 100.0) {
            RCLCPP_WARN(LOGGER, "Near singularity! Condition number: %.1f", cond);
            // 不返回空，阻尼伪逆本身可以处理，但降低速度
            // velocity 已经在调用方限速了
        }
        
        Eigen::MatrixXd J_pinv = J_linear.transpose() * JJt.inverse();  // n×3
        
        // Δq = J⁺ · v · dt
        Eigen::VectorXd delta_q = J_pinv * linear_velocity * dt;
        
        // 构造目标关节角度
        std::vector<double> target_angles(current_joint_angles.size());
        for (size_t i = 0; i < current_joint_angles.size(); ++i) {
            target_angles[i] = current_joint_angles[i] + delta_q(i);
        }
        
        // 检查关节限位
        const auto& joint_models = joint_model_group_->getActiveJointModels();
        for (size_t i = 0; i < joint_models.size(); ++i) {
            const auto& bounds = joint_models[i]->getVariableBounds();
            if (!bounds.empty()) {
                double lo = bounds[0].min_position_;
                double hi = bounds[0].max_position_;
                if (target_angles[i] < lo || target_angles[i] > hi) {
                    RCLCPP_WARN(LOGGER, "Joint %s exceeds limit [%.2f, %.2f]: %.4f, clamping",
                               joint_names_[i].c_str(), lo, hi, target_angles[i]);
                    target_angles[i] = std::clamp(target_angles[i], lo, hi);
                }
            }
        }
        
        return target_angles;
    }
    
    /**
     * @brief 检查关节位置是否到达目标
     */
    bool check_joint_positions_reached(
        const std::vector<double>& current_angles,
        const std::vector<double>& target_angles)
    {
        if (current_angles.size() != target_angles.size() || current_angles.size() != 6) {
            return false;
        }
        
        for (size_t i = 0; i < current_angles.size(); ++i) {
            double error = std::abs(current_angles[i] - target_angles[i]);
            if (error > joint_position_tolerance_) {
                return false;
            }
        }
        
        return true;
    }

    /**
     * @brief 处理 TURN 模式
     * 
     * 以距离末端位置 100mm 的点为圆心，绕 normal_direction 往 up_direction 方向转90度
     */
    void handle_mode_turn() {
        RCLCPP_INFO(LOGGER, "Mode TURN: planning arc trajectory...");
        
        if (!reference_pose_initialized_) {
            RCLCPP_WARN(LOGGER, "Mode TURN: reference pose not initialized, skipping");
            return;
        }
        
        //旋转中心(从当前退100mm)
        Eigen::Vector3d end_position(up_end_pose_.position.x, 
                                     up_end_pose_.position.y, 
                                     up_end_pose_.position.z);
        
        Eigen::Vector3d circle_center = end_position - e_hat_direction_ * 0.1;
        
        RCLCPP_INFO(LOGGER, "  Starting position: [%.3f, %.3f, %.3f]",
                   end_position.x(), end_position.y(), end_position.z());
        RCLCPP_INFO(LOGGER, "  Circle center: [%.3f, %.3f, %.3f]",
                   circle_center.x(), circle_center.y(), circle_center.z());
        RCLCPP_INFO(LOGGER, "  e_hat: [%.3f, %.3f, %.3f]",
                   e_hat_direction_.x(), e_hat_direction_.y(), e_hat_direction_.z());
        RCLCPP_INFO(LOGGER, "  up: [%.3f, %.3f, %.3f]",
                   up_direction_.x(), up_direction_.y(), up_direction_.z());
        
        std::vector<geometry_msgs::msg::Pose> waypoints;
        const int num_points = 30;  
        const double angle_range = M_PI / 2.0; 
        const double radius = 0.1; 
        
        for (int i = 0; i <= num_points; ++i) {
            double angle = angle_range * i / num_points;
            
            // 圆弧上的点
            Eigen::Vector3d point = circle_center + 
                                   radius * (std::sin(angle) * up_direction_ + 
                                            std::cos(angle) * e_hat_direction_);
            
            geometry_msgs::msg::Pose waypoint;
            waypoint.position.x = point.x();
            waypoint.position.y = point.y();
            waypoint.position.z = point.z();
            
            //使用 up_end_pose_ 的pose
            waypoint.orientation = up_end_pose_.orientation;
            
            waypoints.push_back(waypoint);
        }
        
        // 打印前几个路径点用于调试
        RCLCPP_INFO(LOGGER, "  First waypoint: [%.3f, %.3f, %.3f]",
                   waypoints[0].position.x, waypoints[0].position.y, waypoints[0].position.z);
        if (waypoints.size() > 10) {
            RCLCPP_INFO(LOGGER, "  Mid waypoint: [%.3f, %.3f, %.3f]",
                       waypoints[waypoints.size()/2].position.x, 
                       waypoints[waypoints.size()/2].position.y, 
                       waypoints[waypoints.size()/2].position.z);
        }
        RCLCPP_INFO(LOGGER, "  Last waypoint: [%.3f, %.3f, %.3f]",
                   waypoints.back().position.x, waypoints.back().position.y, waypoints.back().position.z);
        
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.01; 
        
        double fraction = move_group_->computeCartesianPath(
            waypoints, eef_step, trajectory);
        
        RCLCPP_INFO(LOGGER, "  Cartesian path: %.2f%% achieved", fraction * 100.0);
        
        if (fraction > 0.8) {
            visual_tools_->publishPath(waypoints, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
            visual_tools_->trigger();
            
            RCLCPP_INFO(LOGGER, "Mode TURN: arc trajectory planned and visualized");
            
            auto trajectory_dict = extractTrajectory(trajectory);
            RCLCPP_INFO(LOGGER, "  Executing trajectory with %zu points", trajectory_dict.size());
            
            for (const auto& point : trajectory_dict) {
                std::vector<double> joint_angles = point.second;
                
                if (joint_angles.size() != 6) {
                    RCLCPP_ERROR(LOGGER, "Invalid trajectory point with %zu joints, skipping", joint_angles.size());
                    continue;
                }
                
                std::vector<float> send_angles = convert_urdf_to_hardware(joint_angles);
                serial_node_->send_roboarm_with_controller(latest_torques_, send_angles);
                
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        } else {
            RCLCPP_WARN(LOGGER, "Mode TURN: failed to plan complete trajectory (%.2f%%), not executing", fraction * 100.0);
            std::vector<float> send_angles = {0, 0, 0, 0, 0, 0};
            serial_node_->send_roboarm_with_controller(latest_torques_, send_angles);
        }
    }
    
    /**
     * @brief 处理 ROLL 模式（雅可比伪逆方案）
     * 
     * 与 UP 类似，但消除 e_hat 分量的同时允许绕 e_hat 轴旋转。
     * 如果你想做纯平移（和UP一样），直接复用 handle_mode_up 即可。
     */
    void handle_mode_roll(
        const std::vector<double>& robo_arm_angles,
        const std::vector<double>& controller_angles)
    {
        // 暂时复用 UP 模式的逻辑
        handle_mode_up(robo_arm_angles, controller_angles);
    }

    /**
     * @brief 处理 RESET 模式（待实现）
     */
    void handle_mode_reset() {

        RCLCPP_DEBUG(LOGGER, "Mode RESET: not implemented yet");
    }

    /**
     * @brief 在 UP 模式的约束平面内规划并执行随机轨迹
     * 
     * 在垂直于 e_hat 方向的平面内随机选取3个目标点，
     * 使用 Cartesian 路径规划生成轨迹并执行
     */
    void plan_and_execute_random_trajectory_in_plane(const std::vector<double>& current_joint_angles) {
        if (!reference_pose_initialized_) {
            RCLCPP_WARN(LOGGER, "Reference pose not initialized, cannot plan random trajectory");
            return;
        }
        
        RCLCPP_INFO(LOGGER, "Planning random trajectory in constraint plane...");
        
        // 获取当前末端位置
        geometry_msgs::msg::Pose current_pose = compute_end_effector_pose(current_joint_angles);
        Eigen::Vector3d current_position(current_pose.position.x, 
                                        current_pose.position.y, 
                                        current_pose.position.z);
        
        // 计算两个与 e_hat 垂直的正交向量（用于在平面内生成随机点）
        Eigen::Vector3d e_hat_normalized = e_hat_direction_.normalized();
        
        // 找一个不与 e_hat 平行的向量
        Eigen::Vector3d arbitrary;
        if (std::abs(e_hat_normalized.x()) < 0.9) {
            arbitrary = Eigen::Vector3d(1, 0, 0);
        } else {
            arbitrary = Eigen::Vector3d(0, 1, 0);
        }
        
        // 通过叉乘得到两个正交向量
        Eigen::Vector3d u = e_hat_normalized.cross(arbitrary).normalized();
        Eigen::Vector3d v = e_hat_normalized.cross(u).normalized();
        
        // 生成3个随机目标点（在约束平面内）
        std::vector<geometry_msgs::msg::Pose> waypoints;
        const double max_radius = 0.08;  // 最大偏移半径 80mm
        
        // 添加起始点
        waypoints.push_back(current_pose);
        
        // 生成3个随机目标点
        srand(time(NULL));
        for (int i = 0; i < 3; ++i) {
            // 随机角度和半径
            double angle = (rand() % 360) * M_PI / 180.0;
            double radius = (rand() % 100) / 100.0 * max_radius;
            
            // 在平面内生成点
            Eigen::Vector3d offset = radius * (std::cos(angle) * u + std::sin(angle) * v);
            Eigen::Vector3d target_position = current_position + offset;
            
            geometry_msgs::msg::Pose target_pose;
            target_pose.position.x = target_position.x();
            target_pose.position.y = target_position.y();
            target_pose.position.z = target_position.z();
            
            // 保持末端姿态不变（继承参考姿态）
            target_pose.orientation = reference_pose_.orientation;
            
            waypoints.push_back(target_pose);
            
            RCLCPP_INFO(LOGGER, "  Target %d: [%.3f, %.3f, %.3f], offset radius: %.3f",
                       i + 1, target_position.x(), target_position.y(), target_position.z(), radius);
        }
        
        // 使用 Cartesian 路径规划
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.01;  // 1cm 步长
        
        double fraction = move_group_->computeCartesianPath(
            waypoints, eef_step, trajectory);
        
        RCLCPP_INFO(LOGGER, "  Cartesian path: %.2f%% achieved", fraction * 100.0);
        
        if (fraction > 0.8) {
            visual_tools_->publishPath(waypoints, rviz_visual_tools::CYAN, rviz_visual_tools::SMALL);
            visual_tools_->trigger();
            
            RCLCPP_INFO(LOGGER, "Random trajectory planned and visualized");
            
            // 提取轨迹点
            auto trajectory_dict = extractTrajectory(trajectory);
            RCLCPP_INFO(LOGGER, "  Executing trajectory with %zu points", trajectory_dict.size());
            
            // 执行轨迹
            for (const auto& point : trajectory_dict) {
                std::vector<double> joint_angles = point.second;
                
                if (joint_angles.size() != 6) {
                    RCLCPP_ERROR(LOGGER, "Invalid trajectory point with %zu joints, skipping", joint_angles.size());
                    continue;
                }
                
                // 计算当前点的重力补偿
                std::vector<float> torques = compute_gravity_compensation_(joint_angles);
                
                // 转换为硬件角度并发送
                std::vector<float> send_angles = convert_urdf_to_hardware(joint_angles);
                //serial_node_->send_roboarm_with_controller(torques, send_angles);
                
                // 控制执行速度
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            
            RCLCPP_INFO(LOGGER, "Random trajectory execution completed");
        } else {
            RCLCPP_WARN(LOGGER, "Failed to plan complete trajectory (%.2f%%), not executing", fraction * 100.0);
        }
    }

    
    /**
     * @brief 计算末端执行器位姿
     */
    geometry_msgs::msg::Pose compute_end_effector_pose(const std::vector<double>& joint_angles) {
        moveit::core::RobotState robot_state(robot_model_);
        
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            robot_state.setJointPositions(joint_names_[i], &joint_angles[i]);
        }
        robot_state.update();
        
        Eigen::Isometry3d ee_transform = 
            robot_state.getGlobalLinkTransform(end_effector_link_);
        
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
        std::vector<float> hardware_angles(urdf_angles.begin(), urdf_angles.end());
        hardware_angles[1] = -hardware_angles[1];
        hardware_angles[4] = 6.28 - hardware_angles[4];
        return hardware_angles;
    }

    // 成员变量
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<SerialRxNode> serial_node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    const moveit::core::JointModelGroup* joint_model_group_;
    moveit::core::RobotModelConstPtr robot_model_;
    std::string end_effector_link_;
    std::vector<std::string> joint_names_;
    
    AngleFilter robo_filter_;
    AngleFilter controller_filter_;

    
    std::function<std::vector<float>(const std::vector<double>&)> compute_gravity_compensation_;
    std::vector<float> latest_torques_;
    
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // RViz 可视化工具
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
    moveit_visual_tools::MoveItVisualToolsPtr controller_visual_tools_;  // 用于控制器状态
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    //坐标轴方向
    Eigen::Vector3d e_hat_direction_;
    Eigen::Vector3d up_direction_;
    Eigen::Vector3d normal_direction_;

    //UP开始时的pose
    geometry_msgs::msg::Pose reference_pose_;
    bool reference_pose_initialized_ = false;
    
    //UP结束时的pose
    geometry_msgs::msg::Pose up_end_pose_;
    
    // 模式切换记录
    uint8_t last_control_mode_;

    //RESET位置
    geometry_msgs::msg::Pose reset_pose_;  // 在构造函数中初始化
    
    // 日志计数器（用于减少日志频率）
    int log_counter_ = 0;
    
    // 初始化完成标志
    bool is_initialized_ = false;
    
    // UP模式相关变量（已改用雅可比方案，不再需要状态机）
    // 保留 joint_position_tolerance_ 供 check_joint_positions_reached 使用（TURN模式可能用到）
    const double joint_position_tolerance_ = 0.05;

};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("real_world_new", node_options);

    auto serial_node = std::make_shared<SerialRxNode>();
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(serial_node);
    
    // 在后台线程中启动 executor，让 MoveGroup 有机会初始化
    std::atomic<bool> should_stop{false};
    std::thread executor_thread([&executor, &should_stop]() {
        while (!should_stop && rclcpp::ok()) {
            executor.spin_some(std::chrono::milliseconds(10));
        }
    });
    
    // 启动串口接收线程
    std::thread receive_thread([serial_node]() {
        serial_node->receive_data();
    });

    // 等待 MoveGroup 初始化（给足够的时间）
    RCLCPP_INFO(LOGGER, "Waiting for MoveGroup to initialize...");
    rclcpp::sleep_for(std::chrono::seconds(5));
    
    // 现在创建 RobotController
    RCLCPP_INFO(LOGGER, "Creating RobotController...");
    auto controller = std::make_shared<RobotController>(node, serial_node);
    controller->start_control_loop();

    RCLCPP_INFO(LOGGER, "System ready, starting main loop...");
    
    // 等待关闭信号
    while (rclcpp::ok()) {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    
    RCLCPP_INFO(LOGGER, "Shutting down...");
    should_stop = true;
    
    if (executor_thread.joinable()) {
        executor_thread.join();
    }
    
    if (receive_thread.joinable()) {
        receive_thread.join();
    }
    
    rclcpp::shutdown();
    return 0;
}
