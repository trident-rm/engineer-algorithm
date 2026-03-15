/**
 ******************************************************************************
 * @file    rviz_simulation.cpp
 * @brief   RViz-based simulation control node for the robotic arm
 * @details
 *          Unlike real_world_new.cpp which receives data from SerialRxNode,
 *          this node uses RViz's native Goal State target to control the robotic arm.
 *          - User drags the Goal State target in RViz (no custom markers needed)
 *          - Goal pose is converted to controller joint angles via IK
 *          - Control mode is changed via keyboard input or RViz UI
 *          - The same control logic (UP/TURN/ROLL/RESET modes) applies
 *          - Two visual arms are displayed: robot arm (green) and restrict arm (blue)
 * @note    Dependencies:
 *          - utils.hpp
 *          - MoveIt MotionPlanning plugin (provides Goal State target)
 ******************************************************************************
 */

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("rviz_simulation");

// 控制状态枚举
enum ControlMode { 
    NO = 0, 
    UP = 1, 
    TURN = 2, 
    ROLL = 3, 
    RESET = 4, 
};

/**
 * @brief 机械臂仿真控制器类 (基于 RViz 交互式标记)
 * 
 */
class RobotSimulationController {
public:
    RobotSimulationController(
        rclcpp::Node::SharedPtr node,
        const std::string& planning_group = "arm")
        : node_(node),
          e_hat_direction_(1, 0, 0),
          up_direction_(0, 0, 1), 
          normal_direction_(0, 1, 0), 
          last_control_mode_(NO),
          current_control_mode_(NO),
          controller_angles_(6, 0.0),
          robo_arm_angles_(6, 0.0)
    {
        RCLCPP_INFO(LOGGER, "Initializing RobotSimulationController...");
        
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
        planning_frame_ = move_group_->getPlanningFrame();
        joint_names_ = joint_model_group_->getActiveJointModelNames();
        
        RCLCPP_INFO(LOGGER, "Planning group: %s, End effector: %s", 
                    planning_group.c_str(), end_effector_link_.c_str());
        RCLCPP_INFO(LOGGER, "Planning frame: %s", planning_frame_.c_str());
        RCLCPP_INFO(LOGGER, "Joint names: %zu joints", joint_names_.size());
        
        // 初始化重力补偿
        compute_gravity_compensation_ = make_gravity_closure(robot_model_);
        
        // 初始化 RESET 位置
        reset_pose_ = compute_end_effector_pose({0, 2.65, -0.86, 3.14, 3.14, 0});
        
        // 初始化 RViz 可视化工具
        RCLCPP_INFO(LOGGER, "Initializing MoveItVisualTools with frame: %s", planning_frame_.c_str());
        visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
            node_, planning_frame_, "simulation_markers", robot_model_);
        visual_tools_->deleteAllMarkers();
        visual_tools_->loadRemoteControl();
        RCLCPP_INFO(LOGGER, "MoveItVisualTools initialized successfully");
        
        // 初始化 DisplayRobotState 发布器
        robot_state_pub_ = node_->create_publisher<moveit_msgs::msg::DisplayRobotState>(
            "display_robot_state", rclcpp::QoS(1).transient_local());
        
        restrict_arm_state_pub_ = node_->create_publisher<moveit_msgs::msg::DisplayRobotState>(
            "restrict_arm_display_robot_state", rclcpp::QoS(1).transient_local());
        
        // 初始化标记发布器
        marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("simulation_text_markers", 10);
        
        // 订阅 RViz MotionPlanning 发布的 controller 显示状态（用户交互）
        controller_state_sub_ = node_->create_subscription<moveit_msgs::msg::DisplayRobotState>(
            "controller_display_robot_state",
            rclcpp::QoS(1).transient_local(),
            std::bind(&RobotSimulationController::controller_state_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(LOGGER, "Subscribed to: controller_display_robot_state (user interaction)");
        RCLCPP_INFO(LOGGER, "Publishing to: display_robot_state (real arm - green)");
        RCLCPP_INFO(LOGGER, "Publishing to: restrict_arm_display_robot_state (constrained arm - blue)");
        
        // 创建控制模式切换服务
        mode_service_ = node_->create_service<std_srvs::srv::SetBool>(
            "set_control_mode",
            std::bind(&RobotSimulationController::handle_mode_request, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        // 初始化关节角度为 RESET 位置
        robo_arm_angles_ = {0, -2.65, -0.86, 3.14, 3.14, 0};
        controller_angles_ = {0, -2.65, -0.86, 3.14, 3.14, 0};
        
        // 标记初始化完成
        is_initialized_ = true;
        
        RCLCPP_INFO(LOGGER, "RobotSimulationController initialized");
        RCLCPP_INFO(LOGGER, "Architecture:");
        RCLCPP_INFO(LOGGER, "  Subscribe: /controller_display_robot_state (user Goal State interaction)");
        RCLCPP_INFO(LOGGER, "  Calculate restrict_arm_angles based on control mode");
        RCLCPP_INFO(LOGGER, "  Publish: /display_robot_state (robo_arm - green arm)");
        RCLCPP_INFO(LOGGER, "  Publish: /restrict_arm_display_robot_state (constrained - blue arm)");
        RCLCPP_INFO(LOGGER, "Available control modes: 0=NO, 1=UP, 2=TURN, 3=ROLL, 4=RESET");
        RCLCPP_INFO(LOGGER, "To change mode, use: ros2 service call /set_control_mode std_srvs/srv/SetBool \"data: <mode>\"");
    }
    
    /**
     * @brief 发布初始 controller 状态（橘色机械臂初始位置）
     */
    void publish_initial_controller_state() {
        try {
            moveit::core::RobotState robot_state(robot_model_);
            for (size_t i = 0; i < joint_names_.size() && i < controller_angles_.size(); ++i) {
                robot_state.setJointPositions(joint_names_[i], &controller_angles_[i]);
            }
            robot_state.update();

            moveit_msgs::msg::DisplayRobotState display_msg;
            moveit::core::robotStateToRobotStateMsg(robot_state, display_msg.state);
            
            robot_state_pub_->publish(display_msg);
            
            RCLCPP_INFO(LOGGER, "[SUCCESS] Published initial controller state to /display_robot_state");
            RCLCPP_INFO(LOGGER, "  Planning frame: %s", planning_frame_.c_str());
            RCLCPP_INFO(LOGGER, "  angles: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                        controller_angles_[0], controller_angles_[1], controller_angles_[2],
                        controller_angles_[3], controller_angles_[4], controller_angles_[5]);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(LOGGER, "Failed to publish initial state: %s", e.what());
        }
    }
    
    /**
     * @brief 启动控制循环（20ms 定时器）
     */
    void start_control_loop() {
        // 确保RViz、TF树和MoveGroup都已初始化
        RCLCPP_INFO(LOGGER, "========================================");
        RCLCPP_INFO(LOGGER, "Starting control loop initialization...");
        RCLCPP_INFO(LOGGER, "Planning frame: '%s'", planning_frame_.c_str());
        RCLCPP_INFO(LOGGER, "Robot model name: '%s'", robot_model_->getName().c_str());
        RCLCPP_INFO(LOGGER, "Joint names: %zu", joint_names_.size());
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            RCLCPP_INFO(LOGGER, "  [%zu] %s", i, joint_names_[i].c_str());
        }
        RCLCPP_INFO(LOGGER, "========================================");
        
        RCLCPP_INFO(LOGGER, "Waiting 3 seconds for TF tree and RViz to initialize...");
        rclcpp::sleep_for(std::chrono::seconds(3));
        
        // 发布初始状态
        RCLCPP_INFO(LOGGER, "Publishing initial arm state...");
        publish_initial_controller_state();
        
        // 再等一段时间让markers发送完成
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        
        control_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&RobotSimulationController::control_callback, this));
        
        RCLCPP_INFO(LOGGER, "========================================");
        RCLCPP_INFO(LOGGER, "Control loop successfully started (20ms cycle)");
        RCLCPP_INFO(LOGGER, "Subscribes from: /controller_display_robot_state");
        RCLCPP_INFO(LOGGER, "Publishes to: /display_robot_state (green arm)");
        RCLCPP_INFO(LOGGER, "Publishes to: /restrict_arm_display_robot_state (blue arm)");
        RCLCPP_INFO(LOGGER, "========================================");
    }
    
private:
    /**
     * @brief Controller 显示状态回调 - 获取用户在 RViz 中的交互目标
     */
    void controller_state_callback(const moveit_msgs::msg::DisplayRobotState::ConstSharedPtr& msg) {
        // 从 DisplayRobotState 消息中提取 controller 的关节角度
        if (msg->state.joint_state.name.size() != msg->state.joint_state.position.size()) {
            RCLCPP_WARN(LOGGER, "controller_state_callback: name size (%zu) != position size (%zu)",
                        msg->state.joint_state.name.size(), msg->state.joint_state.position.size());
            return;
        }
        
        for (size_t i = 0; i < msg->state.joint_state.name.size(); ++i) {
            const auto& joint_name = msg->state.joint_state.name[i];
            const auto& pos = msg->state.joint_state.position[i];
            
            for (size_t j = 0; j < joint_names_.size(); ++j) {
                if (joint_names_[j] == joint_name && j < controller_angles_.size()) {
                    controller_angles_[j] = pos;
                    break;
                }
            }
        }
        
        static int callback_count_local = 0;
        if (++callback_count_local == 1) {
            RCLCPP_INFO(LOGGER, "[FIRST CONTROLLER UPDATE] User dragged Goal State!");
            RCLCPP_INFO(LOGGER, "  controller_angles: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                        controller_angles_[0], controller_angles_[1], controller_angles_[2],
                        controller_angles_[3], controller_angles_[4], controller_angles_[5]);
        }
        
        if (callback_count_local % 50 == 0) {
            RCLCPP_INFO(LOGGER, "[Controller State - Goal Update #%d] controller_angles: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                        callback_count_local,
                        controller_angles_[0], controller_angles_[1], controller_angles_[2],
                        controller_angles_[3], controller_angles_[4], controller_angles_[5]);
        }
        callback_count_++;
    }
    
    /**
     * @brief 控制模式请求服务处理函数
     * @note std_srvs::SetBool 的 request 有 data (bool) 字段，但我们用参数来传递模式值
     */
    void handle_mode_request(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        
        // 从 ROS 参数或环境获取新模式 (建议方案)
        // 如果没有参数，则从 request->data (0/1) 作为 UP/NO 的快速切换
        int new_mode = NO;
        
        // 优先尝试从参数获取具体的模式值 (0-4)
        std::string mode_param = "control_mode";
        if (node_->has_parameter(mode_param)) {
            new_mode = node_->get_parameter(mode_param).as_int();
        } else if (request->data) {
            // 快速切换: true -> UP, false -> NO
            new_mode = request->data ? UP : NO;
        }
        
        // 验证模式值范围
        if (new_mode < 0 || new_mode > 4) {
            response->success = false;
            response->message = "Invalid mode value (0-4), got: " + std::to_string(new_mode);
            RCLCPP_WARN(LOGGER, "Invalid mode request: %d", new_mode);
            return;
        }
        
        current_control_mode_ = new_mode;
        response->success = true;
        response->message = "Mode changed to: " + std::to_string(new_mode);
        RCLCPP_INFO(LOGGER, "Mode changed by service: %d", new_mode);
    }
    
    /**
     * @brief 控制循环回调函数
     */
    void control_callback() {
        if (!is_initialized_) {
            return;
        }
        
        // controller_angles_ 由 controller_state_callback 持续更新
        // (来自 RViz MotionPlanning 的 controller_display_robot_state)
        
        static int control_count = 0;
        if (++control_count == 1) {
            RCLCPP_INFO(LOGGER, "[Control Loop Started]");
            RCLCPP_INFO(LOGGER, "  Initial controller_angles (goal): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                        controller_angles_[0], controller_angles_[1], controller_angles_[2],
                        controller_angles_[3], controller_angles_[4], controller_angles_[5]);
            RCLCPP_INFO(LOGGER, "  Initial robo_arm_angles (actual): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                        robo_arm_angles_[0], robo_arm_angles_[1], robo_arm_angles_[2],
                        robo_arm_angles_[3], robo_arm_angles_[4], robo_arm_angles_[5]);
        }
        
        if (control_count % 100 == 0) {
            RCLCPP_DEBUG(LOGGER, "[Control Loop] Received %d controller updates", callback_count_);
        }
        
        // 计算重力补偿力矩
        std::vector<float> gravity_torques = compute_gravity_compensation_(robo_arm_angles_);
        
        // 检测模式切换
        if (current_control_mode_ != last_control_mode_) {
            RCLCPP_INFO(LOGGER, "Control mode changed: %d -> %d", 
                        last_control_mode_, current_control_mode_);
            
            switch (current_control_mode_) {
                case UP:
                    handle_mode_switch_up();
                    break;
                case TURN:
                    handle_mode_switch_turn();
                    break;
                case ROLL:
                    handle_mode_switch_roll();
                    break;
                case RESET:
                    handle_mode_switch_reset();
                    break;
                case NO:
                default:
                    break;
            }
            
            last_control_mode_ = current_control_mode_;
            mode_just_switched_ = true;
        }
        
        // 清空前一帧的标记
        visual_tools_->deleteAllMarkers();
        
        // 可视化当前机械臂状态
        visualize_robot_state(robo_arm_angles_, rviz_visual_tools::GREEN);
        visualize_restrict_state(robo_arm_angles_);
        
        // 获取末端位置用于约束可视化
        geometry_msgs::msg::Pose current_pose = compute_end_effector_pose(robo_arm_angles_);
        Eigen::Vector3d ee_position(current_pose.position.x, 
                                    current_pose.position.y, 
                                    current_pose.position.z);
        
        // 根据控制模式处理
        switch (current_control_mode_) {
            case NO:
                handle_mode_no();
                break;
            case UP:
                if (reference_pose_initialized_) {
                    Eigen::Vector3d ref_pos(reference_pose_.position.x,
                                           reference_pose_.position.y,
                                           reference_pose_.position.z);
                    visualize_direction_with_constraint_plane(ref_pos, e_hat_direction_, 
                                                             rviz_visual_tools::RED, "e_hat");
                }
                if (!mode_just_switched_) {
                    handle_mode_up();
                } else {
                    mode_just_switched_ = false;
                }
                break;
            case TURN:
                if (reference_pose_initialized_) {
                    Eigen::Vector3d turn_ref_pos(reference_pose_.position.x,
                                                 reference_pose_.position.y,
                                                 reference_pose_.position.z);
                    visualize_direction_with_constraint_plane(turn_ref_pos, normal_direction_, 
                                                             rviz_visual_tools::BLUE, "normal");
                    visualize_up_direction(ee_position);
                }
                if (!mode_just_switched_) {
                    handle_mode_turn();
                } else {
                    mode_just_switched_ = false;
                }
                break;
            case ROLL:
                if (reference_pose_initialized_) {
                    Eigen::Vector3d roll_ref_pos(reference_pose_.position.x,
                                                 reference_pose_.position.y,
                                                 reference_pose_.position.z);
                    visualize_direction_with_constraint_plane(roll_ref_pos, e_hat_direction_, 
                                                             rviz_visual_tools::RED, "e_hat");
                }
                if (!mode_just_switched_) {
                    handle_mode_roll();
                } else {
                    mode_just_switched_ = false;
                }
                break;
            case RESET:
                handle_mode_reset();
                break;
        }
        
        visual_tools_->trigger();
    }
    
    
    /**
     * @brief 处理 NO 模式 - 控制器直接控制机械臂
     */
    void handle_mode_no() {
        robo_arm_angles_ = controller_angles_;
    }
    
    /**
     * @brief 处理 UP 模式
     */
    void handle_mode_up() {
        if (controller_angles_.size() != 6 || robo_arm_angles_.size() != 6) {
            return;
        }
        if (!reference_pose_initialized_) return;

        Eigen::Vector3d reference_position(
            reference_pose_.position.x,
            reference_pose_.position.y,
            reference_pose_.position.z);

        Eigen::Quaterniond ref_orient(
            reference_pose_.orientation.w,
            reference_pose_.orientation.x,
            reference_pose_.orientation.y,
            reference_pose_.orientation.z);
        
        std::vector<double> target_angles = direction_constraint(
            robo_arm_angles_,
            controller_angles_,
            reference_position,
            controller_ee_at_mode_start_,
            e_hat_direction_,
            robot_model_,
            joint_model_group_,
            end_effector_link_,
            joint_names_,
            0.20,
            nullptr,
            &ref_orient,
            false);

        if (target_angles.empty()) {
            RCLCPP_WARN_THROTTLE(LOGGER, *node_->get_clock(), 1000, 
                "UP mode: direction_constraint failed (singular)");
            return;
        }

        robo_arm_angles_ = target_angles;
        visualize_restrict_state(target_angles);
    }
    
    /**
     * @brief 处理 TURN 模式
     */
    void handle_mode_turn() {
        if (!reference_pose_initialized_) return;

        Eigen::Vector3d reference_position(
            reference_pose_.position.x,
            reference_pose_.position.y,
            reference_pose_.position.z);

        Eigen::Quaterniond ref_orient(
            reference_pose_.orientation.w,
            reference_pose_.orientation.x,
            reference_pose_.orientation.y,
            reference_pose_.orientation.z);
            
        std::vector<double> target_angles = direction_constraint(
            robo_arm_angles_,
            controller_angles_,
            reference_position,
            controller_ee_at_mode_start_,
            normal_direction_,
            robot_model_,
            joint_model_group_,
            end_effector_link_,
            joint_names_,
            0.20,
            nullptr,
            nullptr,
            true);

        if (target_angles.empty()) {
            RCLCPP_WARN_THROTTLE(LOGGER, *node_->get_clock(), 1000, 
                "TURN mode: direction_constraint failed (singular)");
            return;
        }

        robo_arm_angles_ = target_angles;
        visualize_restrict_state(target_angles);
    }
    
    /**
     * @brief 处理 ROLL 模式
     */
    void handle_mode_roll() {
        if (!reference_pose_initialized_) return;

        Eigen::Vector3d reference_position(
            reference_pose_.position.x,
            reference_pose_.position.y,
            reference_pose_.position.z);

        Eigen::Quaterniond ref_orient(
            reference_pose_.orientation.w,
            reference_pose_.orientation.x,
            reference_pose_.orientation.y,
            reference_pose_.orientation.z);
            
        std::vector<double> target_angles = direction_constraint(
            robo_arm_angles_,
            controller_angles_,
            reference_position,
            controller_ee_at_mode_start_,
            e_hat_direction_,
            robot_model_,
            joint_model_group_,
            end_effector_link_,
            joint_names_,
            0.20,
            nullptr,
            &ref_orient,
            true);

        if (target_angles.empty()) {
            RCLCPP_WARN_THROTTLE(LOGGER, *node_->get_clock(), 1000, 
                "ROLL mode: direction_constraint failed (singular)");
            return;
        }

        robo_arm_angles_ = target_angles;
        visualize_restrict_state(target_angles);
    }
    
    /**
     * @brief 处理 RESET 模式
     */
    void handle_mode_reset() {
        if (reset_trajectory_points_.empty()) {
            robo_arm_angles_ = {0, 2.65, -0.86, 3.14, 3.14, 0};
            return;
        }

        const auto& target = reset_trajectory_points_[reset_trajectory_index_];
        robo_arm_angles_ = target;

        if (reset_trajectory_index_ + 1 < reset_trajectory_points_.size()) {
            ++reset_trajectory_index_;
        }
    }
    
    /**
     * @brief 模式切换处理 - UP 模式
     */
    void handle_mode_switch_up() {
        reference_pose_ = compute_end_effector_pose(robo_arm_angles_);
        reference_pose_initialized_ = true;
        
        Eigen::Quaterniond quat(
            reference_pose_.orientation.w,
            reference_pose_.orientation.x,
            reference_pose_.orientation.y,
            reference_pose_.orientation.z
        );
        e_hat_direction_ = quat * Eigen::Vector3d(0, 1, 0);

        moveit::core::RobotState ctrl_st(robot_model_);
        for (size_t i = 0; i < joint_names_.size(); ++i)
            ctrl_st.setJointPositions(joint_names_[i], &controller_angles_[i]);
        ctrl_st.update();
        controller_ee_at_mode_start_ = ctrl_st.getGlobalLinkTransform(end_effector_link_).translation();

        RCLCPP_INFO(LOGGER, "UP mode entered:");
        RCLCPP_INFO(LOGGER, "  e_hat direction: [%.3f, %.3f, %.3f]",
                   e_hat_direction_.x(), e_hat_direction_.y(), e_hat_direction_.z());
    }
    
    /**
     * @brief 模式切换处理 - TURN 模式
     */
    void handle_mode_switch_turn() {
        geometry_msgs::msg::Pose new_pose = compute_end_effector_pose(robo_arm_angles_);
        up_direction_ = (Eigen::Vector3d(new_pose.position.x, new_pose.position.y, new_pose.position.z) - 
                         Eigen::Vector3d(reference_pose_.position.x, reference_pose_.position.y, reference_pose_.position.z)).normalized();
        normal_direction_ = e_hat_direction_.cross(up_direction_).normalized();
        reference_pose_ = new_pose;

        moveit::core::RobotState ctrl_st(robot_model_);
        for (size_t i = 0; i < joint_names_.size(); ++i)
            ctrl_st.setJointPositions(joint_names_[i], &controller_angles_[i]);
        ctrl_st.update();
        controller_ee_at_mode_start_ = ctrl_st.getGlobalLinkTransform(end_effector_link_).translation();

        RCLCPP_INFO(LOGGER, "TURN mode entered:");
        RCLCPP_INFO(LOGGER, "  up direction: [%.3f, %.3f, %.3f]",
                   up_direction_.x(), up_direction_.y(), up_direction_.z());
    }
    
    /**
     * @brief 模式切换处理 - ROLL 模式
     */
    void handle_mode_switch_roll() {
        reference_pose_ = compute_end_effector_pose(robo_arm_angles_);

        moveit::core::RobotState ctrl_st(robot_model_);
        for (size_t i = 0; i < joint_names_.size(); ++i)
            ctrl_st.setJointPositions(joint_names_[i], &controller_angles_[i]);
        ctrl_st.update();
        controller_ee_at_mode_start_ = ctrl_st.getGlobalLinkTransform(end_effector_link_).translation();

        RCLCPP_INFO(LOGGER, "ROLL mode entered");
    }
    
    /**
     * @brief 模式切换处理 - RESET 模式
     */
    void handle_mode_switch_reset() {
        reset_trajectory_points_.clear();
        reset_trajectory_index_ = 0;

        std::vector<double> urdf_target = {0.0, 2.65, -0.86, 3.14, 3.14, 0.0};

        moveit::core::RobotState start_state(robot_model_);
        for (size_t i = 0; i < joint_names_.size(); ++i)
            start_state.setJointPositions(joint_names_[i], &robo_arm_angles_[i]);
        start_state.update();
        move_group_->setStartState(start_state);

        move_group_->setJointValueTarget(
            std::map<std::string, double>{
                {joint_names_[0], urdf_target[0]},
                {joint_names_[1], urdf_target[1]},
                {joint_names_[2], urdf_target[2]},
                {joint_names_[3], urdf_target[3]},
                {joint_names_[4], urdf_target[4]},
                {joint_names_[5], urdf_target[5]},
            });
        move_group_->setMaxVelocityScalingFactor(0.05);
        move_group_->setMaxAccelerationScalingFactor(0.05);
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            const auto& jt = plan.trajectory.joint_trajectory;
            for (const auto& pt : jt.points) {
                if (pt.positions.size() == joint_names_.size())
                    reset_trajectory_points_.push_back(
                        std::vector<double>(pt.positions.begin(), pt.positions.end()));
            }
            RCLCPP_INFO(LOGGER, "RESET mode: planned %zu waypoints to home position",
                        reset_trajectory_points_.size());
        } else {
            RCLCPP_WARN(LOGGER, "RESET mode: MoveIt planning failed");
        }
        move_group_->setStartStateToCurrentState();
        move_group_->setMaxVelocityScalingFactor(0.1);
        move_group_->setMaxAccelerationScalingFactor(0.1);
    }
    
    /**
     * @brief 可视化机械臂状态
     */
    void visualize_robot_state(const std::vector<double>& joint_angles, const rviz_visual_tools::Colors& color) {
        if (!is_initialized_ || !robot_model_) {
            return;
        }
        try {
            moveit::core::RobotState robot_state(robot_model_);
            for (size_t i = 0; i < joint_names_.size() && i < joint_angles.size(); ++i) {
                robot_state.setJointPositions(joint_names_[i], &joint_angles[i]);
            }
            robot_state.update();
            visual_tools_->publishRobotState(robot_state, color);
        } catch (const std::exception& e) {
            static int error_count = 0;
            if (++error_count <= 3) {
                RCLCPP_WARN(LOGGER, "visualize_robot_state error: %s (planning_frame=%s)", 
                           e.what(), planning_frame_.c_str());
            }
        }
    }
    
    /**
     * @brief 可视化受限状态 (蓝色)
     */
    void visualize_restrict_state(const std::vector<double>& joint_angles) {
        if (!is_initialized_ || !robot_model_ || joint_angles.size() != joint_names_.size()) {
            return;
        }
        moveit::core::RobotState robot_state(robot_model_);
        for (size_t i = 0; i < joint_names_.size() && i < joint_angles.size(); ++i) {
            robot_state.setJointPositions(joint_names_[i], &joint_angles[i]);
        }
        robot_state.update();

        moveit_msgs::msg::DisplayRobotState display_msg;
        moveit::core::robotStateToRobotStateMsg(robot_state, display_msg.state);

        const auto& link_names = robot_model_->getLinkModelNames();
        for (const auto& link_name : link_names) {
            moveit_msgs::msg::ObjectColor color;
            color.id = link_name;
            color.color.r = 0.0f;
            color.color.g = 0.4f;
            color.color.b = 1.0f;
            color.color.a = 1.0f;
            display_msg.highlight_links.push_back(color);
        }
        restrict_arm_state_pub_->publish(display_msg);
    }
    
    /**
     * @brief 可视化约束方向及其平面
     */
    void visualize_direction_with_constraint_plane(
        const Eigen::Vector3d& position,
        const Eigen::Vector3d& direction,
        const rviz_visual_tools::Colors& color = rviz_visual_tools::RED,
        const std::string& label = "constraint") {
        
        Eigen::Vector3d arrow_end = position + direction * 0.2;
        visual_tools_->publishArrow(
            visual_tools_->convertPoint(position),
            visual_tools_->convertPoint(arrow_end),
            color, rviz_visual_tools::MEDIUM);
        
        geometry_msgs::msg::Pose text_pose;
        text_pose.position = visual_tools_->convertPoint(arrow_end);
        text_pose.orientation.w = 1.0;
        visual_tools_->publishText(text_pose, label, color, rviz_visual_tools::SMALL, false);
        
        visualize_perpendicular_plane(position, direction);
    }
    
    /**
     * @brief 可视化与方向垂直的平面
     */
    void visualize_perpendicular_plane(const Eigen::Vector3d& position, const Eigen::Vector3d& direction) {
        Eigen::Vector3d dir_normalized = direction.normalized();
        
        Eigen::Vector3d arbitrary;
        if (std::abs(dir_normalized.x()) < 0.9) {
            arbitrary = Eigen::Vector3d(1, 0, 0);
        } else {
            arbitrary = Eigen::Vector3d(0, 1, 0);
        }
        
        Eigen::Vector3d u = dir_normalized.cross(arbitrary).normalized();
        Eigen::Vector3d v = dir_normalized.cross(u).normalized();
        
        visualization_msgs::msg::Marker plane_marker;
        plane_marker.header.frame_id = planning_frame_;
        plane_marker.header.stamp = node_->now();
        plane_marker.ns = "constraint_plane";
        plane_marker.id = 1;
        plane_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        plane_marker.action = visualization_msgs::msg::Marker::ADD;
        plane_marker.scale.x = 0.003;
        plane_marker.color.r = 1.0;
        plane_marker.color.g = 0.5;
        plane_marker.color.b = 0.0;
        plane_marker.color.a = 0.6;
        plane_marker.lifetime = rclcpp::Duration(0, 0);
        
        double radius = 0.08;
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
    }
    
    /**
     * @brief 可视化向上方向
     */
    void visualize_up_direction(const Eigen::Vector3d& position) {
        Eigen::Vector3d up_end = position + up_direction_ * 0.2;
        visual_tools_->publishArrow(
            visual_tools_->convertPoint(position),
            visual_tools_->convertPoint(up_end),
            rviz_visual_tools::GREEN, rviz_visual_tools::MEDIUM);
        
        geometry_msgs::msg::Pose text_pose;
        text_pose.position = visual_tools_->convertPoint(up_end);
        text_pose.orientation.w = 1.0;
        visual_tools_->publishText(text_pose, "up", rviz_visual_tools::GREEN, rviz_visual_tools::SMALL, false);
    }
    
    /**
     * @brief 计算末端执行器位姿
     */
    geometry_msgs::msg::Pose compute_end_effector_pose(const std::vector<double>& joint_angles) {
        moveit::core::RobotState robot_state(robot_model_);
        for (size_t i = 0; i < joint_names_.size() && i < joint_angles.size(); ++i) {
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
    
    // 成员变量
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    const moveit::core::JointModelGroup* joint_model_group_;
    moveit::core::RobotModelConstPtr robot_model_;
    std::string end_effector_link_;
    std::string planning_frame_;
    std::vector<std::string> joint_names_;
    
    rclcpp::Subscription<moveit_msgs::msg::DisplayRobotState>::SharedPtr controller_state_sub_;
    int callback_count_ = 0;
    
    bool mode_just_switched_ = false;
    std::function<std::vector<float>(const std::vector<double>&)> compute_gravity_compensation_;
    
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mode_service_;
    
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_pub_;
    rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr restrict_arm_state_pub_;

    Eigen::Vector3d e_hat_direction_;
    Eigen::Vector3d up_direction_;
    Eigen::Vector3d normal_direction_;

    geometry_msgs::msg::Pose reference_pose_;
    bool reference_pose_initialized_ = false;
    Eigen::Vector3d controller_ee_at_mode_start_{0, 0, 0};
    

    uint8_t last_control_mode_;
    uint8_t current_control_mode_;
    
    geometry_msgs::msg::Pose reset_pose_;
    
    bool is_initialized_ = false;
    
    std::vector<double> controller_angles_;
    std::vector<double> robo_arm_angles_;
    
    std::vector<std::vector<double>> reset_trajectory_points_;
    size_t reset_trajectory_index_ = 0;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("rviz_simulation", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    
    // 在后台线程中启动 executor
    std::atomic<bool> should_stop{false};
    std::thread executor_thread([&executor, &should_stop]() {
        while (!should_stop && rclcpp::ok()) {
            executor.spin_some(std::chrono::milliseconds(10));
        }
    });

    // 等待 MoveGroup 初始化
    RCLCPP_INFO(rclcpp::get_logger("rviz_simulation"), "Waiting for MoveGroup to initialize...");
    rclcpp::sleep_for(std::chrono::seconds(5));
    
    // 创建控制器
    RCLCPP_INFO(rclcpp::get_logger("rviz_simulation"), "Creating RobotSimulationController...");
    auto controller = std::make_shared<RobotSimulationController>(node);
    controller->start_control_loop();

    RCLCPP_INFO(rclcpp::get_logger("rviz_simulation"), "System ready");
    
    // 等待关闭信号
    while (rclcpp::ok()) {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    
    RCLCPP_INFO(rclcpp::get_logger("rviz_simulation"), "Shutting down...");
    should_stop = true;
    
    if (executor_thread.joinable()) {
        executor_thread.join();
    }
    
    rclcpp::shutdown();
    return 0;
}
