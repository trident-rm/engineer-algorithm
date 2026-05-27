/**
 * @file complete_control.cpp
 * @brief 完整的控制状态机实现，包含多种控制模式和串口通信
 */

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tools/usb_data.hpp>

//全局常量
static const rclcpp::Logger LOGGER = rclcpp::get_logger("flow_with_pyroki");
static const std::string PLANNING_GROUP = "arm";
static const double STEP = 0.01;
static const std::vector<std::string> JOINT_NAMES = {
    "joint1","joint2","joint3","joint4","joint5","joint6"};
static const std::vector<double> energy_unit_1 = {
    0.0, -1.57, 1.57, 0.0, 0.0, 0.0
};

static const geometry_msgs::msg::Pose energy_unit_1_pose = []() {
    geometry_msgs::msg::Pose p;
    p.position.x = 15.0;
    p.position.y = 0.0;
    p.position.z = 512.5;
    p.orientation.x = 0.7071;
    p.orientation.y = 0;
    p.orientation.z = 0.7071;
    p.orientation.w = 0;
    return p;
}();

static const std::vector<double> energy_unit_2 = {
    0.0, -1.57, 1.57, 0.0, 0.0, 0.0
};

static const geometry_msgs::msg::Pose energy_unit_2_pose = []() {
    geometry_msgs::msg::Pose p;
    p.position.x = 0.3;
    p.position.y = 0.0;
    p.position.z = 0.2;
    p.orientation.w = 1.0;
    return p;
}();
static const std::vector<double> energy_unit_3 = {
    0.0, -1.57, 1.57, 0.0, 0.0, 0.0
};

static const geometry_msgs::msg::Pose energy_unit_3_pose = []() {
    geometry_msgs::msg::Pose p;
    p.position.x = 0.3;
    p.position.y = 0.0;
    p.position.z = 0.2;
    p.orientation.w = 1.0;
    return p;
}();

static const std::vector<double> energy_unit_4 = {
    0.0, -1.57, 1.57, 0.0, 0.0, 0.0
};

static const geometry_msgs::msg::Pose energy_unit_4_pose = []() {
    geometry_msgs::msg::Pose p;
    p.position.x = 0.3;
    p.position.y = 0.0;
    p.position.z = 0.2;
    p.orientation.w = 1.0;
    return p;
}();

static const std::vector<double> energy_unit_5 = {
    0.0, -1.57, 1.57, 0.0, 0.0, 0.0
};

static const geometry_msgs::msg::Pose energy_unit_5_pose = []() {
    geometry_msgs::msg::Pose p;
    p.position.x = 0.3;
    p.position.y = 0.0;
    p.position.z = 0.2;
    p.orientation.w = 1.0;
    return p;
}();

static const std::vector<double> energy_unit_6 = {
    0.0, -1.57, 1.57, 0.0, 0.0, 0.0
};

static const geometry_msgs::msg::Pose energy_unit_6_pose = []() {
    geometry_msgs::msg::Pose p;
    p.position.x = 0.3;
    p.position.y = 0.0;
    p.position.z = 0.2;
    p.orientation.w = 1.0;
    return p;
}();

//全局变量
static std::mutex g_js_mutex;

//关节角度的全局变量
static std::vector<double> g_arm_state(6, 0.0);           //目标机械臂状态
//static std::vector<double> g_last_state(6, 0.0);       // g_arm_state上一刻的状态
//static std::vector<double> g_real_arm_state(6, 0.0);      // 实际状态 (在仿真里和/joint_states一样, 但在真实环境里是读取串口的数据)
static std::vector<double> g_ctrl_js(6, 0.0); //从/controller_joint_states接收的控制器状态
static std::vector<float> g_zero_currents(6, 0.0f); //发送到硬件的零电流

//方向向量的全局变量
static std::vector<double> p_vector = {0, 0, 1}; 
static std::vector<double> q_vector = {0, 1, 0};
static std::vector<double> up_vector = {1, 0, 0};
static std::vector<double> base_x = {1, 0, 0};
static std::vector<double> base_y = {0, 1, 0};
static std::vector<double> base_z = {0, 0, 1};
static std::vector<double> ee_positive = {0, 0, 0}; //末端link当前z轴的方向（正）

//位置向量的全局变量 
static std::vector<double> up_start = {0.0,0.0,0.0}; //upRestrain刚开始的时候末端位置
static std::vector<double> up_end = {0.0,0.0,0.0}; //upRestrain结束时末端位置



// 键盘输入缓存
static std::mutex g_key_mutex;
static char g_last_key = 0;

// pyroki 状态接收标志
static std::atomic<bool> g_got_pyroki_state(false);

enum class ControlMode {
    Following,  // G: 绝对跟随pyroki（跟随控制器）
    RelativeF,  // H: 相对跟随控制器
    UpIncre,    // W: base frame Z增量模式
    DownIncre,  // S: base frame Z减量模式
    RightIncre, // D: base frame Y减量模式
    LeftIncre,  // A: base frame Y增量模式
    ForwardIncre, // E: base frame X增量模式
    BackwardIncre, // Q: base frame X减量模式
    InIncre,    // I: 末端沿当前z轴正向增量模式
    OutIncre,   // K: 末端沿当前z轴负向增
    UpRestrain, // R: 用雅可比矩阵约束末端沿q_vector方向不动（将控制器在这个方向上的位移消除掉然后进行IK）
    Turn, // T: 用execute_turn规划一个绕P轴旋转的动作



    ENERGY_UNIT_1, // 1: 去取能量单元1
    ENERGY_UNIT_2, // 2: 去取能量单元2
    ENERGY_UNIT_3, // 3: 去取能量单元3
    ENERGY_UNIT_4, // 4: 去取能量单元4
    ENERGY_UNIT_5, // 5: 去取能量单元5      
    ENERGY_UNIT_6  // 6: 去取能量单元6
};

/**
 * @brief 状态机: 管理多种控制模式
 */
class ControlStateMachine {
public:
    using MoveGroupPtr = std::shared_ptr<moveit::planning_interface::MoveGroupInterface>;
    using JMGPtr = const moveit::core::JointModelGroup*;
    using VTPtr = std::shared_ptr<moveit_visual_tools::MoveItVisualTools>;
    using JointStatePublisherPtr = rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr;
    using SerialRxNodePtr = std::shared_ptr<SerialRxNode>;
    
    ControlMode mode_;
    MoveGroupPtr mg_;
    JMGPtr jmg_;
    VTPtr vt_;
    rclcpp::Node::SharedPtr node_;
    JointStatePublisherPtr js_pub_;
    SerialRxNodePtr serial_node_;
    
    ControlStateMachine(rclcpp::Node::SharedPtr node, MoveGroupPtr mg, JMGPtr jmg, VTPtr vt, JointStatePublisherPtr js_pub, SerialRxNodePtr serial_node)
        : mode_(ControlMode::Following), mg_(mg), jmg_(jmg), vt_(vt), node_(node), js_pub_(js_pub), serial_node_(serial_node) {}
    
    void start() {
        RCLCPP_INFO(LOGGER, "Started");
        enter_following();
        // activate planning monitor
        mg_->getCurrentState(5.0);

    }
    
    void switch_to(ControlMode new_mode) {
        //if (new_mode == mode_) return;
        mode_ = new_mode;
        
        switch (mode_) {
            case ControlMode::Following:
                enter_following();
                break;
            case ControlMode::UpIncre:
                plan_incremental_motion(base_z);  // +Z
                break;
            case ControlMode::DownIncre:
                plan_incremental_motion({-base_z[0], -base_z[1], -base_z[2]});  // -Z
                break;
            case ControlMode::RightIncre:
                plan_incremental_motion({-base_y[0], -base_y[1], -base_y[2]});  // -Y
                break;
            case ControlMode::LeftIncre:
                plan_incremental_motion(base_y);  // +Y
                break;
            case ControlMode::ForwardIncre:
                plan_incremental_motion(base_x);  // +X
                break;
            case ControlMode::BackwardIncre:
                plan_incremental_motion({-base_x[0], -base_x[1], -base_x[2]});  // -X
                break;
            case ControlMode::InIncre: {
                auto current_state = mg_->getCurrentState(5.0);
                const Eigen::Isometry3d& tf = current_state->getGlobalLinkTransform(mg_->getEndEffectorLink());
                Eigen::Matrix3d rotation = tf.rotation();
                Eigen::Vector3d ee_z = rotation.col(2); 
                plan_incremental_motion({ee_z(0), ee_z(1), ee_z(2)});
                break;
            }
            case ControlMode::OutIncre: {
                auto current_state = mg_->getCurrentState(5.0);
                const Eigen::Isometry3d& tf = current_state->getGlobalLinkTransform(mg_->getEndEffectorLink());
                Eigen::Matrix3d rotation = tf.rotation();
                Eigen::Vector3d ee_z = rotation.col(2); 
                plan_incremental_motion({-ee_z(0), -ee_z(1), -ee_z(2)}); //末端z反方向
                break;
            }
            case ControlMode::UpRestrain: {
                auto current_state = mg_->getCurrentState(5.0);
                const Eigen::Isometry3d& tf = current_state->getGlobalLinkTransform(mg_->getEndEffectorLink());
                
                // 记录末端X轴朝向
                Eigen::Matrix3d rotation = tf.rotation();
                Eigen::Vector3d ee_x = rotation.col(0);  // 末端X轴
                q_vector = {ee_x(0), ee_x(1), ee_x(2)};
                
                // 记录末端位置
                up_start = {tf.translation().x(), tf.translation().y(), tf.translation().z()};
                break;
            }
            case ControlMode::Turn: {
                auto current_state = mg_->getCurrentState(5.0);
                const Eigen::Isometry3d& tf = current_state->getGlobalLinkTransform(mg_->getEndEffectorLink());
                
                // 记录末端位置
                up_end = {tf.translation().x(), tf.translation().y(), tf.translation().z()};
                
                // 计算up_vector = up_end - up_start
                up_vector[0] = up_end[0] - up_start[0];
                up_vector[1] = up_end[1] - up_start[1];
                up_vector[2] = up_end[2] - up_start[2];
                
                //计算p_vector = q_vector x up_vector
                p_vector[0] = q_vector[1] * up_vector[2] - q_vector[2] * up_vector[1];  
                p_vector[1] = q_vector[2] * up_vector[0] - q_vector[0] * up_vector[2];  
                p_vector[2] = q_vector[0] * up_vector[1] - q_vector[1] * up_vector[0];


                plan_turn();
                break;
            }
            case ControlMode::ENERGY_UNIT_1: get_energy_unit(1); break;
            case ControlMode::ENERGY_UNIT_2: get_energy_unit(2); break;
            case ControlMode::ENERGY_UNIT_3: get_energy_unit(3); break;
            case ControlMode::ENERGY_UNIT_4: get_energy_unit(4); break;
            case ControlMode::ENERGY_UNIT_5: get_energy_unit(5); break;
            case ControlMode::ENERGY_UNIT_6: get_energy_unit(6); break;
            default:
                break;
        }
    }
    
    void update() {
        if (mode_ == ControlMode::Following) {
            // Following 模式：持续同步 g_ctrl_js 到 g_arm_state
            std::lock_guard<std::mutex> lk(g_js_mutex);
            g_arm_state = g_ctrl_js;
        } else if (mode_ == ControlMode::UpRestrain) {
            // UpRestrain 模式：持续约束末端在 q_vector 方向不动
            const double dt = 0.02;  // 50Hz update
            std::vector<double> constrained_angles = uprestrain_constraint(dt);
            if (!constrained_angles.empty()) {
                std::lock_guard<std::mutex> lk(g_js_mutex);
                g_arm_state = constrained_angles;
            }
        }
    }
    
private:
    
    void enter_following() {
        std::vector<double> target_joints;
        //std::vector<double> current_joints;
        {
            std::lock_guard<std::mutex> lk(g_js_mutex);
            //current_joints = g_arm_state;
            target_joints = g_ctrl_js;
        }
        
        double max_diff = 0.0;
        auto current_joints = mg_->getCurrentJointValues();
        for (size_t i = 0; i < current_joints.size(); ++i) {
            double diff = std::abs(current_joints[i] - target_joints[i]);
            max_diff = std::max(max_diff, diff);
            RCLCPP_INFO(LOGGER, "Joint %zu: current=%.4f, target=%.4f, diff=%.4f", 
                        i + 1, current_joints[i], target_joints[i], diff);
        }
        
        const double DIFF_THRESHOLD = 0.1;
        if (max_diff > DIFF_THRESHOLD) {
            
            auto state = std::make_shared<moveit::core::RobotState>(mg_->getRobotModel());
            state->setToDefaultValues();
            state->setJointGroupPositions(jmg_, current_joints);
            state->enforceBounds();
            state->update();
            mg_->setStartState(*state);
            
            mg_->setJointValueTarget(target_joints);
            
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto rc = mg_->plan(plan);
            if (rc != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_WARN(LOGGER, "Failed to plan smooth transition (code=%d)", static_cast<int>(rc.val));      // 降级到直接设置目标，而不规划
                {
                    std::lock_guard<std::mutex> lk(g_js_mutex);
                    g_arm_state = target_joints;
                }
                return;
            }

            vt_->deleteAllMarkers();
            vt_->publishTrajectoryLine(plan.trajectory, jmg_->getLinkModel(mg_->getEndEffectorLink()), jmg_);
            vt_->trigger();
            
            write_trajectory(plan.trajectory.joint_trajectory);
        }
        
    }

    void plan_turn() {
        const double radius = 0.1;
        const int num_points = 40;

        // 规划半圆轨迹：绕着p_vector转，往up_vector方向转，圆心在up_end - q_vector*radius
        std::vector<geometry_msgs::msg::Pose> waypoints;
        const double angle_range = M_PI / 2;

        Eigen::Vector3d start_pos(up_end[0], up_end[1], up_end[2]);
        Eigen::Vector3d world_center = start_pos - Eigen::Vector3d(q_vector[0], q_vector[1], q_vector[2]) * radius;
        Eigen::Vector3d radius_vec0 = start_pos - world_center;

        const Eigen::Quaterniond start_quat(
            mg_->getCurrentPose().pose.orientation.w,
            mg_->getCurrentPose().pose.orientation.x,
            mg_->getCurrentPose().pose.orientation.y,
            mg_->getCurrentPose().pose.orientation.z);
        Eigen::Matrix3d start_rot = start_quat.toRotationMatrix();
        Eigen::Vector3d start_x = start_rot.col(0);

        waypoints.reserve(num_points + 1);
        waypoints.push_back(mg_->getCurrentPose().pose);

        for (int i = 1; i <= num_points; ++i) {
            double angle = angle_range * static_cast<double>(i) / static_cast<double>(num_points);
            Eigen::AngleAxisd aa(angle, Eigen::Vector3d(p_vector[0], p_vector[1], p_vector[2]));
            Eigen::Vector3d world_point = world_center + aa * radius_vec0;

            geometry_msgs::msg::Pose target_pose;
            target_pose.position.x = world_point.x();
            target_pose.position.y = world_point.y();
            target_pose.position.z = world_point.z();

            // 姿态半圆：末端（gripper）x 轴绕 p_vector 旋转，y 轴保持 up_vector 方向
            Eigen::Vector3d x_target = (aa * start_x).normalized();
            Eigen::Vector3d y_target(up_vector[0], up_vector[1], up_vector[2]);
            Eigen::Vector3d z_target = x_target.cross(y_target).normalized();

            Eigen::Matrix3d target_rot;
            target_rot.col(0) = x_target;
            target_rot.col(1) = y_target;
            target_rot.col(2) = z_target;

            Eigen::Quaterniond target_quat(target_rot);
            target_quat.normalize();
            target_pose.orientation.x = target_quat.x();
            target_pose.orientation.y = target_quat.y();
            target_pose.orientation.z = target_quat.z();
            target_pose.orientation.w = target_quat.w();
            waypoints.push_back(target_pose);
        }
        
        // 计算笛卡尔路径
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.002;
        double fraction = 0.0;
        int maxtries = 20;
        int attempts = 0;
        
        while (fraction < 0.95 && attempts < maxtries) {
            fraction = mg_->computeCartesianPath(waypoints, eef_step, trajectory);
            attempts++;
            if (attempts % 10 == 0) {
                RCLCPP_WARN(LOGGER, "Cartesian path planning: attempt %d, fraction %.2f", attempts, fraction);
            }
        }

        if (trajectory.joint_trajectory.points.empty()) {
            RCLCPP_WARN(LOGGER, "Half-circle trajectory planning failed");
            return;
        }

        vt_->deleteAllMarkers();
        vt_->publishTrajectoryLine(trajectory, jmg_->getLinkModel(mg_->getEndEffectorLink()), jmg_);
        vt_->trigger();

        write_trajectory(trajectory.joint_trajectory);
    }
    

    void write_trajectory(const trajectory_msgs::msg::JointTrajectory& jt) {
        if (jt.points.empty()) return;
        // 将轨迹点通过串口发送到硬件
        for (const auto& pt : jt.points) {
            if (pt.positions.size() >= 6 && serial_node_) {
                std::vector<float> angles(pt.positions.begin(), pt.positions.end());
                // 发送目标角度和零电流到硬件
                serial_node_->send_roboarm_with_controller(g_zero_currents, angles);
                RCLCPP_DEBUG(LOGGER, "Sent trajectory point: j1=%.4f j2=%.4f j3=%.4f j4=%.4f j5=%.4f j6=%.4f",
                           angles[0], angles[1], angles[2], angles[3], angles[4], angles[5]);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        
        // 轨迹执行完成后，把最后一个点设置为新的目标状态
        if (!jt.points.empty() && jt.points.back().positions.size() >= 6) {
            std::lock_guard<std::mutex> lk(g_js_mutex);
            for (size_t i = 0; i < 6 && i < jt.points.back().positions.size(); ++i) {
                g_arm_state[i] = jt.points.back().positions[i];
            }
            RCLCPP_INFO(LOGGER, "Updated g_arm_state to final trajectory point: j1=%.4f j2=%.4f j3=%.4f j4=%.4f j5=%.4f j6=%.4f",
                       g_arm_state[0], g_arm_state[1], g_arm_state[2], 
                       g_arm_state[3], g_arm_state[4], g_arm_state[5]);
        }
    }

private:
    void get_energy_unit(int unit_id) {        
        std::vector<double> target_pose;
        switch (unit_id) {
            case 1: target_pose = energy_unit_1; break;
            case 2: target_pose = energy_unit_2; break;
            case 3: target_pose = energy_unit_3; break;
            case 4: target_pose = energy_unit_4; break;
            case 5: target_pose = energy_unit_5; break;
            case 6: target_pose = energy_unit_6; break;
        }
        
        RCLCPP_INFO(LOGGER, "Planning to energy_unit_%d", unit_id);
        
        //设置规划起点和终点
        mg_->setStartStateToCurrentState();
        mg_->setJointValueTarget(target_pose);
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto rc = mg_->plan(plan);
        if (rc != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(LOGGER, "Plan to energy_unit_%d failed (code=%d)", unit_id, static_cast<int>(rc.val));
            return;
        }
        
        vt_->deleteAllMarkers();
        vt_->publishTrajectoryLine(plan.trajectory, jmg_->getLinkModel(mg_->getEndEffectorLink()), jmg_);
        vt_->trigger();
        
        // 执行轨迹
        write_trajectory(plan.trajectory.joint_trajectory);
    }
    
    void plan_incremental_motion(const std::vector<double>& direction, double step_size = 0.01) {
        
        mg_->setStartStateToCurrentState();
        
        auto current_state = mg_->getCurrentState(5.0);
        const std::string ee = mg_->getEndEffectorLink();
        const Eigen::Isometry3d& tf = current_state->getGlobalLinkTransform(ee);
        
        // 计算新的目标位置
        geometry_msgs::msg::Pose target;
        target.position.x = tf.translation().x() + direction[0] * step_size;
        target.position.y = tf.translation().y() + direction[1] * step_size;
        target.position.z = tf.translation().z() + direction[2] * step_size;
        
        // 保持原来的姿态
        Eigen::Quaterniond q(tf.rotation());
        target.orientation.x = q.x();
        target.orientation.y = q.y();
        target.orientation.z = q.z();
        target.orientation.w = q.w();
        
        // 使用 Cartesian 路径规划
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(mg_->getCurrentPose().pose);  // 起点
        waypoints.push_back(target);                        // 终点
        
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.005;  // 5mm 的末端步长
        double fraction = 0.0;
        
        fraction = mg_->computeCartesianPath(waypoints, eef_step, trajectory);
        if (fraction < 0.95) {
            RCLCPP_WARN(LOGGER, "Cartesian path planning for incremental motion: fraction %.2f", fraction);
            return;
        }
        
        vt_->deleteAllMarkers();
        vt_->publishTrajectoryLine(trajectory, jmg_->getLinkModel(ee), jmg_);
        vt_->trigger();
        RCLCPP_INFO(LOGGER, "Planned incremental motion with %zu points (fraction: %.2f)", 
                    trajectory.joint_trajectory.points.size(), fraction);
        
        write_trajectory(trajectory.joint_trajectory);
    }
    
    /**
     * @brief UpRestrain 模式：消除控制器在 q_vector 方向的偏移
     */
    std::vector<double> uprestrain_constraint(double dt)
    {
        std::vector<double> ctrl_js_copy, arm_state_copy;
        {
            std::lock_guard<std::mutex> lk(g_js_mutex);
            ctrl_js_copy = g_ctrl_js;
            arm_state_copy = g_arm_state;
        }
        
        moveit::core::RobotState controller_state(mg_->getRobotModel());
        for (size_t i = 0; i < JOINT_NAMES.size(); ++i)
            controller_state.setJointPositions(JOINT_NAMES[i], &ctrl_js_copy[i]);
        controller_state.update();
        Eigen::Vector3d controller_ee =
            controller_state.getGlobalLinkTransform(mg_->getEndEffectorLink()).translation();

        auto arm_state = mg_->getCurrentState(5.0);
        Eigen::Vector3d current_ee =
            arm_state->getGlobalLinkTransform(mg_->getEndEffectorLink()).translation();
        
        Eigen::Vector3d recorded_pos(up_start[0], up_start[1], up_start[2]);

        Eigen::Vector3d constrain_dir_normalized = 
            Eigen::Vector3d(q_vector[0], q_vector[1], q_vector[2]).normalized();
        
        Eigen::Vector3d controller_displacement = controller_ee - recorded_pos;
        double constrain_component = controller_displacement.dot(constrain_dir_normalized);
        
        Eigen::Vector3d constrained_target = recorded_pos + 
            (controller_displacement - constrain_component * constrain_dir_normalized);

        Eigen::Vector3d velocity = constrained_target - current_ee;
        
        double drift_along_constraint = (current_ee - recorded_pos).dot(constrain_dir_normalized);
        const double DRIFT_CORRECTION_GAIN = 2.0;
        velocity -= DRIFT_CORRECTION_GAIN * drift_along_constraint * constrain_dir_normalized;

        double position_error = velocity.norm();
        const double DEAD_ZONE = 0.0005;  // m
        if (position_error < DEAD_ZONE) {
            return arm_state_copy;
        }

        const double MAX_SPEED = 0.45;  // m/s
        if (position_error > MAX_SPEED) {
            velocity *= (MAX_SPEED / position_error);
        }

        Eigen::MatrixXd full_jacobian;
        Eigen::Vector3d ref_pt(0.0, 0.0, 0.0);
        if (!arm_state->getJacobian(
                jmg_,
                arm_state->getLinkModel(mg_->getEndEffectorLink()),
                ref_pt, full_jacobian)) {
            RCLCPP_ERROR(LOGGER, "uprestrain_constraint: failed to compute Jacobian");
            return {};
        }

        Eigen::MatrixXd J = full_jacobian.block(0, 0, 3, full_jacobian.cols());
        Eigen::VectorXd vel_cmd = velocity;

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

        int jrows = static_cast<int>(J.rows());
        Eigen::MatrixXd JJt = J * J.transpose()
                            + (damping * damping) * Eigen::MatrixXd::Identity(jrows, jrows);
        Eigen::MatrixXd J_pinv = J.transpose() * JJt.inverse();
        Eigen::VectorXd delta_q = J_pinv * vel_cmd * dt;

        const double MAX_JOINT_DELTA = 0.1;  // rad
        double max_dq = delta_q.cwiseAbs().maxCoeff();
        if (max_dq > MAX_JOINT_DELTA) {
            delta_q *= (MAX_JOINT_DELTA / max_dq);
        }

        const double JOINT_LIMIT_MARGIN = 0.05;  // rad
        const auto& joint_models = jmg_->getActiveJointModels();
        double limit_scale = 1.0;
        for (size_t i = 0; i < joint_models.size(); ++i) {
            const auto& bounds = joint_models[i]->getVariableBounds();
            if (bounds.empty()) continue;
            double target = arm_state_copy[i] + delta_q(static_cast<Eigen::Index>(i));
            double lo = bounds[0].min_position_ + JOINT_LIMIT_MARGIN;
            double hi = bounds[0].max_position_ - JOINT_LIMIT_MARGIN;
            if (target < lo && delta_q(static_cast<Eigen::Index>(i)) < 0) {
                double allowed = arm_state_copy[i] - lo;
                if (allowed > 0 && std::abs(delta_q(static_cast<Eigen::Index>(i))) > 1e-6)
                    limit_scale = std::min(limit_scale,
                        allowed / std::abs(delta_q(static_cast<Eigen::Index>(i))));
                else
                    limit_scale = 0.0;
            } else if (target > hi && delta_q(static_cast<Eigen::Index>(i)) > 0) {
                double allowed = hi - arm_state_copy[i];
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

        std::vector<double> result(arm_state_copy.size());
        for (size_t i = 0; i < arm_state_copy.size(); ++i) {
            result[i] = arm_state_copy[i] + delta_q(static_cast<Eigen::Index>(i));
        }

        RCLCPP_DEBUG(LOGGER,
            "uprestrain_constraint: pos_err=%.4fm cond=%.1f λ=%.4f",
            position_error, condition_number, damping);

        return result;
    }

};


// 全局状态机实例
std::unique_ptr<ControlStateMachine> g_state_machine = nullptr;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions opts;
    opts.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("flow_with_pyroki", opts);

    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    exec.add_node(node);
    std::thread spin_thread([&exec]() { exec.spin(); });

    auto io_cb_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto control_cb_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // 创建 SerialRxNode 用于与硬件通信（会自动发布 /joint_states）
    auto serial_node = std::make_shared<SerialRxNode>();
    exec.add_node(serial_node);

    rclcpp::SubscriptionOptions ctrl_sub_opts;
    ctrl_sub_opts.callback_group = io_cb_group;

    // 订阅控制器反馈（用于 Following 模式和其他控制）
    auto ctrl_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/controller_joint_states", 10,
        [](const sensor_msgs::msg::JointState::SharedPtr msg) {
            if (msg->position.size() >= 6) {
                std::lock_guard<std::mutex> lk(g_js_mutex);
                for (size_t i = 0; i < 6; ++i) {
                    g_ctrl_js[i] = msg->position[i];
                }
                // Following 模式下直接同步到目标状态
                if (g_state_machine && g_state_machine->mode_ == ControlMode::Following) {
                    g_arm_state = g_ctrl_js;
                }
            }
        },
        ctrl_sub_opts);


    // 等待硬件节点初始化并发布 /joint_states
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 初始化MoveIt
    auto mg = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, PLANNING_GROUP);
    mg->setEndEffectorLink("gripper");
    // const auto* jmg = mg->getRobotModel()->getJointModelGroup(PLANNING_GROUP);
    const moveit::core::JointModelGroup* jmg =
        mg->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    auto vt = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
        node, "base_link", "flow_with_pyroki", mg->getRobotModel());
    vt->deleteAllMarkers();

    // 初始化状态机
    g_state_machine = std::make_unique<ControlStateMachine>(node, mg, jmg, vt, nullptr, serial_node);
    g_state_machine->start();
    
    RCLCPP_INFO(LOGGER, "Ready. Keys: G(Following) W/S/A/D/E/Q(Base) I/K(EE) R(UpRestrain) T(Turn) 1-6(Energy) X(Quit)");

    // 键盘输入订阅
    rclcpp::SubscriptionOptions key_sub_opts;
    key_sub_opts.callback_group = control_cb_group;

    auto key_sub = node->create_subscription<std_msgs::msg::String>(
        "/key_input", 10,
        [](const std_msgs::msg::String::SharedPtr msg) {
            if (msg->data.empty()) return;
            std::lock_guard<std::mutex> lk(g_key_mutex);
            g_last_key = msg->data[0];
        },
        key_sub_opts);

    // 定期发送目标状态到硬件串口
    auto send_timer = node->create_wall_timer(
        std::chrono::milliseconds(20),
        [serial_node]() {
            if (serial_node) {
                std::vector<float> target_angles(6);
                {
                    std::lock_guard<std::mutex> lk(g_js_mutex);
                    for (size_t i = 0; i < 6 && i < g_arm_state.size(); ++i) {
                        target_angles[i] = static_cast<float>(g_arm_state[i]);
                    }
                }
                serial_node->send_roboarm_with_controller(g_zero_currents, target_angles);
                RCLCPP_DEBUG(LOGGER, "Sent target to hardware: j1=%.4f j2=%.4f j3=%.4f j4=%.4f j5=%.4f j6=%.4f",
                           target_angles[0], target_angles[1], target_angles[2], 
                           target_angles[3], target_angles[4], target_angles[5]);
            }
        },
        io_cb_group);

    // 键盘处理和状态机更新
    auto key_timer = node->create_wall_timer(
        std::chrono::milliseconds(20),
        [node]() {
            if (!g_state_machine) {
                return;
            }

            char key = 0;
            {
                std::lock_guard<std::mutex> lk(g_key_mutex);
                key = g_last_key;
                g_last_key = 0;
            }
            
            if (key == 0) {
                if (g_state_machine->mode_ == ControlMode::Following ||
                    g_state_machine->mode_ == ControlMode::UpRestrain) {
                    g_state_machine->update();
                }
            }

            const ControlMode prev_mode = g_state_machine->mode_;
            RCLCPP_INFO(LOGGER, "Current State Mode: %d, Received Key: %c", static_cast<int>(g_state_machine->mode_), key);
            // 处理按键
            switch (key) {
                case 'G':
                    RCLCPP_INFO(LOGGER, "Switched to G mode");
                    g_state_machine->switch_to(ControlMode::Following);
                    break;
                case 'W':
                    RCLCPP_INFO(LOGGER, "Switched to W mode");
                    g_state_machine->switch_to(ControlMode::UpIncre);
                    break;
                case 'S':
                    RCLCPP_INFO(LOGGER, "Switched to S mode");
                    g_state_machine->switch_to(ControlMode::DownIncre);
                    break;
                case 'A':
                    RCLCPP_INFO(LOGGER, "Switched to A mode");
                    g_state_machine->switch_to(ControlMode::LeftIncre);
                    break;
                case 'D':
                    g_state_machine->switch_to(ControlMode::RightIncre);
                    break;
                case 'E':
                    g_state_machine->switch_to(ControlMode::ForwardIncre);
                    break;
                case 'Q':
                    g_state_machine->switch_to(ControlMode::BackwardIncre);
                    break;
                case 'I':
                    g_state_machine->switch_to(ControlMode::InIncre);
                    break;
                case 'O':
                    g_state_machine->switch_to(ControlMode::OutIncre);
                    break;
                case 'R':
                    g_state_machine->switch_to(ControlMode::UpRestrain);
                    break;
                case 'T':
                    g_state_machine->switch_to(ControlMode::Turn);
                    break;
                case 'X':
                    rclcpp::shutdown();
                    break;
                case '1':
                case '2':
                case '3':
                case '4':
                case '5':
                case '6':
                    {
                        ControlMode mode = static_cast<ControlMode>(static_cast<int>(ControlMode::ENERGY_UNIT_1) + (key - '1'));
                        g_state_machine->switch_to(mode);
                    }
                    break;
                default:
                    break;
                
            }
            
            // 仅在模式切换后触发 update()
            if (g_state_machine->mode_ != prev_mode) {
                g_state_machine->update();
            }
        },
        control_cb_group);

    while (rclcpp::ok())
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Destroy state machine first so timer callbacks stop doing work before
    // the node context becomes invalid (prevents vt_->trigger() crash on SIGINT)
    g_state_machine.reset();
    exec.cancel();
    if (spin_thread.joinable()) spin_thread.join();
    return 0;
}