/**
 * @file flow_with_pyroki.cpp
 * @brief 多模态控制工作流: 支持状态机切换
 */

#include <atomic>
#include <chrono>
#include <fstream>
#include <map>
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
#include <moveit/kinematic_constraints/utils.hpp>


//全局常量
static const rclcpp::Logger LOGGER = rclcpp::get_logger("flow_with_pyroki");
static const std::string PLANNING_GROUP = "arm";
static const double STEP = 0.01;
static const std::vector<std::string> JOINT_NAMES = {
    "joint1","joint2","joint3","joint4","joint5","joint6"};
static const std::vector<double> energy_unit_1 = {
    0.0, -0.85, 0.82, 0.0, 0.15, -1.57
};
static const geometry_msgs::msg::Pose energy_unit_1_pose = []() {
    Eigen::Matrix3d rot;
    rot << 0, 0, 1,
           0, 1, 0,
           -1, 0, 0;
    Eigen::Quaterniond q(rot);
    q.normalize();
    
    geometry_msgs::msg::Pose p;
    p.position.x = 0.3;
    p.position.y = 0.0;
    p.position.z = 0.525;
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();
    return p;
}();
static const std::vector<double> energy_unit_2 = {
     0.0, -0.85, 0.82, 0.0, 0.15, 0.55
};
static const geometry_msgs::msg::Pose energy_unit_2_pose = []() {
    Eigen::Matrix3d rot;
    rot << 0, sin(M_PI/6), cos(M_PI/6),
           0, cos(M_PI/6), -sin(M_PI/6),
          -1, 0, 0;
    Eigen::Quaterniond q(rot);
    q.normalize();
    
    geometry_msgs::msg::Pose p;
    p.position.x = 0.3;
    p.position.y = 0.0;
    p.position.z = 0.525;
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();
    return p;
}();
static const std::vector<double> energy_unit_3 = {
    0.0, -0.85, 0.82, 0.0, 0.15,-0.75, 
};
static const geometry_msgs::msg::Pose energy_unit_3_pose = []() {
    Eigen::Matrix3d rot;
    rot << 0, -sin(M_PI/6), cos(M_PI/6),
           0, cos(M_PI/6),  sin(M_PI/6),
          -1, 0, 0;

    Eigen::Quaterniond q(rot);
    q.normalize();
    
    geometry_msgs::msg::Pose p;
    p.position.x = 0.3;
    p.position.y = 0.0;
    p.position.z = 0.20625;
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();
    return p;
}();

static const std::vector<double> energy_unit_4 = {
    0.0, -0.85, 0.82, 0.0, 0.15, -1.57
};
static const geometry_msgs::msg::Pose energy_unit_4_pose = []() {
    Eigen::Matrix3d rot;
    rot << 0, 0, 1,
           0, 1, 0,
          -1, 0, 0;
    Eigen::Quaterniond q(rot);
    q.normalize();
    
    geometry_msgs::msg::Pose p;
    //p.position.x = 0.3;
    //p.position.y = 0.0;
    //p.position.z = 0.1125;
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();
    return p;
}();
static const std::vector<double> energy_unit_5 = {
    0.0, -0.85, 0.82, 0.0, 0.15, -2.23
};
static const geometry_msgs::msg::Pose energy_unit_5_pose = []() {
    Eigen::Matrix3d rot;
    rot << 0, sin(M_PI/6), cos(M_PI/6),
           0, cos(M_PI/6), -sin(M_PI/6),
          -1, 0, 0;
    Eigen::Quaterniond q(rot);
    q.normalize();
    
    geometry_msgs::msg::Pose p;
    p.position.x = 0.3;
    p.position.y = 0.0;
    p.position.z = 0.20625;
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();
    return p;
}();
static const std::vector<double> energy_unit_6 = {
    0.0, -0.85, 0.82, 0.0, 0.15, 2.23
};
static const geometry_msgs::msg::Pose energy_unit_6_pose = []() {
    Eigen::Matrix3d rot;
    rot << 0, -sin(M_PI/6), cos(M_PI/6),
           0, cos(M_PI/6),  sin(M_PI/6),
          -1, 0, 0;
    Eigen::Quaterniond q(rot);
    q.normalize();
    
    geometry_msgs::msg::Pose p;
    p.position.x = 0.3;
    p.position.y = 0.0;
    p.position.z = 0.39375;
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();
    return p;
}();

static const geometry_msgs::msg::Pose save_pose = [](){
    Eigen::Matrix3d rot;
    rot << 0, 0, 1,
           0, 1, 0,
          -1, 0, 0;
    Eigen::Quaterniond q(rot);
    q.normalize();
    
    geometry_msgs::msg::Pose p;
    p.position.x = 0.3;
    p.position.y = 0.0;
    p.position.z = 0.525;
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();
    return p;
}();
//全局变量
static std::mutex g_js_mutex;

//关节角度的全局变量
static std::vector<double> g_arm_state(6, 0.0);           // 当前发布状态
//static std::vector<double> g_last_state(6, 0.0);       // g_arm_state上一刻的状态
//static std::vector<double> g_real_arm_state(6, 0.0);      // 实际状态 (在仿真里和/joint_states一样, 但在真实环境里是读取串口的数据)
static std::vector<double> g_ctrl_js(6, 0.0);

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
    InIncre,    // I: 末端沿当前x轴正向增量模式
    OutIncre,   // O: 末端沿当前x轴负向增
    IncrementalIdle, // 增量动作完成后的中间状态
    UpRestrain, // R: 用雅可比矩阵约束末端沿q_vector方向不动（将控制器在这个方向上的位移消除掉然后进行IK）
    Turn, // T: 用execute_turn规划一个绕P轴旋转的动作
    Save, //0: 存矿,规划到save pose



    ENERGY_UNIT_1, // 1: 去取能量单元1
    ENERGY_UNIT_2, // 2: 去取能量单元2
    ENERGY_UNIT_3, // 3: 去取能量单元3
    ENERGY_UNIT_4, // 4: 去取能量单元4
    ENERGY_UNIT_5, // 5: 去取能量单元5      
    ENERGY_UNIT_6  // 6: 去取能量单元6
};

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
bool appendTrajectoryToFile(
    const std::map<std::string, std::vector<double>>& trajectory_data,
    const std::string& file_path = "/home/hku/robo_arm/last_incremental_motion_trajectory.log")
{
    std::ofstream out(file_path);
    if (!out.is_open()) {
        return false;
    }

    for (const auto& kv : trajectory_data) {
        out << kv.first << "=";
        for (size_t i = 0; i < kv.second.size(); ++i) {
            if (i > 0) {
                out << ",";
            }
            out << kv.second[i];
        }
        out << '\n';
    }
    out << "---" << '\n';

    return true;
}

/**
 * @brief 状态机: 管理多种控制模式
 */
class ControlStateMachine {
public:
    using MoveGroupPtr = std::shared_ptr<moveit::planning_interface::MoveGroupInterface>;
    using JMGPtr = const moveit::core::JointModelGroup*;
    using VTPtr = std::shared_ptr<moveit_visual_tools::MoveItVisualTools>;
    using JointStatePublisherPtr = rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr;
    
    ControlMode mode_;
    MoveGroupPtr mg_;
    JMGPtr jmg_;
    VTPtr vt_;
    rclcpp::Node::SharedPtr node_;
    JointStatePublisherPtr js_pub_;
    
    ControlStateMachine(rclcpp::Node::SharedPtr node, MoveGroupPtr mg, JMGPtr jmg, VTPtr vt, JointStatePublisherPtr js_pub)
        : mode_(ControlMode::Following), mg_(mg), jmg_(jmg), vt_(vt), node_(node), js_pub_(js_pub) {}
    
    void start() {
        RCLCPP_INFO(LOGGER, "Started");
        enter_following();
        // activate planning monitor
        mg_->getCurrentState(5.0);

    }
    
    void switch_to(ControlMode new_mode) {
        if (new_mode == mode_) return;
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
                Eigen::Vector3d ee_x = rotation.col(0); 
                plan_incremental_motion({ee_x(0), ee_x(1), ee_x(2)});
                break;
            }
            case ControlMode::OutIncre: {
                auto current_state = mg_->getCurrentState(5.0);
                const Eigen::Isometry3d& tf = current_state->getGlobalLinkTransform(mg_->getEndEffectorLink());
                Eigen::Matrix3d rotation = tf.rotation();
                Eigen::Vector3d ee_x = rotation.col(0); 
                plan_incremental_motion({-ee_x(0), -ee_x(1), -ee_x(2)}); //末端x反方向
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
            case ControlMode::IncrementalIdle:
                break;
            case ControlMode::Save:
                plan_to_save_pose(); break;
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
        // 按节奏更新 g_arm_state，由 pub_timer 定期发布到 /joint_states
        for (const auto& pt : jt.points) {
            { std::lock_guard<std::mutex> lk(g_js_mutex); g_arm_state = pt.positions; }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        
        // 增量动作执行完成后，切换到 IncrementalIdle 状态
        // 这样下次按同一个键时会从 IncrementalIdle → 该增量模式，从而重新规划
        if (mode_ == ControlMode::UpIncre || mode_ == ControlMode::DownIncre ||
            mode_ == ControlMode::RightIncre || mode_ == ControlMode::LeftIncre ||
            mode_ == ControlMode::ForwardIncre || mode_ == ControlMode::BackwardIncre ||
            mode_ == ControlMode::InIncre || mode_ == ControlMode::OutIncre) {
            mode_ = ControlMode::IncrementalIdle;
            RCLCPP_INFO(LOGGER, "Incremental motion completed, waiting for next command");
        }
    }

private:
    void plan_to_save_pose() {
        RCLCPP_INFO(LOGGER, "Planning to save_pose");
        
        mg_->setStartStateToCurrentState();
        
        // 尝试规划，如果失败则在保持x轴朝向不变的情况下迭代改变y、z轴朝向
        const int MAX_ATTEMPTS = 10;
        const double ANGLE_STEP = M_PI / 18;  // 10度为单位进行旋转
        
        geometry_msgs::msg::Pose original_target = save_pose;
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::core::MoveItErrorCode rc = moveit::core::MoveItErrorCode::PLANNING_FAILED;
        
        for (int attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
            geometry_msgs::msg::Pose target_pose = original_target;
            
            if (attempt > 0) {
                // 提取原始四元数并转换为旋转矩阵
                Eigen::Quaterniond original_quat(
                    original_target.orientation.w,
                    original_target.orientation.x,
                    original_target.orientation.y,
                    original_target.orientation.z);
                Eigen::Matrix3d original_rot = original_quat.toRotationMatrix();
                
                // 获取 x 轴方向（保持固定）
                Eigen::Vector3d x_axis = original_rot.col(0);
                
                // 计算旋转角度：在绕 x 轴的平面内旋转
                double rotation_angle = ANGLE_STEP * attempt;
                Eigen::AngleAxisd rotation_around_x(rotation_angle, x_axis);
                
                // 应用旋转：保持 x 轴不变，改变 y 和 z 轴
                Eigen::Matrix3d new_rot = original_rot * rotation_around_x.toRotationMatrix();
                
                // 转换回四元数
                Eigen::Quaterniond new_quat(new_rot);
                new_quat.normalize();
                
                target_pose.orientation.x = new_quat.x();
                target_pose.orientation.y = new_quat.y();
                target_pose.orientation.z = new_quat.z();
                target_pose.orientation.w = new_quat.w();
                
                RCLCPP_WARN(LOGGER, "Plan to save_pose failed at attempt %d, trying different orientation (angle: %.2f rad)", 
                    attempt, rotation_angle);
            }
            
            mg_->setPoseTarget(target_pose, mg_->getEndEffectorLink());
            rc = mg_->plan(plan);
            
            if (rc == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(LOGGER, "Plan to save_pose succeeded at attempt %d", attempt + 1);
                break;
            }
        }
        
        if (rc != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(LOGGER, "Plan to save_pose failed after %d attempts (code=%d, error='%s', message='%s', source='%s')",   
                MAX_ATTEMPTS,
                static_cast<int>(rc.val),  
                moveit::core::errorCodeToString(rc).c_str(),  
                rc.message.c_str(),  
                rc.source.c_str());  
            return;
        }
        
        vt_->deleteAllMarkers();
        vt_->publishTrajectoryLine(plan.trajectory, jmg_->getLinkModel(mg_->getEndEffectorLink()), jmg_);
        vt_->trigger();
        
        write_trajectory(plan.trajectory.joint_trajectory);
    }
    
    void get_energy_unit(int unit_id) {     
        std::vector<double> target_pose;   
        //geometry_msgs::msg::Pose target_pose;
        /** 
        switch (unit_id) {
            case 1: target_pose = energy_unit_1_pose; break;
            case 2: target_pose = energy_unit_2_pose; break;
            case 3: target_pose = energy_unit_3_pose; break;
            case 4: target_pose = energy_unit_4_pose; break;
            case 5: target_pose = energy_unit_5_pose; break;
            case 6: target_pose = energy_unit_6_pose; break;
        }
        */
        switch (unit_id) {
            case 1: target_pose = energy_unit_1; break;
            case 2: target_pose = energy_unit_2; break;
            case 3: target_pose = energy_unit_3; break;
            case 4: target_pose = energy_unit_4; break;
            case 5: target_pose = energy_unit_5; break;
            case 6: target_pose = energy_unit_6; break;
        }
        
        RCLCPP_INFO(LOGGER, "Planning to energy_unit_%d", unit_id);
        
        auto current_pose = mg_->getCurrentPose(mg_->getEndEffectorLink());
        auto current_joints = mg_->getCurrentJointValues();
        
        mg_->setStartStateToCurrentState();
        
        // 设置目标位置容差（0.05m）
        //mg_->setGoalPositionTolerance(0.05);
        mg_ ->setJointValueTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::core::MoveItErrorCode rc = mg_-> plan(plan);

        // 设置目标朝向容差（相比save_pose更严格的朝向约束）
        //mg_->setGoalOrientationTolerance(0.02);  // ~1.15度容差

        /** 
        // 尝试规划，如果失败则在目标x、y坐标范围内迭代搜索
        const double SEARCH_RADIUS = 0.15;  // 15cm范围
        const int MAX_ATTEMPTS = 10;
        
        geometry_msgs::msg::Pose original_target = target_pose;
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::core::MoveItErrorCode rc = moveit::core::MoveItErrorCode::PLANNING_FAILED;
        
        for (int attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
            mg_->setPoseTarget(target_pose, mg_->getEndEffectorLink());
            //mg_ -> setPose
            rc = mg_->plan(plan);
            
            if (rc == moveit::core::MoveItErrorCode::SUCCESS) {
                auto trajectory_data = extractTrajectory(plan.trajectory);  
                if (appendTrajectoryToFile(trajectory_data)) {
                    RCLCPP_INFO(LOGGER,
                        "Trajectory data written to /tmp/flow_with_pyroki_trajectory.log");
                } else {
                    RCLCPP_WARN(LOGGER,
                        "Failed to write trajectory data to /tmp/flow_with_pyroki_trajectory.log");
                }
                RCLCPP_INFO(LOGGER, "Plan to energy_unit_%d succeeded at attempt %d", unit_id, attempt + 1);
                break;
            }
            
            // 如果失败，在目标的x、y坐标范围内尝试邻近位置
            if (attempt < MAX_ATTEMPTS - 1) {
                // 使用网格搜索：在x、y方向各尝试多个偏移 (3x3网格 = 9种位置)
                int grid_index = attempt % 9;
                double offset_x = ((grid_index % 3) - 1) * SEARCH_RADIUS / 2;  // [-0.075, 0, 0.075]
                double offset_y = ((grid_index / 3) - 1) * SEARCH_RADIUS / 2;  // [-0.075, 0, 0.075]
                
                target_pose.position.x = original_target.position.x + offset_x;
                target_pose.position.y = original_target.position.y + offset_y;
                target_pose.position.z = original_target.position.z;
                target_pose.orientation = original_target.orientation;
                
                RCLCPP_WARN(LOGGER, "Plan to energy_unit_%d failed at attempt %d, trying nearby position (offset: %.3f, %.3f)", 
                    unit_id, attempt + 1, offset_x, offset_y);
            }
            
        }
        
        if (rc != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(LOGGER, "Plan to energy_unit_%d failed after %d attempts (code=%d, error='%s', message='%s', source='%s')",   
                unit_id,
                MAX_ATTEMPTS,
                static_cast<int>(rc.val),  
                moveit::core::errorCodeToString(rc).c_str(),  
                rc.message.c_str(),  
                rc.source.c_str());  
            return;
        }
        */
        
        vt_->deleteAllMarkers();
        vt_->publishTrajectoryLine(plan.trajectory, jmg_->getLinkModel(mg_->getEndEffectorLink()), jmg_);
        vt_->trigger();
        appendTrajectoryToFile(extractTrajectory(plan.trajectory));
        write_trajectory(plan.trajectory.joint_trajectory);
    }
    
    void plan_incremental_motion(const std::vector<double>& direction, double step_size = 0.05) {
        
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
        appendTrajectoryToFile(extractTrajectory(trajectory));
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

    auto js_pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    rclcpp::SubscriptionOptions ctrl_sub_opts;
    ctrl_sub_opts.callback_group = io_cb_group;

    auto ctrl_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/pyroki_joint_states", 10,
        [](const sensor_msgs::msg::JointState::SharedPtr msg) {
            if (msg->position.size() >= 6) {
                std::lock_guard<std::mutex> lk(g_js_mutex);
                for (size_t i = 0; i < 6; ++i) {
                    g_ctrl_js[i] = msg->position[i];
                }
                // Following 模式下直接同步，避免依赖周期性 update()。
                if (g_state_machine && g_state_machine->mode_ == ControlMode::Following) {
                    g_arm_state = g_ctrl_js;
                }
                g_got_pyroki_state = true;
            }
        },
        ctrl_sub_opts);

    // 先启动 /joint_states 发布，再初始化 MoveGroup，避免启动期拿不到当前状态。
    auto pub_timer = node->create_wall_timer(
        std::chrono::milliseconds(20),
        [node, js_pub]() {
            sensor_msgs::msg::JointState m;
            auto stamp = node->now();
            if (stamp.nanoseconds() == 0) {
                // Fallback for missing /clock when sim time is accidentally enabled.
                stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
            }
            m.header.stamp = stamp;
            m.name = JOINT_NAMES;
            { std::lock_guard<std::mutex> lk(g_js_mutex); m.position = g_arm_state; }
            js_pub->publish(m);
        },
        io_cb_group);


    // 初始化MoveIt
    auto mg = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, PLANNING_GROUP);
    mg->setEndEffectorLink("gripper");
    // const auto* jmg = mg->getRobotModel()->getJointModelGroup(PLANNING_GROUP);
    const moveit::core::JointModelGroup* jmg =
        mg->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    auto vt = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
        node, "base_link", "flow_with_pyroki", mg->getRobotModel());
    vt->deleteAllMarkers();

    // 等待初始状态
    
    
    // 初始化状态机
    g_state_machine = std::make_unique<ControlStateMachine>(node, mg, jmg, vt, js_pub);
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
    
    static int log_counter = 0;

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
            
            if (key == 0 && g_state_machine->mode_ == ControlMode::Following) {
                g_state_machine->update();
            }

            const ControlMode prev_mode = g_state_machine->mode_;
            log_counter++;
            if (log_counter % 200 == 0) {
                RCLCPP_INFO(LOGGER, "Current State Mode: %d", static_cast<int>(g_state_machine->mode_));
            }   
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
                case '0':
                    g_state_machine->switch_to(ControlMode::Save);
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