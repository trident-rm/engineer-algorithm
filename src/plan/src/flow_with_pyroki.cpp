/**
 * @file flow_with_pyroki.cpp
 * @brief 多模态控制工作流: 支持状态机切换
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>
#include <boost/variant/get.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <robo_arm_msgs/msg/plane_frame.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace
{
std::string toFileUri(const std::filesystem::path& path)
{
    return "file://" + path.string();
}

geometry_msgs::msg::Pose makePose(double x, double y, double z, double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 1.0)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = qx;
    pose.orientation.y = qy;
    pose.orientation.z = qz;
    pose.orientation.w = qw;
    return pose;
}

moveit_msgs::msg::CollisionObject makeCollisionObject(
    const std::string& id,
    const std::string& mesh_resource,
    const std::string& frame_id,
    const geometry_msgs::msg::Pose& pose)
{
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = frame_id;
    object.id = id;
    object.operation = moveit_msgs::msg::CollisionObject::ADD;

    std::unique_ptr<shapes::Mesh> mesh(shapes::createMeshFromResource(mesh_resource));
    if (!mesh) {
        throw std::runtime_error("Failed to load mesh resource: " + mesh_resource);
    }

    shapes::ShapeMsg shape_msg;
    if (!shapes::constructMsgFromShape(mesh.get(), shape_msg)) {
        throw std::runtime_error("Failed to convert mesh to message: " + mesh_resource);
    }

    object.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(shape_msg));
    object.mesh_poses.push_back(pose);
    return object;
}

geometry_msgs::msg::Pose randomPillarPoseInRange()
{
    static thread_local std::mt19937 rng(std::random_device{}());
    static std::uniform_real_distribution<double> x_dist(-0.85, -0.75);
    static std::uniform_real_distribution<double> y_dist(-0.05, 0.05);
    static std::uniform_real_distribution<double> z_dist(0.5, 0.6);
    static std::uniform_real_distribution<double> roll_dist(-M_PI/6, M_PI/6);
    static std::uniform_real_distribution<double> pitch_dist(-M_PI/6, M_PI/6);
    static std::uniform_real_distribution<double> yaw_dist(-M_PI/3, M_PI/3);

    const double roll = roll_dist(rng);
    const double pitch = pitch_dist(rng);
    const double yaw = yaw_dist(rng);

    const Eigen::AngleAxisd roll_aa(roll, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch_aa(pitch, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw_aa(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yaw_aa * pitch_aa * roll_aa;
    q.normalize();

    return makePose(
        x_dist(rng),
        y_dist(rng),
        z_dist(rng),
        q.x(),
        q.y(),
        q.z(),
        q.w());
}
}  // namespace

//状态机demo的虚假变量
static const geometry_msgs::msg::Pose fake_hover_pose = []() {
    geometry_msgs::msg::Pose p;
    p.position.x = -0.1789;
    p.position.y = -0.0983;
    p.position.z = 0.5492;
    p.orientation.x = -0.0244;
    p.orientation.y = 0.1637;
    p.orientation.z = 0.2011;
    p.orientation.w = 0.9655;
    return p;
}();

//全局常量
static const rclcpp::Logger LOGGER = rclcpp::get_logger("flow_with_pyroki");
static const std::string PLANNING_GROUP = "arm";
static const double STEP = 0.01;
static const std::string WORLD_FRAME = "world";
static const std::string GROUND_FRAME = "reduced_ground_frame";
static const std::string PILLAR_FRAME = "reduced_pillar_frame";
static std::mutex g_env_pose_mutex;
static geometry_msgs::msg::Pose g_ground_pose = makePose(0.0, 0.0, 0.0);
static geometry_msgs::msg::Pose g_pillar_pose = makePose(-0.8, 0.0, 0.55);
static const std::vector<std::string> JOINT_NAMES = {
    "joint1","joint2","joint3","joint4","joint5","joint6"};
static const std::vector<double> energy_unit_1 = {
    0.0, -0.85, 0.82, 0.0, 0.15, -1.57
};
static const geometry_msgs::msg::Pose g_energy_unit_1_pose = []() {
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
static const std::vector<double> g_energy_unit_2 = {
     0.0, -0.85, 0.82, 0.0, 0.15, 0.55
};
static const geometry_msgs::msg::Pose g_energy_unit_2_pose = []() {
    Eigen::Matrix3d rot;
    rot << 0, 0, 1,
           cos(M_PI/3), sin(M_PI/3), 0,
          -sin(M_PI/3), cos(M_PI/3), 0;
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
static const std::vector<double> g_energy_unit_3 = {
    0.0, -0.85, 0.82, 0.0, 0.15,-0.75, 
};
static const geometry_msgs::msg::Pose g_energy_unit_3_pose = []() {
    Eigen::Matrix3d rot;
    rot << 0, 0, 1,
           -cos(M_PI/6), sin(M_PI/6), 0,
          -sin(M_PI/6), -cos(M_PI/6), 0;

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

static const std::vector<double> g_energy_unit_4 = {
    0.0, -0.85, 0.82, 0.0, 0.15, -1.57
};
static const geometry_msgs::msg::Pose g_energy_unit_4_pose = []() {
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
static const std::vector<double> g_energy_unit_5 = {
    0.0, -0.85, 0.82, 0.0, 0.15, -2.23
};
static const geometry_msgs::msg::Pose g_energy_unit_5_pose = []() {
    Eigen::Matrix3d rot;
    rot << 0, 0, 1,
           cos(M_PI/3), sin(M_PI/3), 0,
          -sin(M_PI/3), cos(M_PI/3), 0;
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
static const std::vector<double> g_energy_unit_6 = {
    0.0, -0.85, 0.82, 0.0, 0.15, 2.23
};
static const geometry_msgs::msg::Pose g_energy_unit_6_pose = []() {
    Eigen::Matrix3d rot;
    rot << 0, 0, 1,
           -cos(M_PI/6), sin(M_PI/6), 0,
          -sin(M_PI/6), -cos(M_PI/6), 0;
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

static geometry_msgs::msg::Pose g_hover_pose = [](){
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
static std::vector<double> g_arm_state = {0.0, 0.3, 0.0, 0.0, 0.0, 0.0};           // 当前发布状态
//static std::vector<double> g_last_state(6, 0.0);       // g_arm_state上一刻的状态
//static std::vector<double> g_real_arm_state(6, 0.0);      // 实际状态 (在仿真里和/joint_states一样, 但在真实环境里是读取串口的数据)
static std::vector<double> g_ctrl_js(6, 0.0);

//方向向量的全局变量
static std::vector<double> g_p_vector = {0, 0, 1}; 
static std::vector<double> g_q_vector = {0, 1, 0};
static std::vector<double> g_up_vector = {1, 0, 0};
static std::vector<double> g_base_x = {1, 0, 0};
static std::vector<double> g_base_y = {0, 1, 0};
static std::vector<double> g_base_z = {0, 0, 1};
static std::vector<double> g_ee_positive = {0, 0, 0}; //末端link当前z轴的方向（正）
static std::vector<double> g_plane_center = {0, 0, 0}; // marker平面中心点
static std::vector<std::vector<double>> g_plane_corners; // marker平面四个角点

//位置向量的全局变量 
static std::vector<double> g_up_start = {0.0,0.0,0.0}; //upRestrain刚开始的时候末端位置
static std::vector<double> g_up_end = {0.0,0.0,0.0}; //upRestrain结束时末端位置

// 键盘输入缓存
static std::mutex g_key_mutex;
static std::condition_variable g_key_cv;
static char g_last_key = 0;

// pyroki 状态接收标志
static std::atomic<bool> g_got_pyroki_state(false);

static void publishEnvironmentTF(
    const rclcpp::Node::SharedPtr& node,
    const std::shared_ptr<tf2_ros::TransformBroadcaster>& tf_broadcaster)
{
    geometry_msgs::msg::Pose ground_pose_snapshot;
    geometry_msgs::msg::Pose pillar_pose_snapshot;
    {
        std::lock_guard<std::mutex> lk(g_env_pose_mutex);
        ground_pose_snapshot = g_ground_pose;
        pillar_pose_snapshot = g_pillar_pose;
    }

    geometry_msgs::msg::TransformStamped ground_tf;
    ground_tf.header.stamp = node->now();
    ground_tf.header.frame_id = WORLD_FRAME;
    ground_tf.child_frame_id = GROUND_FRAME;
    ground_tf.transform.translation.x = ground_pose_snapshot.position.x;
    ground_tf.transform.translation.y = ground_pose_snapshot.position.y;
    ground_tf.transform.translation.z = ground_pose_snapshot.position.z;
    ground_tf.transform.rotation = ground_pose_snapshot.orientation;

    geometry_msgs::msg::TransformStamped pillar_tf;
    pillar_tf.header.stamp = ground_tf.header.stamp;
    pillar_tf.header.frame_id = WORLD_FRAME;
    pillar_tf.child_frame_id = PILLAR_FRAME;
    pillar_tf.transform.translation.x = pillar_pose_snapshot.position.x;
    pillar_tf.transform.translation.y = pillar_pose_snapshot.position.y;
    pillar_tf.transform.translation.z = pillar_pose_snapshot.position.z;
    pillar_tf.transform.rotation = pillar_pose_snapshot.orientation;

    tf_broadcaster->sendTransform(ground_tf);
    tf_broadcaster->sendTransform(pillar_tf);
}

static bool updatePillarCollisionObjectInPlanningScene(const geometry_msgs::msg::Pose& pillar_pose)
{
    try {
        const std::filesystem::path workspace_root = "/home/hku/robo_arm/src/arms";
        const std::filesystem::path pillar_mesh = workspace_root / "reduced_pillar/meshes/pillar_link.STL";

        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        const auto pillar_object = makeCollisionObject(
            "reduced_pillar",
            toFileUri(pillar_mesh),
            WORLD_FRAME,
            pillar_pose);
        planning_scene_interface.applyCollisionObject(pillar_object);
        return true;
    } catch (const std::exception& ex) {
        RCLCPP_WARN(LOGGER, "Failed to update reduced_pillar collision object: %s", ex.what());
        return false;
    }
}

enum class ControlMode {
    Idle,       // 全局空闲态：不执行任何动作
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
    Save, //Z，自动化存矿完整状态机
    ENERGY_UNIT_1, // 1: 去取能量单元1
    ENERGY_UNIT_2, // 2: 去取能量单元2
    ENERGY_UNIT_3, // 3: 去取能量单元3
    ENERGY_UNIT_4, // 4: 去取能量单元4
    ENERGY_UNIT_5, // 5: 去取能量单元5      
    ENERGY_UNIT_6  // 6: 去取能量单元6
};

//Save状态内部子状态机 （）une25)
enum class SaveSubState {
    CalPlane,
    Hover,
    Slide,
    Turn_P,
    Turn_Q,
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
    using MarkerArrayPublisherPtr = rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr;
    
    ControlMode mode_;
    SaveSubState save_substate_;
    MoveGroupPtr mg_;
    JMGPtr jmg_;
    VTPtr vt_;
    rclcpp::Node::SharedPtr node_;
    JointStatePublisherPtr js_pub_;
    MarkerArrayPublisherPtr marker_pub_; //绕P、Q轴旋转相关可视化
    MarkerArrayPublisherPtr plane_pub_; //平面相关可视化
    rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr pyroki_ik_client_;
    

    std::mutex plane_mutex_;
    bool has_latest_plane_frame_ = false;
    robo_arm_msgs::msg::PlaneFrame latest_plane_frame_;
    robo_arm_msgs::msg::PlaneFrame save_plane_frame_;

    
    ControlStateMachine(rclcpp::Node::SharedPtr node, MoveGroupPtr mg, 
        JMGPtr jmg, VTPtr vt, JointStatePublisherPtr js_pub, 
        MarkerArrayPublisherPtr marker_pub, MarkerArrayPublisherPtr plane_pub)
        : mode_(ControlMode::Following), save_substate_(SaveSubState::CalPlane), mg_(mg), jmg_(jmg), vt_(vt), node_(node), js_pub_(js_pub), marker_pub_(marker_pub), plane_pub_(plane_pub) {
        pyroki_ik_client_ = node_->create_client<moveit_msgs::srv::GetPositionIK>("/pyroki_get_position_ik");
    }
    
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
            case ControlMode::Save:
                save_substate_ = SaveSubState::CalPlane;
                enter_save();
                break;
            case ControlMode::Idle:
                break;
            case ControlMode::Following:
                enter_following();
                break;
            case ControlMode::UpIncre:
                if (!plan_incremental_motion(g_base_z)) {
                    RCLCPP_WARN(LOGGER, "UpIncre planning failed");
                }
                break;
            case ControlMode::DownIncre:
                if (!plan_incremental_motion({-g_base_z[0], -g_base_z[1], -g_base_z[2]})) {  // -Z
                    RCLCPP_WARN(LOGGER, "DownIncre planning failed");
                }
                break;
            case ControlMode::RightIncre:
                if (!plan_incremental_motion({-g_base_y[0], -g_base_y[1], -g_base_y[2]})) {  // -Y
                    RCLCPP_WARN(LOGGER, "RightIncre planning failed");
                }
                break;
            case ControlMode::LeftIncre:
                if (!plan_incremental_motion(g_base_y)) {  // +Y
                    RCLCPP_WARN(LOGGER, "LeftIncre planning failed");
                }
                break;
            case ControlMode::ForwardIncre:
                if (!plan_incremental_motion(g_base_x)) {  // +X
                    RCLCPP_WARN(LOGGER, "ForwardIncre planning failed");
                }
                break;
            case ControlMode::BackwardIncre:
                if (!plan_incremental_motion({-g_base_x[0], -g_base_x[1], -g_base_x[2]})) {  // -X
                    RCLCPP_WARN(LOGGER, "BackwardIncre planning failed");
                }
                break;
            case ControlMode::InIncre: {
                auto current_state = mg_->getCurrentState(5.0);
                const Eigen::Isometry3d& tf = current_state->getGlobalLinkTransform(mg_->getEndEffectorLink());
                Eigen::Matrix3d rotation = tf.rotation();
                Eigen::Vector3d ee_x = rotation.col(0); 
                if (!plan_incremental_motion({ee_x(0), ee_x(1), ee_x(2)})) {
                    RCLCPP_WARN(LOGGER, "InIncre planning failed");
                }
                break;
            }
            case ControlMode::OutIncre: {
                auto current_state = mg_->getCurrentState(5.0);
                const Eigen::Isometry3d& tf = current_state->getGlobalLinkTransform(mg_->getEndEffectorLink());
                Eigen::Matrix3d rotation = tf.rotation();
                Eigen::Vector3d ee_x = rotation.col(0); 
                if (!plan_incremental_motion({-ee_x(0), -ee_x(1), -ee_x(2)})) { //末端x反方向
                    RCLCPP_WARN(LOGGER, "OutIncre planning failed");
                }
                break;
            }
            case ControlMode::UpRestrain: {
                auto current_state = mg_->getCurrentState(5.0);
                const Eigen::Isometry3d& tf = current_state->getGlobalLinkTransform(mg_->getEndEffectorLink());
                
                // 记录末端X轴朝向
                Eigen::Matrix3d rotation = tf.rotation();
                Eigen::Vector3d ee_x = rotation.col(0);  // 末端X轴
                g_q_vector = {ee_x(0), ee_x(1), ee_x(2)};
                
                // 记录末端位置
                g_up_start = {tf.translation().x(), tf.translation().y(), tf.translation().z()};
                break;
            }
            case ControlMode::Turn: {
                auto current_state = mg_->getCurrentState(5.0);
                const Eigen::Isometry3d& tf = current_state->getGlobalLinkTransform(mg_->getEndEffectorLink());
                
                // 记录末端位置
                g_up_end = {tf.translation().x(), tf.translation().y(), tf.translation().z()};
                
                /*
                // 计算up_vector = up_end - up_start
                g_up_vector[0] = g_up_end[0] - g_up_start[0];
                g_up_vector[1] = g_up_end[1] - g_up_start[1];
                g_up_vector[2] = g_up_end[2] - g_up_start[2];
                
                //计算p_vector = q_vector x up_vector
                g_p_vector[0] = g_q_vector[1] * g_up_vector[2] - g_q_vector[2] * g_up_vector[1];  
                g_p_vector[1] = g_q_vector[2] * g_up_vector[0] - g_q_vector[0] * g_up_vector[2];  
                g_p_vector[2] = g_q_vector[0] * g_up_vector[1] - g_q_vector[1] * g_up_vector[0];
                */
                // center = start - q * radius
                // q=-X -> center is on +X; rotating +90deg around +Y moves end toward +X and +Z.
                g_q_vector = {-1, 0, 0};
                g_up_vector = {0, 1, 0};
                g_p_vector = {0, 1, 0};

                publish_turn_vectors(g_up_end);
                if (!plan_and_exe_turn_p()) {
                    RCLCPP_WARN(LOGGER, "Turn planning around P axis failed");
                }
                break;
            }
            case ControlMode::IncrementalIdle:
                break;
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

    void on_plane_frame(const robo_arm_msgs::msg::PlaneFrame::SharedPtr msg) {
        Eigen::Vector3d up_axis(msg->up_axis.x, msg->up_axis.y, msg->up_axis.z);
        Eigen::Vector3d p_axis(msg->p_axis.x, msg->p_axis.y, msg->p_axis.z);
        Eigen::Vector3d q_axis(msg->q_axis.x, msg->q_axis.y, msg->q_axis.z);
        Eigen::Vector3d center(msg->center.x, msg->center.y, msg->center.z);

        if (!normalize_vector(up_axis) || !normalize_vector(p_axis) ||
            !normalize_vector(q_axis) || !center.allFinite()) {
            RCLCPP_WARN(LOGGER, "on_plane_frame: received non-finite or degenerate plane frame");
            return;
        }

        if (msg->corners.empty()) {
            RCLCPP_WARN(LOGGER, "on_plane_frame: received empty plane corners");
            return;
        }

        std::vector<geometry_msgs::msg::Point32> corners;
        corners.reserve(msg->corners.size());
        for (const auto& c : msg->corners) {
            if (!std::isfinite(c.x) || !std::isfinite(c.y) || !std::isfinite(c.z)) {
                RCLCPP_WARN(LOGGER, "on_plane_frame: received non-finite corner");
                return;
            }
            corners.push_back(c);
        }

        robo_arm_msgs::msg::PlaneFrame plane_frame;
        plane_frame.header = msg->header;
        plane_frame.up_axis.x = up_axis.x();
        plane_frame.up_axis.y = up_axis.y();
        plane_frame.up_axis.z = up_axis.z();
        plane_frame.p_axis.x = p_axis.x();
        plane_frame.p_axis.y = p_axis.y();
        plane_frame.p_axis.z = p_axis.z();
        plane_frame.q_axis.x = q_axis.x();
        plane_frame.q_axis.y = q_axis.y();
        plane_frame.q_axis.z = q_axis.z();
        plane_frame.center.x = center.x();
        plane_frame.center.y = center.y();
        plane_frame.center.z = center.z();
        plane_frame.corners = corners;

        {
            std::lock_guard<std::mutex> lk(plane_mutex_);
            latest_plane_frame_ = plane_frame;
            has_latest_plane_frame_ = true;
        }
    }

private:
    static bool normalize_vector(Eigen::Vector3d& value) {
        const double norm = value.norm();
        if (norm < 1e-8) {
            return false;
        }
        value /= norm;
        return true;
    }

    static std::vector<double> to_std_vector(const Eigen::Vector3d& value) {
        return {value.x(), value.y(), value.z()};
    }

    /**
     * @brief 计算悬停位姿
     * @date  2026-06-27 
     */
    bool calculate_hover_pose(float distance = 0.2){
        Eigen::Vector3d center(g_plane_center[0], g_plane_center[1], g_plane_center[2]);
        Eigen::Vector3d q_axis(g_q_vector[0], g_q_vector[1], g_q_vector[2]);
        if (q_axis.norm() < 1e-8) {
            RCLCPP_WARN(LOGGER, "calculate_hover_pose: invalid q axis");
            return false;
        }
        q_axis.normalize();
        Eigen::Vector3d hover_position = center + distance * q_axis;
        g_hover_pose.position.x = hover_position.x();
        g_hover_pose.position.y = hover_position.y();
        g_hover_pose.position.z = hover_position.z();

        q_axis = -q_axis;  // EE x-axis points opposite to q_vector

        // Build a right-handed frame with EE x-axis aligned to q_axis.
        const Eigen::Vector3d world_z(0.0, 0.0, 1.0);
        const Eigen::Vector3d world_y(0.0, 1.0, 0.0);
        Eigen::Vector3d aux = (std::abs(q_axis.dot(world_y)) < 0.9) ? world_y : world_z;

        Eigen::Vector3d z_axis = q_axis.cross(aux);
        Eigen::Vector3d y_axis = z_axis.cross(q_axis);

        Eigen::Matrix3d rot;
        rot.col(0) = q_axis;  // EE x-axis
        rot.col(1) = y_axis;
        rot.col(2) = z_axis;

        Eigen::Quaterniond quat(rot);
        quat.normalize();
        g_hover_pose.orientation.x = quat.x();
        g_hover_pose.orientation.y = quat.y();
        g_hover_pose.orientation.z = quat.z();
        g_hover_pose.orientation.w = quat.w();
        return true;
    }

    /**
     * @brief 规划绕P轴旋转的动作
     * @details 沿着旋转轴正方向看过去，逆时针转
     * @date 2026-06-27
     * @todo 可能需要根据p_vector和up_vector的方向调整旋转方向
     */
    bool plan_and_exe_turn_p(geometry_msgs::msg::Pose* final_pose = nullptr){
        const double radius = 0.1;
        const int num_points = 40;

        // 规划半圆轨迹：绕着p_vector转，往up_vector方向转，圆心在up_end - q_vector*radius
        std::vector<geometry_msgs::msg::Pose> waypoints;
        const double angle_range = M_PI / 2;

        Eigen::Vector3d p_axis(g_p_vector[0], g_p_vector[1], g_p_vector[2]);
        Eigen::Vector3d q_axis(g_q_vector[0], g_q_vector[1], g_q_vector[2]);
        Eigen::Vector3d up_axis(g_up_vector[0], g_up_vector[1], g_up_vector[2]);
        if (!normalize_vector(p_axis) || !normalize_vector(q_axis) || !normalize_vector(up_axis)) {
            RCLCPP_WARN(LOGGER, "plan_and_exe_turn_p: invalid turn axes");
            return false;
        }

        Eigen::Vector3d start_pos(g_up_end[0], g_up_end[1], g_up_end[2]);
        Eigen::Vector3d world_center = start_pos - q_axis * radius;
        Eigen::Vector3d radius_vec0 = start_pos - world_center;
        const double angle_sign = (p_axis.cross(radius_vec0).dot(up_axis) >= 0.0) ? 1.0 : -1.0;

        waypoints.reserve(num_points + 1);
        waypoints.push_back(mg_->getCurrentPose().pose);

        for (int i = 1; i <= num_points; ++i) {
            double angle = angle_range * static_cast<double>(i) / static_cast<double>(num_points);
            Eigen::AngleAxisd aa(angle_sign * angle, p_axis);

            Eigen::Vector3d world_point = world_center + aa * radius_vec0;

            geometry_msgs::msg::Pose target_pose;
            target_pose.position.x = world_point.x();
            target_pose.position.y = world_point.y();
            target_pose.position.z = world_point.z();

            // 姿态半圆：q 是末端 -x，p 是末端 -y。
            Eigen::Vector3d x_target = -(aa * q_axis).normalized();
            Eigen::Vector3d y_target = -p_axis;
            Eigen::Vector3d z_target = x_target.cross(y_target).normalized();
            y_target = z_target.cross(x_target).normalized();

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

        if (!waypoints.empty()) {
            if (final_pose != nullptr) {
                *final_pose = waypoints.back();
            }
            publish_target_pose_visualization(waypoints.back(), "target_turn_p", 320);
        }

        publish_turn_waypoints(waypoints);
        
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
        if (fraction < 0.95) {
            RCLCPP_WARN(LOGGER, "Half-circle trajectory planning failed: fraction %.2f", fraction);
            return false;
        }
        if (trajectory.joint_trajectory.points.empty()) {
            RCLCPP_WARN(LOGGER, "Half-circle trajectory planning failed");
            return false;
        }
        vt_->deleteAllMarkers();
        vt_->publishTrajectoryLine(trajectory, jmg_->getLinkModel(mg_->getEndEffectorLink()), jmg_);
        vt_->trigger();

        write_trajectory(trajectory.joint_trajectory);
        return true;
    }

    /**
     * @brief 规划绕Q轴旋转的动作
     * @date 2026-06-27
     * @details gripper x轴旋转，其他轴无所谓
     * @todo 目前绕Q轴旋转的动作规划和绕P轴旋转的动作规划是一样的，可能需要根据实际需求进行调整
     */
    bool plan_and_exe_turn_q(geometry_msgs::msg::Pose* final_pose = nullptr){
        const double radius = 0.1;
        const int num_points = 40;

        // 绕 -q_vector 旋转 30 度，圆心在 up_end - up_vector*radius
        std::vector<geometry_msgs::msg::Pose> waypoints;
        const double angle_range = M_PI / 6;

        Eigen::Vector3d q_axis(g_q_vector[0], g_q_vector[1], g_q_vector[2]);
        Eigen::Vector3d up_axis(g_up_vector[0], g_up_vector[1], g_up_vector[2]);
        if (!normalize_vector(q_axis) || !normalize_vector(up_axis)) {
            RCLCPP_WARN(LOGGER, "plan_and_exe_turn_q: invalid turn axes");
            return false;
        }

        const Eigen::Vector3d rotation_axis = -q_axis;
        Eigen::Vector3d start_pos(g_up_end[0], g_up_end[1], g_up_end[2]);
        Eigen::Vector3d world_center = start_pos - up_axis * radius;
        Eigen::Vector3d radius_vec0 = start_pos - world_center;

        Eigen::Quaterniond start_quat(
            mg_->getCurrentPose().pose.orientation.w,
            mg_->getCurrentPose().pose.orientation.x,
            mg_->getCurrentPose().pose.orientation.y,
            mg_->getCurrentPose().pose.orientation.z);
        start_quat.normalize();
        Eigen::Matrix3d start_rot = start_quat.toRotationMatrix();
        Eigen::Vector3d start_y = start_rot.col(1).normalized();
        Eigen::Vector3d fixed_z = start_rot.col(2).normalized();

        waypoints.reserve(num_points + 1);
        waypoints.push_back(mg_->getCurrentPose().pose);

        for (int i = 1; i <= num_points; ++i) {
            double angle = angle_range * static_cast<double>(i) / static_cast<double>(num_points);
            Eigen::AngleAxisd aa(angle, rotation_axis);
            
            Eigen::Vector3d world_point = world_center + aa * radius_vec0;

            geometry_msgs::msg::Pose target_pose;
            target_pose.position.x = world_point.x();
            target_pose.position.y = world_point.y();
            target_pose.position.z = world_point.z();

            Eigen::Vector3d z_target = fixed_z;
            Eigen::Vector3d y_target = aa * start_y;
            y_target -= y_target.dot(z_target) * z_target;
            if (y_target.norm() < 1e-8) {
                RCLCPP_WARN(LOGGER, "plan_and_exe_turn_q: degenerate waypoint orientation");
                return false;
            }
            y_target.normalize();
            Eigen::Vector3d x_target = y_target.cross(z_target).normalized();
            y_target = z_target.cross(x_target).normalized();

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

        if (!waypoints.empty()) {
            if (final_pose != nullptr) {
                *final_pose = waypoints.back();
            }
            publish_target_pose_visualization(waypoints.back(), "target_turn_q", 330);
        }

        publish_turn_waypoints(waypoints);
        
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
        if (fraction < 0.95) {
            RCLCPP_WARN(LOGGER, "Half-circle trajectory planning failed: fraction %.2f", fraction);
            return false;
        }
        if (trajectory.joint_trajectory.points.empty()) {
            RCLCPP_WARN(LOGGER, "Half-circle trajectory planning failed");
            return false;
        }
        vt_->deleteAllMarkers();
        vt_->publishTrajectoryLine(trajectory, jmg_->getLinkModel(mg_->getEndEffectorLink()), jmg_);
        vt_->trigger();

        write_trajectory(trajectory.joint_trajectory);
        return true;
    }

    void wait_for_enter_key() {
        std::unique_lock<std::mutex> lk(g_key_mutex);
        g_key_cv.wait(lk, []() { return g_last_key == 'N'; });
        g_last_key = 0;
    }

    /**
     * @brief 计算g_p_vector, g_q_vector, g_up_vector
     */
    bool calculate_plane(){
        robo_arm_msgs::msg::PlaneFrame plane_frame;

        {
            std::lock_guard<std::mutex> lk(plane_mutex_);
            if (!has_latest_plane_frame_) {
                RCLCPP_WARN(LOGGER, "calculate_plane: no plane frame received yet");
                return false;
            }
            plane_frame = latest_plane_frame_;
            save_plane_frame_ = plane_frame;
        }

        if (plane_frame.corners.size() < 3) {
            RCLCPP_WARN(
                LOGGER,
                "calculate_plane: expected >=3 corners from marker_reco, got %zu",
                plane_frame.corners.size());
            return false;
        }

        Eigen::Vector3d p_axis(plane_frame.p_axis.x, plane_frame.p_axis.y, plane_frame.p_axis.z);
        Eigen::Vector3d q_axis(plane_frame.q_axis.x, plane_frame.q_axis.y, plane_frame.q_axis.z);
        Eigen::Vector3d up_axis(plane_frame.up_axis.x, plane_frame.up_axis.y, plane_frame.up_axis.z);
        Eigen::Vector3d center(plane_frame.center.x, plane_frame.center.y, plane_frame.center.z);
        std::vector<geometry_msgs::msg::Point32> corners = plane_frame.corners;

        if (!q_axis.allFinite() || !p_axis.allFinite() ||
            !up_axis.allFinite() || !center.allFinite()) {
            RCLCPP_WARN(LOGGER, "calculate_plane: filtered marker_reco frame contains non-finite values");
            return false;
        }

        g_p_vector = to_std_vector(p_axis);
        g_q_vector = to_std_vector(q_axis);
        g_up_vector = to_std_vector(up_axis);
        g_plane_center = to_std_vector(center);

        g_plane_corners.clear();
        g_plane_corners.reserve(corners.size());
        for (const auto& c : corners) {
            if (!std::isfinite(c.x) || !std::isfinite(c.y) || !std::isfinite(c.z)) {
                RCLCPP_WARN(LOGGER, "calculate_plane: filtered marker_reco corner contains non-finite values");
                return false;
            }
            g_plane_corners.push_back({c.x, c.y, c.z});
        }

        RCLCPP_INFO(LOGGER,
            "calculate_plane: center=(%.4f, %.4f, %.4f), p=(%.4f, %.4f, %.4f), q=(%.4f, %.4f, %.4f), up=(%.4f, %.4f, %.4f)",
            g_plane_center[0], g_plane_center[1], g_plane_center[2],
            g_p_vector[0], g_p_vector[1], g_p_vector[2],
            g_q_vector[0], g_q_vector[1], g_q_vector[2],
            g_up_vector[0], g_up_vector[1], g_up_vector[2]);

        return true;
    }

    bool build_incremental_target_pose(
        const std::vector<double>& direction,
        double step_size,
        geometry_msgs::msg::Pose& target_pose)
    {
        auto current_state = mg_->getCurrentState(5.0);
        if (!current_state) {
            RCLCPP_WARN(LOGGER, "build_incremental_target_pose: failed to get current robot state");
            return false;
        }

        const std::string ee = mg_->getEndEffectorLink();
        const Eigen::Isometry3d& tf = current_state->getGlobalLinkTransform(ee);
        target_pose.position.x = tf.translation().x() + direction[0] * step_size;
        target_pose.position.y = tf.translation().y() + direction[1] * step_size;
        target_pose.position.z = tf.translation().z() + direction[2] * step_size;

        Eigen::Quaterniond q(tf.rotation());
        q.normalize();
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();
        return true;
    }

    
    void enter_save(){
        while (mode_ == ControlMode::Save) {
            switch(save_substate_){
                case SaveSubState::CalPlane: {
                    // Demo mode: skip plane estimation and use a fixed fake hover pose.
                    // if (!calculate_plane()) {
                    //     RCLCPP_WARN(LOGGER, "Plane calculation failed. Save flow stopped and switched to Idle.");
                    //     save_substate_ = SaveSubState::CalPlane;
                    //     mode_ = ControlMode::Idle;
                    //     break;
                    // }
                    // publish_plane_visualization(
                    //     g_plane_center,
                    //     g_q_vector,
                    //     g_p_vector,
                    //     g_up_vector);
                    
                    const geometry_msgs::msg::Pose new_pillar_pose = randomPillarPoseInRange();
                    {
                        std::lock_guard<std::mutex> lk(g_env_pose_mutex);
                        g_pillar_pose = new_pillar_pose;
                    }
                    if (!updatePillarCollisionObjectInPlanningScene(new_pillar_pose)) {
                        RCLCPP_WARN(LOGGER, "Save demo: pillar TF moved but planning scene collision object update failed.");
                    }

                    g_plane_center = {
                        new_pillar_pose.position.x,
                        new_pillar_pose.position.y,
                        new_pillar_pose.position.z,
                    };
                    Eigen::Quaterniond hover_quat(
                        new_pillar_pose.orientation.w,
                        new_pillar_pose.orientation.x,
                        new_pillar_pose.orientation.y,
                        new_pillar_pose.orientation.z);

                    hover_quat.normalize();
                    const Eigen::Matrix3d hover_rot = hover_quat.toRotationMatrix();
                    g_q_vector = to_std_vector(-hover_rot.col(0));
                    g_p_vector = to_std_vector(-hover_rot.col(1));
                    g_up_vector = to_std_vector(hover_rot.col(2));
                    RCLCPP_INFO(LOGGER,
                        "Save demo: randomized pillar pose to (%.3f, %.3f, %.3f) and updated hover pose position.",
                        new_pillar_pose.position.x,
                        new_pillar_pose.position.y,
                        new_pillar_pose.position.z);
                    wait_for_enter_key(); //等待N键
                    save_substate_ = SaveSubState::Hover;
                    break;
                }

                case SaveSubState::Hover: {
                    // Demo mode: hover pose is derived from the randomized pillar pose/axes.
                    if (!calculate_hover_pose()) {
                        RCLCPP_WARN(LOGGER, "Hover pose calculation failed. Save flow stopped and switched to Idle.");
                        save_substate_ = SaveSubState::CalPlane;
                        mode_ = ControlMode::Idle;
                        break;
                    }
                    publish_hover_pose_visualization(g_hover_pose);
                    RCLCPP_INFO(LOGGER, "Save demo: hover pose calculated from randomized pillar pose.");

                    wait_for_enter_key();
                    if (!get_energy_unit(0)) {
                        RCLCPP_WARN(LOGGER, "Hover planning/execution failed, fallback to hover IK check.");
                        if (!get_energy_unit_target_ik(g_hover_pose)) {
                            RCLCPP_WARN(LOGGER, "Hover target IK check failed. Save flow stopped and switched to Idle.");
                            save_substate_ = SaveSubState::CalPlane;
                            mode_ = ControlMode::Idle;
                            break;
                        }
                        RCLCPP_INFO(LOGGER, "Hover fallback IK check passed.");
                    } else {
                        RCLCPP_INFO(LOGGER, "Hover planning/execution succeeded.");
                    }

                    wait_for_enter_key();
                    const std::vector<double> insert_dir = {-g_q_vector[0], -g_q_vector[1], -g_q_vector[2]};
                    geometry_msgs::msg::Pose insert_target_pose;
                    if (!build_incremental_target_pose(insert_dir, 0.2, insert_target_pose)) {
                        RCLCPP_WARN(LOGGER, "Insert target pose construction failed. Save flow stopped and switched to Idle.");
                        save_substate_ = SaveSubState::CalPlane;
                        mode_ = ControlMode::Idle;
                        break;
                    }

                    if (!plan_incremental_motion(insert_dir, 0.2)) {
                        RCLCPP_WARN(LOGGER, "Insert planning failed, fallback to target IK check.");
                        if (!get_energy_unit_target_ik(insert_target_pose)) {
                            RCLCPP_WARN(LOGGER, "Insert fallback IK check failed. Save flow stopped and switched to Idle.");
                            save_substate_ = SaveSubState::CalPlane;
                            mode_ = ControlMode::Idle;
                            break;
                        }
                        RCLCPP_INFO(LOGGER, "Insert fallback IK check passed.");
                    } else {
                        RCLCPP_INFO(LOGGER, "Finish Planning and Executing Insert...");
                    }

                    save_substate_ = SaveSubState::Slide;
                    break;
                }

                case SaveSubState::Slide: {
                    const std::vector<double> slide_dir = {g_up_vector[0], g_up_vector[1], g_up_vector[2]};
                    geometry_msgs::msg::Pose slide_target_pose;
                    wait_for_enter_key();
                    if (!build_incremental_target_pose(slide_dir, 0.1, slide_target_pose)) {
                        RCLCPP_WARN(LOGGER, "Slide target pose construction failed. Save flow stopped and switched to Idle.");
                        save_substate_ = SaveSubState::CalPlane;
                        mode_ = ControlMode::Idle;
                        break;
                    }

                    if (!plan_incremental_motion(slide_dir, 0.1)) {
                        RCLCPP_WARN(LOGGER, "Slide planning failed, fallback to target IK check.");
                        if (!get_energy_unit_target_ik(slide_target_pose)) {
                            RCLCPP_WARN(LOGGER, "Slide fallback IK check failed. Save flow stopped and switched to Idle.");
                            save_substate_ = SaveSubState::CalPlane;
                            mode_ = ControlMode::Idle;
                            break;
                        }
                        RCLCPP_INFO(LOGGER, "Slide fallback IK check passed.");
                    } else {
                        RCLCPP_INFO(LOGGER, "Finish Planning and Executing Slide...");
                    }

                    wait_for_enter_key();
                    save_substate_ = SaveSubState::Turn_P;
                    break;
                }

                case SaveSubState::Turn_P: {
                    auto current_state = mg_->getCurrentState(5.0);
                    const Eigen::Isometry3d& tf = current_state->getGlobalLinkTransform(mg_->getEndEffectorLink());
                    g_up_end = {tf.translation().x(), tf.translation().y(), tf.translation().z()};
                    publish_turn_vectors(g_up_end);

                    geometry_msgs::msg::Pose turn_p_target_pose;
                    if (!plan_and_exe_turn_p(&turn_p_target_pose)) {
                        RCLCPP_WARN(LOGGER, "Turn around P axis failed, fallback to target IK check.");
                        if (!get_energy_unit_target_ik(turn_p_target_pose)) {
                            RCLCPP_WARN(LOGGER, "Turn P fallback IK check failed. Save flow stopped and switched to Idle.");
                            save_substate_ = SaveSubState::CalPlane;
                            mode_ = ControlMode::Idle;
                            break;
                        }
                        RCLCPP_INFO(LOGGER, "Turn P fallback IK check passed.");
                    } else {
                        RCLCPP_INFO(LOGGER, "Finish Planning and Executing Turn around P axis...");
                    }

                    wait_for_enter_key();
                    save_substate_ = SaveSubState::Turn_Q;
                    break;
                }

                case SaveSubState::Turn_Q: {
                    auto current_state = mg_->getCurrentState(5.0);
                    const Eigen::Isometry3d& tf = current_state->getGlobalLinkTransform(mg_->getEndEffectorLink());
                    g_up_end = {tf.translation().x(), tf.translation().y(), tf.translation().z()};
                    publish_turn_vectors(g_up_end);

                    geometry_msgs::msg::Pose turn_q_target_pose;
                    if (!plan_and_exe_turn_q(&turn_q_target_pose)) {
                        RCLCPP_WARN(LOGGER, "Turn around Q axis failed, fallback to target IK check.");
                        if (!get_energy_unit_target_ik(turn_q_target_pose)) {
                            RCLCPP_WARN(LOGGER, "Turn Q fallback IK check failed. Save flow stopped and switched to Idle.");
                            save_substate_ = SaveSubState::CalPlane;
                            mode_ = ControlMode::Idle;
                            break;
                        }
                        RCLCPP_INFO(LOGGER, "Turn Q fallback IK check passed.");
                    } else {
                        RCLCPP_INFO(LOGGER, "Finish Planning and Executing Turn around Q axis...");
                    }

                    wait_for_enter_key();
                    save_substate_ = SaveSubState::CalPlane;
                    mode_ = ControlMode::Idle;
                    break;
                }
            }
        }

        if (mode_ == ControlMode::Idle) {
            RCLCPP_INFO(LOGGER, "Save operation completed or failed. Switched to global Idle.");
        }
    }
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
                RCLCPP_WARN(
                    LOGGER,
                    "Failed to plan smooth transition (code=%d). "
                    "Likely start-state collision; fallback to direct sync.",
                    static_cast<int>(rc.val));
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

    void publish_plane_visualization(const std::vector<double>& center, const std::vector<double>& q_axis, const std::vector<double>& p_axis, const std::vector<double>& up_axis){
        if (!plane_pub_ || center.size() < 3 || q_axis.size() < 3 || p_axis.size() < 3 || up_axis.size() < 3) {
            return;
        }

        const bool has_four_corners =
            g_plane_corners.size() >= 4 &&
            g_plane_corners[0].size() >= 3 &&
            g_plane_corners[1].size() >= 3 &&
            g_plane_corners[2].size() >= 3 &&
            g_plane_corners[3].size() >= 3;

        if (!has_four_corners) {
            RCLCPP_WARN(
                LOGGER,
                "publish_plane_visualization: corners <4, skip corner markers and keep axis markers only (got %zu)",
                g_plane_corners.size());
        }

        const auto stamp = node_->now();

        geometry_msgs::msg::Point center_point;
        center_point.x = center[0];
        center_point.y = center[1];
        center_point.z = center[2];

        auto make_arrow = [this, &center_point, stamp](
            int id,
            const std::vector<double>& axis,
            float r,
            float g,
            float b,
            double length,
            const std::string& ns)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "base_link";
            marker.header.stamp = stamp;
            marker.ns = ns;
            marker.id = id;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.01;
            marker.scale.y = 0.02;
            marker.scale.z = 0.02;
            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
            marker.color.a = 1.0f;
            marker.lifetime = rclcpp::Duration::from_seconds(0.0);
            marker.frame_locked = true;

            Eigen::Vector3d dir(axis[0], axis[1], axis[2]);
            if (dir.norm() < 1e-8) {
                marker.action = visualization_msgs::msg::Marker::DELETE;
                return marker;
            }
            dir.normalize();
            dir *= length;

            geometry_msgs::msg::Point end_point;
            end_point.x = center_point.x + dir.x();
            end_point.y = center_point.y + dir.y();
            end_point.z = center_point.z + dir.z();

            marker.points.push_back(center_point);
            marker.points.push_back(end_point);
            return marker;
        };

        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker center_marker;
        center_marker.header.frame_id = "base_link";
        center_marker.header.stamp = stamp;
        center_marker.ns = "plane_center";
        center_marker.id = 0;
        center_marker.type = visualization_msgs::msg::Marker::SPHERE;
        center_marker.action = visualization_msgs::msg::Marker::ADD;
        center_marker.pose.position = center_point;
        center_marker.pose.orientation.w = 1.0;
        center_marker.scale.x = 0.015;
        center_marker.scale.y = 0.015;
        center_marker.scale.z = 0.015;
        center_marker.color.r = 1.0f;
        center_marker.color.g = 1.0f;
        center_marker.color.b = 0.0f;
        center_marker.color.a = 1.0f;
        center_marker.lifetime = rclcpp::Duration::from_seconds(0.0);
        center_marker.frame_locked = true;
        
        marker_array.markers.push_back(center_marker);
        if (has_four_corners) {
            geometry_msgs::msg::Point corner_0;
            geometry_msgs::msg::Point corner_1;
            geometry_msgs::msg::Point corner_2;
            geometry_msgs::msg::Point corner_3;
            corner_0.x = g_plane_corners[0][0];
            corner_0.y = g_plane_corners[0][1];
            corner_0.z = g_plane_corners[0][2];

            corner_1.x = g_plane_corners[1][0];
            corner_1.y = g_plane_corners[1][1];
            corner_1.z = g_plane_corners[1][2];

            corner_2.x = g_plane_corners[2][0];
            corner_2.y = g_plane_corners[2][1];
            corner_2.z = g_plane_corners[2][2];

            corner_3.x = g_plane_corners[3][0];
            corner_3.y = g_plane_corners[3][1];
            corner_3.z = g_plane_corners[3][2];

            visualization_msgs::msg::Marker corner_0_marker = center_marker;
            corner_0_marker.id = 1;
            corner_0_marker.pose.position = corner_0;

            visualization_msgs::msg::Marker corner_1_marker = center_marker;
            corner_1_marker.id = 2;
            corner_1_marker.pose.position = corner_1;

            visualization_msgs::msg::Marker corner_2_marker = center_marker;
            corner_2_marker.id = 3;
            corner_2_marker.pose.position = corner_2;

            visualization_msgs::msg::Marker corner_3_marker = center_marker;
            corner_3_marker.id = 4;
            corner_3_marker.pose.position = corner_3;

            marker_array.markers.push_back(corner_0_marker);
            marker_array.markers.push_back(corner_1_marker);
            marker_array.markers.push_back(corner_2_marker);
            marker_array.markers.push_back(corner_3_marker);
        }
        marker_array.markers.push_back(make_arrow(1, q_axis, 1.0f, 0.2f, 0.1f, 0.12, "plane_q_axis"));
        marker_array.markers.push_back(make_arrow(2, p_axis, 0.1f, 1.0f, 0.1f, 0.10, "plane_p_axis"));
        marker_array.markers.push_back(make_arrow(3, up_axis, 0.1f, 0.4f, 1.0f, 0.10, "plane_up_axis"));

        plane_pub_->publish(marker_array);
    }

    void publish_turn_vectors(const std::vector<double>& origin) {
        if (!marker_pub_ || origin.size() < 3) {
            return;
        }

        auto make_arrow = [this, &origin](
            int id,
            const std::vector<double>& direction,
            float r,
            float g,
            float b,
            double length,
            const std::string& label)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "base_link";
            marker.header.stamp = node_->now();
            marker.ns = "turn_vectors";
            marker.id = id;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.scale.x = 0.01;
            marker.scale.y = 0.02;
            marker.scale.z = 0.02;

            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
            marker.color.a = 0.95f;

            marker.lifetime = rclcpp::Duration::from_seconds(0.0);
            marker.frame_locked = true;
            marker.text = label;

            if (direction.size() < 3) {
                marker.action = visualization_msgs::msg::Marker::DELETE;
                return marker;
            }

            geometry_msgs::msg::Point start;
            start.x = origin[0];
            start.y = origin[1];
            start.z = origin[2];

            Eigen::Vector3d dir(direction[0], direction[1], direction[2]);
            if (dir.norm() < 1e-8) {
                marker.action = visualization_msgs::msg::Marker::DELETE;
                return marker;
            }
            dir.normalize();
            dir *= length;

            geometry_msgs::msg::Point end;
            end.x = start.x + dir.x();
            end.y = start.y + dir.y();
            end.z = start.z + dir.z();

            marker.points.push_back(start);
            marker.points.push_back(end);
            return marker;
        };

        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.reserve(3);
        marker_array.markers.push_back(make_arrow(0, g_up_vector, 1.0f, 0.2f, 0.2f, 0.12, "up"));
        marker_array.markers.push_back(make_arrow(1, g_p_vector, 0.2f, 1.0f, 0.2f, 0.12, "p"));
        marker_array.markers.push_back(make_arrow(2, g_q_vector, 0.2f, 0.2f, 1.0f, 0.12, "q"));

        marker_pub_->publish(marker_array);
    }

    void publish_turn_waypoints(const std::vector<geometry_msgs::msg::Pose>& waypoints) {
        if (!marker_pub_ || waypoints.empty()) {
            return;
        }

        visualization_msgs::msg::Marker points_marker;
        points_marker.header.frame_id = "base_link";
        points_marker.header.stamp = node_->now();
        points_marker.ns = "turn_waypoints";
        points_marker.id = 10;
        points_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        points_marker.action = visualization_msgs::msg::Marker::ADD;
        points_marker.scale.x = 0.008;
        points_marker.scale.y = 0.008;
        points_marker.scale.z = 0.008;
        points_marker.color.r = 1.0f;
        points_marker.color.g = 0.9f;
        points_marker.color.b = 0.2f;
        points_marker.color.a = 0.95f;
        points_marker.lifetime = rclcpp::Duration::from_seconds(0.0);
        points_marker.frame_locked = true;

        visualization_msgs::msg::Marker line_marker;
        line_marker.header = points_marker.header;
        line_marker.ns = "turn_waypoints";
        line_marker.id = 11;
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        line_marker.scale.x = 0.003;
        line_marker.color.r = 1.0f;
        line_marker.color.g = 0.5f;
        line_marker.color.b = 0.0f;
        line_marker.color.a = 0.9f;
        line_marker.lifetime = rclcpp::Duration::from_seconds(0.0);
        line_marker.frame_locked = true;

        visualization_msgs::msg::Marker x_axis_marker;
        x_axis_marker.header = points_marker.header;
        x_axis_marker.ns = "turn_waypoints";
        x_axis_marker.id = 12;
        x_axis_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        x_axis_marker.action = visualization_msgs::msg::Marker::ADD;
        x_axis_marker.scale.x = 0.002;
        x_axis_marker.color.r = 1.0f;
        x_axis_marker.color.g = 0.2f;
        x_axis_marker.color.b = 0.2f;
        x_axis_marker.color.a = 0.95f;
        x_axis_marker.lifetime = rclcpp::Duration::from_seconds(0.0);
        x_axis_marker.frame_locked = true;

        visualization_msgs::msg::Marker y_axis_marker;
        y_axis_marker.header = points_marker.header;
        y_axis_marker.ns = "turn_waypoints";
        y_axis_marker.id = 13;
        y_axis_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        y_axis_marker.action = visualization_msgs::msg::Marker::ADD;
        y_axis_marker.scale.x = 0.002;
        y_axis_marker.color.r = 0.2f;
        y_axis_marker.color.g = 1.0f;
        y_axis_marker.color.b = 0.2f;
        y_axis_marker.color.a = 0.95f;
        y_axis_marker.lifetime = rclcpp::Duration::from_seconds(0.0);
        y_axis_marker.frame_locked = true;

        visualization_msgs::msg::Marker z_axis_marker;
        z_axis_marker.header = points_marker.header;
        z_axis_marker.ns = "turn_waypoints";
        z_axis_marker.id = 14;
        z_axis_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        z_axis_marker.action = visualization_msgs::msg::Marker::ADD;
        z_axis_marker.scale.x = 0.002;
        z_axis_marker.color.r = 0.2f;
        z_axis_marker.color.g = 0.2f;
        z_axis_marker.color.b = 1.0f;
        z_axis_marker.color.a = 0.95f;
        z_axis_marker.lifetime = rclcpp::Duration::from_seconds(0.0);
        z_axis_marker.frame_locked = true;

        const double axis_length = 0.02;

        points_marker.points.reserve(waypoints.size());
        line_marker.points.reserve(waypoints.size());
        x_axis_marker.points.reserve(waypoints.size() * 2);
        y_axis_marker.points.reserve(waypoints.size() * 2);
        z_axis_marker.points.reserve(waypoints.size() * 2);
        for (const auto& pose : waypoints) {
            geometry_msgs::msg::Point p;
            p.x = pose.position.x;
            p.y = pose.position.y;
            p.z = pose.position.z;
            points_marker.points.push_back(p);
            line_marker.points.push_back(p);

            Eigen::Quaterniond q(
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z);
            q.normalize();
            const Eigen::Matrix3d rot = q.toRotationMatrix();

            geometry_msgs::msg::Point x_end;
            x_end.x = p.x + axis_length * rot(0, 0);
            x_end.y = p.y + axis_length * rot(1, 0);
            x_end.z = p.z + axis_length * rot(2, 0);
            x_axis_marker.points.push_back(p);
            x_axis_marker.points.push_back(x_end);

            geometry_msgs::msg::Point y_end;
            y_end.x = p.x + axis_length * rot(0, 1);
            y_end.y = p.y + axis_length * rot(1, 1);
            y_end.z = p.z + axis_length * rot(2, 1);
            y_axis_marker.points.push_back(p);
            y_axis_marker.points.push_back(y_end);

            geometry_msgs::msg::Point z_end;
            z_end.x = p.x + axis_length * rot(0, 2);
            z_end.y = p.y + axis_length * rot(1, 2);
            z_end.z = p.z + axis_length * rot(2, 2);
            z_axis_marker.points.push_back(p);
            z_axis_marker.points.push_back(z_end);
        }

        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.push_back(points_marker);
        marker_array.markers.push_back(line_marker);
        marker_array.markers.push_back(x_axis_marker);
        marker_array.markers.push_back(y_axis_marker);
        marker_array.markers.push_back(z_axis_marker);
        marker_pub_->publish(marker_array);
    }

    void publish_target_pose_visualization(
        const geometry_msgs::msg::Pose& target_pose,
        const std::string& ns,
        int marker_id_base = 300)
    {
        if (!plane_pub_) {
            return;
        }

        Eigen::Quaterniond quat(
            target_pose.orientation.w,
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z);
        if (quat.norm() < 1e-8) {
            RCLCPP_WARN(LOGGER, "publish_target_pose_visualization: invalid target quaternion (ns=%s)", ns.c_str());
            return;
        }
        quat.normalize();
        const Eigen::Matrix3d rot = quat.toRotationMatrix();
        const auto stamp = node_->now();

        geometry_msgs::msg::Point origin;
        origin.x = target_pose.position.x;
        origin.y = target_pose.position.y;
        origin.z = target_pose.position.z;

        auto make_axis_arrow = [stamp, &origin, &ns](
            int id,
            const Eigen::Vector3d& axis,
            float r,
            float g,
            float b,
            double length)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "base_link";
            marker.header.stamp = stamp;
            marker.ns = ns;
            marker.id = id;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.005;
            marker.scale.y = 0.010;
            marker.scale.z = 0.010;
            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
            marker.color.a = 0.95f;
            marker.lifetime = rclcpp::Duration::from_seconds(0.0);
            marker.frame_locked = true;

            geometry_msgs::msg::Point end;
            end.x = origin.x + axis.x() * length;
            end.y = origin.y + axis.y() * length;
            end.z = origin.z + axis.z() * length;
            marker.points.push_back(origin);
            marker.points.push_back(end);
            return marker;
        };

        visualization_msgs::msg::Marker pose_marker;
        pose_marker.header.frame_id = "base_link";
        pose_marker.header.stamp = stamp;
        pose_marker.ns = ns;
        pose_marker.id = marker_id_base;
        pose_marker.type = visualization_msgs::msg::Marker::SPHERE;
        pose_marker.action = visualization_msgs::msg::Marker::ADD;
        pose_marker.pose.position = origin;
        pose_marker.pose.orientation.w = 1.0;
        pose_marker.scale.x = 0.016;
        pose_marker.scale.y = 0.016;
        pose_marker.scale.z = 0.016;
        pose_marker.color.r = 1.0f;
        pose_marker.color.g = 0.4f;
        pose_marker.color.b = 0.1f;
        pose_marker.color.a = 1.0f;
        pose_marker.lifetime = rclcpp::Duration::from_seconds(0.0);
        pose_marker.frame_locked = true;

        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.reserve(4);
        marker_array.markers.push_back(pose_marker);
        marker_array.markers.push_back(make_axis_arrow(marker_id_base + 1, rot.col(0), 1.0f, 0.2f, 0.2f, 0.07));
        marker_array.markers.push_back(make_axis_arrow(marker_id_base + 2, rot.col(1), 0.2f, 1.0f, 0.2f, 0.06));
        marker_array.markers.push_back(make_axis_arrow(marker_id_base + 3, rot.col(2), 0.2f, 0.3f, 1.0f, 0.06));
        plane_pub_->publish(marker_array);
    }

    void publish_hover_pose_visualization(const geometry_msgs::msg::Pose& hover_pose) {
        if (!plane_pub_) {
            return;
        }

        const auto stamp = node_->now();

        geometry_msgs::msg::Point origin;
        origin.x = hover_pose.position.x;
        origin.y = hover_pose.position.y;
        origin.z = hover_pose.position.z;

        Eigen::Quaterniond quat(
            hover_pose.orientation.w,
            hover_pose.orientation.x,
            hover_pose.orientation.y,
            hover_pose.orientation.z);
        if (quat.norm() < 1e-8) {
            RCLCPP_WARN(LOGGER, "publish_hover_pose_visualization: invalid hover quaternion");
            return;
        }
        quat.normalize();
        const Eigen::Matrix3d rot = quat.toRotationMatrix();

        auto make_axis_arrow = [stamp, &origin](
            int id,
            const Eigen::Vector3d& axis,
            float r,
            float g,
            float b,
            double length,
            const std::string& ns)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = stamp;
            marker.ns = ns;
            marker.id = id;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.006;
            marker.scale.y = 0.012;
            marker.scale.z = 0.012;
            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
            marker.color.a = 0.95f;
            marker.lifetime = rclcpp::Duration::from_seconds(0.0);
            marker.frame_locked = true;

            geometry_msgs::msg::Point end;
            end.x = origin.x + axis.x() * length;
            end.y = origin.y + axis.y() * length;
            end.z = origin.z + axis.z() * length;

            marker.points.push_back(origin);
            marker.points.push_back(end);
            return marker;
        };

        visualization_msgs::msg::Marker pose_marker;
        pose_marker.header.frame_id = "world";
        pose_marker.header.stamp = stamp;
        pose_marker.ns = "hover_pose";
        pose_marker.id = 90;
        pose_marker.type = visualization_msgs::msg::Marker::SPHERE;
        pose_marker.action = visualization_msgs::msg::Marker::ADD;
        pose_marker.pose.position = origin;
        pose_marker.pose.orientation.w = 1.0;
        pose_marker.scale.x = 0.018;
        pose_marker.scale.y = 0.018;
        pose_marker.scale.z = 0.018;
        pose_marker.color.r = 1.0f;
        pose_marker.color.g = 0.7f;
        pose_marker.color.b = 0.0f;
        pose_marker.color.a = 1.0f;
        pose_marker.lifetime = rclcpp::Duration::from_seconds(0.0);
        pose_marker.frame_locked = true;

        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.reserve(4);
        marker_array.markers.push_back(pose_marker);
        marker_array.markers.push_back(make_axis_arrow(91, rot.col(0), 1.0f, 0.2f, 0.2f, 0.08, "hover_pose_axis_x"));
        marker_array.markers.push_back(make_axis_arrow(92, rot.col(1), 0.2f, 1.0f, 0.2f, 0.07, "hover_pose_axis_y"));
        marker_array.markers.push_back(make_axis_arrow(93, rot.col(2), 0.2f, 0.3f, 1.0f, 0.07, "hover_pose_axis_z"));

        plane_pub_->publish(marker_array);
    }

    void plan_to_save_pose() {
        RCLCPP_INFO(LOGGER, "Planning to save_pose");
        
        mg_->setStartStateToCurrentState();
        
        // 尝试规划，如果失败则在保持x轴朝向不变的情况下迭代改变y、z轴朝向
        const int MAX_ATTEMPTS = 10;
        const double ANGLE_STEP = M_PI / 18;  // 10度为单位进行旋转
        
        geometry_msgs::msg::Pose original_target = g_hover_pose;
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

            publish_target_pose_visualization(target_pose, "target_save_pose", 340);
            
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
    
    /**
     * @brief 对目标位置求IK，探究规划失败的原因
     */
    bool get_energy_unit_target_ik(const geometry_msgs::msg::Pose& target_pose){

        publish_target_pose_visualization(target_pose, "target_energy_unit_ik", 350);

        if (!pyroki_ik_client_) {
            RCLCPP_ERROR(LOGGER, "pyroki IK client is not initialized");
            return false;
        }

        if (!pyroki_ik_client_->wait_for_service(std::chrono::seconds(3))) {
            RCLCPP_WARN(LOGGER, "IK service /pyroki_get_position_ik is unavailable");
            return false;
        }

        auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
        request->ik_request.group_name = PLANNING_GROUP;
        request->ik_request.ik_link_name = mg_->getEndEffectorLink();
        request->ik_request.pose_stamped.header.frame_id = "base_link";
        request->ik_request.pose_stamped.header.stamp = node_->now();
        request->ik_request.pose_stamped.pose = target_pose;
        request->ik_request.avoid_collisions = false;
        request->ik_request.timeout.sec = 3;
        request->ik_request.timeout.nanosec = 0;

        const auto current_joint_values = mg_->getCurrentJointValues();
        request->ik_request.robot_state.joint_state.name = JOINT_NAMES;
        request->ik_request.robot_state.joint_state.position = current_joint_values;

        RCLCPP_INFO(LOGGER,
            "IK check request sent for target pose: pos=(%.4f, %.4f, %.4f)",
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z);

        auto future = pyroki_ik_client_->async_send_request(request);
        const auto status = future.wait_for(std::chrono::seconds(10));
        if (status != std::future_status::ready) {
            RCLCPP_WARN(LOGGER, "IK service call timeout for target pose");
            return false;
        }

        const auto response = future.get();
        if (!response) {
            RCLCPP_WARN(LOGGER, "IK service returned null response for target pose");
            return false;
        }

        if (response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            RCLCPP_WARN(LOGGER,
                "IK check failed for target pose (error_code=%d)",
                response->error_code.val);
            return false;
        }

        std::vector<double> ik_solution;
        ik_solution.resize(JOINT_NAMES.size(), 0.0);
        const auto& names = response->solution.joint_state.name;
        const auto& positions = response->solution.joint_state.position;
        for (size_t i = 0; i < JOINT_NAMES.size(); ++i) {
            for (size_t j = 0; j < names.size() && j < positions.size(); ++j) {
                if (names[j] == JOINT_NAMES[i]) {
                    ik_solution[i] = positions[j];
                    break;
                }
            }
        }

        RCLCPP_INFO(LOGGER,
            "IK check passed for target pose: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
            ik_solution[0], ik_solution[1], ik_solution[2],
            ik_solution[3], ik_solution[4], ik_solution[5]);

        // 仅做可视化预览：把IK结果写入g_arm_state，由/joint_states发布到RViz。
        {
            std::lock_guard<std::mutex> lk(g_js_mutex);
            g_arm_state = ik_solution;
        }
        RCLCPP_INFO(LOGGER, "IK preview applied to /joint_states for target pose");
        return true;

    }
    
    bool get_energy_unit(int unit_id) {     
        geometry_msgs::msg::Pose target_pose;
        
        switch (unit_id) {
            case 0: target_pose = g_hover_pose; break;
            case 1: target_pose = g_energy_unit_1_pose; break;
            case 2: target_pose = g_energy_unit_2_pose; break;
            case 3: target_pose = g_energy_unit_3_pose; break;
            case 4: target_pose = g_energy_unit_4_pose; break;
            case 5: target_pose = g_energy_unit_5_pose; break;
            case 6: target_pose = g_energy_unit_6_pose; break;
        }

        publish_target_pose_visualization(target_pose, "target_energy_unit_plan", 360);
        
        RCLCPP_INFO(LOGGER, "Planning to energy_unit_%d", unit_id);
        
        mg_->setStartStateToCurrentState();
        
        const bool target_ok = mg_->setPoseTarget(target_pose, mg_->getEndEffectorLink());
        if (!target_ok) {
            RCLCPP_WARN(LOGGER,
                "setPoseTarget failed for energy_unit_%d (likely IK target setup failure). "
                "target_pose: pos=(%.4f, %.4f, %.4f), quat=(%.4f, %.4f, %.4f, %.4f)",
                unit_id,
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z,
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.z,
                target_pose.orientation.w);
            return false;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::core::MoveItErrorCode rc = mg_-> plan(plan);
        if (rc != moveit::core::MoveItErrorCode::SUCCESS || plan.trajectory.joint_trajectory.points.empty()) {
            RCLCPP_WARN(LOGGER, "Planning to energy_unit_%d failed (code=%d, error='%s', message='%s', source='%s'). "
                "target_pose: pos=(%.4f, %.4f, %.4f), quat=(%.4f, %.4f, %.4f, %.4f)",
                unit_id,
                static_cast<int>(rc.val),
                moveit::core::errorCodeToString(rc).c_str(),
                rc.message.c_str(),
                rc.source.c_str(),
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z,
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.z,
                target_pose.orientation.w);
            mg_->clearPoseTargets();
            return false;
        }
        
        vt_->deleteAllMarkers();
        vt_->publishTrajectoryLine(plan.trajectory, jmg_->getLinkModel(mg_->getEndEffectorLink()), jmg_);
        vt_->trigger();
        mg_->clearPoseTargets();
        appendTrajectoryToFile(extractTrajectory(plan.trajectory));
        write_trajectory(plan.trajectory.joint_trajectory);
        return true;
    }
    /**
     * @brief 增量运动规划：规划沿给定方向移动末端执行器并写入话题（串口）去执行
     */
    bool plan_incremental_motion(const std::vector<double>& direction, double step_size = 0.05) {
        
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

        publish_target_pose_visualization(target, "target_incremental", 370);
        
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
            return false;
        }
        if (trajectory.joint_trajectory.points.empty()) {
            RCLCPP_WARN(LOGGER, "Cartesian path planning for incremental motion failed: empty trajectory");
            return false;
        }
        
        vt_->deleteAllMarkers();
        vt_->publishTrajectoryLine(trajectory, jmg_->getLinkModel(ee), jmg_);
        vt_->trigger();
        RCLCPP_INFO(LOGGER, "Planned incremental motion with %zu points (fraction: %.2f)", 
                    trajectory.joint_trajectory.points.size(), fraction);
        appendTrajectoryToFile(extractTrajectory(trajectory));
        write_trajectory(trajectory.joint_trajectory);
        return true;
    }

    /**
     * @brief 虚假的增量运动规划，仅用于做状态机demo
     */
    
    
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
        
        Eigen::Vector3d recorded_pos(g_up_start[0], g_up_start[1], g_up_start[2]);

        Eigen::Vector3d constrain_dir_normalized = 
            Eigen::Vector3d(g_q_vector[0], g_q_vector[1], g_q_vector[2]).normalized();
        
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
    auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("/turn_vectors", 10);
    auto plane_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("/plane_visualization", 10);
    auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    rclcpp::SubscriptionOptions io_sub_opts;
    io_sub_opts.callback_group = io_cb_group;

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
        io_sub_opts);

    auto marker_plane_sub = node->create_subscription<robo_arm_msgs::msg::PlaneFrame>(
        "/marker_reco/filtered/plane", 10,
        [](const robo_arm_msgs::msg::PlaneFrame::SharedPtr msg) {
            if (g_state_machine) {
                g_state_machine->on_plane_frame(msg);
            }
        },
        io_sub_opts);

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
    // mg->setPoseReferenceFrame("base_link");
    mg->setEndEffectorLink("gripper");
    mg->setPlanningPipelineId("ompl");
    mg->setPlannerId("RRTConnect");
    mg->setPoseReferenceFrame("world");
    RCLCPP_INFO(LOGGER, "MoveIt pose reference frame: %s", mg->getPoseReferenceFrame().c_str());
    RCLCPP_INFO(LOGGER, "MoveIt planning frame: %s", mg->getPlanningFrame().c_str());

    auto env_tf_timer = node->create_wall_timer(
        std::chrono::milliseconds(50),
        [node, tf_broadcaster]() {
            publishEnvironmentTF(node, tf_broadcaster);
        },
        io_cb_group);
    publishEnvironmentTF(node, tf_broadcaster);
    
    try {
        const std::filesystem::path workspace_root = "/home/hku/robo_arm/src/arms";
        const std::filesystem::path ground_mesh = workspace_root / "reduced_ground/meshes/ground_link.STL";
        const std::filesystem::path pillar_mesh = workspace_root / "reduced_pillar/meshes/pillar_link.STL";

        geometry_msgs::msg::Pose ground_pose_snapshot;
        geometry_msgs::msg::Pose pillar_pose_snapshot;
        {
            std::lock_guard<std::mutex> lk(g_env_pose_mutex);
            ground_pose_snapshot = g_ground_pose;
            pillar_pose_snapshot = g_pillar_pose;
        }

        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        collision_objects.push_back(makeCollisionObject(
            "reduced_ground",
            toFileUri(ground_mesh),
            WORLD_FRAME,
            ground_pose_snapshot));
        collision_objects.push_back(makeCollisionObject(
            "reduced_pillar",
            toFileUri(pillar_mesh),
            WORLD_FRAME,
            pillar_pose_snapshot));
        planning_scene_interface.applyCollisionObjects(collision_objects);
        RCLCPP_INFO(LOGGER, "Added reduced_ground and reduced_pillar as collision objects to planning scene");
    } catch (const std::exception& ex) {
        RCLCPP_WARN(LOGGER, "Failed to add environment collision objects: %s", ex.what());
    }

    // const auto* jmg = mg->getRobotModel()->getJointModelGroup(PLANNING_GROUP);
    const moveit::core::JointModelGroup* jmg =
        mg->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    auto vt = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
        node, "base_link", "flow_with_pyroki", mg->getRobotModel());
    vt->deleteAllMarkers();

    // 初始化状态机
    g_state_machine = std::make_unique<ControlStateMachine>(node, mg, jmg, vt, js_pub, marker_pub, plane_pub);
    g_state_machine->start();
    
    RCLCPP_INFO(LOGGER, "Ready. Keys: G(Following) W/S/A/D/E/Q(Base) I/K(EE) R(UpRestrain) T(Turn) 1-6(Energy) X(Quit)");

    // 键盘输入订阅
    rclcpp::SubscriptionOptions key_sub_opts;
    key_sub_opts.callback_group = control_cb_group;

    auto key_sub = node->create_subscription<std_msgs::msg::String>(
        "/key_input", 10,
        [](const std_msgs::msg::String::SharedPtr msg) {
            if (msg->data.empty()) return;
            {
                std::lock_guard<std::mutex> lk(g_key_mutex);
                g_last_key = msg->data[0];
            }
            g_key_cv.notify_all();
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
                case 'Z':
                    RCLCPP_INFO(LOGGER, "Switched to Z mode");
                    g_state_machine->switch_to(ControlMode::Save);
                    break;

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