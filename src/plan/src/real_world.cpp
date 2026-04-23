#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Geometry>
#include <moveit/robot_state/conversions.hpp>
#include <chrono>
#include <thread>
#include <map>
#include <vector>
#include <cmath>
#include <mutex>
#include "utils.hpp"
#include "usb_data.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("z_plan_real_world");

enum State { 
    IDLE, 
    PLANNING,
    SENDING,
    CHECKING,
};
State current_state = IDLE;
geometry_msgs::msg::Pose last_target_pose;
bool planning_command_received = true;  // 伪代码,假设一开始就收到规划指令

std::vector<std::vector<float>> current_trajectory;
size_t trajectory_index = 0;
std::vector<float> last_sent_angles;
std::chrono::steady_clock::time_point last_send_time;
// Latest joint state and computed torques (protected by mutex)
std::mutex js_mutex;
std::vector<double> latest_positions;
std::vector<float> latest_torques;
// 角度低通滤波器状态
std::vector<double> filtered_angles(6, 0.0);
bool filter_initialized = false;
const double ANGLE_FILTER_ALPHA = 0.3;  // 低通滤波系数（0-1，越小越平滑）

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("z_plan_real_world", node_options);

    auto serial_node = std::make_shared<SerialRxNode>();
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    executor.add_node(serial_node);
    
    std::thread receive_thread([serial_node]() {
        serial_node->receive_data();
    });

    static const std::string PLANNING_GROUP = "arm";
    
    RCLCPP_INFO(LOGGER, "Initializing MoveGroupInterface...");
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    
    moveit::core::RobotModelConstPtr robot_model = move_group.getRobotModel();
    if (!robot_model) {
        RCLCPP_ERROR(LOGGER, "Failed to get robot model!");
        rclcpp::shutdown();
        return -1;
    }
    
    const moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup(PLANNING_GROUP);
    if (!joint_model_group) {
        RCLCPP_ERROR(LOGGER, "Failed to get joint model group: %s", PLANNING_GROUP.c_str());
        rclcpp::shutdown();
        return -1;
    }
    
    const std::string end_effector_link = move_group.getEndEffectorLink();
    const std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();
    
    RCLCPP_INFO(LOGGER, "Planning group: %s, End effector: %s, Joints: %zu", 
                PLANNING_GROUP.c_str(), end_effector_link.c_str(), joint_names.size());
    
    // 等待机器人状态就绪
    RCLCPP_INFO(LOGGER, "Waiting for robot state...");
    rclcpp::sleep_for(std::chrono::seconds(2));
    // publisher for displaying robot state in RViz
    auto display_pub = move_group_node->create_publisher<moveit_msgs::msg::DisplayRobotState>("/display_robot_state", 10);

    auto compute_gravity_compensation = make_gravity_closure(robot_model);
    
    auto joint_state_sub = move_group_node->create_subscription<sensor_msgs::msg::JointState>(
        "/robo_arm_joint_states", 10,
        [&](const sensor_msgs::msg::JointState::SharedPtr msg) {
            RCLCPP_INFO(LOGGER, "Received joint state message");
            if (!msg) {
                RCLCPP_WARN(LOGGER, "Received null JointState");
                return;
            }
            if (msg->position.size() < joint_names.size()) {
                RCLCPP_WARN(LOGGER, "Received JointState with %zu positions but %zu active joints expected", msg->position.size(), joint_names.size());
                return;
            }

            float original_value1 = msg->position[1];
            float original_value4 = msg->position[4];
            msg->position[1] = -original_value1;
            msg->position[4] = 6.28 - original_value4;

            // 转换为 double 并应用低通滤波，减少角度噪声
            std::vector<double> urdf_angles(msg->position.begin(), msg->position.end());
            
            if (!filter_initialized) {
                // 首次初始化滤波器
                filtered_angles = urdf_angles;
                filter_initialized = true;
            } else {
                // 一阶低通滤波: filtered = alpha * new + (1-alpha) * old
                for (size_t i = 0; i < urdf_angles.size() && i < filtered_angles.size(); ++i) {
                    filtered_angles[i] = ANGLE_FILTER_ALPHA * urdf_angles[i] + 
                                        (1.0 - ANGLE_FILTER_ALPHA) * filtered_angles[i];
                }
            }

            // 使用滤波后的角度计算重力补偿
            std::vector<float> current = compute_gravity_compensation(filtered_angles);

            // cache latest positions and torques for periodic sender
            {
                std::lock_guard<std::mutex> lk(js_mutex);
                latest_positions = msg->position;  // 保存原始转换后的角度用于其他用途
                latest_torques = current;
            }

            // 状态机逻辑
            // 使用 do-while 让同一次回调中可以连续执行多个状态转换
            bool state_changed = true;
            int transition_count = 0;
            const int MAX_TRANSITIONS = 5;  // 防止死循环
            
            while (state_changed && transition_count < MAX_TRANSITIONS) {
                state_changed = false;
                transition_count++;
                
                switch (current_state) {
                    case IDLE: {
                        // 空闲状态，等待规划指令
                        if (planning_command_received) {
                            current_state = PLANNING;
                            state_changed = true;  // 继续执行 PLANNING 分支
                            RCLCPP_INFO(LOGGER, "State: IDLE -> PLANNING");
                        }
                        break;
                    }

                    case PLANNING: {
                    // 计算笛卡尔路径
                    moveit::core::RobotState start_state(robot_model);
                    start_state.setToDefaultValues();
                    for (size_t i = 0; i < joint_names.size(); ++i) {
                        double v = static_cast<double>(msg->position[i]);
                        start_state.setJointPositions(joint_names[i], &v);
                    }
                    start_state.update();
                    move_group.setStartState(start_state);

                    Eigen::Isometry3d ee_transform = start_state.getGlobalLinkTransform(end_effector_link);
                    geometry_msgs::msg::Pose current_end_effector_pose;
                    Eigen::Vector3d trans = ee_transform.translation();
                    Eigen::Quaterniond q(ee_transform.rotation());
                    current_end_effector_pose.position.x = trans.x();
                    current_end_effector_pose.position.y = trans.y();
                    current_end_effector_pose.position.z = trans.z();
                    current_end_effector_pose.orientation.x = q.x();
                    current_end_effector_pose.orientation.y = q.y();
                    current_end_effector_pose.orientation.z = q.z();
                    current_end_effector_pose.orientation.w = q.w();

                    std::vector<geometry_msgs::msg::Pose> waypoints;
                    waypoints.push_back(current_end_effector_pose);
                    Eigen::Vector3d z_prime_direction(0, 0, 1); 
                    
                    // 沿 Z' 轴方向移动 0.1 米 (10cm)
                    geometry_msgs::msg::Pose target_end_effector_pose = 
                        compute_target_pose_along_direction(current_end_effector_pose, z_prime_direction, 0.1);
                    
                    RCLCPP_INFO(LOGGER, "Target pose: [%.3f, %.3f, %.3f]", 
                                target_end_effector_pose.position.x,
                                target_end_effector_pose.position.y,
                                target_end_effector_pose.position.z);
                                    
                    waypoints.push_back(target_end_effector_pose);

                    const double eef_step = 0.01;
                    moveit_msgs::msg::RobotTrajectory trajectory;
                    double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory);

                    if (fraction < 0.9) {
                        RCLCPP_WARN(LOGGER, "Failed to compute Cartesian path, fraction: %.2f", fraction);
                        current_state = IDLE;
                        //planning_command_received = false;
                    } else {
                        RCLCPP_INFO(LOGGER, "Computed Cartesian path with fraction: %.2f", fraction);
                        auto trajectory_dict = extractTrajectory(trajectory);
                        printTrajectory(trajectory_dict);
                        std::vector<std::vector<float>> trajectory_array = trajectory_2d_array(trajectory_dict, joint_names);
                        RCLCPP_INFO(LOGGER, "Trajectory converted to 2D array with %zu time points", trajectory_array.size());

                        current_trajectory = trajectory_array;
                        trajectory_index = 0;
                        last_target_pose = target_end_effector_pose;
                        current_state = SENDING;
                        state_changed = true;  // 继续执行 SENDING 分支
                        planning_command_received = false;
                        RCLCPP_INFO(LOGGER, "State: PLANNING -> SENDING");
                    }
                    break;
                }

                case SENDING: {
                    // 发送一帧，然后转到 CHECKING
                    if (trajectory_index < current_trajectory.size()) {
                        auto ja = current_trajectory[trajectory_index];
                        std::vector<float> send_angles = ja;
                        if (joint_names.size() > 1) {
                            send_angles[1] = -send_angles[1];
                            send_angles[4] = 6.28 - send_angles[4];
                        }
                        serial_node->send_roboarm_with_controller(current, send_angles);
                        RCLCPP_INFO(LOGGER, "Sent frame %zu/%zu", trajectory_index + 1, current_trajectory.size());
                        last_sent_angles = send_angles;
                        last_send_time = std::chrono::steady_clock::now();
                        current_state = CHECKING;
                        RCLCPP_INFO(LOGGER, "State: SENDING -> CHECKING");
                    } else {
                        // 所有帧已发完（理论上不应该到这里，因为在 CHECKING 会判断）
                        RCLCPP_WARN(LOGGER, "All frames sent, going to IDLE");
                        current_state = IDLE;
                    }
                    break;
                }

                case CHECKING: {
                    // 检查当前角度是否接近目标角度
                    auto now = std::chrono::steady_clock::now();
                    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send_time).count();
                    
                    bool reached = false;
                    if (elapsed > 2000) {  // 超时 2 秒认为到位（根据实际调整）
                        reached = true;
                        RCLCPP_WARN(LOGGER, "Timeout: %ld ms elapsed, considering reached", elapsed);
                    } else {
                        // 检查角度误差（推荐启用！）
                        reached = true;
                        for (size_t i = 0; i < joint_names.size() && i < last_sent_angles.size(); ++i) {
                            double diff;
                            if (i == 1) {
                                // joint2 需要反向比较
                                diff = std::abs(msg->position[i] + last_sent_angles[i]);
                            } else if (i == 4) {
                                // joint5 需要偏移比较
                                diff = std::abs(msg->position[i] - (6.28 - last_sent_angles[i]));
                            } else {
                                diff = std::abs(msg->position[i] - last_sent_angles[i]);
                            }
                            
                            if (diff > 0.05) {  // 容差 0.05 弧度 (约 2.86 度)
                                reached = false;
                                RCLCPP_DEBUG(LOGGER, "Joint %zu not reached: diff=%.4f", i, diff);
                                break;
                            }
                        }
                        if (reached) {
                            RCLCPP_INFO(LOGGER, "All joints reached target (elapsed: %ld ms)", elapsed);
                        }
                    }

                    if (reached) {
                        trajectory_index++;
                        if (trajectory_index < current_trajectory.size()) {
                            // 还有剩余帧，继续发送
                            current_state = SENDING;
                            state_changed = true;  // 继续执行 SENDING 分支
                            RCLCPP_INFO(LOGGER, "Frame reached, State: CHECKING -> SENDING");
                        } else {
                            // 所有帧发完，检查是否到达目标位置
                            moveit::core::RobotState start_state(robot_model);
                            start_state.setToDefaultValues();
                            for (size_t i = 0; i < joint_names.size(); ++i) {
                                double v = static_cast<double>(msg->position[i]);
                                start_state.setJointPositions(joint_names[i], &v);
                            }
                            start_state.update();

                            Eigen::Isometry3d ee_transform = start_state.getGlobalLinkTransform(end_effector_link);
                            geometry_msgs::msg::Pose current_end_effector_pose;
                            Eigen::Vector3d trans = ee_transform.translation();
                            Eigen::Quaterniond q(ee_transform.rotation());
                            current_end_effector_pose.position.x = trans.x();
                            current_end_effector_pose.position.y = trans.y();
                            current_end_effector_pose.position.z = trans.z();
                            current_end_effector_pose.orientation.x = q.x();
                            current_end_effector_pose.orientation.y = q.y();
                            current_end_effector_pose.orientation.z = q.z();
                            current_end_effector_pose.orientation.w = q.w();

                            double dx = current_end_effector_pose.position.x - last_target_pose.position.x;
                            double dy = current_end_effector_pose.position.y - last_target_pose.position.y;
                            double dz = current_end_effector_pose.position.z - last_target_pose.position.z;
                            double distance_to_target = sqrt(dx * dx + dy * dy + dz * dz);

                            if (distance_to_target <= 0.005) {  // 到达目标位置范围内（0.5cm）
                                RCLCPP_INFO(LOGGER, "Target reached! Distance: %.4f m, State: CHECKING -> IDLE", distance_to_target);
                                current_state = IDLE;
                            } else {
                                RCLCPP_INFO(LOGGER, "All frames sent but not at target. Distance: %.4f m, staying in CHECKING", distance_to_target);
                                // 可以选择：继续等待到位，或者直接回 IDLE
                                current_state = IDLE;  // 或者保持 CHECKING 继续等待
                            }
                        }
                    } else {
                        RCLCPP_DEBUG(LOGGER, "Still moving to frame %zu", trajectory_index);
                    }
                    break;
                }
            }
            }  // end of while loop

            // 更新 RobotState 用于 RViz 显示和规划
            moveit::core::RobotState start_state(robot_model);
            start_state.setToDefaultValues();
            for (size_t i = 0; i < joint_names.size(); ++i) {
                double v = static_cast<double>(msg->position[i]);
                start_state.setJointPositions(joint_names[i], &v);
            }
            start_state.update();
            move_group.setStartState(start_state);

            // Publish robot state to RViz
            moveit_msgs::msg::RobotState robot_state_msg;
            moveit::core::robotStateToRobotStateMsg(start_state, robot_state_msg);
            moveit_msgs::msg::DisplayRobotState display_msg;
            display_msg.state = robot_state_msg;
            display_pub->publish(display_msg);

            Eigen::Isometry3d ee_transform = start_state.getGlobalLinkTransform(end_effector_link);
            geometry_msgs::msg::Pose current_end_effector_pose;
            // Manual conversion from Eigen::Isometry3d to geometry_msgs::msg::Pose
            Eigen::Vector3d trans = ee_transform.translation();
            Eigen::Quaterniond q(ee_transform.rotation());
            current_end_effector_pose.position.x = trans.x();
            current_end_effector_pose.position.y = trans.y();
            current_end_effector_pose.position.z = trans.z();
            current_end_effector_pose.orientation.x = q.x();
            current_end_effector_pose.orientation.y = q.y();
            current_end_effector_pose.orientation.z = q.z();
            current_end_effector_pose.orientation.w = q.w();

            RCLCPP_DEBUG(LOGGER, "Current end effector pose - Position: [%.3f, %.3f, %.3f], Orientation: [%.3f, %.3f, %.3f, %.3f]",
                        current_end_effector_pose.position.x,
                        current_end_effector_pose.position.y,
                        current_end_effector_pose.position.z,
                        current_end_effector_pose.orientation.x,
                        current_end_effector_pose.orientation.y,
                        current_end_effector_pose.orientation.z,
                        current_end_effector_pose.orientation.w);
        });
    // Periodic timer: when not actively sending a trajectory, keep sending gravity torques
    auto torque_timer = move_group_node->create_wall_timer(std::chrono::milliseconds(50),
        [&, serial_node]() {
            std::vector<float> torques;
            {
                std::lock_guard<std::mutex> lk(js_mutex);
                if (latest_torques.empty()) return;
                torques = latest_torques;
            }
            // 在 IDLE、PLANNING、CHECKING 状态下周期性发送力矩（保持姿态）
            // SENDING 状态下不发送（每帧已经包含力矩）
            if (current_state == IDLE || current_state == PLANNING || current_state == CHECKING) {
                std::vector<float> send_angles = last_sent_angles.empty() 
                    ? std::vector<float>(joint_names.size(), 0.0f) 
                    : last_sent_angles;
                serial_node->send_roboarm_with_controller(torques, send_angles);
                RCLCPP_DEBUG(LOGGER, "Periodic torque send (state: %d)", static_cast<int>(current_state));
            }
        });

    RCLCPP_INFO(LOGGER, "Starting executor spin...");
    executor.spin();  // 阻塞在这里，直到收到关闭信号
    
    RCLCPP_INFO(LOGGER, "Shutting down...");
    rclcpp::shutdown();
    
    // 等待接收线程结束
    if (receive_thread.joinable()) {
        receive_thread.join();
    }
    
    return 0;
}