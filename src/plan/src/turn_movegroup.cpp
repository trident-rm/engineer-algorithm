/** 
 * @file turn_movegroup.cpp
 * @brief  串口控制模式：IDLE/HOME/TURN 三种模式控制机械臂
 * @author  Grace
 * @date    2026-03-31
 * @modified 2026-04-24
*/

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <thread>
#include <chrono>
#include "utils.hpp"
#include "tools/usb_data.hpp"

enum SerialControlMode {
    SERIAL_MODE_IDLE = 0,
    SERIAL_MODE_HOME = 1,
    SERIAL_MODE_TURN = 2
};

static const rclcpp::Logger LOGGER = rclcpp::get_logger("z_plan_movegroup_robo_arm");
static const std::vector<double> RESET_JOINT_VALUES = {-1.55, -0.77, 0.67, 0.0, 1.37, -0.05};
static uint8_t last_control_mode(255);
static std::vector<float> torques(6, 0.0f);

/**
 * @brief 逐帧发送轨迹到串口
 */
void send_trajectory_to_serial(
    const moveit_msgs::msg::RobotTrajectory& trajectory,
    std::shared_ptr<SerialRxNode> serial_node,
    int frame_interval_ms = 50)
{
    const auto& points = trajectory.joint_trajectory.points;
    for (size_t i = 0; i < points.size(); ++i) {
        const auto& point = points[i];
        std::vector<float> frame_angles(point.positions.begin(), point.positions.end());
        serial_node->send_roboarm_with_controller(torques, convert_urdf_to_hardware(frame_angles));
        
        RCLCPP_DEBUG(LOGGER, "Sent trajectory frame %zu/%zu", i + 1, points.size());
        std::this_thread::sleep_for(std::chrono::milliseconds(frame_interval_ms));
    }
    
    RCLCPP_INFO(LOGGER, "Trajectory transmission completed");
}

void execute_home(
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
    std::shared_ptr<SerialRxNode> serial_node)
{
    RCLCPP_INFO(LOGGER, "Executing HOME (reset) command");
    move_group->setJointValueTarget(RESET_JOINT_VALUES);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            move_group->execute(plan);
            RCLCPP_INFO(LOGGER, "HOME execution started");
            send_trajectory_to_serial(plan.trajectory, serial_node, 50);
            return;
    } else {
            RCLCPP_WARN(LOGGER, "Failed to plan HOME motion");
    }  
}

void execute_turn(
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
    const moveit::core::JointModelGroup* joint_model_group,
    moveit_visual_tools::MoveItVisualTools& visual_tools,
    std::shared_ptr<SerialRxNode> serial_node)
{
    RCLCPP_INFO(LOGGER, "Executing TURN command");
    const geometry_msgs::msg::Pose start_pose = move_group->getCurrentPose().pose;
    const double radius = 0.1;
    const int num_points = 40;
    const Eigen::Quaterniond start_quat(
        start_pose.orientation.w,
        start_pose.orientation.x,
        start_pose.orientation.y,
        start_pose.orientation.z);

    const Eigen::Vector3d x_axis_world = (start_quat * Eigen::Vector3d::UnitX()).normalized();
    const Eigen::Vector3d y_axis_world = (start_quat * Eigen::Vector3d::UnitY()).normalized();

    const Eigen::Vector3d normal_direction = y_axis_world;
    const Eigen::Vector3d e_hat_direction = x_axis_world;

    moveit_msgs::msg::RobotTrajectory trajectory =
        plan_half_circle_turn(start_pose, radius, num_points, normal_direction, e_hat_direction, move_group);

    if (trajectory.joint_trajectory.points.empty()) {
        RCLCPP_ERROR(LOGGER, "Half-circle trajectory planning failed");
        return;
    }

    visual_tools.deleteAllMarkers();
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "Half_Circle_Path", rviz_visual_tools::BLACK, rviz_visual_tools::XLARGE);
    visual_tools.publishTrajectoryLine(trajectory, joint_model_group);
    visual_tools.trigger();

    moveit::planning_interface::MoveGroupInterface::Plan half_circle_plan;
    half_circle_plan.trajectory = trajectory;
    move_group->execute(half_circle_plan);
    
    send_trajectory_to_serial(trajectory, serial_node, 50);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("z_plan_movegroup_robo_arm", node_options);
    
    std::shared_ptr<SerialRxNode> serial_node = std::make_shared<SerialRxNode>();
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    executor.add_node(serial_node);
    std::thread([&executor]() { executor.spin(); }).detach();


    RCLCPP_INFO(LOGGER, "Waiting for serial port to open...");
    while (true) {
        auto angles = serial_node->get_robo_arm_angles();
        if (angles.size() == 6) {
            RCLCPP_INFO(LOGGER, "Serial port opened, entering control mode");
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    static const std::string PLANNING_GROUP = "arm";
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node, PLANNING_GROUP);
    move_group->setEndEffectorLink("gripper");

    const moveit::core::JointModelGroup* joint_model_group =
        move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "base_link", "turn_plan",
                                                        move_group->getRobotModel());

    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group->getEndEffectorLink().c_str());

    last_control_mode = 255;
    while (true) {
        uint8_t current_mode = serial_node->get_control_mode();
        if (current_mode == last_control_mode) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        // 从串口读取当前关节角度并设置为起始状态
        auto real_joint_angles = convert_hardware_to_urdf(serial_node->get_robo_arm_angles());
        if (real_joint_angles.size() == 6) {
            auto current_state = move_group->getCurrentState();
            std::vector<double> joint_angles_double(real_joint_angles.begin(), real_joint_angles.end());
            current_state->setJointGroupPositions(joint_model_group, joint_angles_double);
            move_group->setStartState(*current_state);
            RCLCPP_DEBUG(LOGGER, "Set start state from serial: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                        joint_angles_double[0], joint_angles_double[1], joint_angles_double[2],
                        joint_angles_double[3], joint_angles_double[4], joint_angles_double[5]);
        }

        switch (current_mode) {
            case SERIAL_MODE_HOME: {
                RCLCPP_INFO(LOGGER, "Received HOME command");
                execute_home(move_group, serial_node);
                break;
            }
            case SERIAL_MODE_TURN: {
                RCLCPP_INFO(LOGGER, "Received TURN command");
                execute_turn(move_group, joint_model_group, visual_tools, serial_node);
                break;
            }
            default:
                break;
        }
        if (current_mode != SERIAL_MODE_IDLE) {
            last_control_mode = current_mode;
        }
    }

    rclcpp::shutdown();
    return 0;
}
