/** 
 * @file key_step_control.cpp
 * @brief   先回到基础位置，根据按键输入，一次朝对应方向移动1cm
 * @date    2026-04-23
*/

#include <algorithm>
#include <iostream>
#include <iterator>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <chrono>
#include <atomic>

#include <termios.h>
#include <unistd.h>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/conversions.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "utils.hpp"
#include "tools/usb_data.hpp"
enum SerialControlMode {
    SERIAL_MODE_IDLE = 0,
    SERIAL_MODE_HOME = 1,
    SERIAL_MODE_UP = 2,
    SERIAL_MODE_DOWN = 3,
    SERIAL_MODE_LEFT = 4,
    SERIAL_MODE_RIGHT = 5,
    SERIAL_MODE_FORWARD = 6,
    SERIAL_MODE_BACKWARD = 7,
    SERIAL_MODE_PUSH = 8,
    SERIAL_MODE_PULL = 9,
    SERIAL_MODE_TURN = 10
};

static const rclcpp::Logger LOGGER = rclcpp::get_logger("key_step_control");
static const double MOVE_STEP = 0.01;
static const std::vector<double> RESET_JOINT_VALUES = {0, -1, 0.67, -3.14, 5.8, -1.3};
static uint8_t last_control_mode(255); 
static std::vector<float> torques(6, 0.0f); 
static std::vector<double> joint_angles(6, 0.0);
static std::vector<double> current_joint_states(6, 0.0);  // 从 /joint_states 话题订阅



/**
 * @brief 逐帧发送轨迹到串口
 * @param trajectory 规划出的轨迹
 * @param serial_node 串口节点
 * @param frame_interval_ms 每帧间隔（毫秒）
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
        
        // 打印 torques 的16进制形式
        std::string torques_hex = "Torques (hex): ";
        for (size_t j = 0; j < torques.size(); ++j) {
            char hex_buf[16];
            uint32_t bits = *reinterpret_cast<uint32_t*>(&torques[j]);
            snprintf(hex_buf, sizeof(hex_buf), "%08x ", bits);
            torques_hex += hex_buf;
        }
        RCLCPP_INFO(LOGGER, "%s", torques_hex.c_str());
        
        // 打印 frame_angles 的16进制形式
        std::string angles_hex = "Frame angles (hex): ";
        for (size_t j = 0; j < frame_angles.size(); ++j) {
            char hex_buf[16];
            uint32_t bits = *reinterpret_cast<uint32_t*>(&frame_angles[j]);
            snprintf(hex_buf, sizeof(hex_buf), "%08x ", bits);
            angles_hex += hex_buf;
        }
        RCLCPP_INFO(LOGGER, "%s", angles_hex.c_str());
        
        // 打印转换后的硬件数据
        auto hw_angles = convert_urdf_to_hardware(frame_angles);
        std::string hw_hex = "Hardware angles (hex): ";
        for (size_t j = 0; j < hw_angles.size(); ++j) {
            char hex_buf[16];
            uint32_t bits = *reinterpret_cast<uint32_t*>(&hw_angles[j]);
            snprintf(hex_buf, sizeof(hex_buf), "%08x ", bits);
            hw_hex += hex_buf;
        }
        RCLCPP_INFO(LOGGER, "%s", hw_hex.c_str());
        
        serial_node->send_roboarm_with_controller(torques, hw_angles);
        
        RCLCPP_DEBUG(LOGGER, "Sent trajectory frame %zu/%zu", i + 1, points.size());
        std::this_thread::sleep_for(std::chrono::milliseconds(frame_interval_ms));
    }
    
    RCLCPP_INFO(LOGGER, "Trajectory transmission completed");
}

void execute_single_move(
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
    const moveit::core::JointModelGroup* joint_model_group,
    const std::string& direction,
    moveit_visual_tools::MoveItVisualTools& visual_tools,
    std::shared_ptr<SerialRxNode> serial_node)
{
    geometry_msgs::msg::Pose current_pose = move_group->getCurrentPose().pose;
    geometry_msgs::msg::Pose target_pose = current_pose;
    
    if (direction == "UP") {
        target_pose.position.z += MOVE_STEP;
    } else if (direction == "DOWN") {
        target_pose.position.z -= MOVE_STEP;
    } else if (direction == "LEFT") {
        target_pose.position.y += MOVE_STEP;
    } else if (direction == "RIGHT") {
        target_pose.position.y -= MOVE_STEP;
    } else if (direction == "FORWARD") {
        target_pose.position.x += MOVE_STEP;
    } else if (direction == "BACKWARD") {
        target_pose.position.x -= MOVE_STEP;
    }
    
    move_group->setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = false;
    while(!success) {
        success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (!success) {
            RCLCPP_WARN(LOGGER, "Failed to plan motion. Retrying...");
        }
    }
    
    visual_tools.publishTrajectoryLine(plan.trajectory, joint_model_group);
    // move_group->execute(plan);
    send_trajectory_to_serial(plan.trajectory, serial_node, 50);
}


void execute_home(
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
    std::shared_ptr<SerialRxNode> serial_node)
{
    RCLCPP_INFO(LOGGER, "Executing HOME (reset) command");
    move_group->setJointValueTarget(RESET_JOINT_VALUES);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(LOGGER, "HOME execution started");
            //move_group->execute(plan);
            send_trajectory_to_serial(plan.trajectory, serial_node, 50);
            return;
    } else {
            RCLCPP_WARN(LOGGER, "Failed to plan HOME motion");
    }  
}


int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("key_step_control", node_options);
  std::shared_ptr<SerialRxNode> serial_node = std::make_shared<SerialRxNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(serial_node);
  std::thread executor_thread([&executor]() { executor.spin(); });

  //std::thread receive_thread([serial_node]() {
  //      serial_node->receive_data();
  //});

  static const std::string PLANNING_GROUP = "arm";
  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, PLANNING_GROUP);
  move_group->setEndEffectorLink("gripper");
  const moveit::core::JointModelGroup* joint_model_group =
      move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  namespace rvt = rviz_visual_tools;  
  moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link", "key_step_control",
                                                      move_group->getRobotModel());
  //auto current_state = move_group->getCurrentState();
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  last_control_mode = 255;
  
  // 订阅 /joint_states 话题获取实时关节状态
  auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [](const sensor_msgs::msg::JointState::SharedPtr msg) {
          if (msg->position.size() >= 6) {
              for (size_t i = 0; i < 6; ++i) {
                  current_joint_states[i] = msg->position[i];
              }
          }
      });
  
  // 创建 joint_state 发布器
  //auto joint_state_pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  
  // 创建 display_robot_state 发布器（RViz 期望的话题名）
  //auto display_state_pub = node->create_publisher<moveit_msgs::msg::DisplayRobotState>(
  //    "/display_robot_state", rclcpp::QoS(1).transient_local());


  while(true) {  
    //std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    static auto last_print_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    
    //if (now-last_print_time >= std::chrono::seconds(5)) {
        RCLCPP_INFO(LOGGER, "========== Current Joint States (from /joint_states) ==========");
        for (size_t i = 0; i < current_joint_states.size(); ++i) {
            RCLCPP_INFO(LOGGER, "Current joint[%zu] angle: %.4f rad", i + 1, current_joint_states[i]);
        }
        RCLCPP_INFO(LOGGER, "==============================================================");
        last_print_time = now;
    //}
    
    //current_state->setJointGroupPositions(joint_model_group, joint_angles);
    //move_group->setStartState(*current_state);
    visual_tools.publishRobotState(*move_group->getCurrentState(), rviz_visual_tools::GREEN);
    
    uint8_t current_mode = serial_node->get_control_mode();
    RCLCPP_INFO(LOGGER, "Current control mode from serial: %d", current_mode);
    current_mode = 0;
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move to reset joint state");
    current_mode = 1;
    switch(current_mode) {
      case SERIAL_MODE_HOME: {
        RCLCPP_INFO(LOGGER, "Received HOME command");
        execute_home(move_group, serial_node);
        break;
      }
      case SERIAL_MODE_UP: {
        RCLCPP_INFO(LOGGER, "Received UP command");
        execute_single_move(move_group, joint_model_group, "UP", visual_tools, serial_node);
        break;
      }
      case SERIAL_MODE_DOWN: {
        RCLCPP_INFO(LOGGER, "Received DOWN command");
        execute_single_move(move_group, joint_model_group, "DOWN", visual_tools, serial_node);
        break;
      }
      case SERIAL_MODE_LEFT: {
        RCLCPP_INFO(LOGGER, "Received LEFT command");
        execute_single_move(move_group, joint_model_group, "LEFT", visual_tools, serial_node);
        break;
      }
      case SERIAL_MODE_RIGHT: {
        RCLCPP_INFO(LOGGER, "Received RIGHT command");
        execute_single_move(move_group, joint_model_group, "RIGHT", visual_tools, serial_node);
        break;
      }
      case SERIAL_MODE_FORWARD: {
        RCLCPP_INFO(LOGGER, "Received FORWARD command");
        execute_single_move(move_group, joint_model_group, "FORWARD", visual_tools, serial_node);
        break;
      }
      case SERIAL_MODE_BACKWARD: {
        RCLCPP_INFO(LOGGER, "Received BACKWARD command");
        execute_single_move(move_group, joint_model_group, "BACKWARD", visual_tools, serial_node);
        break;
      }

      case SERIAL_MODE_TURN: {
        RCLCPP_INFO(LOGGER, "Received TURN command");
        break;
      }


      default:
        break;
    }
    
    if (current_mode != SERIAL_MODE_IDLE) {
      last_control_mode = current_mode;
    }
  }

  executor_thread.join();
  rclcpp::shutdown();
  return 0;
}