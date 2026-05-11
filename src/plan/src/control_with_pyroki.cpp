/**
 * @file control_with_pyroki.cpp
 * @brief 配合 pyroki_ik_node 使用：
 *        - 订阅 /joint_states 拿到 pyroki 实时算出的关节角作为规划起点
 *        - 启动时通过命令行参数选择动作
 *            1 = UP, 2 = DOWN, 3 = LEFT, 4 = RIGHT, 5 = TURN
 *        - 在 RViz 的 RvizVisualToolsGui 里点 "Next" 触发一次规划+执行
 */

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/msg/display_robot_state.hpp>

#include "utils.hpp"

enum ControlAction {
    ACTION_NONE  = 0,
    ACTION_UP    = 1,
    ACTION_DOWN  = 2,
    ACTION_LEFT  = 3,
    ACTION_RIGHT = 4,
    ACTION_TURN  = 5,
};

static const rclcpp::Logger LOGGER = rclcpp::get_logger("control_with_pyroki");
static const std::string PLANNING_GROUP = "arm";
static const double MOVE_STEP = 0.01;  // 1cm

// pyroki 发来的关节角缓存
static std::mutex g_js_mutex;
static std::vector<double> g_current_joint_states(6, 0.0);
static std::atomic<bool> g_have_joint_states{false};

static const std::vector<std::string> JOINT_NAMES = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};


/**
 * @brief 把缓存的关节角设为 move_group 的规划起点
 */
static void set_start_state_from_pyroki(
    const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group,
    const moveit::core::JointModelGroup* jmg)
{
    if (!g_have_joint_states.load()) {
        RCLCPP_WARN(LOGGER, "Still no /joint_states from pyroki, using current robot state");
        return;
    }
    std::vector<double> joints;
    {
        std::lock_guard<std::mutex> lk(g_js_mutex);
        joints = g_current_joint_states;
    }
    auto state = move_group->getCurrentState();
    state->setJointGroupPositions(jmg, joints);
    move_group->setStartState(*state);
    RCLCPP_INFO(LOGGER, "Start state from pyroki: [%.3f %.3f %.3f %.3f %.3f %.3f]",
                joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);
}


static void execute_translate(
    const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group,
    const moveit::core::JointModelGroup* jmg,
    moveit_visual_tools::MoveItVisualTools& visual_tools,
    const std::string& dir)
{
    set_start_state_from_pyroki(move_group, jmg);

    geometry_msgs::msg::Pose target = move_group->getCurrentPose().pose;
    if (dir == "UP")    target.position.z += MOVE_STEP;
    if (dir == "DOWN")  target.position.z -= MOVE_STEP;
    if (dir == "LEFT")  target.position.y += MOVE_STEP;
    if (dir == "RIGHT") target.position.y -= MOVE_STEP;

    move_group->setPoseTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(LOGGER, "Plan %s failed", dir.c_str());
        return;
    }
    visual_tools.publishTrajectoryLine(plan.trajectory, jmg);
    visual_tools.trigger();
    // 仿真节点: 不调用 move_group->execute(), 也不发到任何硬件 controller
}


static void execute_turn(
    const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group,
    const moveit::core::JointModelGroup* jmg,
    moveit_visual_tools::MoveItVisualTools& visual_tools)
{
    set_start_state_from_pyroki(move_group, jmg);

    const geometry_msgs::msg::Pose start_pose = move_group->getCurrentPose().pose;
    const double radius = 0.1;
    const int num_points = 40;

    const Eigen::Quaterniond start_quat(
        start_pose.orientation.w, start_pose.orientation.x,
        start_pose.orientation.y, start_pose.orientation.z);
    const Eigen::Vector3d x_axis_world = (start_quat * Eigen::Vector3d::UnitX()).normalized();
    const Eigen::Vector3d y_axis_world = (start_quat * Eigen::Vector3d::UnitY()).normalized();

    auto trajectory = plan_half_circle_turn(
        start_pose, radius, num_points,
        /*normal=*/y_axis_world, /*e_hat=*/x_axis_world, move_group);

    if (trajectory.joint_trajectory.points.empty()) {
        RCLCPP_ERROR(LOGGER, "TURN trajectory planning failed");
        return;
    }
    visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(trajectory, jmg);
    visual_tools.trigger();

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory = trajectory;
    // 仿真节点: 不真执行
    (void)plan;
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    if (argc < 2) {
        RCLCPP_ERROR(LOGGER,
            "Usage: control_with_pyroki <action>\n"
            "  1 = UP, 2 = DOWN, 3 = LEFT, 4 = RIGHT, 5 = TURN");
        rclcpp::shutdown();
        return 1;
    }
    int action = std::atoi(argv[1]);
    if (action < 1 || action > 5) {
        RCLCPP_ERROR(LOGGER, "Invalid action %d (expected 1..5)", action);
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("control_with_pyroki", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    // 订阅 pyroki 发的 /joint_states
    auto js_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [](const sensor_msgs::msg::JointState::SharedPtr msg) {
            std::lock_guard<std::mutex> lk(g_js_mutex);
            for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
                for (size_t j = 0; j < JOINT_NAMES.size(); ++j) {
                    if (msg->name[i] == JOINT_NAMES[j]) {
                        g_current_joint_states[j] = msg->position[i];
                        break;
                    }
                }
            }
            g_have_joint_states.store(true);
        });

    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        node, PLANNING_GROUP);
    move_group->setEndEffectorLink("gripper");
    const moveit::core::JointModelGroup* jmg =
        move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    moveit_visual_tools::MoveItVisualTools visual_tools(
        node, "base_link", "control_with_pyroki", move_group->getRobotModel());
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    RCLCPP_INFO(LOGGER, "Waiting for /joint_states from pyroki...");
    while (rclcpp::ok() && !g_have_joint_states.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(LOGGER, "Got /joint_states. Selected action = %d", action);

    while (rclcpp::ok()) {
        visual_tools.prompt(
            "Press 'next' in the RvizVisualToolsGui window to plan & execute action");

        switch (action) {
            case ACTION_UP:    execute_translate(move_group, jmg, visual_tools, "UP");    break;
            case ACTION_DOWN:  execute_translate(move_group, jmg, visual_tools, "DOWN");  break;
            case ACTION_LEFT:  execute_translate(move_group, jmg, visual_tools, "LEFT");  break;
            case ACTION_RIGHT: execute_translate(move_group, jmg, visual_tools, "RIGHT"); break;
            case ACTION_TURN:  execute_turn(move_group, jmg, visual_tools);               break;
            default: break;
        }
    }

    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
