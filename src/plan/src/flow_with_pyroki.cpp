/**
 * @file flow_with_pyroki.cpp
 * @brief 纯仿真工作流第一版: 收到 /key_input == "w" => 末端 +z 1cm.
 *        起点关节角来自 /joint_states (pyroki_ik_node 发的).
 *        不调用 execute, 只在 RViz 画轨迹线.
 *
 * 配套用 ros2 run plan keyboard_input 把键盘输入转成 /key_input 话题.
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
#include <std_msgs/msg/string.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("flow_with_pyroki");
static const std::string PLANNING_GROUP = "arm";
static const double STEP = 0.01;

static const std::vector<std::string> JOINT_NAMES = {
    "joint1","joint2","joint3","joint4","joint5","joint6"};

static std::mutex g_js_mutex;
static std::vector<double> g_current_joints(6, 0.0);
static std::atomic<bool> g_have_js{false};

static void plan_translate_z(
    const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& mg,
    const moveit::core::JointModelGroup* jmg,
    moveit_visual_tools::MoveItVisualTools& vt,
    double dz)
{
    if (g_have_js.load()) {
        std::vector<double> joints;
        { std::lock_guard<std::mutex> lk(g_js_mutex); joints = g_current_joints; }
        auto state = mg->getCurrentState();
        state->setJointGroupPositions(jmg, joints);
        state->enforceBounds();
        mg->setStartState(*state);
        RCLCPP_INFO(LOGGER, "Start joints: [%.3f %.3f %.3f %.3f %.3f %.3f]",
                    joints[0],joints[1],joints[2],joints[3],joints[4],joints[5]);
    }
    geometry_msgs::msg::Pose target = mg->getCurrentPose().pose;
    target.position.z += dz;
    mg->setPoseTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto rc = mg->plan(plan);
    if (rc != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(LOGGER, "Plan failed (code=%d)", static_cast<int>(rc.val));
        return;
    }
    vt.deleteAllMarkers();
    vt.publishTrajectoryLine(plan.trajectory, jmg);
    vt.trigger();
    RCLCPP_INFO(LOGGER, "Planned %zu points (sim only)",
                plan.trajectory.joint_trajectory.points.size());
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions opts;
    opts.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("flow_with_pyroki", opts);

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    std::thread spin_thread([&exec]() { exec.spin(); });

    auto js_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [](const sensor_msgs::msg::JointState::SharedPtr msg) {
            std::lock_guard<std::mutex> lk(g_js_mutex);
            for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
                for (size_t j = 0; j < JOINT_NAMES.size(); ++j)
                    if (msg->name[i] == JOINT_NAMES[j]) {
                        g_current_joints[j] = msg->position[i]; break;
                    }
            g_have_js.store(true);
        });

    auto mg = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, PLANNING_GROUP);
    mg->setEndEffectorLink("gripper");
    const auto* jmg = mg->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    moveit_visual_tools::MoveItVisualTools vt(
        node, "base_link", "flow_with_pyroki", mg->getRobotModel());
    vt.deleteAllMarkers();

    RCLCPP_INFO(LOGGER, "Waiting for /joint_states...");
    while (rclcpp::ok() && !g_have_js.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    RCLCPP_INFO(LOGGER, "Ready. Send 'w' on /key_input to plan +z 1cm, 'q' to quit.");

    auto key_sub = node->create_subscription<std_msgs::msg::String>(
        "/key_input", 10,
        [&](const std_msgs::msg::String::SharedPtr msg) {
            if (msg->data.empty()) return;
            char c = msg->data[0];
            if (c == 'w' || c == 'W') {
                plan_translate_z(mg, jmg, vt, +STEP);
            } else if (c == 'q' || c == 'Q') {
                RCLCPP_INFO(LOGGER, "Quit");
                rclcpp::shutdown();
            } else {
                RCLCPP_INFO(LOGGER, "Key '%c' - no action", c);
            }
        });

    while (rclcpp::ok())
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (spin_thread.joinable()) spin_thread.join();
    return 0;
}